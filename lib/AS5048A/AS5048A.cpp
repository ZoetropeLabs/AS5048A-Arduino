#include "Arduino.h"

#include <AS5048A.h>

// #define AS5048A_DEBUG

static const uint16_t AS5048A_CLEAR_ERROR_FLAG              = 0x0001;
static const uint16_t AS5048A_PROGRAMMING_CONTROL           = 0x0003;
static const uint16_t AS5048A_OTP_REGISTER_ZERO_POS_HIGH    = 0x0016;
static const uint16_t AS5048A_OTP_REGISTER_ZERO_POS_LOW     = 0x0017;
static const uint16_t AS5048A_DIAG_AGC                      = 0x3FFD;
static const uint16_t AS5048A_MAGNITUDE                     = 0x3FFE;
static const uint16_t AS5048A_ANGLE                         = 0x3FFF;
static const uint16_t AS5048A_NOP 	                        = 0x0000;
static const uint16_t AS5048A_READ_FLAG						= 0b0100000000000000; 
static const uint16_t AS5048A_WRITE_FLAG					= 0b0000000000000000; 

static const float AS5048A_MAX_VALUE = 8191.0;
static const float AS5048A_TWICE_MAX_VALUE = AS5048A_MAX_VALUE * 2.0;
static const float AS5048A_PI  = 3.14159265358979323846;

// Hamming weight computation masks
static const uint16_t mask_01  = 0x5555; //binary: 0101...
static const uint16_t mask_0011  = 0x3333; //binary: 00110011..
static const uint16_t mask_00001111  = 0x0f0f; //binary:  4 zeros,  4 ones ...
static const uint64_t h01 = 0x0101010101010101; //the sum of 256 to the power of 0,1,2,3...


/**
 * Constructor usign response delay (ESP32 and similars)
 */
AS5048A::AS5048A(uint8_t arg_cs, uint8_t arg_response_delay_millis):
	_cs(arg_cs),
	response_delay_millis(arg_response_delay_millis),
	errorFlag(false) {
}

/**
 * Constructor zero response delay (Arduino UNO and similars)
 */
AS5048A::AS5048A(uint8_t arg_cs):
	_cs(arg_cs),
	response_delay_millis(0),
	errorFlag(false) {
}

/**
 * Initialiser
 * Sets up the SPI interface
 */
void AS5048A::init(){
	// 1MHz clock (AMS should be able to accept up to 10MHz)
	settings = SPISettings(1000000, MSBFIRST, SPI_MODE1);

	//setup pins
	pinMode(_cs, OUTPUT);
	digitalWrite(_cs, HIGH);

	//SPI has an internal SPI-device counter, it is possible to call "begin()" from different devices
	SPI.begin();
}

/**
 * Closes the SPI connection
 * SPI has an internal SPI-device counter, for each init()-call the close() function must be called exactly 1 time
 */
void AS5048A::close(){
	SPI.end();
}

/**
 * Utility function used to calculate even parity of an unigned 16 bit integer.
 * It computes the hamming weight using wikipedia article: https://en.wikipedia.org/wiki/Hamming_weight
 * checking if it's even.
 */
uint16_t AS5048A::spiCalcEvenParity(uint16_t x){
	x -= (x >> 1) & mask_01;             //put count of each 2 bits into those 2 bits
    x = (x & mask_0011) + ((x >> 2) & mask_0011); //put count of each 4 bits into those 4 bits 
    x = (x + (x >> 4)) & mask_00001111;        //put count of each 8 bits into those 8 bits 
    uint16_t hamming_weight = (x * h01) >> 56;  //returns left 8 bits of x + (x<<8) + (x<<16) + (x<<24) + ... 
	uint16_t is_even = hamming_weight & 0x0001;
	return (uint16_t)(is_even<<15);
}

/**
 * Get the rotation of the sensor relative to the zero position.
 *
 * @return {int32_t} between -2^13 and 2^13
 */
int32_t AS5048A::getRotation(){
	uint16_t data = AS5048A::getRawRotation();
	int32_t rotation = (int32_t)data;
	if(rotation > 8191) rotation = -((0x3FFF)-rotation); //more than -180
	return rotation;
}

/**
 * Get the rotation of the sensor relative to the zero position in degrees.
 *
 * @return {float} between 0 and 360
 */

float AS5048A::getRotationInDegrees(){
	int32_t rotation = getRotation();
	float degrees = 360.0 * (rotation + AS5048A_MAX_VALUE) / AS5048A_TWICE_MAX_VALUE;
	return degrees;
}

/**
 * Get the rotation of the sensor relative to the zero position in radians.
 *
 * @return {float} between 0 and 2 * PI
 */

float AS5048A::getRotationInRadians(){
	int32_t rotation = getRotation();
	float degrees = AS5048A_PI * (rotation + AS5048A_MAX_VALUE) / AS5048A_MAX_VALUE;
	return degrees;
}

/**
 * Returns the raw angle directly from the sensor
 */
uint16_t AS5048A::getRawRotation(){
	return AS5048A::read(AS5048A_ANGLE);
}

/**
 * returns the value of the state register
 * @return unsigned 16 bit integer containing flags
 */
uint16_t AS5048A::getState(){
	return AS5048A::read(AS5048A_DIAG_AGC);
}

/**
 * Print the diagnostic register of the sensor
 */
void AS5048A::printState(){
	uint16_t data = AS5048A::getState();
	if(AS5048A::error()){
		Serial.print("Error bit was set!");
	}
	Serial.println(data, BIN);
}

/**
 * Returns the value used for Automatic Gain Control (Part of diagnostic
 * register)
 */
uint8_t AS5048A::getGain(){
	uint16_t data = AS5048A::getState();
	return (uint8_t) data & 0xFF;
}

/*
 * Get and clear the error register by reading it
 */
uint16_t AS5048A::getErrors(){
	return AS5048A::read(AS5048A_CLEAR_ERROR_FLAG);
}

/*
 * Set the zero position
 */
bool AS5048A::setZeroPosition(uint16_t position){
	uint8_t low_byte     = (position & 0b0000000000111111);
	uint16_t result_low  = write(AS5048A_OTP_REGISTER_ZERO_POS_LOW, low_byte);
	Serial.println(result_low);
	uint8_t high_byte    = (position >> 6 & 0b0000000011111111);
	uint16_t result_high = write(AS5048A_OTP_REGISTER_ZERO_POS_HIGH, high_byte);
	Serial.println(result_high);
	return (result_high == high_byte) && (result_low == low_byte);
}

/**
 * Returns the current zero position
 */
uint16_t AS5048A::getZeroPosition(){
	uint16_t low_byte  = read(AS5048A_OTP_REGISTER_ZERO_POS_LOW);
	uint16_t high_byte = read(AS5048A_OTP_REGISTER_ZERO_POS_HIGH);
	uint16_t position = ( ( (high_byte & 0x00FF) << 8 ) & 0xFF00) | (low_byte & 0x00FF);
	return position;
}

/**
 * Check if an error has been encountered.
 */
bool AS5048A::error(){
	return errorFlag;
}

/**
 * Read a register from the sensor
 * Takes the address of the register as an unsigned 16 bit
 * Returns the value of the register
 */
uint16_t AS5048A::read(uint16_t registerAddress){
	uint16_t command = registerAddress | AS5048A_READ_FLAG;
	command |= spiCalcEvenParity(command);

#ifdef AS5048A_DEBUG
	Serial.println("Read (0x" + String(registerAddress, HEX) + 
		") with command: 0b" + String(command, BIN));
#endif

	SPI.beginTransaction(settings);

	digitalWrite(_cs, LOW);
	SPI.transfer16(command);
	digitalWrite(_cs,HIGH);

	if(response_delay_millis > 0) {
		delay(response_delay_millis);
	}

	/** 
	 * According to the specs:
	 * "The content of the desired register is available in the MISO register of the
	 * master device at the end of the second transmission cycle"
	 * We send a NOP and expect the response to be the value of the register 
	 * we requested to read.
	 */
	digitalWrite(_cs, LOW);
	uint16_t response = SPI.transfer16(AS5048A_NOP);
	digitalWrite(_cs, HIGH);

	SPI.endTransaction();

#ifdef AS5048A_DEBUG
	Serial.print("Read returned: " + String(response, BIN));
#endif

	//Check if the error bit is set
	errorFlag = response & 0x4000;
#ifdef AS5048A_DEBUG
	if (errorFlag) {
		Serial.println("Setting error bit");
	}
#endif

	//Return the data, stripping the parity and error bits
	return response & ~0xC000;
}


/**
 * Write to a register
 * Takes the 16-bit  address of the target register and the unsigned 16 bit of data
 * to be written to that register
 * Returns the value of the register after the write has been performed. This
 * is read back from the sensor to ensure a sucessful write.
 */
uint16_t AS5048A::write(uint16_t registerAddress, uint16_t data) {

	uint16_t command = registerAddress | AS5048A_WRITE_FLAG;
	command |= spiCalcEvenParity(command);

#ifdef AS5048A_DEBUG
	Serial.println("Write (0x" + String(registerAddress, HEX) + 
		") with command: 0b" + String(command, BIN));
#endif

	SPI.beginTransaction(settings);

	digitalWrite(_cs, LOW);
	SPI.transfer16(command);
	digitalWrite(_cs,HIGH);

	uint16_t dataToSend = data | AS5048A_WRITE_FLAG;
	dataToSend |= spiCalcEvenParity(dataToSend);

#ifdef AS5048A_DEBUG
	Serial.println("Sending data to write: " + String(dataToSend, BIN));
#endif

	digitalWrite(_cs,LOW);
	SPI.transfer16(dataToSend);
	digitalWrite(_cs,HIGH);

	if(response_delay_millis > 0) {
		delay(response_delay_millis);
	}
	
	SPI.endTransaction();
	SPI.beginTransaction(settings);
	//Send a NOP to get the new data in the register
	digitalWrite(_cs, LOW);
	uint16_t response = SPI.transfer16(AS5048A_NOP);
	digitalWrite(_cs, HIGH);

	SPI.endTransaction();

	//Return the data, stripping the parity and error bits
	return response & ~0xC000;
}
