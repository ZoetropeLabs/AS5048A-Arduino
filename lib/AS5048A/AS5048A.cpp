#include "Arduino.h"

#include <AS5048A.h> 

//#define AS5048A_DEBUG

const int AS5048A_CLEAR_ERROR_FLAG              = 0x0001; //Регистр ошибок. Все ошибки очищаются путем доступа.
const int AS5048A_PROGRAMMING_CONTROL           = 0x0003; //Регистр управления программированием. Программирование должно быть включено до прожига памяти. Перед программированием проверка является обязательной. См. Процедуру программирования.
const int AS5048A_OTP_REGISTER_ZERO_POS_HIGH    = 0x0016; //Нулевое значение с высоким байтом
const int AS5048A_OTP_REGISTER_ZERO_POS_LOW     = 0x0017; //Нулевая позиция остается 6 младших младших разрядов
const int AS5048A_DIAG_AGC                      = 0x3FFD; //(0-7)Значение автоматического регулирования усиления. 0 десятичной представляет высокое магнитное поле, 255 десятичных представляет низкое магнитное поле. (8-13)Флаги диагностики
const int AS5048A_MAGNITUDE                     = 0x3FFE; //Значение выходной мощности CORDIC 
const int AS5048A_ANGLE                         = 0x3FFF; //Угловое выходное значение, включая коррекцию нулевой позиции Resolution_ADC 14-bit resolution (0.0219°/LSB)

/**
 * Constructor
 */
AS5048A::AS5048A(byte arg_cs){
	_cs = arg_cs;
	errorFlag = false; 
	position = 0;
}


/**
 * Initialiser
 * Sets up the SPI interface
 */
void AS5048A::init(){
	/** 
	* 1MHz clock (AMS should be able to accept up to 10MHz)
	* mySettting (speedMaximum, dataOrder, dataMode)
	* speedMaximum - максимальная скорость связи. Для чипа SPI, рассчитанного на частоту до 20 МГц , используйте 20000000.
	* dataOrder - порядок вывода даннах в/из шины SPI,  может быть LSBFIRST (наименьший разряд(бит) первый) или MSBFIRST (старший разряд первый)
	* dataMode - устанавливает режим работы шины SPI, задавая уровень сигнала синхронизации и фазу синхронизации
	* SPI_MODE0 (Уровень сигнала (CPOL)-0, Фаза (CPHA)-0)
	* SPI_MODE1 (Уровень сигнала (CPOL)-0, Фаза (CPHA)-1) 
	* SPI_MODE2 (Уровень сигнала (CPOL)-1, Фаза (CPHA)-0)
	* SPI_MODE3 (Уровень сигнала (CPOL)-1, Фаза (CPHA)-1)
	* f(sample) = Min-10.2, Typ-11.25, Max-12.4. (kHz) 
	* Поддерживаемые режимы  I2C AS5048B:
	* • Случайное / последовательное чтение
	* • Байт / Запись страницы
	* • Стандартный: от 0 до 100 кГц, тактовая частота (ведомый режим)
	* • Быстрый режим: тактовая частота от 0 до 400 кГц (ведомый режим)
	* • Высокая скорость: от 0 до 3,4 МГц тактовой частоты (ведомый режим)
	*/		
	settings = SPISettings(1000000, MSBFIRST, SPI_MODE1);
	//инициализация пина Slave Select если LOW ведомый взаимодействует с ведущим если HIGH ведомый игнорирует сигналы от ведущего
	pinMode(_cs, OUTPUT);
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
 * Utility function used to calculate even parity of word
 */ 
byte AS5048A::spiCalcEvenParity(word value){
	//byte cnt = 0;
	//byte i;
	//for (i = 0; i < 16; i++)
	//{
	//   if (value & 0x1)
	//	{
	//		cnt++;
	//	}
	//	value >>= 1;
	//}
	//return cnt & 0x1;
	byte operand_compare =  value;
	byte i = 0;
	do{
		value >>= 1;
		operand_compare ^= value;
	} while ((i++) < 14);
	return operand_compare & 0x1;
}



/**
 * Get the rotation of the sensor relative to the zero position.
 *
 * @return {int} between -2^13 and 2^13
 */
int AS5048A::getRotation(){
	word data;
	int rotation;
	data = AS5048A::getRawRotation();
	rotation = (int)data - (int)position;
	if(rotation > 8191) rotation = -((0x3FFF)-rotation); //more than -180
	//if(rotation < -0x1FFF) rotation = rotation+0x3FFF;
	return rotation;
}

/**
 * Returns the raw angle directly from the sensor
 */
word AS5048A::getRawRotation(bool EnableMedianValue = false){
	return AS5048A::read(AS5048A_ANGLE, EnableMedianValue);
}

/**
 *Возвращает физическую величину в угловых градусах, полученное из двоичного числа АЦП
 */
float RotationRawToAngle (word DiscreteCode){
	return DiscreteCode * (360.0 / float(AS5048A_ANGLE));
}

/**
 * Возвращает инкрементный и декрементный угол поворота в переменную RotationAngle в процедуру прередаються адреса переменных 
 */
void AbsoluteAngleRotation (float *RotationAngle, float *AngleCurrent, float *AnglePrevious){

	if (*AngleCurrent != *AnglePrevious){
	
        if ( (*AngleCurrent < 90) && (*AnglePrevious > 270) ){
            *RotationAngle += abs(360 - abs(*AngleCurrent - *AnglePrevious));
		}  
		
        if ( (*AnglePrevious < 90) && (*AngleCurrent > 270) ){
            *RotationAngle -= abs(360 - abs(*AngleCurrent - *AnglePrevious));
		}
        
        if (*AngleCurrent > *AnglePrevious && ((*AngleCurrent < 90) && (*AnglePrevious > 270))!=true && ((*AnglePrevious < 90) && (*AngleCurrent > 270))!=true){
            *RotationAngle += abs(*AngleCurrent - *AnglePrevious);
		} 
            
        if (*AnglePrevious > *AngleCurrent && ((*AngleCurrent < 90) && (*AnglePrevious > 270))!=true && ((*AnglePrevious < 90) && (*AngleCurrent > 270))!=true){
            *RotationAngle -= abs(*AnglePrevious - *AngleCurrent);
		}
	}

    *AnglePrevious = *AngleCurrent;		
}


float AS5048A::GetAngularMinutes (float AngleAbsolute){
	return ( AngleAbsolute - int(AngleAbsolute) ) *60;
}

float AS5048A::GetAngularSeconds (float AngleAbsolute){
	return (AS5048A::GetAngularMinutes(AngleAbsolute) - int(AS5048A::GetAngularMinutes(AngleAbsolute)) ) * 60;
}

/**
 * returns the value of the state register
 * @return 16 bit word containing flags
 */
word AS5048A::getState(){
	return read(AS5048A_DIAG_AGC);
}

/**
 * Print the diagnostic register of the sensor
 */
void AS5048A::printState(){
	word data;
	data = AS5048A::getState();
	if(AS5048A::error()){
		Serial.print("Error bit was set!");
	}
	Serial.println(data, BIN);
}


/**
 * Returns the value used for Automatic Gain Control (Part of diagnostic
 * register)
 */
byte AS5048A::getGain(){
	word data = AS5048A::getState();
	return (byte) data & 0xFF;
}

/*
 * Get and clear the error register by reading it
 */
word AS5048A::getErrors(){
	return AS5048A::read(AS5048A_CLEAR_ERROR_FLAG);
}

/*
 * Set the zero position
 */
void AS5048A::setZeroPosition(word arg_position){
	position = arg_position % 0x3FFF;
}

/*
 * Returns the current zero position
 */
word AS5048A::getZeroPosition(){
	return position;
}

//функция для сортировки по возрастанию
word AS5048A::SortingUp (const void * a, const void * b){
	return ( *(word*)a - *(int*)b );
}

/*
 * Check if an error has been encountered.
 */
bool AS5048A::error(){
	return errorFlag;
}

/*
 * Read a register from the sensor
 * Takes the address of the register as a 16 bit word
 * Returns the value of the register
 */
word AS5048A::read(word registerAddress, bool MeaValueMedian){
	word buffer = 0x00;
	word array_buffer [15];
	word command = 0b0100000000000000; // PAR=0 R/W=R
	command = command | registerAddress;
	//Add a parity bit on the the MSB
	command |= ((word)spiCalcEvenParity(command)<<15);

#ifdef AS5048A_DEBUG
	Serial.print("Read (0x");
	Serial.print(registerAddress, HEX);
	Serial.print(") with command: 0b");
	Serial.println(command, BIN);
#endif

	//SPI - begin transaction
	SPI.beginTransaction(settings);
	
	//Send the command and Now read the response
	digitalWrite(_cs, LOW);
	if (MeaValueMedian == true){
		for (i = 0; i < 15; i++){
			array_buffer [i] = SPI.transfer16(command);
		}
	}else{
		buffer = SPI.transfer16(command);
	}	
    digitalWrite(_cs, HIGH);

	//SPI - end transaction
	SPI.endTransaction();
	
	if (MeaValueMedian == true){
		qsort (array_buffer, 15, sizeof(word), AS5048A::SortingUp);
		buffer = (array_buffer[7] + array_buffer[8])/2;
	}

#ifdef AS5048A_DEBUG
	Serial.print("Read returned: ");
	//Serial.print(left_byte, BIN);
	Serial.print(highByte(buffer), BIN);
	Serial.print(" ");
	//Serial.println(right_byte, BIN);
	Serial.print(lowByte(buffer), BIN);
#endif

	//Check if the error bit is set
	if (/*left_byte*/ highByte(buffer) & 0x40) {
#ifdef AS5048A_DEBUG
		Serial.println("Setting error bit");
#endif
		errorFlag = true;
	}else {
		errorFlag = false;
	}

	//Return the data, stripping the parity and error bits
	return /*(( ( left_byte & 0xFF ) << 8 ) | ( right_byte & 0xFF )) & ~0xC000*/ buffer & ~0xC000;
}


/*
 * Write to a register
 * Takes the 16-bit  address of the target register and the 16 bit word of data
 * to be written to that register
 * Returns the value of the register after the write has been performed. This
 * is read back from the sensor to ensure a sucessful write.
 */
word AS5048A::write(word registerAddress, word data) {

	word command = 0b0000000000000000; // PAR=0 R/W=W
	command |= registerAddress;

	//Add a parity bit on the the MSB
	command |= ((word)spiCalcEvenParity(command)<<15);

	//Split the command into two bytes
	byte right_byte = command & 0xFF;
	byte left_byte = ( command >> 8 ) & 0xFF;

#ifdef AS5048A_DEBUG
	Serial.print("Write (0x");
	Serial.print(registerAddress, HEX);
	Serial.print(") with command: 0b");
	Serial.println(command, BIN);
#endif

	//SPI - begin transaction
	SPI.beginTransaction(settings);

	//Start the write command with the target address
	digitalWrite(_cs, LOW);
	SPI.transfer(left_byte);
	SPI.transfer(right_byte);
	digitalWrite(_cs,HIGH);
	
	word dataToSend = 0b0000000000000000;
	dataToSend |= data;

	//Craft another packet including the data and parity
	dataToSend |= ((word)spiCalcEvenParity(dataToSend)<<15);
	right_byte = dataToSend & 0xFF;
	left_byte = ( dataToSend >> 8 ) & 0xFF;

#ifdef AS5048A_DEBUG
	Serial.print("Sending data to write: ");
	Serial.println(dataToSend, BIN);
#endif

	//Now send the data packet
	digitalWrite(_cs,LOW);
	SPI.transfer(left_byte);
	SPI.transfer(right_byte);
	digitalWrite(_cs,HIGH);
	
	//Send a NOP to get the new data in the register
	digitalWrite(_cs, LOW);
	left_byte =-SPI.transfer(0x00); // - ?
	right_byte = SPI.transfer(0x00);
	digitalWrite(_cs, HIGH);

	//SPI - end transaction
	SPI.endTransaction();

	//Return the data, stripping the parity and error bits
	return (( ( left_byte & 0xFF ) << 8 ) | ( right_byte & 0xFF )) & ~0xC000;
