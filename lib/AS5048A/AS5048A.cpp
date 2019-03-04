#include "Arduino.h"

#include <AS5048A.h> 

//#define AS5048A_DEBUG

const int AS5048A_NOP                           = 0x0000; // Dummy operation, no information.
const int AS5048A_CLEAR_ERROR_FLAG              = 0x0001; // Error register. All errors are cleared by access.
const int AS5048A_PROGRAMMING_CONTROL           = 0x0003; // Register control programming. Programming must be enabled before the memory is burned. Before programming verification is mandatory. See Programming Procedure.
const int AS5048A_OTP_REGISTER_ZERO_POS_HIGH    = 0x0016; // Zero value of 8 bits high
const int AS5048A_OTP_REGISTER_ZERO_POS_LOW     = 0x0017; // Zero value 6 bits lower
const int AS5048A_DIAG_AGC                      = 0x3FFD; // (0-7) The value of the automatic gain control. 0 decimal represents a high magnetic field, 255 decimal represents a low magnetic field. (8-13) Diagnostics Flags
const int AS5048A_MAGNITUDE                     = 0x3FFE; // The value of the output power CORDIC
const int AS5048A_ANGLE                         = 0x3FFF; // Angular output value, including zero position correction Resolution_ADC 14-bit resolution (0.0219 ° / LSB)

/**
 * Constructor
 * Initialization of the AS5048A library
 */
AS5048A::AS5048A(byte Arg_Cs){
  _cs = Arg_Cs;
  _errorFlag = false; 
  _position = 0;
}


/**
 * Initialiser
 * Sets up the SPI interface
 */
void AS5048A::init(){
  /** 
  * 1MHz clock (AMS should be able to accept up to 10MHz)
  * mySettting (speedMaximum, dataOrder, dataMode)
  * speedMaximum - maximum connection speed. For an SPI chip rated for up to 20 MHz, use 20,000,000.
  * dataOrder - the order of data output to / from the SPI bus, it can be LSBFIRST (least significant bit (bit) first) or MSBFIRST (most significant first bit)
  * dataMode - sets the SPI bus mode, setting the level of the synchronization signal and the synchronization phase
  * SPI_MODE0 (Signal level (CPOL) -0, Phase (CPHA) -0)
  * SPI_MODE1 (Signal level (CPOL) -0, Phase (CPHA) -1)
  * SPI_MODE2 (Signal level (CPOL) -1, Phase (CPHA) -0)
  * SPI_MODE3 (Signal level (CPOL) -1, Phase (CPHA) -1)
  * f(sample) = Min-10.2, Typ-11.25, Max-12.4. (kHz) 
  * Supported I2C AS5048B Modes:
  * • Random / sequential read
  * • Byte / Write page
  * • Standard: 0 to 100 kHz, clock frequency (slave mode)
  * • Fast mode: clock frequency from 0 to 400 kHz (slave mode)
  * • High speed: 0 to 3.4 MHz clock frequency (slave mode)
  */    
  settings = SPISettings(1000000, MSBFIRST, SPI_MODE1);
 // initialization of the Slave Select pin if the LOW slave interacts with the master if the HIGH slave ignores the signals from the master
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
 * Calculation of the parity bit of the 14-bit address and writing to the 15th bit of the returned 16-bit word
 */ 
byte AS5048A::spiCalcEvenParity(word Value){
  /**byte cnt = 0;
  byte i;
  for (i = 0; i < 15; i++)
  {
     if (Value & 0x1)
    {
      cnt++;
    }
    Value >>= 1;
  }
  return cnt & 0x1;*/
  
  byte operand_compare =  bitRead(Value,0);
  byte i = 1;
  do{
    operand_compare ^= bitRead(Value,i);
  } while ((i++) < 14);
  return operand_compare & 0x1;
}



/**
 * Get the rotation of the sensor relative to the zero _position.
 *
 * @return {int} between -2^13 and 2^13
 */
int AS5048A::getRotation(){
  word data;
  int rotation;
  data = AS5048A::getRawRotation();
  rotation = (int)data - (int)_position;
  if(rotation > 8191) rotation = -((0x3FFF)-rotation); //more than -180
  //if(rotation < -0x1FFF) rotation = rotation+0x3FFF;
  return rotation;
}

/**
 * Returns the raw angle directly from the sensor
 * Returns an angular binary 14 bit angular value (DEC 16383)
 * Angle output value including zero offset.
 */
word AS5048A::getRawRotation(bool EnableMedianValue){
  return AS5048A::read(AS5048A_ANGLE, EnableMedianValue);
}

/**
 *Returns the physical quantity in angular degrees, obtained from a binary 14 bit ADC number
 */
float AS5048A::RotationRawToAngle(word DiscreteCode){
  return DiscreteCode * (360.0 / float(AS5048A_ANGLE));
}

/**
 * Returns the physical quantity in angular radians, obtained from the binary 14-bit ADC number
 */
float AS5048A::RotationRawToRadian(word DiscreteCode){
  return DiscreteCode * ((2 * PI) / float(AS5048A_ANGLE));
}

/**
 * Returns the incremental and decrementing angle of rotation into the variable RotationAngle. The variable addresses are changed in the procedure.
 */
void AS5048A::AbsoluteAngleRotation (float *RotationAngle, float *AngleCurrent, float *AnglePrevious){

  if (*AngleCurrent != *AnglePrevious){
    //a circle is made to increase from 360 to 1
        if ((*AngleCurrent < 90) && (*AnglePrevious > 270) /*|| 
    (*AngleCurrent < 1.5707963267948966192313216916398) && (*AnglePrevious > 4.7123889803846898576939650749193) */){
            *RotationAngle += abs(360 - abs(*AngleCurrent - *AnglePrevious));
      _reverse = true;
    }  
    //the circle is made to decrease from 1 to 360
        if ((*AnglePrevious < 90) && (*AngleCurrent > 270) /*|| 
    (*AnglePrevious < 1.5707963267948966192313216916398) && (*AngleCurrent > 4.7123889803846898576939650749193) */){
            *RotationAngle -= abs(360 - abs(*AngleCurrent - *AnglePrevious));
      _reverse = false;
    }
        //ascending circle
        if (*AngleCurrent > *AnglePrevious && ((*AngleCurrent < 90) && (*AnglePrevious > 270))!=true && ((*AnglePrevious < 90) && (*AngleCurrent > 270))!=true /*||
    *AngleCurrent > *AnglePrevious && ((*AngleCurrent < 1.5707963267948966192313216916398) && (*AnglePrevious > 4.7123889803846898576939650749193))!=true && ((*AnglePrevious < 1.5707963267948966192313216916398) && (*AngleCurrent > 4.7123889803846898576939650749193))!=true*/){
            *RotationAngle += abs(*AngleCurrent - *AnglePrevious);
      _reverse = true;
    } 
        //descending circle
        if (*AnglePrevious > *AngleCurrent && ((*AngleCurrent < 90) && (*AnglePrevious > 270))!=true && ((*AnglePrevious < 90) && (*AngleCurrent > 270))!=true /*||
    *AnglePrevious > *AngleCurrent && ((*AngleCurrent < 1.5707963267948966192313216916398) && (*AnglePrevious > 4.7123889803846898576939650749193))!=true && ((*AnglePrevious < 1.5707963267948966192313216916398) && (*AngleCurrent > 4.7123889803846898576939650749193))!=true*/){
            *RotationAngle -= abs(*AnglePrevious - *AngleCurrent);
      _reverse = false;
    }   
  }

    *AnglePrevious = *AngleCurrent;   
}

/**
*returns the angle minutes
*/
float AS5048A::GetAngularMinutes (float AngleAbsolute){
  return ( AngleAbsolute - int(AngleAbsolute) ) * 60;

}

/**
*returns seconds of angle
*/
float AS5048A::GetAngularSeconds (float AngleAbsolute){
  return (AS5048A::GetAngularMinutes(AngleAbsolute) - int(AS5048A::GetAngularMinutes(AngleAbsolute)) ) * 60;
}

/**
*returns the movement of the spur gear in mm
*WheelRotationAngle - Angle of rotation of the wheel
*NormalModule - Module normal
*NumberGearTeeth - The number of teeth of the wheel or the number of visits of the worm
*(PI * NormalModule) - Front Pitch
*20 - the angle of the tooth
*/ 
float AS5048A::LinearDisplacementRack ( float WheelRotationAngle, float NormalModule, float NumberGearTeeth){
  return WheelRotationAngle * (( ( (PI * NormalModule) / cos(radians(20)) ) * NumberGearTeeth) / 360);
} 

/**
*returns the movement of the screw in mm
*StepGroove - screw thread pitch
*ScrewRotationAngle - screw rotation angle
*/ 
float AS5048A::LinearMotionHelicalGear ( float ScrewRotationAngle, float StepGroove){  
  return (ScrewRotationAngle * (StepGroove / 360));
}

/**
 * returns the value of the state register
 * @return 16 bit word containing flags
 * Returns the value of the diagnostic sensor register
 * 16 bits in size, 13 of them are significant (example 1101100110101)
 */
word AS5048A::getState(){
  return AS5048A::read(AS5048A_DIAG_AGC,false) & ~0xC000;
}

/**
 * Print the diagnostic register of the sensor
 * Output to the Serial port the value of the diagnostic register of the sensor
 */
void AS5048A::printState(){
  word data;
  data = AS5048A::getState();
  if(AS5048A::error()){
    Serial.println("Error bit was set! (function printState register Diagnostics + Automatic Gain Control (AGC) )");
  }
  Serial.println(" ");
  Serial.println("The value of the automatic control of the magnetic field gain");
  Serial.println("255 is a low magnetic field");
  Serial.println("0 is a high magnetic field");
  //Serial.println(lowByte(data), BIN);
  Serial.println(lowByte(data), DEC);
  
/**Diagnostic Functions AS5048
  * See Figure 22 register address x3FFD (AS5048A) or Figure 31 register address 251 dec (AS5048B)
  * • OCF (Offset Compensation Finished), logic high indicates the finished Offset Compensation Algorithm. After power up the flag remains always to logic high.
  * • COF (CORDIC Overflow), logic high indicates an out of range error in the CORDIC part. When this bit is set, the angle and magnitude data is invalid. The absolute output maintains the last valid angular value.
  * • COMP low, indicates a high magnetic field. It is recommended to monitor in addition the magnitude value.
  * • COMP high, indicated a weak magnetic field. It is recommended to monitor the magnitude value.
 */
  Serial.print("Diagnostic flags");
  Serial.print(" OCF-");
  Serial.print(bitRead(data,8), DEC);
  Serial.print(" COF-");
  Serial.print(bitRead(data,9), DEC);
  Serial.print(" Comp Low-");
  Serial.print(bitRead(data,10), DEC);
  Serial.print(" Comp High-");
  Serial.println(bitRead(data,11), DEC);
  Serial.println(" ");
  //Serial.println(data, BIN);
}


/**
 * Returns the value used for Automatic Gain Control (Part of diagnostic
 * register)
 * Returns the value of the automatic gain control diagnostic register
 */
byte AS5048A::getGain(){
  word data = AS5048A::getState();
  if(AS5048A::error()){
    Serial.print("Error bit was set! (function getGain register Diagnostics + Automatic Gain Control (AGC) )");
  }
  return (byte) data & 0xFF;
}

/**
 * Get and clear the error register by reading it
 * Clear error flag and return three bits (0bit Framing Error, 1bit Command Invalid, 2bit Parity Error)
 * Register of errors. All errors are cleared by access.
  Possible conditions that force to install ERROR FLAG:
  • Invalid parity
  • Incorrect hours (without full transfer cycle or too many hours)
  • Invalid command
  • Frame error
  Note (s): If the error flag is set to high due to
  communication problem, the flag remains set until it is
  cleared by the CLEAR ERROR FLAG command.
 */
word AS5048A::getErrors(){
  return AS5048A::read(AS5048A_CLEAR_ERROR_FLAG,false) & ~0xC000;
}

/**
 * Retrieve and clear the error register and output the register value to the serial port
 */
void AS5048A::printErrors(){
  word data;
  data = AS5048A::getErrors();
  if(AS5048A::error()){
    Serial.println("Error bit was set! (function printErrors register Clear Error Flag)");
  }
  Serial.println("Error register");
  Serial.print("Command frame (packet) error ");
  Serial.println(bitRead(data,0), DEC);
  Serial.print("Invalid command ");
  Serial.println(bitRead(data,1), DEC);
  Serial.print("Parity bit error ");
  Serial.println(bitRead(data,2), DEC);
  Serial.println(" ");
  //Serial.println(data, BIN);
} 

/**
 *The function sends the NOP command and returns the contents of the register. The NOP team is a fake
 *write to register x0000 sensor AS5048
 */ 
word AS5048A::DummyOperNoInf(){
  return AS5048A::read(AS5048A_NOP,false);  
}

/**
 *The procedure records the absolute value measured by the AS5048 sensor, a randomly located magnet on the axis of rotation,
 *as zero angle position

AS5048 programming
Zero Position Programming: The absolute angle position can be programmed through the interface. This can be useful for randomly 
placing the magnet on the axis of rotation. A mechanical zero reading can be performed and written back to the IC. With constant 
programming, the position is not reversible, stored in the IC. This programming can only be done once. To simplify the calculation 
of the zero position, it is only necessary to record the value in the IC, which was previously read from the angle register.

Programming sequence with verification: to program a zero position, the following sequence must be performed:
1. Write 0 to the OTP zero position register to clear.
2. Read current angle information
3. Write the read angle position in the OTP zero register.

Now record the zero position. If you want to write the value of the OTP register, send:

4. Set the programming bit to write the value of the OTP control register.
5. Set the write bit (Burn) to start the automatic programming procedure.
6. Read the current angle information if (equal to 0) then.
7. Set the Verify bit to reload OTP data into internal registers.
8. Read the current angle information for verification (equal to 0).

Programming can be done in 5 V mode using an internal LDO or 3V, but with a minimum supply voltage of 3.3 V. 
In the case of 3 V operation, a 10 µF capacitor on the VDD3 pin is also required.
 */

void AS5048A::ProgAbsolAngleZeroPosit(){
  word rotationzero = 0b0000000000000000;
  word programcontrol = 0b00000000000000;
  
  AS5048A::write(AS5048A_OTP_REGISTER_ZERO_POS_HIGH, AS5048A_NOP & ~0xFF00); 
  AS5048A::write(AS5048A_OTP_REGISTER_ZERO_POS_LOW, AS5048A_NOP & ~0xFFC0); 
  
  rotationzero |= AS5048A::getRawRotation();
  
  AS5048A::write(AS5048A_OTP_REGISTER_ZERO_POS_HIGH, (rotationzero >> 6) & 0xFF);
  AS5048A::write(AS5048A_OTP_REGISTER_ZERO_POS_LOW, rotationzero & 0x3F);
  
  AS5048A::write(AS5048A_PROGRAMMING_CONTROL, bitSet(programcontrol,0));
  AS5048A::write(AS5048A_PROGRAMMING_CONTROL, bitSet(programcontrol,3));
  
  if (1 < AS5048A::getRawRotation() < -1){
    AS5048A::write(AS5048A_PROGRAMMING_CONTROL, bitSet(programcontrol,6));
  }

  Serial.println(AS5048A::getRawRotation(), DEC); 
}

/**
 * Set the zero _position
 */
void AS5048A::setZeroPosition(word Arg_Position){
  _position = Arg_Position % 0x3FFF;
}

/**
 * Returns the current zero _position
 */
word AS5048A::getZeroPosition(){
  return _position;
}

/**
 * ascending sort function
 */
void AS5048A::quickSort(word *Arr, int Left, int Right) { 
  int i = Left, j = Right; 
  int tmp; 
  word pivot = Arr[(Left + Right) / 2]; 

  /* partition */ 
  while (i <= j) { 
    while (Arr[i] < pivot) 
      i++; 
    while (Arr[j] > pivot) 
      j--; 
    if (i <= j) { 
      tmp = Arr[i]; 
      Arr[i] = Arr[j]; 
      Arr[j] = tmp; 
      i++; 
      j--; 
    } 
  } 

  /* recursion */ 
  if (Left < j) 
    AS5048A::quickSort(Arr, Left, j); 
  if (i < Right) 
    AS5048A::quickSort(Arr, i, Right); 
}

/**
 * Check if an error has been encountered.
 * Error flag indicating a transmission error in a previous master transmission (Master)
 */
bool AS5048A::error(){
  return _errorFlag;
}

/**
 * Read a register from the sensor
 * Takes the address of the register as a 16 bit word
 * Returns the value of the register
 * Sending a command to read AS5048A sensor registers
 */
word AS5048A::read(word RegisterAddress, bool MeanValueMedian){
  word readdata;
  word array_data[16];
  word command = 0b0100000000000000; // PAR=0 R/W=R
  
  command |= RegisterAddress;
  
  //Add a parity bit on the the MSB
  command |= ((word)spiCalcEvenParity(command)<<15);
  
  //SPI - begin transaction
  SPI.beginTransaction(settings);
  
  digitalWrite(_cs, LOW);
  SPI.transfer16(command);
  digitalWrite(_cs, HIGH);
  SPI.endTransaction();

  #ifdef AS5048A_DEBUG
    Serial.print("Read (0x");
    Serial.print(RegisterAddress, HEX);
    Serial.print(") with command: 0b");
    Serial.println(command, BIN);
  #endif
  
  //Send the command and Now read the response
  if (MeanValueMedian == true){
  
    for ( byte i = 0; i < 16; i++){
      digitalWrite(_cs, LOW);
      array_data[i] = SPI.transfer16(command) & ~0xC000;
      digitalWrite(_cs, HIGH);
      //Serial.println(array_data[i], BIN);   
    }

    AS5048A::quickSort(array_data, 0, 15);
    readdata = ( array_data[8]  + array_data[9]  ) / 2 ;  
    
    SPI.endTransaction();
    //SPI - end transaction
    
    //Return the data, stripping the parity and error bits
    return readdata;  
  }else{
    digitalWrite(_cs, LOW);
    readdata = SPI.transfer16(command);
    digitalWrite(_cs, HIGH);
    
    #ifdef AS5048A_DEBUG
      Serial.print("Read returned: ");
      Serial.print(highByte(readdata), BIN);
      Serial.print(lowByte(readdata), BIN);
    #endif

    //Check if the error bit is set
    //If the 15 bit is set to 1 (transmission error in the previous transfer of the master) _errorFlag set to 1 otherwise 0
    if (bitRead(readdata,14)) {
      #ifdef AS5048A_DEBUG
        Serial.println("Setting error bit");
      #endif
      _errorFlag = true;
    }else {
      _errorFlag = false;
    }
    
    SPI.endTransaction();
    //SPI - end transaction
    
    //Return the data, stripping the parity and error bits
    return readdata & ~0xC000;
  }
  
}


/**
 * Write to a register
 * Takes the 16-bit  address of the target register and the 16 bit word of WriteData
 * to be written to that register
 * Returns the value of the register after the write has been performed. This
 * is read back from the sensor to ensure a sucessful write.
 */
word AS5048A::write(word RegisterAddress, word WriteData) {
  word command = 0b0000000000000000; // PAR=0 R/W=W
  word dataToSend = 0b0000000000000000;
  
  command |= RegisterAddress;
  dataToSend |= WriteData;
  
  //Add a parity bit on the the MSB
  command |= ((word)spiCalcEvenParity(command) << 15);
  
  //Craft another packet including the data and parity
  dataToSend |= ((word)spiCalcEvenParity(dataToSend) << 15);
  
#ifdef AS5048A_DEBUG
  Serial.print("Write (0x");
  Serial.print(RegisterAddress, HEX);
  Serial.print(") with command: 0b");
  Serial.println(command, BIN);
#endif

  //SPI - begin transaction
  SPI.beginTransaction(settings);

  //Start the write command with the target address
  digitalWrite(_cs, LOW);
  SPI.transfer16(command);
  digitalWrite(_cs, HIGH);

#ifdef AS5048A_DEBUG
  Serial.print("Sending data to write: ");
  Serial.println(dataToSend, BIN);
#endif

  //Now send the data packet
  digitalWrite(_cs, LOW);
  SPI.transfer16(dataToSend);
  digitalWrite(_cs, HIGH);
  
  //Send a NOP to get the new data in the register
  digitalWrite(_cs, LOW);
  dataToSend = SPI.transfer16(AS5048A_NOP);
  digitalWrite(_cs, HIGH);
  SPI.endTransaction();

  //SPI - end transaction
  SPI.endTransaction();

  //Return the data, stripping the parity and error bits
  return dataToSend & ~0xC000;
}
