#ifndef as5048_h
#define as5048_h
#define LIBRARY_VERSION 1.0.1

#include <SPI.h>
//#include <math.h>

//#include <stdio.h> 
//#include <stdlib.h>   

class AS5048A{

  bool _errorFlag;
  byte _cs;
  //byte cs;
  //byte dout;
  //byte din;
  //byte clk;
  //word transaction(word data);
  word _position;
  bool _reverse; // Flag of the direction of movement of the machine ram

  SPISettings settings;

  public:

  /**
   *  Constructor
   */
  AS5048A(byte Arg_Cs);

  /**
   * Initialiser
   * Sets up the SPI interface
   */
  void init();

  /**
   * Closes the SPI connection
   */
  void close();

  /**
   * Read a register from the sensor
   * Takes the address of the register as a 16 bit word
   * Returns the value of the register
   * MeaValueMedian allows you to find the median average of 16 measurements because
   * after 16 CLK cycles, CSn must be returned to a high state in order to reset
   * some parts of the kernel interface.
   */
  word read(word RegisterAddress, bool MeanValueMedian = false);

  /**
   * Write to a register
   * Takes the 16-bit  address of the target register and the 16 bit word of data
   * to be written to that register
   * Returns the value of the register after the write has been performed. This
   * is read back from the sensor to ensure a sucessful write.
   */
  word write(word RegisterAddress, word WriteData);

  /**
   * Get the rotation of the sensor relative to the zero position.
   *
   * @return {int} between -2^13 and 2^13
   */
  int getRotation();

  /**
   * Returns the raw angle directly from the sensor
   */
  word getRawRotation(bool EnableMedianValue = false);

  /**
   * Returns the physical quantity in angular degrees, obtained from a binary 14 bit ADC number
   */
  float RotationRawToAngle (word DiscreteCode);
  
  /**
   * Returns the physical quantity in angular radians, obtained from the binary 14-bit ADC number
   */
  float RotationRawToRadian(word DiscreteCode);

  /**
  * Returns the incremental and decrementing rotation angle in the variable RotationAngle. The variable addresses are passed to the procedure. 
  */
  void AbsoluteAngleRotation (float *RotationAngle, float *AngleCurrent, float *AnglePrevious);

  /**
  * Function to sort ascending
  */
  void quickSort(word *Arr, int Left, int Right);

  /**
  * Returns the angle minutes
  */
  float GetAngularMinutes (float AngleAbsolute);

  /**
  * Returns seconds of angle
  */
  float GetAngularSeconds (float AngleAbsolute);
  
  /**
  *Returns the movement of the spur gear in mm
  *WheelRotationAngle - Angle of rotation of the wheel
  *NormalModule - normal gear module
  *NumberGearTeeth - The number of wheel teeth (gears) or the number of worm hits
  *(PI * NormalModule) - Front Pitch
  *20 - the angle of the tooth
  */ 
  float LinearDisplacementRack ( float WheelRotationAngle, float NormalModule, float NumberGearTeeth);
  
  /**
  *Returns the movement of the screw in mm
  *StepGroove - screw thread pitch
  *ScrewRotationAngle - screw rotation angle
  */ 
  float LinearMotionHelicalGear ( float ScrewRotationAngle, float StepGroove);

  /**
   * returns the value of the state register
   * @return 16 bit word containing flags
   */
  word getState();

  /**
   * Print the diagnostic register of the sensor
   */
  void printState();

  /**
   * Returns the value used for Automatic Gain Control (Part of diagnostic
   * register)
   */
  byte getGain();

  /**
   * Get and clear the error register by reading it
   */
  word getErrors();
  
  /**
   * Get and clear the error register and output the register value to the Serial port
   */
  void printErrors();
  
  /**
   *The function sends the NOP command and returns the contents of the register. The NOP team is a fake
   *write to register x0000 sensor AS5048
   */
  word DummyOperNoInf();
  
  /**
   *The procedure records the absolute value measured by the sensor AS5048, a randomly located magnet on the axis of rotation,
   *as zero angle position
   */
  void ProgAbsolAngleZeroPosit ();
  
  /**
   * Set the zero position
   */
  void setZeroPosition(word Arg_Position);

  /**
   * Returns the current zero position
   */
  word getZeroPosition();

  /**
   * Check if an error has been encountered.
   */
  bool error();

  
  
  private:
  /**
   * returns the parity bit
   */
  byte spiCalcEvenParity(word);
};
#endif
