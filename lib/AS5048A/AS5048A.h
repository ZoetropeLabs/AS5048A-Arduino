#ifndef as5048_h
#define as5048_h
#define LIBRARY_VERSION 1.0.0


#define AS5048A_CLEAR_ERROR_FLAG 0x0001
#define AS5048A_PROGRAMMING_CONTROL 0x0003
#define AS5048A_OTP_REGISTER_ZERO_POS_HIGH 0x0016
#define AS5048A_OTP_REGISTER_ZERO_POS_LOW 0x0017
#define AS5048A_DIAG_AGC 0x3FFD
#define AS5048A_MAGNITUDE 0x3FFE
#define AS5048A_ANGLE 0x3FFF

class AS5048A{

  bool errorFlag;
  byte _cs;
  byte cs;
  byte dout;
  byte din;
  byte clk;
  word position;
  word transaction(word data);

  public:

  /**
   *
   */
  AS5048A(byte arg_cs);
  void init();
  void close();


	word read(word registerAddress);
	word write(word registerAddress, word data);

  //most important!
  int getRotation();

  //return the raw position
  word getRawRotation();

  //just for diagnostics
  word getState();

  //print the state, over the serial line
  void printState();

  //like get start, but just returns the gain
  byte getGain();

  //get the error flags and reset the error register
  word getErrors();

  //set what value is the zero position, returned
  //values will be relative to this
  void setZeroPosition(word arg_position);

  word getZeroPosition();

  //returns true if there's an error
  bool error();

  private:

  byte spiCalcEvenParity(word);
};
#endif
