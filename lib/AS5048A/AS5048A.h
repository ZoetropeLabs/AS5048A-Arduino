#ifndef as5048_h
#define as5048_h
#define LIBRARY_VERSION 1.0.1

#include <SPI.h>

class AS5048A{

	bool errorFlag;
	byte _cs;
	byte cs;
	byte dout;
	byte din;
	byte clk;
	word position;
	word transaction(word data);
	
	SPISettings settings;
	
	public:

	/**
	 *	Constructor
	 */
	AS5048A(byte arg_cs);

	/**
	 * Initialiser
	 * Sets up the SPI interface
	 */
	void init();

	/**
	 * Closes the SPI connection
	 */
	void close();

	/*
	 * Read a register from the sensor
	 * Takes the address of the register as a 16 bit word
	 * Returns the value of the register
	 */
	word read(word registerAddress);

	/*
	 * Write to a register
	 * Takes the 16-bit  address of the target register and the 16 bit word of data
	 * to be written to that register
	 * Returns the value of the register after the write has been performed. This
	 * is read back from the sensor to ensure a sucessful write.
	 */
	word write(word registerAddress, word data);

	/**
	 * Get the rotation of the sensor relative to the zero position.
	 *
	 * @return {int} between -2^13 and 2^13
	 */
	int getRotation();

	/**
	 * Returns the raw angle directly from the sensor
	 */
	word getRawRotation();


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

	/*
	 * Get and clear the error register by reading it
	 */
	word getErrors();

	/*
	 * Set the zero position
	 */
	void setZeroPosition(word arg_position);

	/*
	 * Returns the current zero position
	 */
	word getZeroPosition();

	/*
	 * Check if an error has been encountered.
	 */
	bool error();

	private:

	byte spiCalcEvenParity(word);
};
#endif
