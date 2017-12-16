#ifndef as5048_h
#define as5048_h
#define LIBRARY_VERSION 1.0.1

#include <SPI.h>

class AS5048A{

	bool errorFlag;
	uint8_t _cs;
	// if the microcontroller is fast (e.g. ESP32) it should take a value around 50 so that the slave has time to send the response
	uint8_t response_delay_millis;
	uint16_t position;
	uint16_t transaction(uint16_t data);

	SPISettings settings;

	public:

	/**
	 *	Constructor
	 */
	AS5048A(uint8_t arg_cs, uint8_t arg_response_delay_millis);

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
	 * Takes the address of the register as a 16 bit uint16_t
	 * Returns the value of the register
	 */
	uint16_t read(uint16_t registerAddress);

	/*
	 * Write to a register
	 * Takes the 16-bit  address of the target register and the 16 bit uint16_t of data
	 * to be written to that register
	 * Returns the value of the register after the write has been performed. This
	 * is read back from the sensor to ensure a sucessful write.
	 */
	uint16_t write(uint16_t registerAddress, uint16_t data);

	/**
	 * Get the rotation of the sensor relative to the zero position.
	 *
	 * @return {int} between -2^13 and 2^13
	 */
	int32_t getRotation();

	/**
	 * Returns the raw angle directly from the sensor
	 */
	uint16_t getRawRotation();


	/**
	 * returns the value of the state register
	 * @return 16 bit uint16_t containing flags
	 */
	uint16_t getState();

	/**
	 * Print the diagnostic register of the sensor
	 */
	void printState();

	/**
	 * Returns the value used for Automatic Gain Control (Part of diagnostic
	 * register)
	 */
	uint8_t getGain();

	/*
	 * Get and clear the error register by reading it
	 */
	uint16_t getErrors();

	/*
	 * Set the zero position
	 */
	void setZeroPosition(uint16_t arg_position);

	/*
	 * Returns the current zero position
	 */
	uint16_t getZeroPosition();

	/**
	 * Get the rotation of the sensor relative to the zero position in degrees.
	 *
	 * @return {float} between 0 and 360
	 */
	float getRotationInDegrees();

	/**
	 * Get the rotation of the sensor relative to the zero position in radians.
	 *
	 * @return {float} between 0 and 2 * PI
	 */
	float getRotationInRadians();

	/*
	 * Check if an error has been encountered.
	 */
	bool error();

	private:

	uint8_t spiCalcEvenParity(uint16_t);
};
#endif
