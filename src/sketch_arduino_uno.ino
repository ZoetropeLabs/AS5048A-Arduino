#include <AS5048A.h>


AS5048A angleSensor(10);

void setup()
{
	Serial.begin(19200);
	angleSensor.init();
}

void loop()
{
	delay(1000);

	word val = angleSensor.getRawRotation();
	Serial.print("Got rotation of: 0x");
	Serial.println(val, HEX);
	Serial.print("State: ");
	angleSensor.printState();
	Serial.print("Errors: ");
	Serial.println(angleSensor.getErrors());
}
