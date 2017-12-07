#include <GPS.h>
#include <H2RoverXbee.h>
#include <IMU.h>
#include <MotorControl.h>
#include <PinDeclarations.h>
#include <Ultrasound.h>

Ultrasound sensor;
void setup() {
	// put your setup code here, to run once:
	Serial.begin(9600);
	//Give the sensor an unique ID number
	sensor.initialize(0);

}

void loop() {
	//100 is the number of samples to be averaged
	//See the Ultrasound class for more information
	Serial.println(sensor.getDistance(100));
}
