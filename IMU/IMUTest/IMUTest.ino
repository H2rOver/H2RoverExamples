
/*	Created By: Daniel Benusovich
	Created On: 4 November 2017
	Last Edited By: Daniel Benusovich
	Last Edited On: 4 November 2017
*/

#include <IMU.h>
#include <MotorControl.h>
#include <PinDeclarations.h>


IMU thingy;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(1000);
 /* 	The number passed in is the ID number for the sensor. It 	must be unique.
	The IMU will attempt to calibrate itself by turning and 	stopping repeatedly. Please contact the creator of the 	library if the IMU does not calibrates after several cycles
*/
  thingy.initialize(0);
}

void loop() {
  // put your main code here, to run repeatedly:
 //Declare a 16 bit int array of size three
 // 16 bits to fully represent the numbers
 // 3 indices for X Y and Z
 int16_t array[3];
 //Pass in the array for it to get populated
 thingy.getXYZ(array);
 //Read from the array! Easy day
 Serial.print("X: ");
 Serial.print(array[0], DEC);
 Serial.print("     Y: ");
 Serial.print(array[1], DEC);
 Serial.print("     Z: ");
 Serial.println(array[2], DEC);
}
