
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
  thingy.initialize(0);
}

void loop() {
  // put your main code here, to run repeatedly:
 int16_t array[3];
 thingy.getXYZ(array);
 Serial.print("X: ");
 Serial.print(array[0], DEC);
 Serial.print("     Y: ");
 Serial.print(array[1], DEC);
 Serial.print("     Z: ");
 Serial.println(array[2], DEC);
}
