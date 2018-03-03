/*	Created By: Daniel Benusovich
	Created On: 8 November 2017
	Last Edited By: Daniel Benusovich
	Last Edited On: 8 November 2017
*/

#include <H2RoverXbee.h>
#include <IMU.h>
#include <MotorControl.h>
#include <PinDeclarations.h>

H2RoverXbee xbee(1);
H2RoverXbee xbee1(0);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

}

void loop() {
xbee.setMaximumPacketSize(10);
  // put your main code here, to run repeatedly:
//First value holds the length of the payload DO NOT INCLUDE THE  // LENGTH OF THE PAYLOAD. Notice ten characters <= 10
  uint8_t payload[] = {6, 'A' , 'B', 'C', '1', '2', '3'};
  xbee.sendPacket(payload);
  xbee1.sendPacket(payload);
  delay(1000);
}
