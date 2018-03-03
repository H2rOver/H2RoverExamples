/*	Created By: Daniel Benusovich
	Created On: 8 November 2017
	Last Edited By: Daniel Benusovich and Jonathan Midolo
	Last Edited On: 3 March 2018
*/

#include <H2RoverXbee.h>
#include <IMU.h>
#include <MotorControl.h>
#include <PinDeclarations.h>

// If you want to transmit from END DEVICE to COORDINATOR
//     comment out all xbee1 references and uncomment all xbee references
// If you want to transmit from COORDINATOR to END DEVICE
//     comment out all xbee references and uncomment all xbee1 references

// For EC07(END) to transmit to EC02(COORD) use ID=1
//H2RoverXbee xbee(1);
// For EC02(COORD) to transmit to EC07(END) use ID=0
H2RoverXbee xbee1(0);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

}

void loop() {
//xbee.setMaximumPacketSize(10);
xbee1.setMaximumPacketSize(10);
  // put your main code here, to run repeatedly:
//First value holds the length of the payload DO NOT INCLUDE THE  // LENGTH OF THE PAYLOAD. Notice ten characters <= 10
  uint8_t payload[] = {6, 'A' , 'B', 'C', '1', '2', '3'};
  //xbee.sendPacket(payload);
  xbee1.sendPacket(payload);
  delay(1000);
}
