
/*	Created By: Daniel Benusovich
	Created On: 8 November 2017
	Last Edited By: Daniel Benusovich
	Last Edited On: 8 November 2017
*/

#include <H2RoverXbee.h>
#include <IMU.h>
#include <MotorControl.h>
#include <PinDeclarations.h>

H2RoverXbee xbee(0);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  uint8_t payload[H2RoverXbee::MAXIMUM_PACKET_SIZE];
  int status = xbee.getPacket(payload);
  if (status == H2RoverXbee::RECIEVED_PACKET_ACK){
    for(int i = 0; i < H2RoverXbee::MAXIMUM_PACKET_SIZE; i++){
      Serial.print(" Index: ");
      Serial.print(i, DEC);
      Serial.print(" Value: ");
      Serial.print(payload[i]);
      Serial.println();
    }
  } else if (status == H2RoverXbee::RECEIVED_RX_PACKET) {
    Serial.println("No Packet");
  }
}
