
/*	Created By: Daniel Benusovich
	Created On: 8 November 2017
	Last Edited By: Daniel Benusovich
	Last Edited On: 8 November 2017
*/

#include <H2RoverXbee.h>
#include <IMU.h>
#include <MotorControl.h>
#include <PinDeclarations.h>

//Setting up xbee as end device
H2RoverXbee xbee(0);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  //Make the maximum packet size available as the array
  //Not all of the array may be used in every message
  //This issue will be fixed shortly
  uint8_t payload[H2RoverXbee::MAXIMUM_PACKET_SIZE];
  
  //Populate array
  /* The status is important to check. A proper payload must be 	acknowledged and read
  */
  int status = xbee.getPacket(payload);
  
  //Check if the packet status is good then print the results
  //Or process them as ones could requires.
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
