
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
	//This should match on all xbees
  xbee.setMaximumPacketSize(10);
}

void loop() {
  // put your main code here, to run repeatedly:
  //Make the maximum packet size available as the array
  //Not all of the array may be used in every message
  //The plus one size accounts for the length of the 
  //payload that is returned. It can be a max of     	//MAXIMUM_PACKET_SIZE 
  uint8_t payload[xbee.getMaximumPacketSize() + 1];
  
  //Populate array
  /* The status is important to check. A proper payload must be 	acknowledged and read
  */
  int status = xbee.getPacket(payload);
  
  //Check if the packet status is good then print the results
  //Or process them as ones could requires.
  if (status == H2RoverXbee::RECIEVED_PACKET_ACK){

	//We read the first payload value as it contains the 	//length of the payload
    for(int i = 0; i < payload[0]; i++){
      Serial.print(" Index: ");
      Serial.print(i, DEC);
      Serial.print(" Value: ");
	// You could not cast this and use jus the number
	//The cast is just for show
      Serial.print((char) payload[i + 1]);
      Serial.println();
    }
  } else if (status == H2RoverXbee::RECEIVED_RX_PACKET) {
    Serial.println("No Packet");
  }
  
}
