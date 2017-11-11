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
  uint8_t payload[] = {'0' , '1', '2', '3', '4', '5', '6', '7', '8', '9'};
  xbee.sendPacket(payload);
  delay(1000);
}
