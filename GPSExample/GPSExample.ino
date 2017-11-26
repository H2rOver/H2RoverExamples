#include <GPS.h>
#include <H2RoverXbee.h>
#include <IMU.h>
#include <MotorControl.h>
#include <PinDeclarations.h>


GPS gps;
uint32_t timer = millis();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Starting");
  gps.initialize();
}

void loop() {
  // put your main code here, to run repeatedly:
  gps.updateLocation();
  if (timer > millis())  timer = millis();
  
  if (millis() - timer > 2000) {
      timer = millis(); // reset the timer
      String array[8];
      gps.getData(array);
      Serial.print("\nTime: ");
      Serial.print(array[0]); Serial.print(':');
      Serial.print(array[1]); Serial.print(':');
      Serial.print(array[2]); Serial.print('.');
      Serial.print("Date: ");
      Serial.print(array[3]); Serial.print('/');
      Serial.print(array[4]); Serial.print("/20");
      Serial.println(array[5]);
      Serial.print("Fix: ");
      Serial.print(" quality: ");
        Serial.print("Location (in degrees, works with Google Maps): ");
        Serial.print(array[6]);
        Serial.print(", "); 
        Serial.println(array[7]);
       
      }
   }

