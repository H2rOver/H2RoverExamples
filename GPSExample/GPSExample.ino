#include <GPS.h>
#include <H2RoverXbee.h>
#include <IMU.h>
#include <MotorControl.h>
#include <PinDeclarations.h>

//Declare the gps object to be used
GPS gps;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Starting");
  //We must initialize the GPS
  //It is possible to be stuck in this state when attempting to aquire a fix
  gps.initialize();
}

void loop() {
  // put your main code here, to run repeatedly:
	//Need an array of size 8 to hold alll related data
	String array[8];
	//This methods populates the array and does all the necessary waiting 
	gps.getData(array);
	//Print out the values! Easy day!
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

