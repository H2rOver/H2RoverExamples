#include <GPS.h>
#include <H2RoverXbee.h>
#include <IMU.h>
#include <MotorControl.h>
#include <PinDeclarations.h>

H2RoverXbee xbee(1); // XBee module is end-device
GPS gps;

String array[8]; // for array of Strings from GPS library
String timeString;
String dateString;
String latString;
String longString;
char timeChar[9]; // HH:MM:SS = 8 characters + terminating = 9
char dateChar[11]; // MM/DD/YYYY = 10 characters + terminating = 11
char latChar[11]; // (-)###.#####(#) = 10 characters [1 () optional] + terminating = 11
char longChar[11]; // (-)###.#####(#) = 10 characters [1 () optional] + terminating = 11

void setup() {
      Serial.begin(9600);
      Serial.println("Starting");
      xbee.setMaximumPacketSize(10); // initialize XBee packet size
      gps.initialize(); // Initialize GPS settings
}

void loop() {

      // get new data from GPS module
      gps.getData(array);

      // print GPS data in Serial Monitor
      Serial.print("\nTime: ");
      Serial.print(array[0]); Serial.print(':');
      Serial.print(array[1]); Serial.print(':');
      Serial.print(array[2]);

      Serial.print("\nDate: ");
      Serial.print(array[3]); Serial.print('/');
      Serial.print(array[4]); Serial.print("/20"); // prefix year for Y2K
      Serial.print(array[5]);

      Serial.print("\nLocation: ");
      Serial.print(array[6]); Serial.print(", ");
      Serial.print(array[7]);

      // create character arrays from GPS library array of strings
      timeString = array[0]+":"+array[1]+":"+array[2];
      timeString.toCharArray(timeChar,9);
      dateString = array[3]+"/"+array[4]+"/20"+array[5];
      dateString.toCharArray(dateChar,11);
      latString = array[6];
      latString.toCharArray(latChar,11);
      longString = array[7];
      longString.toCharArray(longChar,11);

      Serial.println("\nXBee Data: "); // to preempt odd XBee serial data

      // send XBee payloads: Time -> Date -> Latitude -> Longitude
      uint8_t timeData[] = {strlen(timeChar),timeChar[0],timeChar[1],timeChar[2],timeChar[3],
            timeChar[4],timeChar[5],timeChar[6],timeChar[7]};

      xbee.sendPacket(timeData);

      uint8_t dateData[] = {strlen(dateChar),dateChar[0],dateChar[1],dateChar[2],dateChar[3],
            dateChar[4],dateChar[5],dateChar[6],dateChar[7],dateChar[8],dateChar[9]};

      xbee.sendPacket(dateData);

      uint8_t latData[] = {strlen(latChar),latChar[0],latChar[1],latChar[2],latChar[3],
            latChar[4],latChar[5],latChar[6],latChar[7],latChar[8],latChar[9]};

      xbee.sendPacket(latData);

      uint8_t longData[] = {strlen(longChar),longChar[0],longChar[1],longChar[2],longChar[3],
            longChar[4],longChar[5],longChar[6],longChar[7],longChar[8],longChar[9]};

      xbee.sendPacket(longData);
      }
