#include <GPS.h>
#include <H2RoverXbee.h>
#include <IMU.h>
#include <MotorControl.h>
#include <PinDeclarations.h>

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(ULTRASOUND_TRIGGER, OUTPUT);
  pinMode(ULTRASOUND_ECHO, INPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  long duration, distance;
  digitalWrite(ULTRASOUND_TRIGGER, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASOUND_TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASOUND_TRIGGER, LOW);
  duration = pulseIn(ULTRASOUND_ECHO, HIGH);
  distance = (duration/2) / 29.1;
  if(distance >= 200 || distance <= 0) {
    Serial.print(distance);
    Serial.println(" Out of Range");
  }
  else {
    Serial.print(distance);
    Serial.println(" cm");
  }
}

