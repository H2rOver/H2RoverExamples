#include <H2RoverXbee.h>
#include <IMU.h>
#include <MotorControl.h>
#include <PinDeclarations.h>

MotorControl Red;
IMU Imu_obj;

void setup() {
  // put your setup code here, to run once:
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FEELER), stopRed, LOW);
  Serial.begin(9600);
  delay(1000);

  Imu_obj.initialize(0); //ID? is zero
  delay(5000);
}

void loop() {
  int bumper;
    forward_heading();
      
    while(bumper == 1)
    {
      bumper = not(digitalRead(FEELER));
      Red.motorOff();
    }
}

void forward_heading() {
  int16_t imu_readings1[3];
  int16_t imu_readings2[3];

  Imu_obj.getXYZ(imu_readings1);

  while(1)
  {
    Red.motorForward(255);
    delay(100);
    Imu_obj.getXYZ(imu_readings2);
    while((imu_readings2[0]) + 1 < imu_readings1[0])
    {
      Red.motorForwardRight(150);
      Imu_obj.getXYZ(imu_readings2);
    }
    while((imu_readings2[0]) - 1 > imu_readings1[0])
    {
      Red.motorForwardLeft(150);
      Imu_obj.getXYZ(imu_readings2);
    }
  }
}

void stopRed() {
  Red.motorOff();
  delay(100);
}

