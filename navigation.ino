#include <H2RoverXbee.h>
#include <IMU.h>
#include <MotorControl.h>
#include <PinDeclarations.h>

MotorControl Red;
IMU Imu_obj;
  
int16_t imu_readings1[3];
int16_t imu_readings2[3];

void setup() {
  // put your setup code here, to run once:
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FEELER), stopRed, FALLING);
  Serial.begin(9600);
  delay(1000);

  Imu_obj.initialize(0); //ID? is zero
  delay(1000);
}

void loop() {
    forward_heading();
}

void forward_heading() {

  Imu_obj.getXYZ(imu_readings1);

  while(1)
  {
    Red.motorForward(255);
    delay(100);
    Imu_obj.getXYZ(imu_readings2);

    //when off by at least 2 degrees, correct, and then overcorrect by 2 degrees
    if(imu_readings2[0] + 2 < imu_readings1[0])
    {
      while((imu_readings2[0]) - 2 < imu_readings1[0])
      {
        Red.motorForwardRight(150);
        Imu_obj.getXYZ(imu_readings2);
      }
    }
    
    if(imu_readings2[0] - 2 < imu_readings1[0])
    {
      while((imu_readings2[0]) + 2 > imu_readings1[0])
      {
        Red.motorForwardLeft(150);
        Imu_obj.getXYZ(imu_readings2);
      }
    }
  }
}

void stopRed() {

  detachInterrupt(digitalPinToInterrupt(FEELER));
  Red.motorOff();
  delay(100);

  Serial.println("1");
  
  Imu_obj.getXYZ(imu_readings1);
  Serial.println("2");
  Imu_obj.getXYZ(imu_readings2);

  Serial.println("3");
  
  Red.motorBackward(255);
  delay(1000);

  Serial.println("4");
  //turn left 180 degrees
  while(imu_readings2[0] + 180 > imu_readings1[0])
  {
    //Serial.println("in loop");
    Red.motorLeft(255);
    Imu_obj.getXYZ(imu_readings2);
  }

  forward_heading(); 
}

