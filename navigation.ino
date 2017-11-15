#include <H2RoverXbee.h>
#include <IMU.h>
#include <MotorControl.h>
#include <PinDeclarations.h>

MotorControl Red;
IMU Imu_obj;

boolean bumperFlag;
int16_t imu_readings1[3];

void setup() {
  // put your setup code here, to run once:
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FEELER), stopRed, FALLING);
  Serial.begin(9600);
  
  bumperFlag = false;

  delay(1000);
  Imu_obj.initialize(0); //ID? is zero
  delay(5000);
}

void loop() {
  Imu_obj.getXYZ(imu_readings1);
  while(bumperFlag == false)
  {
    forward_heading();
  }
  if(bumperFlag == true)
  {
    turn_around();
    bumperFlag = false;
  }
}

void forward_heading() {

    int timeStart;
    int timeNow;
    int16_t imu_readings2[3];
    
    Imu_obj.getXYZ(imu_readings2);
    
    //when off by at least 2 degrees, correct, and then overcorrect by 2 degrees
    if(imu_readings2[0] + 2 < imu_readings1[0])
    {
      timeStart = millis();
      do{
        Red.motorForwardRight(150);
        Imu_obj.getXYZ(imu_readings2);
        timeNow = millis();
      }while((imu_readings2[0] - 2 < imu_readings1[0]) && (timeNow - timeStart < 5000));
    }
    else if(imu_readings2[0] - 2 < imu_readings1[0])
    {
      timeStart = millis();
      do{
        Red.motorForwardLeft(150);
        Imu_obj.getXYZ(imu_readings2);
        timeNow = millis();
      }while((imu_readings2[0] + 2 > imu_readings1[0]) && (timeNow - timeStart < 5000));
    }
    else
    {
      Red.motorForward(255);
    }

}

void turn_around()
{
  int16_t imu_readings2[3];
  
  Imu_obj.getXYZ(imu_readings2);
  Red.motorBackward(255);
  delay(1000);

  if(imu_readings1[0] < 180) {
    imu_readings1[0] = imu_readings1[0] + 360;
  }
  //turn left 180 degrees
  do{
    //Serial.println("in loop");
    Red.motorLeft(255);
    Imu_obj.getXYZ(imu_readings2);
    if(imu_readings2[0] < 180) {
      imu_readings2[0] = imu_readings2[0] + 360;
    }
    Serial.print("X1 = ");
    Serial.print(imu_readings1[0]);
    Serial.print(" ~~ X2 = ");
    Serial.println(imu_readings2[0]);
  } while(imu_readings2[0] + 180 > imu_readings1[0]);
}

void stopRed() {
  Red.motorOff();
  bumperFlag = true;
}

