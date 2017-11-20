#include <H2RoverXbee.h>
#include <IMU.h>
#include <MotorControl.h>
#include <PinDeclarations.h>

#define TWO_FIVE_m 8964 //2.5m = 8964 encoder ticks
#define TEN_cm 359 //10cm = 359 ticks
#define DISTANCE_TRAVELED ticksNow - ticksStart

MotorControl Red;
IMU Imu_obj;

volatile int encoderTicks;
volatile boolean bumperFlag;
int16_t imu_readings1[3];

void setup() {
  // bumper setup:
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FEELER), stopRed, FALLING);

  //encoder setup
  encoderTicks = 0; //900 ticks = 25.1cm
  pinMode(ENCODER_FRONT_LEFT_INC, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_FRONT_LEFT_INC), EncoderEvent, FALLING);
  
  Serial.begin(9600);
  
  bumperFlag = false;

  delay(1000);
  Imu_obj.initialize(0); //ID? is zero
  delay(5000);
}

void loop() {
  int ticksStart, ticksNow;
	
  Imu_obj.getXYZ(imu_readings1);
  while(bumperFlag == false)
  {
    //start counting encoder ticks when we start moving forward
	  ticksStart = encoderTicks;
	  ticksNow = encoderTicks;
	  while(bumperFlag == false && (DISTANCE_TRAVELED < TWO_FIVE_m))
	  {		  
	    forward_heading();
	    ticksNow = encoderTicks;
	  }
	  if(DISTANCE_TRAVELED >= TWO_FIVE_m)
	  {
      //take a probe reading
      //take a GPS reading 
		  Red.motorOff();
		  delay(3000);
	  }
  }
  if(bumperFlag == true)
  {
    turn_around();
    bumperFlag = false;
  }
}

void forward_heading() {

    int16_t imu_readings2[3];
    
    Imu_obj.getXYZ(imu_readings2);
    
    //when off by at least 2 degrees, correct, and then overcorrect by 2 degrees
    if(imu_readings2[0] + 2 < imu_readings1[0])
    {
	  Red.motorForwardRight(255);
      do{
        Imu_obj.getXYZ(imu_readings2);
      }while((imu_readings2[0] - 2 < imu_readings1[0]) && !bumperFlag);
    }
    else if(imu_readings2[0] - 2 < imu_readings1[0])
    {
	  Red.motorForwardLeft(255);
      do{
        Imu_obj.getXYZ(imu_readings2);
      }while((imu_readings2[0] + 2 > imu_readings1[0]) && !bumperFlag);  //&& (timeNow - timeStart < 5000) 
    }
    else
    {
      Red.motorForward(255);
    }

}

void turn_around()
{
  int timeStart;
  int timeNow;
  int16_t imu_readings2[3];
  int ticksStart, ticksNow;
  
  //wait for motors to fully stop
  Red.motorOff();
  timeStart = millis();
  do{timeNow = millis();}while(timeNow - timeStart < 800);

  //reverse 10cm and stop
  Red.motorBackward(255);
  ticksStart = encoderTicks;
  do {
    ticksNow = encoderTicks;
  } while(DISTANCE_TRAVELED < TEN_cm); //backup 10cm
  //timeStart = millis();
  //do{timeNow = millis();}while(timeNow - timeStart < 1000);
  Red.motorOff();
  timeStart = millis();
  do{timeNow = millis();}while(timeNow - timeStart < 800);

  imu_readings1[0] = (imu_readings1[0] + 180)%360;

  do{
    Red.motorLeft(150);
    Imu_obj.getXYZ(imu_readings2);
  } while(imu_readings2[0] != imu_readings1[0]);
}

void stopRed() {
  bumperFlag = true;
}

void EncoderEvent() {
  encoderTicks++;
}
