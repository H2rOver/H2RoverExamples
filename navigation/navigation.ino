#include <H2RoverXbee.h>
#include <IMU.h>
#include <MotorControl.h>
#include <PinDeclarations.h>

MotorControl Red;
IMU Imu_obj;

volatile int ticks;
volatile boolean bumperFlag;
int16_t imu_readings1[3];

void setup() {
  // bumper setup:
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FEELER), stopRed, FALLING);

  //encoder setup
  ticks = 0; //900 ticks = 25.1cm
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
	ticksStart = ticks;
	ticksNow = ticks;
	while(bumperFlag == false && (ticksNow - ticksStart < 9000))
	{		  
	  forward_heading();
	  ticksNow = ticks;
	}
	if(ticksNow - ticksStart >= 9000)
	{
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

    //int timeStart;
    //int timeNow;
    int16_t imu_readings2[3];
	int w = 0;
    
    Imu_obj.getXYZ(imu_readings2);
    
    //when off by at least 2 degrees, correct, and then overcorrect by 2 degrees
	
	//Each of the adjustment methods have a print in there to stop the I2C from reading
	//This allows the bumper to add an interrupt to the queue, which could not happen
	//otherwise due to saturation. (This is a theory by Daniel B.)
    if(imu_readings2[0] + 2 < imu_readings1[0])
    {
	  Red.motorForwardRight(150);
      do{
        Imu_obj.getXYZ(imu_readings2);
        //Serial.print((int)bumperFlag);
      }while((imu_readings2[0] - 2 < imu_readings1[0]) && !bumperFlag);
    }
    else if(imu_readings2[0] - 2 < imu_readings1[0])
    {
	  Red.motorForwardLeft(150);
      do{
        Imu_obj.getXYZ(imu_readings2);
        //Serial.print((int)bumperFlag);
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
  
  Red.motorOff();
  timeStart = millis();
  do{
    timeNow = millis();
  }while(timeNow - timeStart < 1000);
  
  Imu_obj.getXYZ(imu_readings2);
  
  Red.motorBackward(255);
  timeStart = millis();
  do{
    timeNow = millis();
  }while(timeNow - timeStart < 1000);

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
  ticks++;
}