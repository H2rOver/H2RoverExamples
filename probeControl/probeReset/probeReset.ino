#include <GPS.h>
#include <H2RoverXbee.h>
#include <IMU.h>
#include <MotorControl.h>
#include <PinDeclarations.h>
#include <Ultrasound.h>

MotorControl Red;
volatile boolean bumperFlag;
static unsigned long time;

//time constants
#define TIME_WAITED millis() - time

void setup() {
  // put your setup code here, to run once:
  bumperFlag = false;
  Serial.begin(9600);
  time = 0;
  pinMode(FEELER, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FEELER), stopRed, FALLING);
  Red.probeUp(150);
}

void loop() {
      if (bumperFlag == true){        
        Red.probeOff();
        //bumperFlag == false;
        Red.probeDown(150);
        time = millis(); // start stopwatch
        while (TIME_WAITED <= 500){
          Serial.println(TIME_WAITED);
        }
        bumperFlag = false;
        Red.probeOff();
      }
}

void stopRed() {
  bumperFlag = true;
}

