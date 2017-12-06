#include <GPS.h>
#include <H2RoverXbee.h>
#include <IMU.h>
#include <MotorControl.h>
#include <PinDeclarations.h>
#include <Ultrasound.h>

//time constants
#define DELAY_HALFs 500
#define DELAY_THIRDSs 750
#define DELAY_1s 1000
#define DELAY_3s 3000
#define TIME_WAITED millis() - time

enum states {
  forward1, //drive forward
  probe1,  //stop and probe until probe finished
  sampleData,
  probe2,
  sendData,
  stop8
};

MotorControl Red;

static states currentState, nextState, previousState;

volatile boolean bumperFlag;
//Be sure to redefine time when using millis
static unsigned long time;

void setup() {
  // put your setup code here, to run once:
    // bumper setup
  bumperFlag = false;
  Serial.begin(9600);
  time = 0;
  pinMode(FEELER, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FEELER), stopRed, FALLING);
  previousState = forward1;
  currentState = forward1;
}

void loop() {
  // put your main code here, to run repeatedly:
Serial.println(currentState);
switch (currentState) {
  
    case forward1:
        nextState = probe1;
        break;
        
    //stop and probe until probe finished
    case probe1: 
      if(previousState != probe1){
        Red.motorOff();
        Red.probeDown(150);
        // do I need to make probe reading pin (MOISTURE_INPUT) an output/input or do we already...???
      }
      if(bumperFlag == true){        
        Serial.println("Testing Soil #1");
        
        Red.probeOff();
        Red.probeUp(150);
        
        time = millis(); // start stopwatch
        while (TIME_WAITED <= 100){
          Serial.println(TIME_WAITED);
        }
        bumperFlag = false;
        Red.probeOff();
        nextState = sampleData;
      }
      else nextState = probe1;
      break;

    case sampleData:
      if(previousState != sampleData){
        int testCount = 0;
        int moistureTest = 0;
        int moistureSum = 0;
        int moistureRead = 0;
        while(testCount < 100)
        {
          moistureTest = analogRead(MOISTURE_INPUT);
          moistureSum = moistureSum + moistureTest;
          time = millis(); // start stopwatch
          while (TIME_WAITED <= 10){
            Serial.println(TIME_WAITED);
          }
          testCount++;
        }
      moistureRead = moistureSum / 100;
      Serial.println(moistureRead);
}
      else nextState = probe2;
      break;
      
    case probe2:
      if(previousState != probe2){
        Red.probeUp(150);
        Serial.println("Probe2 bumperflag");
        Serial.println(bumperFlag);
      }
      if (bumperFlag == true){
        Serial.println("Raising the bar");
        
        Red.probeOff();
        Red.probeDown(150);
        time = millis(); // start stopwatch
        while (TIME_WAITED <= 500){
          Serial.println(TIME_WAITED);
        }
        bumperFlag = false;
        Red.probeOff();
        nextState = sendData;
      }
      else nextState = probe2;
      break;
    
    // data comms within...
    case sendData:
      nextState = stop8;
      break;

    case stop8:
      break;
}
  previousState = currentState;
  currentState = nextState;
}

void stopRed() {
  bumperFlag = true;
}

