#include <H2RoverXbee.h>
#include <IMU.h>
#include <MotorControl.h>
#include <PinDeclarations.h>

//distance constants
#define TWO_FIVE_m 8964 //2.5m = 8964 encoder ticks
#define TEN_cm 359 //10cm = 359 ticks
#define DISTANCE_TRAVELED encoderTicks - ticksStart

//time constants
#define DELAY_1s 1000
#define DELAY_3s 3000
#define TIME_WAITED millis() - time

//enumerated state type
enum states {
  forward1, //drive forward
  probe,  //stop and probe until probe finished
  stop1,  //stop when bumper is hit
  backup1,  //backup after bumper is hit
  stop2,  //stop backing up
  left1,  //turn left 90 after bumper backup
  stop3,  //stop turning left, turn on ultrasound
  forward2  //drive forward while listening to ultrasound
};

//global objects
MotorControl Red;
IMU Imu_obj;

//global variables
static int16_t imu_readings1[3];
volatile boolean bumperFlag;
static boolean overCorrectFlag;
static unsigned long time;
volatile unsigned long encoderTicks;
static unsigned long ticksStart;
static states currentState, nextState, previousState;

void setup() {
  // bumper setup
  bumperFlag = false;
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FEELER), stopRed, FALLING);

  //encoder setup
  encoderTicks = 0; //900 ticks = 25.1cm
  pinMode(ENCODER_FRONT_LEFT_INC, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_FRONT_LEFT_INC), EncoderEvent, FALLING);
  
  Serial.begin(9600);

  delay(1000);
  Imu_obj.initialize(0); //ID? is zero
  delay(5000);

  //initialize static variables
  overCorrectFlag = false;  //for driving straight
  time = 0;
  currentState = forward1;
  nextState = forward1;
  Imu_obj.getXYZ(imu_readings1); //establish imu starting point
}

void loop() {
  int16_t imu_readings2[3];
  
  switch (currentState) {
    
    //drive forward until bumper or 2.5m
    case forward1:
      if(previousState != forward1) {
        ticksStart = encoderTicks;  //start counting encoder ticks
        Imu_obj.getXYZ(imu_readings1); //establish imu starting point
      }
      forward_heading();
      if(bumperFlag == true) nextState = stop1;
      else if(DISTANCE_TRAVELED >= TWO_FIVE_m) nextState = probe;
      else nextState = forward1;
      break;
      
    //stop and probe until probe finished
    case probe: 
      if(previousState != probe) time = millis(); //start stopwatch
      Red.motorOff();
      //replace the time wait logic with check for "probe finished" logic
      if(TIME_WAITED >= DELAY_3s) nextState = forward1;
      else nextState = probe;
      break;
      
    //stop when bumper is hit
    case stop1: 
      bumperFlag = false; //reset bumper flag for next time
      if(previousState != stop1) time = millis(); //start stopwatch
      Red.motorOff();
      if(TIME_WAITED >= DELAY_1s) nextState = backup1;
      else nextState = stop1;
      break;
      
    //backup after bumper is hit for 1s (
    case backup1: 
      if(previousState != backup1) time = millis(); //start stopwatch
      Red.motorBackward(255);
      if(TIME_WAITED >= DELAY_1s) nextState = stop2;
      else nextState = backup1;
      break;
      
    //stop backing up
    case stop2: 
      if(previousState != stop2) time = millis(); //start stopwatch
      Red.motorOff();
      if(TIME_WAITED >= DELAY_1s) nextState = left1;
      else nextState = stop2;
      break;
      
    //turn left 90 after bumper backup
    case left1: 
      if(previousState != left1) imu_readings1[0] = (imu_readings1[0] + 270)%360;  //destination = current - 90 degrees
      Red.motorLeft(150);
      Imu_obj.getXYZ(imu_readings2);
      if(imu_readings2[0] == imu_readings1[0]) nextState = stop3;
      else nextState = left1;
      break;
      
    //stop turning left, turn on ultrasound
    case stop3: 
      if(previousState != stop3) time = millis(); //start stopwatch
      Red.motorOff();
      if(TIME_WAITED >= DELAY_1s) nextState = forward2;
      else nextState = stop3;
      break;
      
    //drive forward while listening to ultrasound
    case forward2:
      if(previousState != forward1) {
        ticksStart = encoderTicks;  //start counting encoder ticks
        Imu_obj.getXYZ(imu_readings1); //establish imu starting point
      }
      forward_heading();
      /* temporary holding place logic */
      nextState = forward1;
      /* real logic needs to check ultrasound until it stops seeing the object, then goto turn right state */
      break;
  }
  previousState = currentState;
  currentState = nextState;
}

void forward_heading() {

    int16_t imu_readings2[3];
    
    Imu_obj.getXYZ(imu_readings2);
    if(overCorrectFlag == true) {
      imu_readings2[0] = imu_readings2[0] - 2;
    }
    
    //when off by at least 1 degrees, correct, and then overcorrect by 2 degrees
    if(imu_readings2[0] + 1 < imu_readings1[0]) {
	    Red.motorForwardRight(255);
      overCorrectFlag = true;
    }
    else if(imu_readings2[0] - 1 < imu_readings1[0]) {
	    Red.motorForwardLeft(255);
      overCorrectFlag = true;
    }
    else {
      Red.motorForward(255);
      overCorrectFlag = false;
    }
}

void stopRed() {
  bumperFlag = true;
}

void EncoderEvent() {
  encoderTicks++;
}
