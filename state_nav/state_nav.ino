#include <H2RoverXbee.h>
#include <IMU.h>
#include <MotorControl.h>
#include <PinDeclarations.h>
#include <Ultrasound.h>

//distance constants
#define TWO_FIVE_m 8964 //2.5m = 8964 encoder ticks
#define TEN_cm 359 //10cm = 359 ticks
#define DISTANCE_TRAVELED encoderTicks - ticksStart

//time constants
#define DELAY_HALFs 500
#define DELAY_THIRDSs 750
#define DELAY_1s 1000
#define DELAY_3s 3000
#define TIME_WAITED millis() - time

//enumerated state type
enum states {
  forward1, //drive forward
  probe1,  //stop and probe until probe finished
  sampleData,
  probe2,
  sendData,
  stop1,  //stop when bumper is hit
  backup1,  //backup after bumper is hit
  stop2,  //stop backing up
  left1,  //turn left 90 after bumper backup
  stop3,  //stop turning left, turn on ultrasound
  forward2a,  //drive forward while listening to ultrasound
  forward2b,
  stop4,
  right1, //temporary state for debgging
  stop5,
  forward3,
  stop7,
  forward4,
  stop6
};

//global objects
MotorControl Red;
IMU Imu_obj;
Ultrasound ultra;

//global variables
static int16_t imu_readings1[3];
volatile boolean bumperFlag;
static boolean overCorrectFlag;
//Be sure to redefine time when using millis
static unsigned long time;
volatile unsigned long encoderTicks;
static unsigned long ticksStart;
static states currentState, nextState, previousState, preProbeState;
uint16_t initialUltDist;
uint32_t forwardTicks;
uint32_t ticksTraveledSide;
uint32_t backwardTicks;
// when outside 255;
uint8_t turningStrength = 150; 
 
void setup() {
  // bumper setup
  bumperFlag = false;
  pinMode(FEELER, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FEELER), stopRed, FALLING);

  //encoder setup
  encoderTicks = 0; //900 ticks = 25.1cm
  pinMode(ENCODER_FRONT_LEFT_INC, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_FRONT_LEFT_INC), EncoderEvent, FALLING);
  
  Serial.begin(9600);
  ultra.initialize(1);
  delay(1000);
  Imu_obj.initialize(0); //ID? is zero
  delay(5000);

  //initialize static variables
  overCorrectFlag = false;  //for driving straight
  time = 0;
  previousState = forward1;
  currentState = forward1;
  nextState = forward1;
  Imu_obj.getXYZ(imu_readings1); //establish imu starting point
}

void loop() {
  int16_t imu_readings2[3];
  uint16_t newUltDist;
  
  switch (currentState) {
    
    //drive forward until bumper or 2.5m
    case forward1:
      if(previousState != forward1) {
        ticksStart = encoderTicks;  //start counting encoder ticks
        Imu_obj.getXYZ(imu_readings1); //establish imu starting point
      }
      forward_heading();
      if(bumperFlag == true) nextState = stop1;
      else if(DISTANCE_TRAVELED >= TWO_FIVE_m) {
		  preProbeState = forward1;
		  nextState = probe1;
	  }
      else nextState = forward1;
      break;
      
    //stop and probe until probe finished
	/*
    case probe: 
      if(previousState != probe) time = millis(); //start stopwatch
      Red.motorOff();
      //replace the time wait logic with check for "probe finished" logic
      if(TIME_WAITED >= DELAY_3s) nextState = forward1;
      else nextState = probe;
      break;
	*/
	  
	//new probe logic
    //stop and probe until probe finished
    case probe1: 
      if(previousState != probe1){
        Red.motorOff();
        Red.probeDown(150);
        // do I need to make probe reading pin (MOISTURE_INPUT) an output/input or do we already...???
      }
      if(bumperFlag == true){        
        //Serial.println("Testing Soil #1");
        
        Red.probeOff();
        Red.probeUp(150);
        
        time = millis(); // start stopwatch
        while (TIME_WAITED <= 100){
          //Serial.println(TIME_WAITED);
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
            //Serial.println(TIME_WAITED);
          }
          testCount++;
        }
      moistureRead = moistureSum / 100;
	  Serial.print("Moisture sensor reading: ");
      Serial.println(moistureRead);
	  nextState = sampleData;
	  }
      else nextState = probe2;
      break;
      
    case probe2:
      if(previousState != probe2){
        Red.probeUp(150);
        //Serial.println("Probe2 bumperflag");
        //Serial.println(bumperFlag);
      }
      if (bumperFlag == true){
        //Serial.println("Raising the bar");
        
        Red.probeOff();
        Red.probeDown(150);
        time = millis(); // start stopwatch
        while (TIME_WAITED <= 500){
          //Serial.println(TIME_WAITED);
        }
        bumperFlag = false;
        Red.probeOff();
        nextState = sendData;
      }
      else nextState = probe2;
      break;
    
    // data comms within...
    case sendData:
	  //ticksStart = encoderTicks;
      nextState = preProbeState;
      break;
      
    //stop when bumper is hit
    case stop1: 
      bumperFlag = false; //reset bumper flag for next time
      if(previousState != stop1) {
		forwardTicks = encoderTicks - ticksStart;
		time = millis(); //start stopwatch
		Red.motorOff();
		  Serial.print("encoder ticks1: "); Serial.println(encoderTicks);
	  }
      if(TIME_WAITED >= DELAY_1s) nextState = backup1;
      else nextState = stop1;
      break;
      
    //backup after bumper is hit for 1s (
    case backup1: 
      if(previousState != backup1) {
		  time = millis(); //start stopwatch
		  ticksStart = encoderTicks;
		  Serial.print("encoder ticks2: "); Serial.println(encoderTicks);
	  }
      Red.motorBackward(255);
      if(TIME_WAITED >= DELAY_HALFs) {
		  backwardTicks = encoderTicks - ticksStart;
		  Serial.println(backwardTicks);
		  nextState = stop2;
	  }
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
      Red.motorLeft(turningStrength);
      Imu_obj.getXYZ(imu_readings2);
	  Serial.println(imu_readings2[0]);
      if(imu_readings2[0] == imu_readings1[0]) nextState = stop3;
      else nextState = left1;
      break;
      
    //stop turning left, turn on ultrasound
    case stop3: 
      if(previousState != stop3) time = millis(); //start stopwatch
      Red.motorOff();
      if(TIME_WAITED >= DELAY_1s) {
		  nextState = forward2a;
	  }
      else nextState = stop3;
      break;
      
    //drive forward while listening to ultrasound
    case forward2a:
      if(previousState != forward2a) {
        ticksStart = encoderTicks;  //start counting encoder ticks
        Imu_obj.getXYZ(imu_readings1); //establish imu starting point
		initialUltDist = ultra.getDistance(100);
      }
      /* temporary holding place logic */
	  //add 5 cm for a small buffer. To be changed later
	  newUltDist = ultra.getDistance(1);
	  Serial.println("Forward 3");
	  Serial.println("Inital then new");
	  Serial.println(initialUltDist);
	  Serial.println(newUltDist);
      if (initialUltDist + 5 < newUltDist){
		ticksTraveledSide = encoderTicks - ticksStart;
		nextState = forward2b;
	  } else {
		nextState = forward2a;
		forward_heading();
	  }
		break;
		
	//after ultrasound obstacle passed, drive additional distance
	case forward2b:
      if(previousState != forward2b) {
        ticksStart = encoderTicks;  //start counting encoder ticks
        Imu_obj.getXYZ(imu_readings1); //establish imu starting point
		time = millis();
      }
	  Serial.print("sideTicks: "); Serial.println(encoderTicks - ticksStart);
	  Serial.print("backwardTicks: "); Serial.println(backwardTicks);
	  //changing to time
      // if (encoderTicks - ticksStart >= backwardTicks % 50){
		// ticksTraveledSide += encoderTicks - ticksStart;
		// nextState = stop4;
	  // } else {
	  if (TIME_WAITED >= DELAY_HALFs){
		nextState = stop4;
	  }
	  else {
		nextState = forward2b;
		forward_heading();
	  }
		break;
	  
	//Stop to begin turning right
	case stop4: 
		if(previousState != stop4){
			time = millis(); //start stopwatch
			Red.motorOff();
		}
		if(TIME_WAITED >= DELAY_1s) nextState = right1;
		else nextState = stop4;
		break;
	  
	//Turn 90 degrees to the right after passing the obstacle
	case right1:
		if(previousState != right1) {
			imu_readings1[0] = (imu_readings1[0] + 90)%360;  //destination = current + 90 degrees
			Red.motorRight(turningStrength);
		}
		Imu_obj.getXYZ(imu_readings2);
		if(imu_readings2[0] == imu_readings1[0]) nextState = stop5;
		else nextState = right1;
		break;
		
	//Stop turning to prepare to move forward
	case stop5: 
		if(previousState != stop5){
			time = millis(); //start stopwatch
			Red.motorOff();
		}
		if(TIME_WAITED >= DELAY_1s) nextState = forward3;
		else nextState = stop5;
		break;
		
	//Drive forward while listening to ultrasound to change values drastically twice
	//This means we encounter the obstacle once, then leave its area of effect
	//We have not encountered the object again yet
	case forward3:
      if(previousState != forward3) {
        ticksStart = encoderTicks - ticksTraveledSide - (forwardTicks - backwardTicks);  //start counting encoder ticks
        Imu_obj.getXYZ(imu_readings1); //establish imu starting point
		initialUltDist = ultra.getDistance(100);
		if (previousState == sendData) {
		  ticksStart = encoderTicks;
		}
      }
      /* temporary holding place logic */
	  //add 5 cm for a small buffer. To be changed later
	  newUltDist = ultra.getDistance(1);
	  Serial.println("Forward 3");
	  Serial.println("Inital then new");
	  Serial.println(initialUltDist);
	  Serial.println(newUltDist);
      if (initialUltDist > newUltDist + 20){
		  nextState = stop7;
	  } 
	  //Stops and takes a reading whenever we travel 2.5m. no matter what!
	  else if(DISTANCE_TRAVELED >= TWO_FIVE_m) {
		  preProbeState = forward3;
		  nextState = probe1;
	  }
	  else {
		  nextState = forward3;
		  forward_heading();
	  }
		break;
		
	case stop7:
		if(previousState != stop7){
			time = millis(); //start stopwatch
			Red.motorOff();
		}
		if(TIME_WAITED >= DELAY_1s) nextState = forward4;
		else nextState = stop7;
		break;

	
	//We have encountered the pbject again
	//drive forward while listening to ultrasound
    case forward4:
      if(previousState != forward4) {
        Imu_obj.getXYZ(imu_readings1); //establish imu starting point
		initialUltDist = ultra.getDistance(100);
		if (previousState == sendData) {
		  ticksStart = encoderTicks;
		}
      }
      /* temporary holding place logic */
	  //add 5 cm for a small buffer. To be changed later
	  newUltDist = ultra.getDistance(1);
	  Serial.println("Forward 4");
	  Serial.println("Inital then new");
	  Serial.println(initialUltDist);
	  Serial.println(newUltDist);
      if (initialUltDist + 5 < newUltDist){
		  forwardTicks += encoderTicks - ticksStart;
		  nextState = stop6;
	  } 
	  else if(DISTANCE_TRAVELED >= TWO_FIVE_m) {
		  preProbeState = forward4;
		  nextState = probe1;
	  }
	  else {
		  nextState = forward4;
		  forward_heading();
	  }
		break;
		
	case stop6:
		Red.motorOff();
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
