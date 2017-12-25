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
  stop6,
  forward4a,
  forward4b,
  stop7,
  right2,
  stop8,
  forward5,
  stop9,
  left2,
  stop10
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
const uint8_t turningStrength = 150; 
const uint8_t initalUltSampleCount = 100;
 
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
   
	/*
	Default starting state.
	Drives forward until bumper is triggered or 2.5m has been traveled.
	Remains in forward1 until the switch on the bumber is triggered or the encoders indicate 2.5m has been traveled.
	Proceeds to stop1 once the bumper is triggered.
	Proceeeds to probe1 when 2.5m have been traveled.
	*/
    case forward1:
		Serial.println("forward1");
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
	  
	/*
	1/4 probing states.
	Lowers the probe into the ground.
	Remains in probe1 until the switch on the probing assembly is triggered.
	Proceeds to sampleData once triggered.
	*/
    case probe1: 
		Serial.println("probe1");
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

	/*
	2/4 probing states.
	Samples ground moisture levels.
	Remains in sampleData until 100 samples are taken.
	Proceeds to probe2 once finished taking samples.
	*/
    case sampleData:
		Serial.println("sampleData");
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
      
	/*
	3/4 probing states.
	Raises the probe up.
	Remains in probe2 until the switch on the probing assembly is triggered.
	Proceeds to sendData once triggered.
	*/
    case probe2:
		Serial.println("probe2");
		if(previousState != probe2){
			Red.probeUp(150);
		}
		if (bumperFlag == true){
			Red.probeOff();
			Red.probeDown(150);
			time = millis(); // start stopwatch
			while (TIME_WAITED <= 250){
			  //Serial.println(TIME_WAITED);
			}
			bumperFlag = false;
			Red.probeOff();
			nextState = sendData;
		}
		else nextState = probe2;
		break;
    
	/*
	CURRENTLY INCOMPLETE
	4/4 probing states.
	Transmits moisture content data to the Master vehicle
	Remains in sendData until the transmission is complete.
	Sets nextState depending on preProbeState allowing for probe1 to be called anywhere.
	*/
    case sendData:
		Serial.println("sendData");
		//ticksStart = encoderTicks;
		nextState = preProbeState;
		break;
    
	/*
	1/21 obstacle avoidance.
	Stops the rover after a bumper trigger.
	Remains in stop1 until DELAY_1s has been reached.
	Proceeds to backup1 once complete.
	*/
    case stop1: 
		Serial.println("stop1");
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
    
	/*
	2/21 obstacle avoidance.
	Back the rover up for DELAY_HALFs time to move away from the obstacle.
	Remains in backup1 until DELAY_HALFs has been reached.
	Proceeds to stop2 once complete.
	*/
    case backup1: 
		Serial.println("backup1");
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
    
	/*
	3/21 obstacle avoidance.
	Stops the rover after backing up.
	Remains in stop2 until DELAY_1s has been reached.
	Proceeds to left1 once complete.
	*/
    case stop2: 
		Serial.println("stop2");
		if(previousState != stop2) time = millis(); //start stopwatch
		Red.motorOff();
		if(TIME_WAITED >= DELAY_1s) nextState = left1;
		else nextState = stop2;
		break;
      
	/*
	4/21 obstacle avoidance.
	Turns the rover to the left 90 degrees using an IMU.
	Remains in left1 until the correct angle has been reached.
	Proceeds to stop3 once complete.
	*/
    case left1: 
		Serial.println("left1");
		if(previousState != left1) imu_readings1[0] = (imu_readings1[0] + 270)%360;  //destination = current - 90 degrees
		Red.motorLeft(turningStrength);
		Imu_obj.getXYZ(imu_readings2);
		Serial.println(imu_readings2[0]);
		if(imu_readings2[0] == imu_readings1[0]) nextState = stop3;
		else nextState = left1;
		break;
      
	/*
	5/21 obstacle avoidance.
	Stops the rover after turning left.
	Remains in stop3 until DELAY_1s has been reached.
	Proceeds to stop3 once complete.
	*/
    case stop3: 
		Serial.println("stop3");
		if(previousState != stop3) time = millis(); //start stopwatch
		Red.motorOff();
		if(TIME_WAITED >= DELAY_1s) {
			nextState = forward2a;
		}
		else nextState = stop3;
		break;
    
	/*
	6/21 obstacle avoidance.
	Turn on ultrasound, take 100 samples, and drive forward. RECORD ticksTraveledSide.
	Remains in forward2a until the distance the ultrasound sees is greater than the distance to the obstacle + 5.
	Proceeds to forward2b once complete.
	*/
    case forward2a:
		Serial.println("forward2a");
		if(previousState != forward2a) {
			ticksStart = encoderTicks;  //start counting encoder ticks
			Imu_obj.getXYZ(imu_readings1); //establish imu starting point
			initialUltDist = ultra.getDistance(initalUltSampleCount);
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
		
	/*
	7/21 obstacle avoidance.
	Drive forward for DELAY_HALFs time. RECORD ticksTraveledSide.
	Remains in forward2b until the time has elapsed.
	Proceeds to stop4 once complete.
	*/
	case forward2b:
		Serial.println("forward2b");
		if(previousState != forward2b) {
			ticksStart = encoderTicks;  //start counting encoder ticks
			Imu_obj.getXYZ(imu_readings1); //establish imu starting point
			time = millis();
		}
		Serial.print("sideTicks: "); Serial.println(encoderTicks - ticksStart);
		Serial.print("backwardTicks: "); Serial.println(backwardTicks);
		if (TIME_WAITED >= DELAY_HALFs){
			ticksTraveledSide += encoderTicks - ticksStart;
			nextState = stop4;
		}
		else {
			nextState = forward2b;
			forward_heading();
		}
		break;
	
	/*
	8/21 obstacle avoidance.
	Stops the rover after driving forward.
	Remains in stop4 until DELAY_1s has been reached.
	Proceeds to right1 once complete.
	*/
	case stop4: 
		Serial.println("stop4");
		if(previousState != stop4){
			time = millis(); //start stopwatch
			Red.motorOff();
		}
		if(TIME_WAITED >= DELAY_1s) nextState = right1;
		else nextState = stop4;
		break;
	
	/*
	9/21 obstacle avoidance.
	Turns the rover to the right 90 degrees using an IMU.
	Remains in right1 until the correct angle has been reached.
	Proceeds to stop5 once complete.
	*/
	case right1:
		Serial.println("right1");
		if(previousState != right1) {
			imu_readings1[0] = (imu_readings1[0] + 90)%360;  //destination = current + 90 degrees
			Red.motorRight(turningStrength);
		}
		Imu_obj.getXYZ(imu_readings2);
		if(imu_readings2[0] == imu_readings1[0]) nextState = stop5;
		else nextState = right1;
		break;
	
	/*
	10/21 obstacle avoidance.
	Stops the rover after turning right.
	Remains in stop5 until DELAY_1s has been reached.
	Proceeds to forward3 once complete.
	*/
	case stop5: 
		Serial.println("stop5");
		if(previousState != stop5){
			time = millis(); //start stopwatch
			Red.motorOff();
		}
		if(TIME_WAITED >= DELAY_1s) nextState = forward3;
		else nextState = stop5;
		break;
	
	/*
	11/21 obstacle avoidance.
	Turn on ultrasound, take 100 samples, and drive forward.
	Remains in forward3 until the distance the ultrasound sees is less than the distance past the obstacle - 5. Accounts for the possibility of a probing required.
	Proceeds to stop6 once the ultrasound sees the obstacle again.
	Proceeds to probe1 if 2.5m has been traveled.
	*/
	case forward3:
		Serial.println("forward3");
		if(previousState != forward3) {
			ticksStart = encoderTicks - ticksTraveledSide - (forwardTicks - backwardTicks);  //start counting encoder ticks
			Imu_obj.getXYZ(imu_readings1); //establish imu starting point
			initialUltDist = ultra.getDistance(initalUltSampleCount);
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
		if (initialUltDist > newUltDist + 20) nextState = stop6;
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
		
	/*
	12/21 obstacle avoidance.
	Stops the rover after encountering the obstacle via ultrasound.
	Remains in stop6 until DELAY_1s has been reached.
	Proceeds to forward4a once complete.
	*/
	case stop6:
		Serial.println("stop6");
		if(previousState != stop6){
			time = millis(); //start stopwatch
			Red.motorOff();
		}
		if(TIME_WAITED >= DELAY_1s) nextState = forward4a;
		else nextState = stop6;
		break;

	
	/*
	13/21 obstacle avoidance.
	Turn on ultrasound, take 100 samples, and drive forward.
	Remains in forward4a until the distance the ultrasound sees is greater than the distance to the obstacle + 5.
	Proceeds to forward4b once complete.
	*/
    case forward4a:
		Serial.println("forward4a");
		if(previousState != forward4a) {
			Imu_obj.getXYZ(imu_readings1); //establish imu starting point
			initialUltDist = ultra.getDistance(initalUltSampleCount);
			if (previousState == sendData) ticksStart = encoderTicks;
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
			nextState = forward4b;
		} 
		else if(DISTANCE_TRAVELED >= TWO_FIVE_m) {
			preProbeState = forward4a;
			nextState = probe1;
		}
		else {
			nextState = forward4a;
			forward_heading();
		}
		break;
		
	/*
	14/21 obstacle avoidance.
	Drive forward for DELAY_HALFs time.
	Remains in forward4b until the time has elapsed.
	Proceeds to stop7 once complete.
	*/
	case forward4b:
		Serial.println("forward4b");
		if(previousState != forward4b) {
			Imu_obj.getXYZ(imu_readings1); //establish imu starting point
			time = millis();
		}
		//Time delay
		if (TIME_WAITED >= DELAY_HALFs) nextState = stop7;
		else if(DISTANCE_TRAVELED >= TWO_FIVE_m) {
			preProbeState = forward4b;
			nextState = probe1;
		}
		else {
			nextState = forward4b;
			forward_heading();
		}
		break;
		
	/*
	15/21 obstacle avoidance.
	Stops the rover after passing the obstacle via ultrasound.
	Remains in stop7 until DELAY_1s has been reached.
	Proceeds to right2 once complete.
	*/
	case stop7: 
		Serial.println("stop7");
		if(previousState != stop7){
			time = millis(); //start stopwatch
			Red.motorOff();
		}
		if(TIME_WAITED >= DELAY_1s) nextState = right2;
		else nextState = stop7;
		break;
	
	/*
	16/21 obstacle avoidance.
	Turns the rover to the right 90 degrees using an IMU.
	Remains in right2 until the correct angle has been reached.
	Proceeds to stop8 once complete.
	*/
	case right2:
		Serial.println("right2");
		if(previousState != right2) {
			imu_readings1[0] = (imu_readings1[0] + 90)%360;  //destination = current + 90 degrees
			Red.motorRight(turningStrength);
		}
		Imu_obj.getXYZ(imu_readings2);
		if(imu_readings2[0] == imu_readings1[0]) nextState = stop8;
		else nextState = right2;
		break;
		
	/*
	17/21 obstacle avoidance.
	Stops the rover after turning right.
	Remains in stop8 until DELAY_1s has been reached.
	Proceeds to forward5 once complete.
	*/
	case stop8: 
		Serial.println("stop8");
		if(previousState != stop8){
			time = millis(); //start stopwatch
			Red.motorOff();
		}
		if(TIME_WAITED >= DELAY_1s) nextState = forward5;
		else nextState = stop8;
		break;

	/*
	18/21 obstacle avoidance.
	Drive forward the distance recorded in forward2a + forward2b via ticksTraveledSide.
	Remains in forward5 until the distance via ticksTraveledSide is traveled.
	Proceeds to stop9 once complete.
	*/
	case forward5:
		Serial.println("forward5");
		if(previousState != forward5){
			ticksStart = encoderTicks;
			Red.motorForward(255);
		}
		Serial.println(ticksTraveledSide);
		Serial.println(DISTANCE_TRAVELED);
		if (DISTANCE_TRAVELED >= ticksTraveledSide) nextState = stop9;
		else nextState = forward5;
		break;

	/*
	19/21 obstacle avoidance.
	Stops the rover after driving ticksTraveledSide distance.
	Remains in stop9 until DELAY_1s has been reached.
	Proceeds to left2 once complete.
	*/		
	case stop9:
		Serial.println("stop9");
		if(previousState != stop9){
			time = millis(); //start stopwatch
			Red.motorOff();
		}
		if(TIME_WAITED >= DELAY_1s) nextState = left2;
		else nextState = stop9;
		break;
		
	/*
	20/21 obstacle avoidance.
	Turns the rover to the left 90 degrees using an IMU.
	Remains in left2 until the correct angle has been reached.
	Proceeds to stop10 once complete.
	*/
    case left2: 
		Serial.println("left2");
		if(previousState != left2) imu_readings1[0] = (imu_readings1[0] + 270)%360;  //destination = current - 90 degrees
		Red.motorLeft(turningStrength);
		Imu_obj.getXYZ(imu_readings2);
		Serial.println(imu_readings2[0]);
		if(imu_readings2[0] == imu_readings1[0]) nextState = stop10;
		else nextState = left2;
		break;
	
	/*
	COMPLETE
	21/21 obstacle avoidance.
	Stops the rover after turning left.
	Remains in stop10 until DELAY_1s has been reached.
	Proceeds to forward1 once complete.
	*/	
  	case stop10: 
		Serial.println("stop10");
		if(previousState != stop10){
			time = millis(); //start stopwatch
			Red.motorOff();
		}
		if(TIME_WAITED >= DELAY_1s) nextState = forward1;
		else nextState = stop10;
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
