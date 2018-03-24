#include <GPS.h>
#include <H2RoverXbee.h>
#include <IMU.h>
#include <MoistureSensor.h>
#include <MotorControl.h>
#include <PinDeclarations.h>
#include <Ultrasound.h>

//distance constants
#define TWO_FIVE_m 8964 //2.5m = 8964 encoder ticks // MIDOLO???
#define TEN_cm 359 //10cm = 359 ticks // MIDOLO???
#define DISTANCE_TRAVELED encoderTicks - ticksStart

//time constants
#define DELAY_HALFs 500
#define DELAY_THIRDSs 750
#define DELAY_1s 1000
#define DELAY_3s 3000
#define TIME_WAITED millis() - time
#define TIMEOUT_WAITED millis() - timeOut
#define PROBE_TIMEOUT 100000

#ifndef M_PI
#define M_PI 3.14159265358979323846 // pi to 20 decimal places
#endif

// MIDOLO math...stdio????

/* IMPORTANT NOTE!:
     Henceforth, all references to "left" and "right" may actually be misnomers,
     depending on which way Red is moving and in which position she started in.
     For the purposes of understanding, we assume Red begins its navigation in a
     left to right, or eastward, direction. The nomenclature "Left" then refers
     to a left turn if Red is moving eastward. The nomenclature "Right" refers to
     a right turn if Red is moving eastward. When Red is moving right to left, or
     westward, the terms "right" and "left" will actually mean the opposite.
*/

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
  stop10,
  plannedLeftA,		//stop
  plannedLeftB,		//90 degree left turn
  plannedLeftC,		//stop again
  rtm1,
  rtm2,
  rtm3,
  rtm4
};

//State Variables
static states currentState, nextState, previousState, preProbeState;

//global objects
MotorControl Red;
IMU Imu_obj;
Ultrasound ultrasound;
//We send to the coordinator from the end device
//We can read from the xbee without distinction of mac address
//MAC address ends with 07. transmits to MAC address 02.
H2RoverXbee xbee_end_to_coordinator(1);
GPS gps;
MoistureSensor moisture(1);

//Xbee packet Length
uint8_t packet_length = 12; // MIDOLO CHANGE

//global control variables
static int16_t imu_readings1[3];
volatile boolean bumperFlag;
static boolean overCorrectFlag;
// motor strength -- grass=? / dirt=?
const uint8_t forwardStrength = 130;
const uint8_t turningStrength = 130;
const uint8_t probeMotorStength = 130;


//Timing Variables
//Be sure to redefine time when using millis
static unsigned long time;
static unsigned long timeOut;
static boolean timeOutFlag;

//Distance Variables
volatile unsigned long encoderTicks;
static unsigned long ticksStart;
uint16_t initialUltDist;
uint32_t forwardTicks;
uint32_t ticksTraveledSide;
uint32_t backwardTicks;

//Sampling Variables
static uint8_t sampleCount;

//Sensor variables
const uint8_t initalUltSampleCount = 100; // MIDOLO what is this? -> rename?
const uint8_t newUltSampleCount = 5;
const uint8_t initalMoistureSampleCount = 50; // MIDOLO what is this? -> rename?
uint16_t moistureLevel = 1023; //This implies it is very dry

//Constant to control how many samples red takes before moving to next row
const uint8_t SAMPLES_PER_ROW = 12;		//Minimum is 2

//Variables to control which direction Red is turning, initialized in setup
uint16_t leftAngle;
uint16_t rightAngle;

//GPS variables
//GPS data array
String array[9];

String timeString;
String dateString;
String latString;
String longString;
// Create character arrays from GPS library array of strings
char timeChar[9]; // HH:MM:SS = 8 characters + terminating = 9
char dateChar[11]; // MM/DD/YYYY = 10 characters + terminating = 11
char latChar[11]; // (-)###.#####(#) = 10 characters [1 () optional] + terminating = 11
char longChar[12]; // (-)###.#####(#) = 10 characters [1 () optional] + terminating = 11
// MIDOLO - long needs 6 decimal precision...upping longChar to 12 -> 11 char + terminating
// MIDOLO - need to check if time is moving...need to add battery backup

// uint8_t timeData[9];
// uint8_t dateData[11];
// uint8_t latData[11];
// uint8_t longData[11];
////MOISTURE Sdata variables
// uint8_t moisture_multiplier;
// uint8_t moisture_remainder;
// uint8_t moisture_data[4];


double masterLatitude; // degrees
double masterLongitude; // degrees
double redLatitude; // degrees
double redLongitude; // degrees
double redLatitudeA; // degrees
double redLongitudeA; // degrees
double redLatitudeB; // degrees
double redLongitudeB; // degrees
// double destLatitude;
// double destLongitude;
double homeBearing;

void setup() {

  // For terminal output debugging
  Serial.begin(9600);

  //GPS setup // MIDOLO...do fix=2 right away or wait till sample and see...?
  gps.initialize();

  //Xbee send/recieve setup
  xbee_end_to_coordinator.setMaximumPacketSize(packet_length);

  // bumper setup
  bumperFlag = false;
  pinMode(FEELER, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FEELER), stopRed, FALLING);

  //encoder setup
  encoderTicks = 0; //900 ticks = 25.1cm // MIDOLO ??
  pinMode(ENCODER_FRONT_LEFT_INC, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_FRONT_LEFT_INC), EncoderEvent, FALLING);

  ultrasound.initialize(1);
  Imu_obj.initialize(0, turningStrength); //ID is 0

  //initialize static variables
  overCorrectFlag = false;  //for driving straight
  timeOutFlag = false;
  time = 0;
  sampleCount = 0;
  leftAngle = 270;
  rightAngle = 90;
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
		Red.motorOff();
		while(!bumperFlag){
			Red.probeDown(probeMotorStength);
		}

		bumperFlag = false;
		Red.probeOff();
		Red.probeUp(probeMotorStength);

		time = millis(); // start stopwatch
		while (TIME_WAITED <= 400){
		//Serial.println(TIME_WAITED);
		}

		bumperFlag = false;
		Red.probeOff();
		nextState = sampleData;
/* 		if(previousState != probe1){
			Red.motorOff();
			Red.probeDown(probeMotorStength);
			Serial.println("probe down till bumper");
			// do I need to make probe reading pin (MOISTURE_INPUT) an output/input or do we already...???
			timeOut = millis();
		}
		if(bumperFlag == true){
		//Serial.println("Testing Soil #1");
		Serial.println("bumper hit");
			Red.probeOff();
			Red.probeUp(probeMotorStength);

			time = millis(); // start stopwatch
			while (TIME_WAITED <= 400){
			  //Serial.println(TIME_WAITED);
			}
			bumperFlag = false;
			Red.probeOff();
			nextState = sampleData;
		}
		//if it has been in this state for more than 3 seconds
		else if(TIMEOUT_WAITED >= PROBE_TIMEOUT) {
			nextState = sampleData;
			timeOutFlag = true;
		}
		else nextState = probe1; */
		break;

	/*
	2/4 probing states.
	Samples ground moisture levels.
	Remains in sampleData until 100 samples are taken. // MIDOLO this is not happening anymore?
	Proceeds to probe2 once finished taking samples.
	*/
    case sampleData:
		Serial.println("sampleData");
		if(previousState != sampleData){
			moistureLevel = moisture.getData(initalMoistureSampleCount);
			Serial.print("Moisture sensor reading: ");
			Serial.println(moistureLevel);
			sampleCount += 1;		//increment sampleCount
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
			Red.probeUp(probeMotorStength);
			timeOut = millis();
		}
		if (bumperFlag == true){
			Red.probeOff();
			Red.probeDown(probeMotorStength);
			time = millis(); // start stopwatch
			while (TIME_WAITED <= 400){
			  //Serial.println(TIME_WAITED);
			}
			bumperFlag = false;
			Red.probeOff();
			nextState = sendData;
		}
		//if it has been in this state for more than 3 seconds
		else if(TIMEOUT_WAITED >= PROBE_TIMEOUT) {
			nextState = sendData;
			timeOutFlag = true;
		}
		else nextState = probe2;
		break;

	/*
	CURRENTLY INCOMPLETE
	4/4 probing states.
	Transmits moisture content data to the Master vehicle
	Remains in sendData until the transmission is complete.
	Sets nextState depending on preProbeState allowing for probe1 to be called anywhere.
	When it reaches the 12th sample (in the row), will initiate a planned right or left turn (depending)
	When it reaches the 1st sample (in the row), will turn right or left (depending) to align itself horizontally with next row
	*/
    case sendData:
		Serial.println("sendData");
		gps_data();
		send_data();
		if((sampleCount%SAMPLES_PER_ROW == 0) || (sampleCount%SAMPLES_PER_ROW == 1 && sampleCount != 1)){
			nextState = plannedLeftA;
		}
		else nextState = preProbeState;
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
		Red.motorBackward(forwardStrength);
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
		/*
		if(previousState != left1) {
			imu_readings1[0] = (imu_readings1[0] + leftAngle)%360;
			if(leftAngle == 270) Red.motorLeft(turningStrength);
			else Red.motorRight(turningStrength);
		}*/
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
			initialUltDist = ultrasound.getDistance(initalUltSampleCount);
		}
		/* temporary holding place logic */
		//add 5 cm for a small buffer. To be changed later
		newUltDist = ultrasound.getDistance(newUltSampleCount);
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
			/*
			imu_readings1[0] = (imu_readings1[0] + rightAngle)%360;
			if(rightAngle == 90) Red.motorRight(turningStrength);
			else Red.motorLeft(turningStrength);
			*/
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
			initialUltDist = ultrasound.getDistance(initalUltSampleCount);
			if (previousState == sendData) {
				ticksStart = encoderTicks;
			}
		}
		/* temporary holding place logic */
		//add 5 cm for a small buffer. To be changed later
		newUltDist = ultrasound.getDistance(newUltSampleCount);
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
			initialUltDist = ultrasound.getDistance(initalUltSampleCount);
			if (previousState == sendData) ticksStart = encoderTicks;
		}
		/* temporary holding place logic */
		//add 5 cm for a small buffer. To be changed later
		newUltDist = ultrasound.getDistance(newUltSampleCount);
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
			/*
			imu_readings1[0] = (imu_readings1[0] + rightAngle)%360;
			if(rightAngle == 90) Red.motorRight(turningStrength);
			else Red.motorLeft(turningStrength);
			*/
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
			//Red.motorForward(forwardStrength);
			forward_heading();
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
		/*
		if(previousState != left2) {
			imu_readings1[0] = (imu_readings1[0] + leftAngle)%360;
			if(leftAngle == 270) Red.motorLeft(turningStrength);
			else Red.motorRight(turningStrength);
		}
		*/
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

	/*
	1/3 Planned Left Turn.
	Stops the rover after reaching the Nth sample (default SAMPLES_PER_ROW = 12) in odd row, or 1st sample in even row.
	Remains in plannedLeftA until DELAY_1s has been reached.
	Proceeds to plannedLeftB once complete.
	*/
	case plannedLeftA:
		Serial.println("plannedLeftA");
		if(previousState != plannedLeftA){
			time = millis(); //start stopwatch
			Red.motorOff();
		}
		if(TIME_WAITED >= DELAY_1s) nextState = plannedLeftB;
		else nextState = plannedLeftA;
		break;

	/*
	2/3 Planned Left Turn.
	Turns the rover to the left 90 degrees using the IMU.
	Remains in plannedLeftB until the correct angle has been reached.
	Proceeds to plannedLeftC once complete.
	*/
    case plannedLeftB:
		Serial.println("plannedLeftB");
		//if(previousState != plannedLeftB) imu_readings1[0] = (imu_readings1[0] + 270)%360;  //destination = current - 90 degrees
		if(previousState != plannedLeftB) {
			imu_readings1[0] = (imu_readings1[0] + leftAngle)%360;
			if(leftAngle == 270) Red.motorLeft(turningStrength);
			else Red.motorRight(turningStrength);
		}
		Imu_obj.getXYZ(imu_readings2);
		Serial.println(imu_readings2[0]);
		if(imu_readings2[0] == imu_readings1[0]) nextState = plannedLeftC;
		else nextState = plannedLeftB;
		break;

	/*
	3/3 Planned Left Turn.
	Stops the rover after reaching the end of the left turn (which has occurred after 12th sample in row, or 1st sample in row).
	Remains in plannedLeftC until DELAY_1s has been reached.
	Proceeds to forward1 once complete.
	*/
	case plannedLeftC:
		Serial.println("plannedLeftC");
		if(previousState != plannedLeftC){
			time = millis(); //start stopwatch
			Red.motorOff();
			//redifine "left" and "right" angles when starting a new row
			if(sampleCount%SAMPLES_PER_ROW == 1){
				uint8_t tempAngle = leftAngle;
				leftAngle = rightAngle;
				rightAngle = tempAngle;
			}
		}
		if(TIME_WAITED >= DELAY_1s) nextState = forward1;
		else nextState = plannedLeftC;
		break;

	// MIDOLO RTM(return to master routine/algorithm) find bearing
	// rtm1 - stop and get first gps reading
	case rtm1:
		Serial.println("rtm1");
		if(previousState != rtm1){
			time = millis(); //start stopwatch
			Red.motorOff();
			gps_data();
			redLatitudeA = latString.toFloat(); // MIDOLO - will need chararraytostring to do reverse for master
			redLongitudeA = longString.toFloat();
		}
		if(TIME_WAITED >= DELAY_1s) nextState = rtm2;
		else nextState = rtm1;
		break;

	// rtm2 - drive straight 2.5m
	case rtm2:
		Serial.println("rtm2");
		if(previousState != rtm2) {
			ticksStart = encoderTicks;  //start counting encoder ticks
			Imu_obj.getXYZ(imu_readings1); //establish imu starting point
		}
		forward_heading();
		//if(bumperFlag == true) nextState = stop1; // MIDOLO no obstacle avoidance on rtm
		//else
		if(DISTANCE_TRAVELED >= TWO_FIVE_m) {
		  nextState = rtm3;
		}
		else nextState = rtm2;
		break;

	// rtm3 - stop and get second gps reading
	case rtm3:
		Serial.println("rtm3");
		if(previousState != rtm3){
			time = millis(); //start stopwatch
			Red.motorOff();
			gps_data();
			redLatitudeB = latString.toFloat(); // MIDOLO - will need chararraytostring to do reverse for master
			redLongitudeB = longString.toFloat();
		}
		if(TIME_WAITED >= DELAY_1s) nextState = rtm4;
		else nextState = rtm3;
		break;

	// rtm4 - calculate bearing and distance and turn red
	//if homeBearing negative then ccw so turn left else cw turn right
	//can we reset imu 0 degrees to redBearing?
	case rtm4:
		Serial.println("rtm4");
		if(previousState != rtm4){
			rtm_data();
		}
		break;

  } // end of switch / "state machine"

    int status = recieve_data(); // recieve command from master after every state or move to a specific state
        // MIDOLO if command = 0 continue if command = 1 return if command = 2 stop
		// opcode 0 init 1 normal 2 something 3 retunrn
    previousState = currentState;
    currentState = nextState;
} // end of void loop

void forward_heading() {

    int16_t imu_readings2[3];

    Imu_obj.getXYZ(imu_readings2);
    if(overCorrectFlag == true) {
        imu_readings2[0] = imu_readings2[0] - 2;
    }

    // when off by at least 1 degree, correct, and then overcorrect by 2 degrees
    if(imu_readings2[0] + 1 < imu_readings1[0]) {
	    Red.motorForwardRight(forwardStrength);
        overCorrectFlag = true;
    }
    else if(imu_readings2[0] - 1 < imu_readings1[0]) {
	    Red.motorForwardLeft(forwardStrength);
        overCorrectFlag = true;
    }
    else {
        Red.motorForward(forwardStrength);
        overCorrectFlag = false;
    }
}

void stopRed() {
    bumperFlag = true;
}

void EncoderEvent() {
    encoderTicks++;
}

void gps_data() {

	// Check 1 of 3 tests below...
    // INDOOR TEST - Check time counting
    while(array[0].toInt() == 0 && array[1].toInt() == 0 && array[2].toInt() == 0){
        gps.getData(array);
    }

    // OUTDOOR TEST - Check latitude population
    // while(array[6].toInt() == 0){
        // gps.getData(array);
    // }

	// FUNCTIONAL TEST - Check for fixquality == 2
    // while(array[8].toInt() != 2){
        // gps.getData(array);
    // }

	// FINAL DATA PULL (Test Pull Cleared)
	gps.getData(array);

	// MIDOLO removed string declar here
	// Construct GPS data into proper formatting
	timeString = array[0]+":"+array[1]+":"+array[2];
	timeString.toCharArray(timeChar,9);
	dateString = array[3]+"/"+array[4]+"/20"+array[5];
	dateString.toCharArray(dateChar,11);
	latString = array[6];
	latString.toCharArray(latChar,11);
	longString = array[7];
	longString.toCharArray(longChar,11);

	// GPS Serial Output for Debugging
	Serial.println("GPS Data:");
	for(int i = 0; i < 8; i++){
		Serial.println(array[i]);
	}
	Serial.println("Format Check:");
	Serial.print("Time: ");
	Serial.println(timeString);
	Serial.print("Date: ");
	Serial.println(dateString);
	Serial.print("Latitude: ");
	Serial.println(latString);
	Serial.print("Longitude: ");
	Serial.println(longString);
}

void send_data(){

	// Prepare moisture data for transmission
	char moisture_multiplier = moistureLevel / 255;
	char moisture_remainder = moistureLevel % 255;

	// Send XBee payloads: Time -> Date -> Latitude -> Longitude -> Moisture Data w/ Sample #
	uint8_t timeData[] = {strlen(timeChar),timeChar[0],timeChar[1],timeChar[2],timeChar[3],
		timeChar[4],timeChar[5],timeChar[6],timeChar[7]};
	xbee_end_to_coordinator.sendPacket(timeData);

	uint8_t dateData[] = {strlen(dateChar),dateChar[0],dateChar[1],dateChar[2],dateChar[3],
		dateChar[4],dateChar[5],dateChar[6],dateChar[7],dateChar[8],dateChar[9]};
	xbee_end_to_coordinator.sendPacket(dateData);

	uint8_t latData[] = {strlen(latChar),latChar[0],latChar[1],latChar[2],latChar[3],
		latChar[4],latChar[5],latChar[6],latChar[7],latChar[8],latChar[9]};
	xbee_end_to_coordinator.sendPacket(latData);

	uint8_t longData[] = {strlen(longChar),longChar[0],longChar[1],longChar[2],longChar[3],
		longChar[4],longChar[5],longChar[6],longChar[7],longChar[8],longChar[9]};
	xbee_end_to_coordinator.sendPacket(longData);
	// MIDOLO need to adjust longData for change in array length

	// MIDOLO i think the length is 4...
	uint8_t moisture_data[] = {3, moisture_multiplier, moisture_remainder, sampleCount};
	xbee_end_to_coordinator.sendPacket(moisture_data);
}

int recieve_data() {
    // MIDOLO need to add fxn - return status
	// receive op code ... need to send something to master to know what to send back
	// need to recieve GPS ALSO !!!!!

	  //Make the maximum packet size available as the array
  //Not all of the array may be used in every message
  //The plus one size accounts for the length of the
  //payload that is returned. It can be a max of     	//MAXIMUM_PACKET_SIZE
  uint8_t payload[xbee_end_to_coordinator.getMaximumPacketSize() + 1];

  //Populate array
  /* The status is important to check. A proper payload must be 	acknowledged and read
  */
  int status = xbee_end_to_coordinator.getPacket(payload);

  //Check if the packet status is good then print the results
  //Or process them as ones could requires.
  if (status == H2RoverXbee::RECIEVED_PACKET_ACK){

	//We read the first payload value as it contains the 	//length of the payload
    for(int i = 0; i < payload[0]; i++){
      Serial.print((char) payload[i + 1]);
    }
      Serial.println();
  } else if (status == H2RoverXbee::RECEIVED_RX_PACKET) {
    Serial.println("No Packet");
  }
	return 0;
}

void rtm_data() {
	// variables for calculating bearing
	double bearingX;
    double bearingY;
	// bearing angles (radians)
    double redBearing; // from due N positive CW
    double masterBearing; // from red location (as if due N) positive CW


	// Haversine formula variables
    double a,c;
    double radiusEarth = 6371; // km (maybe int...?)
    double distToMaster; // m

    double distBuffer = 2.5; // m (maybe int...?)

	// MIDOLO fake master location to test (lamppost by JACARANDA/SEQUIOA)
	// need to get from XBee receive from master
	masterLatitude = 34.240780;
	masterLongitude = -118.528521;

    // Determine red's actual bearing (angles in radians)
	// Bearing start 0 degree due North +CW/-CCW angle (Ng is geographic or true north, not magnetic)
    bearingX = cos(redLatitudeB*M_PI/180) * sin((redLongitudeB-redLongitudeA)*M_PI/180);
    bearingY = cos(redLatitudeA*M_PI/180)*sin(redLatitudeB*M_PI/180) - sin(redLatitudeA*M_PI/180)*cos(redLatitudeB*M_PI/180)*cos((redLongitudeB-redLongitudeA)*M_PI/180);
    redBearing = atan2(bearingX,bearingY); // Swap bearing X,Y for normal cartesian coordinates (90 deg vertical +CCW/-CW angle)

    // Determine bearing to master
	// Bearing start 0 degree due North +CW/-CCW angle (Ng is geographic or true north, not magnetic)
    bearingX = cos(masterLatitude*M_PI/180) * sin((masterLongitude-redLongitudeB)*M_PI/180);
    bearingY = cos(redLatitudeB*M_PI/180)*sin(masterLatitude*M_PI/180) - sin(redLatitudeB*M_PI/180)*cos(masterLatitude*M_PI/180)*cos((masterLongitude-redLongitudeB)*M_PI/180);
    masterBearing = atan2(bearingX,bearingY); // Swap bearing X,Y for normal cartesian coordinates (90 deg vertical +CCW/-CW angle)

    homeBearing = (masterBearing-redBearing)*180/M_PI;

	// Bearing serial output for debugging
    printf("Current Bearing: %f degrees\n",redBearing*180/M_PI);
    printf("Master Bearing from N: %f degrees\n",masterBearing*180/M_PI);
    printf("Angle left to turn: %f degrees\n",homeBearing);

	// Determine distance from red to master
    a = sin((masterLatitude-redLatitudeB)*M_PI/180/2)*sin((masterLatitude-redLatitudeB)*M_PI/180/2) + cos(redLatitudeB*M_PI/180)*cos(masterLatitude*M_PI/180)*sin((masterLongitude-redLongitudeB)*M_PI/180/2)*sin((masterLongitude-redLongitudeB)*M_PI/180/2);
    c = 2*atan2(sqrt(a),sqrt(1-a));
    distToMaster = radiusEarth*c;
    printf("Total Distance to master: %fm\n",distToMaster);

	//MIDOLO do we need the actual destination coordinates or just keep checking distance
    //obstacle avoidance
    //use encoder and distance checking? or time? or redo routine after certain distances?
    // destLatitude = asin(sin(redLatitudeB*M_PI/180)*cos((d)/radiusEarth*M_PI/180)+cos(redLatitudeB*M_PI/180)*sin((d)/radiusEarth*M_PI/180)*cos((M_PI+masterBearing)));
    // destLongitude = redLongitudeB/180*M_PI + atan2(cos((d-distBuffer)/radiusEarth*M_PI/180)-sin(redLatitudeB)*sin(masterLatitude), sin((M_PI+masterBearing))*sin((d-distBuffer)/radiusEarth*M_PI/180)*cos(redLatitudeB));
    // printf("Stopping location: %f,%f\n",destLatitude*180/M_PI,destLongitude*180/M_PI - 90);
}
