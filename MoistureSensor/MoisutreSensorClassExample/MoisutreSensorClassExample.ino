#include <GPS.h>
#include <H2RoverXbee.h>
#include <IMU.h>
#include <MoistureSensor.h>
#include <MotorControl.h>
#include <PinDeclarations.h>
#include <Ultrasound.h>

MoistureSensor moisture(0); //ID of sensor is 0

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  sensor_t sensor;
  moisture.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
}

void loop() {
  // put your main code here, to run repeatedly:
  //Number in the parameters is the number of samples to be taken
  //There is a 100 millisecond delay between tests so do not put a huge number in here
  Serial.println(moisture.getData(10));
}
