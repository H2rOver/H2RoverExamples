 void setup() {
 Serial.begin(9600);
}

void loop() {
  int sensorValue = analogRead(A6);
  Serial.println(sensorValue);
  delay(1000);
}

