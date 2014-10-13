// Example from https://www.sparkfun.com/tutorials/176
void setup() {
  Serial.begin(57600);
  Serial.println("$PMTK104*37"); // FULL COLD RESTART
}
void loop() {
  if (Serial.available()) {
    Serial.write(Serial.read());
  }
}
