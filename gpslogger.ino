// Example from https://www.sparkfun.com/tutorials/176
void setup() {
  Serial.begin(57600);
}
void loop() {
  if (Serial.available()) {
    Serial.write(Serial.read());
  }
}
