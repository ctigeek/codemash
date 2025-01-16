
// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  for (int i=0; i<4; i++) {
    BlinkShort();
  }
  BlinkLong();
}

void BlinkShort() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(200);
  digitalWrite(LED_BUILTIN, LOW);
  delay(200);
}

void BlinkLong() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(600);
  digitalWrite(LED_BUILTIN, LOW);
  delay(200);
}
