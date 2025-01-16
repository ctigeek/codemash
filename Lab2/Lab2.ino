
const char CLOCK=41;


bool clockState = false;  //false==clock low    true==clock high.


// use internal clock, (a potentiometer connected to analog input 7, and a N-O button connected to digital input 7)
const char analogClockSpeedPin = A7;  // center pin of potentiometer
const char stepClockButtonPin = 7;   // button that steps the clock

int analogClockSpeed = 0;
unsigned long clockDelayInMs = 0;
unsigned long clockFlopStart = 0;
unsigned int superFastCounter = 0;
const unsigned int SuperFastCounterResetValue = 0x0FFF;


void setup() {

  //initialize internal clock
  pinMode(CLOCK, OUTPUT);
  digitalWrite(CLOCK, LOW);
  pinMode(stepClockButtonPin, INPUT);
  clockFlopStart = millis();

  Serial.begin(57600);
  Serial.println("Clock started.");

}

void loop() {
  checkInternalClock();
}


void DumpData() {
  Serial.print(" potentiometer input:  ");
  Serial.print(analogClockSpeed);
  Serial.print("   CLOCK:");
  Serial.println(clockState);
}

void checkInternalClock() {

  if (superFastCounter > 0) {
    superFastCounter--;
    //delayMicroseconds(50);
    flopTheClock();
    if (superFastCounter == 0) {
      int newAnalogClockSpeed = analogRead(analogClockSpeedPin);
      if (newAnalogClockSpeed < 15) {
        superFastCounter = SuperFastCounterResetValue;
        Serial.println("Continuing super fast mode....");
      }
    }
    return;
  }


  unsigned long timediff = (millis() - clockFlopStart);
  int newAnalogClockSpeed = analogRead(analogClockSpeedPin);
  if (newAnalogClockSpeed > 1000) {  //clock is off... 
    if (analogClockSpeed < 1000) {
      Serial.println("Clock is stopped.");
    }
    char hilow = digitalRead(stepClockButtonPin);
    analogClockSpeed = newAnalogClockSpeed;
    if (timediff > 50 && hilow != clockState) {  // we are making sure 100ms has passed because we have to de-bounce the input.
      flopTheClock();
      DumpData();
    }
  }
  else {
    if (abs(newAnalogClockSpeed - analogClockSpeed) > 5) { //it has changed!
      analogClockSpeed = newAnalogClockSpeed;
      if (analogClockSpeed < 15) { // MAX!
        clockDelayInMs = 0;
        superFastCounter = SuperFastCounterResetValue;  // 4095
        Serial.println("");
        Serial.println("entering super fast mode....");
      }
      else if (analogClockSpeed < 200) {
        clockDelayInMs = 0;
      }
      else if (analogClockSpeed < 400) {
        clockDelayInMs = analogClockSpeed >> 3;
      }
      else if (analogClockSpeed < 600) {
        clockDelayInMs = analogClockSpeed >> 2;
      }
      else if (analogClockSpeed < 800) {
        clockDelayInMs = analogClockSpeed >> 1;
      }
      else {
        clockDelayInMs = analogClockSpeed;
      }
    }
    if (timediff > clockDelayInMs) {
      char hilow = digitalRead(stepClockButtonPin);
      if (!hilow) {
        flopTheClock();
        DumpData();
      }
    }
  }
}
void flopTheClock() {
  clockState = !clockState;
  digitalWrite(CLOCK, clockState);
  clockFlopStart = millis();
}
