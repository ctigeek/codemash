// BE Bus Enable. Set high for normal operation.
const char BE = 40; // OUTPUT ONLY
// PHI2 the clock.
const char CLOCK=41;
//RWB Read-Write bit. When high, the CPU is reading data. 
const char RWB = 42;   // INPUT ONLY
//RESB When low for 2+ cycles, the CPU is reset and PC is set to FFFC-D.
const char RESET = 43;  // OUTPUT ONLY
//VPB vector pull. When low, the CPU is reading the interrupt vector .
const char VPB = 44;  // INPUT ONLY
//RDY READY. When low, the CPU will stop. Keep high for normal operation.
const char RDY = 45;  // OUTPUT
//IRQB Interrupt Request. Low will trigger an interrupt.
const char IRQB = 46; // OUTPUT ONLY
//NMIB Non Maskable Interrupt. Low will trigger an interrupt.
const char NMIB = 47; // OUTPUT ONLY
// Synchronize OpCode fetch. Goes high when CPU is fetching an op code. 
const char SYNC = 48; // INPUT ONLY


// special addresses
const uint16_t RESET_ADDRESS_LB = 0xFFFC;
const uint16_t RESET_ADDRESS_HB = 0xFFFD;
const uint8_t RESET_VECTOR_LB = 0x00;
const uint8_t RESET_VECTOR_HB = 0x80;


//global vars to track state...
bool clockState = false;  //false==clock low    true==clock high.
uint8_t RWB_status = LOW;
uint8_t VPB_status = HIGH;
uint8_t SYNC_status = HIGH;
uint16_t address = 0x00;
uint8_t data = 0x00;


// use internal clock, (a potentiometer connected to analog input 7, and a N-O button connected to digital input 7)
const char analogClockSpeedPin = A7;  // center pin of potentiometer
const char stepClockButtonPin = 7;   // button that steps the clock
int analogClockSpeed = 0;
unsigned long clockDelayInMs = 0;
unsigned long clockFlopStart = 0;
unsigned int superFastCounter = 0;
const unsigned int SuperFastCounterResetValue = 0xFFF;

const uint8_t Op_Code_NOP = 0xEA;
const uint8_t Op_Code_JMP = 0x4C;

const uint16_t ROM_SIZE = 0x1800; //6KB
byte ROM[ROM_SIZE];

void setup() {
  SetAddressPinsToInput();
  SetDataPinsToInput();

  //set Bus Enable to high. If set to low, address and data go HI-Z
  pinMode(BE, OUTPUT);
  digitalWrite(BE, HIGH);

  //when Read-Write is high, the processor is reading data. When low, writing data.
  pinMode(RWB, INPUT);

  pinMode(RESET, OUTPUT);
  digitalWrite(RESET, HIGH);

  // interrupt Vector Pull
  pinMode(VPB, INPUT);
  
  // Ready
  pinMode(RDY, OUTPUT);
  digitalWrite(RDY, HIGH);

  // interrupt request
  pinMode(IRQB, OUTPUT);
  digitalWrite(IRQB, HIGH);
  // non-maskable interrupt request
  pinMode(NMIB, OUTPUT);
  digitalWrite(NMIB, HIGH);
  // op code fetch Sync
  pinMode(SYNC, INPUT);


  //initialize internal clock
  pinMode(CLOCK, OUTPUT);
  digitalWrite(CLOCK, LOW);
  pinMode(stepClockButtonPin, INPUT);
  clockFlopStart = millis();

  Serial.begin(57600);
  Serial.println("Emulator started.");

  FillRom(ROM);

  ResetCPU();
}

void loop() {
  
  if (clockState) {
    WaitForClockLow();
  }
  else {
    WaitForClockHigh();
  }

  if (address >= 0x8000) {
    uint16_t romaddress = address & 0x7FFF;  //subtract 32k from address so we can use the array index 
    WriteDataLines(ROM[romaddress]);
  }
  else {
    ReadDataLines();
  }

  if (clockState && superFastCounter == 0) {
    DumpData();
  }
}


void ResetCPU() {
  Serial.println("Resetting CPU.... make sure clock is running...");
  digitalWrite(RESET, LOW);

  WaitForClockLow();
  WaitForClockHigh();
  WaitForClockLow();
  WaitForClockHigh();
  Serial.println("Resetting: 2 more clock cycles");
  WaitForClockLow();
  WaitForClockHigh();
  Serial.println("Resetting: 1 more clock cycle");
  WaitForClockLow();
  WaitForClockHigh();

  digitalWrite(RESET, HIGH);

  Serial.println(" Keep the clock running....");

  for (int i=25; i>=0; i--)
  {
    if (clockState) {
      WaitForClockLow();
    }
    else {
      WaitForClockHigh();
    }
    
    if (VPB_status == LOW) {
      if (address == RESET_ADDRESS_LB) {
        if (clockState) { Serial.println("Reset address LB"); }
        WriteDataLines(RESET_VECTOR_LB);
      }
      else if (address == RESET_ADDRESS_HB) {
        if (clockState) { Serial.println("Reset address HB"); }
        WriteDataLines(RESET_VECTOR_HB);
        i=1;
      }
      else {
        Serial.println("This should never happen....");
      }
    }
    if (clockState) {
      DumpData();
    }
  }
  
  WaitForClockLow();
  SetDataPinsToInput();
  Serial.println("Reset complete.");
}


void DumpData() {
  //Serial.print(" analogClockSpeed:  ");
  //Serial.print(analogClockSpeed);
  //Serial.print("    ");
  //Serial.print(superFastCounter);
  Serial.print("   CLOCK:");
  Serial.print(clockState);
  Serial.print("   RWB=");
  Serial.print(RWB_status == HIGH ? "READ " : "WRITE");
  Serial.print("  VPB=");
  Serial.print(VPB_status == HIGH ? "_________" : "INTERRUPT");
  Serial.print("  SYNC=");
  Serial.print(SYNC_status == HIGH ? "OP FETCH   " : "_______    ");
  Serial.print(" ADDRESS=");
  Serial.print(address,HEX);
  Serial.print("   data=");
  Serial.println(data,HEX);
}

void checkInternalClock() {

  if (superFastCounter > 0) {
    superFastCounter--;
    delayMicroseconds(50);
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
      }
    }
  }
}

void flopTheClock() {
  clockState = !clockState;
  digitalWrite(CLOCK, clockState);
  clockFlopStart = millis();
}

void WaitForClockLow() {
  while (clockState) {
    checkInternalClock();
  }
  RWB_status = digitalRead(RWB);
  VPB_status = digitalRead(VPB);
  SYNC_status = digitalRead(SYNC);
  address = ReadAddressLines();
  // do not read data here!
}

void WaitForClockHigh() {
  while (!clockState) {
    checkInternalClock();
  }
  RWB_status = digitalRead(RWB);
  VPB_status = digitalRead(VPB);
  SYNC_status = digitalRead(SYNC);
  address = ReadAddressLines();
  // do not read data here!
}

void WriteDataLines(uint8_t somedata) {
  DDRA = B11111111;  // set pins to output
  PORTA = somedata;
  data = somedata;
}

uint8_t ReadDataLines() {
  DDRA = B00000000;  //all input
  uint8_t byteread = PINA;
  data = byteread;
  return byteread;
}

uint16_t ReadAddressLines() {
  uint16_t addressread = PINB | (PINC << 8);
  return addressread;
}

void SetDataPinsToInput() {
  DDRA = B00000000;
}

void SetAddressPinsToInput() {
  DDRB = B00000000; 
  DDRC = B00000000;
}