//define the pins and variables
#define RCPin 2
#define LPin 3
#define RelayPin1 12
#define RelayPin2 11
#define RelayPin3 8

volatile long StartTime = 0;
volatile long CurrentTime = 0;
volatile long Pulses = 0;
volatile long Lumina = 0;
int PulseWidth = 0;
int LuminaWidth = 0;

#define numRelays 3
int relays[numRelays] = { RelayPin1, RelayPin2, RelayPin3 };
void setup() {

  //set up the serial monitor, pin mode, and external interrupt.
  Serial.begin(9600);
  pinMode(RCPin, INPUT_PULLUP);
  pinMode(LPin, INPUT_PULLUP);
  pinMode(RelayPin1, OUTPUT);
  pinMode(RelayPin3, OUTPUT);
  pinMode(RelayPin2, OUTPUT);
  digitalWrite(RelayPin1, HIGH);
  digitalWrite(RelayPin2, HIGH);
  digitalWrite(RelayPin3, HIGH);

  attachInterrupt(digitalPinToInterrupt(RCPin), PulseTimer, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LPin), LightTimer, CHANGE);
}

void loop() {
  //only save pulse lengths that are less than 2000 microseconds
  if (Pulses < 2000) {
    PulseWidth = Pulses;
  }
  if (Lumina < 2000) {
    LuminaWidth = Lumina;
  }
  if (900 < PulseWidth && PulseWidth < 1100) {
    Serial.print("STOP");
    digitalWrite(RelayPin1, LOW);
    digitalWrite(RelayPin2, LOW);
  } else if (1100 < PulseWidth && PulseWidth < 1600) {
    Serial.print("R1 and R2");
    digitalWrite(RelayPin1, HIGH);
    digitalWrite(RelayPin2, LOW);
  } else if (1800 < PulseWidth && PulseWidth < 2000) {
    Serial.print("R1 and R2");
    digitalWrite(RelayPin1, HIGH);
    digitalWrite(RelayPin2, HIGH);
  }
  ///light
  if (900 < LuminaWidth && LuminaWidth < 1100) {
    Serial.print("Lights ON");
    digitalWrite(RelayPin3, LOW);

  } else if (1800 < LuminaWidth && LuminaWidth < 2000) {
    Serial.print("Lights OFF");
    digitalWrite(RelayPin3, HIGH);
  }

  Serial.println(LuminaWidth);
}


void PulseTimer() {
  //measure the time between interrupts
  CurrentTime = micros();
  if (CurrentTime > StartTime) {
    Pulses = CurrentTime - StartTime;
    StartTime = CurrentTime;
  }
}

void LightTimer() {
  //measure the time between interrupts
  CurrentTime = micros();
  if (CurrentTime > StartTime) {
    Lumina = CurrentTime - StartTime;
    StartTime = CurrentTime;
  }
}