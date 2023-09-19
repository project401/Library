/*
Reading Values from an RC Receiver using Arduino
Brandon Tsuge

In this example, I demonstrate how to use Arduino to read RC (50Hz PWM) values using pulseIn() and external interrupts.
https://www.theboredrobot.com/post/reading-values-from-an-rc-receiver-using-arduino
 */

//define the pins and variables
#define RCPin 2
#define RelayPin1 12  
#define RelayPin2 11
volatile long StartTime = 0;
volatile long CurrentTime = 0;
volatile long Pulses = 0;
int PulseWidth = 0;

#define numRelays 2
int relays[numRelays]={RelayPin1, RelayPin2};
void setup(){

  //set up the serial monitor, pin mode, and external interrupt.
  Serial.begin(9600);
  pinMode(RCPin, INPUT_PULLUP);
  pinMode(RelayPin1, OUTPUT);
  pinMode(RelayPin2, OUTPUT);  
  digitalWrite(RelayPin1,HIGH);
  digitalWrite(RelayPin2,HIGH);
  attachInterrupt(digitalPinToInterrupt(RCPin),PulseTimer,CHANGE);
}

void loop() {
  //only save pulse lengths that are less than 2000 microseconds
  if (Pulses < 2000){
    PulseWidth = Pulses;
  }
  if(900<PulseWidth && PulseWidth<1100){
  Serial.print("STOP");
  digitalWrite(RelayPin1,LOW);
  digitalWrite(RelayPin2,LOW);
 }  
  else if(1100<PulseWidth && PulseWidth<1600){
    Serial.print("R1 and R2");
    digitalWrite(RelayPin1,HIGH);
    digitalWrite(RelayPin2,LOW);
  }
  else if(1800<PulseWidth && PulseWidth<2000){
    Serial.print("R1 and R2");
    digitalWrite(RelayPin1,HIGH);
    digitalWrite(RelayPin2,HIGH);
  }
  Serial.println(PulseWidth);
}


void PulseTimer(){
  //measure the time between interrupts
  CurrentTime = micros();
  if (CurrentTime > StartTime){
    Pulses = CurrentTime - StartTime;
    StartTime = CurrentTime;
  }
}
