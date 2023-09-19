const int rc_pin = 7;
const int Extend_pin = 10;
const int Retract_pin = 9;
const int Position_pin = A5;

int RC_duration;
int RC_location;
int actual_location;

const int deadband = 10;
const int lower_limit = 160;
const int upper_limit = 250;
const int default_position = 220;

void setup() {
    Serial.begin(9600);
    pinMode(rc_pin, INPUT);
    pinMode(Extend_pin, OUTPUT);
    pinMode(Retract_pin, OUTPUT);
    digitalWrite(Extend_pin, LOW);
    digitalWrite(Retract_pin, LOW);
}

void loop() {
    RC_duration = constrain(pulseIn(rc_pin, HIGH), 1000, 2000);
    RC_location = map(RC_duration, 980, 2000, lower_limit, upper_limit);

    actual_location = analogRead(Position_pin);
    
    Serial.print("Actual: ");
    Serial.print(actual_location); 
    Serial.print("  Target: ");
    Serial.print(RC_location); 
    Serial.print('\n');

    if (actual_location < lower_limit) {
        // If below range, move to default position
        if (actual_location < default_position - deadband) {
            extend();
        } else {
            stopMotor();
        }
    } else if (actual_location > upper_limit) {
        // If above range, move to default position
        if (actual_location > default_position + deadband) {
            retract();
        } else {
            stopMotor();
        }
    } else {
        // If within range, operate normally
        if ((actual_location <= RC_location + deadband) && (actual_location >= RC_location - deadband)) {
            stopMotor();
        } else if (actual_location > RC_location + deadband) {
            retract();
        } else if (actual_location < RC_location - deadband) {
            extend();
        }
    }
    
    delay(50);
}

void extend() {
    digitalWrite(Extend_pin, HIGH);
    digitalWrite(Retract_pin, LOW);
    Serial.println("EXTEND");
}

void retract() {
    digitalWrite(Extend_pin, LOW);
    digitalWrite(Retract_pin, HIGH);
    Serial.println("RETRACT");
}

void stopMotor() {
    digitalWrite(Extend_pin, LOW);
    digitalWrite(Retract_pin, LOW);
    Serial.println("STOP");
}
