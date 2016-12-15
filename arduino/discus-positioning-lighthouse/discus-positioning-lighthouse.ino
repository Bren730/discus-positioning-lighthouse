byte sensor1Pin = 5;

volatile unsigned long sensor1Start;
volatile unsigned long sensor1Length;

volatile unsigned long pulseStart;

// Pulse length definitions
#define BASE1_ROTOR0_DATA0 63
#define BASE1_ROTOR0_DATA1 83
#define BASE1_ROTOR1_DATA0 73
#define BASE1_ROTOR1_DATA1 94
#define BASE2_ROTOR0_DATA0 104
#define BASE2_ROTOR0_DATA1 125
#define BASE2_ROTOR1_DATA0 115
#define BASE2_ROTOR1_DATA1 135

bool sawSyncPulse;

void setup() {
  // put your setup code here, to run once:

  pinMode(sensor1Pin, INPUT);
  attachInterrupt(sensor1Pin, interruptHandler, CHANGE);

  Serial.begin(115200);

  delay(5000);

  Serial.println("starting input capture");

}

void loop() {
  // put your main code here, to run repeatedly:

}

void interruptHandler() {

  // This is the rising edge of the pulse
  // If it is a sync pulse, the 
  if(digitalReadFast(sensor1Pin) == HIGH) {

    sensor1Start = micros();
    
  } else {

    sensor1Length = micros() - sensor1Start;

    if(sensor1Length > BASE1_ROTOR0_DATA0 - 5 && sensor1Length < BASE1_ROTOR0_DATA1) {

      // This is a pulse

      pulseStart = sensor1Start;

      sawSyncPulse = true;
      
//      Serial.println("Base 1, rotor 0, data 0");
      
    } else if (sensor1Length < BASE1_ROTOR0_DATA0 && sawSyncPulse) {

      // This is a sweep
      // A pulse-sweep length is at max 8333 µs
      // Thus, the angle is the delta t between this pulse and the sync pulse
      // divided by 8333 * 180 degrees
      // TODO: Check if * 180 is correct

      unsigned long diff = micros() - pulseStart;
      double angle = (double)diff / 180.0;
      Serial.println("Angle: " + String(angle));

      sawSyncPulse = false;
      
    }
    
  }
  
}

