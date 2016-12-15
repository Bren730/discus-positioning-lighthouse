//#define SYNC_PULSE_DEBUG

byte sensor1Pin = 5;

volatile unsigned long sensor1Start;
volatile unsigned long sensor1Length;

volatile unsigned long syncPulseStart;

// Pulse length definitions
// Defined as:
// Skip 0/1, if rotor will skip laser
// Rotor 0/1, signifies which rotor is performing a sweep
// Data 0/1, data bit of the OOTX frame
#define S0R0D0 63
#define S0R0D1 83
#define S0R1D0 73
#define S0R1D1 94
#define S1R0D0 104
#define S1R0D1 125
#define S1R1D0 115
#define S1R1D1 135

bool sawSyncPulse;
bool isXSweep;

void setup() {
  // put your setup code here, to run once:

  pinMode(sensor1Pin, INPUT);
  attachInterrupt(sensor1Pin, interruptHandler, CHANGE);

  Serial.begin(115200);

  delay(5000);

  Serial.println("starting input capture");

#ifdef SYNC_PULSE_DEBUG
  Serial.println("Printing sync pulse debug info");
#endif

}

void loop() {
  // put your main code here, to run repeatedly:

}

void interruptHandler() {

  // Disable interrupts
  cli();

  if (digitalReadFast(sensor1Pin) == HIGH) {

    // This is the rising edge of the pulse
    // Record the starttime
    sensor1Start = micros();

  } else {

    // This is the falling edge of the pulse
    // Record the pulse length
    sensor1Length = micros() - sensor1Start;

    if (sensor1Length > S0R0D0 - 5 && sensor1Length < S0R0D0 + 5) {

#ifdef SYNC_PULSE_DEBUG
      Serial.println("Skip 0, rotor 0, data 0");
#endif

      syncPulseStart = micros();

      sawSyncPulse = true;
      isXSweep = true;

    }

    if (sensor1Length > S0R0D1 - 5 && sensor1Length < S0R0D1 + 5) {

#ifdef SYNC_PULSE_DEBUG
      Serial.println("Skip 0, rotor 0, data 1");
#endif

      syncPulseStart = micros();

      sawSyncPulse = true;
      isXSweep = true;

    }

    if (sensor1Length > S0R1D0 - 5 && sensor1Length < S0R1D0 + 5) {

#ifdef SYNC_PULSE_DEBUG
      Serial.println("Skip 0, rotor 1, data 0");
#endif

      syncPulseStart = micros();

      sawSyncPulse = true;
      isXSweep = false;

    }

    if (sensor1Length > S0R1D1 - 5 && sensor1Length < S0R1D1 + 5) {

#ifdef SYNC_PULSE_DEBUG
      Serial.println("Skip 0, rotor 1, data 1");
#endif

      syncPulseStart = micros();

      sawSyncPulse = true;
      isXSweep = false;

    }

    // Skip = 1 from here--------------------------

    if (sensor1Length > S1R0D0 - 5 && sensor1Length < S1R0D0 + 5) {

#ifdef SYNC_PULSE_DEBUG
      Serial.println("Skip 1, rotor 0, data 0");
#endif

    }

    if (sensor1Length > S1R0D1 - 5 && sensor1Length < S1R0D1 + 5) {

#ifdef SYNC_PULSE_DEBUG
      Serial.println("Skip 1, rotor 0, data 1");
#endif

    }

    if (sensor1Length > S1R1D0 - 5 && sensor1Length < S1R1D0 + 5) {

#ifdef SYNC_PULSE_DEBUG
      Serial.println("Skip 1, rotor 1, data 0");
#endif

    }

    if (sensor1Length > S1R1D1 - 5 && sensor1Length < S1R1D1 + 5) {

#ifdef SYNC_PULSE_DEBUG
      Serial.println("Skip 1, rotor 1, data 1");
#endif

    }


    if (sensor1Length < S0R0D0 && sawSyncPulse) {

      // This is a sweep
      // A pulse-sweep length is at max 8333 µs
      // Thus, the angle is the delta t between this pulse and the sync pulse
      // divided by 8333 * 180 degrees

      // TODO: Check if * 180 is correct

      String type;

      // Check the angle type
      (isXSweep)? type = "X ": type = "Y ";

      double diff = micros() - syncPulseStart;
      Serial.println(type + String(diff));

      sawSyncPulse = false;

    }

    if (micros() - syncPulseStart > 8333) {

      sawSyncPulse = false;
      syncPulseStart = 0;

    }

  }

  // Enable interrupts
  sei();

}

