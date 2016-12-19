//#define SYNC_PULSE_DEBUG

#include "PulsePosition.h"

// Pulse length definitions
// Formatted as:
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

byte sensor1Pin = 5;

volatile unsigned long sensor1Start;
volatile unsigned long sensor1Length;

volatile unsigned long syncPulseStart;

bool sawSyncPulse;
bool isXSweep;

PulsePosition pulsePosition;
LighthouseSensor sensor(sensor1Pin, 0);

void setup() {

  // Enable clock cycle counter
  ARM_DEMCR |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;

  pinMode(sensor1Pin, INPUT);
  attachInterrupt(sensor1Pin, interruptHandler, CHANGE);

  Serial.begin(115200);

  delay(2000);

  Serial.println("starting input capture");

#ifdef SYNC_PULSE_DEBUG
  Serial.println("Printing sync pulse debug info");
#endif

}

void loop() {

  
}

void interruptHandler() {
  
    pulsePosition.parsePulse(sensor);
    
}


