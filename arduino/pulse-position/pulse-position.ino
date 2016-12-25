//#define SYNC_PULSE_DEBUG

#include "PulsePosition.h"

// Teensy 3.2 can run interrupts on pins 0-23 (exposed) and 24-33 (underside)
#define IC_0 0
#define IC_1 1
#define IC_2 2
#define IC_3 3
#define IC_4 4
#define IC_5 5
#define IC_6 6
#define IC_7 7
#define IC_8 8
#define IC_9 9

PulsePosition pulsePosition;

LighthouseSensor ic0(IC_0);
LighthouseSensor ic1(IC_1);
LighthouseSensor ic2(IC_2);
LighthouseSensor ic3(IC_3);
LighthouseSensor ic4(IC_4);
LighthouseSensor ic5(IC_5);
LighthouseSensor ic6(IC_6);
LighthouseSensor ic7(IC_7);
LighthouseSensor ic8(IC_8);
LighthouseSensor ic9(IC_9);

IntervalTimer cycleTimer;

static byte sensorCount = 10;
static byte syncPulseSensor = 0;

void setup() {

  // Enable clock cycle counter
  ARM_DEMCR |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;

  pulsePosition.begin(sensorCount, syncPulseSensor);

  attachInterrupt(syncPulseSensor, ic0ISR, CHANGE);
  //  attachInterrupt(IC_1, ic1ISR, CHANGE);
  //  attachInterrupt(IC_2, ic2ISR, CHANGE);
  //  attachInterrupt(IC_3, ic3ISR, CHANGE);
  //  attachInterrupt(IC_4, ic4ISR, CHANGE);
  //  attachInterrupt(IC_5, ic5ISR, CHANGE);
  //  attachInterrupt(IC_6, ic6ISR, CHANGE);
  //  attachInterrupt(IC_7, ic7ISR, CHANGE);
  //  attachInterrupt(IC_8, ic8ISR, CHANGE);
  //  attachInterrupt(IC_9, ic9ISR, CHANGE);

  Serial.begin(115200);

  delay(2000);

  Serial.println("starting input capture");

#ifdef SYNC_PULSE_DEBUG
  Serial.println("Printing sync pulse debug info");
#endif

}

void loop() {


}

void ic0ISR() {

  Pulse pulse = pulsePosition.parsePulse(pulsePosition.sensors[syncPulseSensor]);

  if (pulse.pulseType == Pulse::PulseType::SYNC_PULSE) {

    //    Serial.println("Sync pulse!!!");
    pulsePosition.writeData();

    // We captured a sync pulse, attatch interrupts for all sensors
    attachInterrupts();

    // Set a timer to detach interrupts just before the end of a sweep
    // This ensures only the syncPulseSensor captures the sync pulse
    //    Serial.println("Setting timer")
    //    cycleTimer.begin(detachInterrupts, 8333);

  }

}

void ic1ISR() {

  pulsePosition.writePulseTime(pulsePosition.sensors[1]);

}

void ic2ISR() {

  pulsePosition.writePulseTime(pulsePosition.sensors[2]);

}

void ic3ISR() {

  pulsePosition.writePulseTime(pulsePosition.sensors[3]);

}
void ic4ISR() {

  pulsePosition.writePulseTime(pulsePosition.sensors[4]);

}

void ic5ISR() {

  pulsePosition.writePulseTime(pulsePosition.sensors[5]);

}

void ic6ISR() {

  pulsePosition.writePulseTime(pulsePosition.sensors[6]);

}

void ic7ISR() {

  pulsePosition.writePulseTime(pulsePosition.sensors[7]);

}
void ic8ISR() {

  pulsePosition.writePulseTime(pulsePosition.sensors[8]);

}

void ic9ISR() {

  pulsePosition.writePulseTime(pulsePosition.sensors[9]);

}

void attachInterrupts() {

  //  Serial.println("Attaching interrupts");

  attachInterrupt(1, ic1ISR, RISING);
  attachInterrupt(2, ic2ISR, RISING);
  attachInterrupt(3, ic3ISR, RISING);
  attachInterrupt(4, ic4ISR, RISING);
  attachInterrupt(5, ic5ISR, RISING);
  attachInterrupt(6, ic6ISR, RISING);
  attachInterrupt(7, ic7ISR, RISING);
  attachInterrupt(8, ic8ISR, RISING);
  attachInterrupt(9, ic9ISR, RISING);

}

void detachInterrupts() {
  //  pulsePosition.writeData();
  Serial.println("Detaching interrupts");
  cycleTimer.end();
  pulsePosition.sawSyncPulse = false;

  detachInterrupt(1);
  detachInterrupt(2);
  detachInterrupt(3);
  detachInterrupt(4);
  detachInterrupt(5);
  detachInterrupt(6);
  detachInterrupt(7);
  detachInterrupt(8);
  detachInterrupt(9);

}

