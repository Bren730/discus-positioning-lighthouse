//#define SYNC_PULSE_DEBUG
//#define BLUETOOTH

#include "DataDiscus.h"
#include "PulsePosition.h"
#include "NeoPixel.h"
#include <SPI.h>
#include <Math.h>

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
DataDiscus dataDiscus;

NeoPixel ring = NeoPixel(24, 23);

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

const byte sensorCount = 10;
const byte syncPulseSensor = 2;
const byte sensorDataLen = 5;
const byte msgLen = 2 + 1 + (sensorDataLen * sensorCount);

// 2 Init bytes, 1 meta byte followed by X sensor data
uint8_t outBuffer[msgLen];

void setup() {
  Serial.begin(115200);

  // Wait for Serial to start
  while (!Serial);

  // Enable clock cycle counter
  ARM_DEMCR |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;


  Serial.println("starting PulsePosition");

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


  //  Serial3.begin(115200);


  //  Serial3.println("starting input capture");

  // Attatch interrupts for all sensors
  attachInterrupts();

#ifdef SYNC_PULSE_DEBUG
  Serial.println("Printing sync pulse debug info");
  delay(2000);
  //  Serial3.println("Printing sync pulse debug info");
#endif

}

void loop() {

  byte baseColor[] = {3, 10, 25};
  byte highlightColor[] = {5, 100, 255};
  float lengths[] = {.4, .2};
  float durations[] = {2500, 2000};


  if (dataDiscus.state == DataDiscus::State::STATE_DISCONNECTED) {
    ring.setWaiting(baseColor, highlightColor, lengths, durations, 500, false, true, true);
  }
  ring.showWaiting();

  String inString = "";

  while (Serial1.available()) {

    char c = Serial1.read();
    inString += c;
    // This delay is necessary, otherwise the message would get cut into multiple pieces
    delay(10);

  }

  if (inString != "") {
    Serial.println(inString);
    if (inString.indexOf("NEW_PAIRING") > 0 ) {

      dataDiscus.state = DataDiscus::State::STATE_PAIRING;
      ring.isWaiting = false;

      byte baseColor[] = {3, 10, 25};
      byte highlightColor[] = {5, 100, 255};
      float lengths[] = {.3, .3};
      float durations[] = {1000, 800};
      
      ring.setWaiting(baseColor, highlightColor, lengths, durations, 500, false, true, true);
      
      Serial.println("DataDiscus is pairing");
    }

    if (inString.indexOf("CONNECT") > 0 && inString.indexOf("DISCONNECT") < 0) {
      dataDiscus.state = DataDiscus::State::STATE_CONNECTED;
      ring.isWaiting = false;

      byte baseColor[] = {0, 25, 10};
      byte highlightColor[] = {5, 255, 100};
      float lengths[] = {.5, .5};
      float durations[] = {2500, -2500};
      
      ring.setWaiting(baseColor, highlightColor, lengths, durations, 500, false, true, true);
      
      Serial.println("DataDiscus connected");
    }

    if (inString.indexOf("DISCONNECT") > 0 ) {
      
      dataDiscus.state = DataDiscus::State::STATE_DISCONNECTED;
      ring.isWaiting = false;
      
      Serial.println("DataDiscus disconnected");
    }
  }

  //  ring.setPercentage(random(0, 100) / 100.0, highlightColor, 2000, 1000);
  //  ring.showPercentage();

}

void ic0ISR() {

  Pulse pulse = pulsePosition.parsePulse(pulsePosition.sensors[syncPulseSensor]);

  if (pulse.pulseType == Pulse::PulseType::SYNC_PULSE) {

    pulsePosition.writeData();

    // Enable interrupts for all other pins again
    cli();
    attachInterrupts();
    sei();

  }

}

void ic1ISR() {

  pulsePosition.writePulseTime(pulsePosition.sensors[1]);
  detachInterrupt(pulsePosition.sensors[1].pin);

}

void ic2ISR() {

  pulsePosition.writePulseTime(pulsePosition.sensors[2]);
  detachInterrupt(pulsePosition.sensors[2].pin);

}

void ic3ISR() {

  pulsePosition.writePulseTime(pulsePosition.sensors[3]);
  detachInterrupt(pulsePosition.sensors[3].pin);

}
void ic4ISR() {

  pulsePosition.writePulseTime(pulsePosition.sensors[4]);
  detachInterrupt(pulsePosition.sensors[4].pin);

}

void ic5ISR() {

  pulsePosition.writePulseTime(pulsePosition.sensors[5]);
  detachInterrupt(pulsePosition.sensors[5].pin);

}

void ic6ISR() {

  pulsePosition.writePulseTime(pulsePosition.sensors[6]);
  detachInterrupt(pulsePosition.sensors[6].pin);

}

void ic7ISR() {

  pulsePosition.writePulseTime(pulsePosition.sensors[7]);
  detachInterrupt(pulsePosition.sensors[7].pin);

}
void ic8ISR() {

  pulsePosition.writePulseTime(pulsePosition.sensors[8]);
  detachInterrupt(pulsePosition.sensors[8].pin);

}

void ic9ISR() {

  pulsePosition.writePulseTime(pulsePosition.sensors[9]);
  detachInterrupt(pulsePosition.sensors[9].pin);

}

void ic10ISR() {

  pulsePosition.writePulseTime(pulsePosition.sensors[10]);
  detachInterrupt(pulsePosition.sensors[10].pin);

}

void ic11ISR() {

  pulsePosition.writePulseTime(pulsePosition.sensors[11]);
  detachInterrupt(pulsePosition.sensors[11].pin);

}

void attachInterrupts() {

  //  Serial.println("Attaching interrupts");

  //  attachInterrupt(1, ic1ISR, RISING);
  //  attachInterrupt(2, ic2ISR, RISING);
  attachInterrupt(3, ic3ISR, RISING);
  attachInterrupt(4, ic4ISR, RISING);
  attachInterrupt(5, ic5ISR, RISING);
  attachInterrupt(6, ic6ISR, RISING);
  attachInterrupt(7, ic7ISR, RISING);
  attachInterrupt(8, ic8ISR, RISING);
  attachInterrupt(9, ic9ISR, RISING);
  attachInterrupt(10, ic10ISR, RISING);
  attachInterrupt(11, ic11ISR, RISING);
  //  attachInterrupt(12, ic9ISR, RISING);

}

void detachInterrupts() {
  //  pulsePosition.writeData();
  Serial.println("Detaching interrupts");
  pulsePosition.sawSyncPulse = false;

  for (byte i = 0; i < sensorCount; i++) {

    detachInterrupt(i);

  }

}

