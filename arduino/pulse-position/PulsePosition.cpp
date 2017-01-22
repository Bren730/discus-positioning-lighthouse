#include "PulsePosition.h"
#include <Arduino.h>

//#define SYNC_PULSE_DEBUG
//#define HUMAN_READABLE
//#define BLUETOOTH

void printBits(byte myByte) {
  for (byte mask = 0x80; mask; mask >>= 1) {
    if (mask  & myByte)
#ifdef BLUETOOTH
      Serial.print('1');
#endif
#ifndef BLUETOOTH
    Serial.print('1');
#endif
    else
#ifdef BLUETOOTH
      Serial.print('0');
#endif
#ifndef BLUETOOTH
    Serial.print('0');
#endif
  }
}

PulsePosition::PulsePosition() {

}

void PulsePosition::begin(byte _sensorCount, byte _syncPulseSensor) {

  sensorCount = _sensorCount;
  syncPulseSensor = _syncPulseSensor;

  // Pins 0 and 1 are used for Bluetooth Serial, so sensors start from pin 2
  for (byte i = 2; i < sensorCount + 2; i++) {

    LighthouseSensor sensor(i);
    sensors[i] = sensor;

    // Check if Serial was initialised
    // Otherwise the microcontroller might get stuck
    if (Serial) {
      Serial.println("Sensor " + String(i) + " initialised");
    }


  }

  //  Serial.begin(115200);
  Serial1.begin(115200);
  // Wait for Serial1 to start
  while (!Serial1);

  // Set Bluetooth to highest speed
  //  Serial1.print('$');
  //  Serial1.print('$');
  //  Serial1.print('$');
  //  while(Serial1.available()) {
  //
  //    Serial1.println(Serial1.read());
  //
  //  }
  //  Serial1.print("SU,92");
  //  delay(100);
  //  Serial1.begin(921600);

#ifdef BLUETOOTH
  Serial1.println("PulsePosition initialised");
#endif
#ifndef BLUETOOTH
  Serial.println("PulsePosition initialised");
#endif

}

Pulse PulsePosition::parsePulse(LighthouseSensor& sensor) {

  // Disable interrupts
  //  cli();

  if (digitalReadFast(sensor.pin) == HIGH) {
    // This is the rising edge of the pulse
    // Record start time of pulse as CPU Cycle count
    // Overflows every 44.73s
    sensor.pulseStart = ARM_DWT_CYCCNT;
    Pulse pulse(0, 0, 0, Pulse::PulseType::PULSE_START);

    return pulse;

  } else {
    // This is the falling edge of the pulse
    // Record the pulse length

    sensor.pulseLength = ARM_DWT_CYCCNT - sensor.pulseStart;
    Pulse pulse = parsePulseType(sensor.pulseLength);

    if (pulse.pulseType == Pulse::PulseType::SYNC_PULSE) {

      prevSyncPulse = syncPulse;
      syncPulse = pulse;

      prevSyncPulseStart = syncPulseStart;
      syncPulseStart = sensor.pulseStart;

      sawSyncPulse = true;

      syncPulseCounter++;

      if (resetSyncPulseTimer) {

        syncPulseTimer = syncPulseStart;
        resetSyncPulseTimer = false;

      }



    }

    if (pulse.pulseType == Pulse::PulseType::SWEEP) {

      // Prevent double sweep registering due to reflections
      if (!sensor.sawSweep) {

        resetSyncPulseTimer = true;

        // Check if the pulse falls within one sweep cycle time
        // And check if we saw a sync pulse before it
        if (ARM_DWT_CYCCNT - syncPulseStart < SWEEP_CYCLE_CLOCK_CYCLES - 10000 && sawSyncPulse) {

          // Assign the base station
          // If we saw more than 1 sync pulse, it's basestation b
          // todo: Improve base station detection
          (syncPulseCounter > 1) ? station = false : station = true;

          // Sweep registered, reset sync pulse counter
          syncPulseCounter = 0;

          sensor.deltaT = ARM_DWT_CYCCNT - syncPulseStart;
          // Serial.println(String(sensor.deltaT) + ", " + String(sensor.skip) + ", " + String(sensor.rotor) + ", " + String(sensor.data));

          sensor.sawSweep = true;

        }

      } else {

        // Sweep pulse registered, but no sync pulse came before it
        // Ignore data
        sawSyncPulse = false;

      }

    }

    return pulse;

  }



  // Enable interrupts again
  //  sei();

}

void PulsePosition::writePulseTime(LighthouseSensor& sensor) {

  //  cli();

  // Prevent double sweep registering due to reflections
  // Also only write sweep value if the time does not exceed the max cycle time
  if (!sensor.sawSweep && (ARM_DWT_CYCCNT - syncPulseStart) < SWEEP_CYCLE_CLOCK_CYCLES) {

    sensor.deltaT = ARM_DWT_CYCCNT - syncPulseStart;
    // Serial.println(String(sensor.deltaT) + ", " + String(sensor.skip) + ", " + String(sensor.rotor) + ", " + String(sensor.data));

    //#if !defined(HUMAN_READABLE) && !defined(SYNC_PULSE_DEBUG)
    //
    //    Serial.write(sensor.id);
    //    Serial.write((sensor.deltaT >> 24));
    //    Serial.write((sensor.deltaT >> 16));
    //    Serial.write((sensor.deltaT >> 8));
    //    Serial.write((sensor.deltaT & 0x00FF));
    //#endif
    //
    //#ifdef HUMAN_READABLE
    //    Serial.print(String(sensor.id) + ", ");
    //    Serial.print(String(sensor.deltaT) + ", ");
    //    Serial.println();
    //#endif

    sensor.sawSweep = true;

  }

  //      sei();

}

Pulse PulsePosition::parsePulseType(unsigned long pulseLength) {

  //  Serial.println(pulseLength);

  if (pulseLength >= S0R0D0 - PULSE_CHANNEL_WIDTH && pulseLength < S0R0D0 + PULSE_CHANNEL_WIDTH) {

#ifdef SYNC_PULSE_DEBUG
    Serial.println("Skip 0, rotor 0, data 0");
#endif

    Pulse pulse(0, 0, 0, Pulse::PulseType::SYNC_PULSE);
    return pulse;

  }

  if (pulseLength >= S0R0D1 - PULSE_CHANNEL_WIDTH && pulseLength < S0R0D1 + PULSE_CHANNEL_WIDTH) {

#ifdef SYNC_PULSE_DEBUG
    Serial.println("Skip 0, rotor 0, data 1");
#endif

    Pulse pulse(0, 0, 1, Pulse::PulseType::SYNC_PULSE);
    return pulse;

  }

  if (pulseLength >= S0R1D0 - PULSE_CHANNEL_WIDTH && pulseLength < S0R1D0 + PULSE_CHANNEL_WIDTH) {

#ifdef SYNC_PULSE_DEBUG
    Serial.println("Skip 0, rotor 1, data 0");
#endif

    Pulse pulse(0, 1, 0, Pulse::PulseType::SYNC_PULSE);
    return pulse;

  }

  if (pulseLength >= S0R1D1 - PULSE_CHANNEL_WIDTH && pulseLength < S0R1D1 + PULSE_CHANNEL_WIDTH) {

#ifdef SYNC_PULSE_DEBUG
    Serial.println("Skip 0, rotor 1, data 1");
#endif

    Pulse pulse(0, 1, 1, Pulse::PulseType::SYNC_PULSE);
    return pulse;

  }

  if (pulseLength >= S1R0D0 - PULSE_CHANNEL_WIDTH && pulseLength < S1R0D0 + PULSE_CHANNEL_WIDTH) {

#ifdef SYNC_PULSE_DEBUG
    Serial.println("Skip 1, rotor 0, data 0");
#endif

    Pulse pulse(1, 0, 0, Pulse::PulseType::SYNC_PULSE);
    return pulse;

  }

  if (pulseLength >= S1R0D1 - PULSE_CHANNEL_WIDTH && pulseLength < S1R0D1 + PULSE_CHANNEL_WIDTH) {

#ifdef SYNC_PULSE_DEBUG
    Serial.println("Skip 1, rotor 0, data 1");
#endif

    Pulse pulse(1, 0, 1, Pulse::PulseType::SYNC_PULSE);
    return pulse;

  }

  if (pulseLength >= S1R1D0 - PULSE_CHANNEL_WIDTH && pulseLength < S1R1D0 + PULSE_CHANNEL_WIDTH) {

#ifdef SYNC_PULSE_DEBUG
    Serial.println("Skip 1, rotor 1, data 0");
#endif

    Pulse pulse(1, 1, 0, Pulse::PulseType::SYNC_PULSE);
    return pulse;

  }

  if (pulseLength >= S1R1D1 - PULSE_CHANNEL_WIDTH && pulseLength < S1R1D1 + PULSE_CHANNEL_WIDTH) {

#ifdef SYNC_PULSE_DEBUG
    Serial.println("Skip 1, rotor 1, data 1");
#endif

    Pulse pulse(1, 1, 1, Pulse::PulseType::SYNC_PULSE);
    return pulse;

  }

  if (pulseLength < S0R0D0 - PULSE_CHANNEL_WIDTH) {

#ifdef SYNC_PULSE_DEBUG
    Serial.println("sweep");
#endif

    Pulse pulse(0, 0, 0, Pulse::PulseType::SWEEP);
    return pulse;

  }

  // Pulse does not fit any type, cast as outlier
#ifdef SYNC_PULSE_DEBUG
  //  Serial.println("outlier, " + String(pulseLength));
#endif

  Pulse pulse(0, 0, 0, Pulse::PulseType::OUTLIER);
  return pulse;

}

void PulsePosition::writeData() {

  cli();

  station = prevSyncPulse.station;
  skip = prevSyncPulse.skip;
  rotor = prevSyncPulse.rotor;
  data = prevSyncPulse.data;
  meta = 0;

  // Construct meta byte
  (station) ? bitSet(meta, 3) : false;
  (skip) ? bitSet(meta, 2) : false;
  (rotor) ? bitSet(meta, 1) : false;
  (data) ? bitSet(meta, 0) : false;

#if !defined(HUMAN_READABLE) && !defined(SYNC_PULSE_DEBUG)

#ifdef BLUETOOTH
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(meta);
#endif
#ifndef BLUETOOTH
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(meta);
#endif
#endif

#ifdef HUMAN_READABLE
#ifdef BLUETOOTH
  Serial1.println();
  Serial1.println("Sync pulse, meta:");
  printBits(meta);
  Serial1.println();
  Serial1.println();
  Serial1.println("Sensors:");
#endif
#ifndef BLUETOOTH
  Serial.println();
  Serial.println("Sync pulse, meta:");
  printBits(meta);
  Serial.println();
  Serial.println();
  Serial.println("Sensors:");
#endif
#endif

  for (byte i = 2; i < sensorCount + 2; i++) {

    if (sensors[i].sawSweep) {
#if !defined(HUMAN_READABLE) && !defined(SYNC_PULSE_DEBUG)
#ifdef BLUETOOTH
      Serial1.write(sensors[i].id);
      Serial1.write((sensors[i].deltaT >> 24));
      Serial1.write((sensors[i].deltaT >> 16));
      Serial1.write((sensors[i].deltaT >> 8));
      Serial1.write((sensors[i].deltaT & 0x00FF));
#endif
#ifndef BLUETOOTH
      Serial.write(sensors[i].id);
      Serial.write((sensors[i].deltaT >> 24));
      Serial.write((sensors[i].deltaT >> 16));
      Serial.write((sensors[i].deltaT >> 8));
      Serial.write((sensors[i].deltaT & 0x00FF));
#endif
#endif

#ifdef HUMAN_READABLE
#ifdef BLUETOOTH
      Serial1.print(String(sensors[i].id) + ", ");
      Serial1.print(String(sensors[i].deltaT) + ", ");
      Serial1.println();
#endif
#ifndef BLUETOOTH
      Serial.print(String(sensors[i].id) + ", ");
      Serial.print(String(sensors[i].deltaT) + ", ");
      Serial.println();
#endif
#endif
    }
  }

  for (byte i = 2; i < sensorCount + 2; i++) {
    // reset all sensors
    // sawSweep is used for reflection elimination
    sensors[i].sawSweep = false;

  }

  sei();

}

void PulsePosition::getOutputBuffer(uint8_t *buffer) {

  byte bufPos = 0;

  station = prevSyncPulse.station;
  skip = prevSyncPulse.skip;
  rotor = prevSyncPulse.rotor;
  data = prevSyncPulse.data;
  meta = 0;

  // Construct meta byte
  (station) ? bitSet(meta, 3) : false;
  (skip) ? bitSet(meta, 2) : false;
  (rotor) ? bitSet(meta, 1) : false;
  (data) ? bitSet(meta, 0) : false;

#if !defined(HUMAN_READABLE) && !defined(SYNC_PULSE_DEBUG)
  buffer[bufPos] = 0xff;
  bufPos++;
  buffer[bufPos] = 0xff;
  bufPos++;
  buffer[bufPos] = meta;
  bufPos++;
#endif

  for (byte i = 2; i < sensorCount + 2; i++) {

    if (sensors[i].sawSweep) {
#if !defined(HUMAN_READABLE) && !defined(SYNC_PULSE_DEBUG)
      buffer[bufPos] = sensors[i].id;
      bufPos++;
      buffer[bufPos] = (sensors[i].deltaT >> 24);
      bufPos++;
      buffer[bufPos] = (sensors[i].deltaT >> 16);
      bufPos++;
      buffer[bufPos] = (sensors[i].deltaT >> 8);
      bufPos++;
      buffer[bufPos] = (sensors[i].deltaT & 0x00FF);
      bufPos++;
#endif

    }
  }

  for (byte i = 2; i < sensorCount + 2; i++) {
    // reset all sensors
    // sawSweep is used for reflection elimination
    sensors[i].sawSweep = false;

  }

}

