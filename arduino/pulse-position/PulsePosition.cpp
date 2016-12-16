#include "PulsePosition.h"
#include <Arduino.h>

// #define SYNC_PULSE_DEBUG
//#define HUMAN_READABLE

PulsePosition::PulsePosition() {

}

void PulsePosition::begin() {



}

void PulsePosition::parsePulse(LighthouseSensor& sensor) {

  // Disable interrupts
  cli();

  if (digitalReadFast(sensor.pin) == HIGH) {
    // This is the rising edge of the pulse
    // Record start time of pulse
    sensor.pulseStart = micros();

  } else {
    // This is the falling edge of the pulse
    // Record the pulse length

    sensor.pulseLength = micros() - sensor.pulseStart;
    Pulse pulse = parsePulseType(sensor.pulseLength);

    if (pulse.pulseType == Pulse::PulseType::SYNC_PULSE) {

      sensor.syncPulseStart = sensor.pulseStart;
      sensor.sawSyncPulse = true;

      sensor.station = pulse.station;
      sensor.skip = pulse.skip;
      sensor.rotor = pulse.rotor;
      sensor.data = pulse.data;

      if (resetSyncPulseTimer) {

        syncPulseTimer = sensor.syncPulseStart;
        resetSyncPulseTimer = false;

      }

    }

    if (pulse.pulseType == Pulse::PulseType::SWEEP) {

      resetSyncPulseTimer = true;

      // Check if the pulse falls within one sweep cycle time
      // And check if we saw a sync pulse before it
      if (micros() - sensor.syncPulseStart < SWEEP_CYCLE_TIME && sensor.sawSyncPulse) {

        sensor.deltaT = micros() - sensor.syncPulseStart;
        // Serial.println(String(sensor.deltaT) + ", " + String(sensor.skip) + ", " + String(sensor.rotor) + ", " + String(sensor.data));

        byte meta = 0;

        // Construct meta byte
        (sensor.station) ? bitSet(meta, 3) : false;
        (sensor.skip) ? bitSet(meta, 2) : false;
        (sensor.rotor) ? bitSet(meta, 1) : false;
        (sensor.data) ? bitSet(meta, 0) : false;

#ifndef HUMAN_READABLE
        Serial.write(0xff);
        Serial.write(0xff);
        Serial.write((sensor.deltaT >> 8));
        Serial.write((sensor.deltaT & 0x00FF));
        Serial.write(meta);
#endif

#ifdef HUMAN_READABLE
        Serial.print(String(sensor.deltaT) + ", ");
        Serial.print(meta, BIN);
        Serial.println();
#endif

        // Reset sawSyncPulse for next pulse/sweep
        sensor.sawSyncPulse = false;

      } else {

        // Sweep pulse registered, but no sync pulse came before it
        // Ignore data
        sensor.sawSyncPulse = false;

      }

    }

  }

  // Enable interrupts again
  sei();

}

Pulse PulsePosition::parsePulseType(unsigned long pulseLength) {

  if (pulseLength >= S0R0D0 - 5 && pulseLength < S0R0D0 + 5) {

#ifdef SYNC_PULSE_DEBUG
    Serial.println("Skip 0, rotor 0, data 0");
#endif

    Pulse pulse(0, 0, 0, Pulse::PulseType::SYNC_PULSE);
    return pulse;

  }

  if (pulseLength >= S0R0D1 - 5 && pulseLength < S0R0D1 + 5) {

#ifdef SYNC_PULSE_DEBUG
    Serial.println("Skip 0, rotor 0, data 1");
#endif

    Pulse pulse(0, 0, 1, Pulse::PulseType::SYNC_PULSE);
    return pulse;

  }

  if (pulseLength >= S0R1D0 - 5 && pulseLength < S0R1D0 + 5) {

#ifdef SYNC_PULSE_DEBUG
    Serial.println("Skip 0, rotor 1, data 0");
#endif

    Pulse pulse(0, 1, 0, Pulse::PulseType::SYNC_PULSE);
    return pulse;

  }

  if (pulseLength >= S0R1D1 - 5 && pulseLength < S0R1D1 + 5) {

#ifdef SYNC_PULSE_DEBUG
    Serial.println("Skip 0, rotor 1, data 1");
#endif

    Pulse pulse(0, 1, 1, Pulse::PulseType::SYNC_PULSE);
    return pulse;

  }

  if (pulseLength >= S1R0D0 - 5 && pulseLength < S1R0D0 + 5) {

#ifdef SYNC_PULSE_DEBUG
    Serial.println("Skip 1, rotor 0, data 0");
#endif

    Pulse pulse(1, 0, 0, Pulse::PulseType::SYNC_PULSE);
    return pulse;

  }

  if (pulseLength >= S1R0D1 - 5 && pulseLength < S1R0D1 + 5) {

#ifdef SYNC_PULSE_DEBUG
    Serial.println("Skip 1, rotor 0, data 1");
#endif

    Pulse pulse(1, 0, 1, Pulse::PulseType::SYNC_PULSE);
    return pulse;

  }

  if (pulseLength >= S1R1D0 - 5 && pulseLength < S1R1D0 + 5) {

#ifdef SYNC_PULSE_DEBUG
    Serial.println("Skip 1, rotor 1, data 0");
#endif

    Pulse pulse(1, 1, 0, Pulse::PulseType::SYNC_PULSE);
    return pulse;

  }

  if (pulseLength >= S1R1D1 - 5 && pulseLength < S1R1D1 + 5) {

#ifdef SYNC_PULSE_DEBUG
    Serial.println("Skip 1, rotor 1, data 1");
#endif

    Pulse pulse(1, 1, 1, Pulse::PulseType::SYNC_PULSE);
    return pulse;

  }

  if (pulseLength < S0R0D0 - 5) {

#ifdef SYNC_PULSE_DEBUG
    Serial.println("sweep");
#endif

    Pulse pulse(0, 0, 0, Pulse::PulseType::SWEEP);
    return pulse;

  }

  // Pulse does not fit any type, cast as outlier
#ifdef SYNC_PULSE_DEBUG
  Serial.println("outlier, " + String(pulseLength));
#endif

  Pulse pulse(0, 0, 0, Pulse::PulseType::OUTLIER);
  return pulse;

}
