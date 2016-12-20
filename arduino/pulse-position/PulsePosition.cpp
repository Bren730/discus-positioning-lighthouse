#include "PulsePosition.h"
#include <Arduino.h>

//#define SYNC_PULSE_DEBUG
//#define HUMAN_READABLE

void printBits(byte myByte) {
  for (byte mask = 0x80; mask; mask >>= 1) {
    if (mask  & myByte)
      Serial.print('1');
    else
      Serial.print('0');
  }
}

PulsePosition::PulsePosition(byte _sensorCount) {

  sensorCount = _sensorCount;

}

void PulsePosition::begin() {

}

void PulsePosition::parsePulse(LighthouseSensor& sensor) {

  // Disable interrupts
  cli();

  if (digitalReadFast(sensor.pin) == HIGH) {
    // This is the rising edge of the pulse
    // Record start time of pulse as CPU Cycle count
    // Overflows every 44.73s
    sensor.pulseStart = ARM_DWT_CYCCNT;

  } else {
    // This is the falling edge of the pulse
    // Record the pulse length

    sensor.pulseLength = ARM_DWT_CYCCNT - sensor.pulseStart;
    Pulse pulse = parsePulseType(sensor.pulseLength);

    if (pulse.pulseType == Pulse::PulseType::SYNC_PULSE) {

      // If we did not see a sync pulse, or if a single sweep cycle has elapsed
      // Write out the start flags and sync pulse metadata
      if (!sawSyncPulse || ARM_DWT_CYCCNT - syncPulseStart > SWEEP_CYCLE_CLOCK_CYCLES - 10000) {
        // Only register pulseStart if it is the first sync pulse

        for (byte i = 0; i < sensorCount; i++) {
          // reset all sensors
          // sawSweep is used for reflection elimination
          sawSweep[i] = false;
          
        }
        
        station = pulse.station;
        skip = pulse.skip;
        rotor = pulse.rotor;
        data = pulse.data;
        meta = 0;

        // Construct meta byte
        (station) ? bitSet(meta, 3) : false;
        (skip) ? bitSet(meta, 2) : false;
        (rotor) ? bitSet(meta, 1) : false;
        (data) ? bitSet(meta, 0) : false;

#ifndef HUMAN_READABLE
        Serial.write(0xff);
        Serial.write(0xff);
        Serial.write(meta);
#endif

#ifdef HUMAN_READABLE
        Serial.println();
        Serial.println("Sync pulse, meta:");
        printBits(meta);
        Serial.println();
        Serial.println();
        Serial.println("Sensors:");
#endif

        syncPulseStart = sensor.pulseStart;

        sawSyncPulse = true;

      }

      syncPulseCounter++;

      if (resetSyncPulseTimer) {

        syncPulseTimer = syncPulseStart;
        resetSyncPulseTimer = false;

      }

    }

    if (pulse.pulseType == Pulse::PulseType::SWEEP) {

      #ifdef SYNC_PULSE_DEBUG
      Serial.println("Sensor " + String(sensor.id) + " Registered a sweep");
      #endif

      // Prevent double sweep registering due to reflections
      if(!sawSweep[sensor.id]){

        #ifdef SYNC_PULSE_DEBUG
      Serial.println("Sensor " + String(sensor.id) + " Did not see a sweep earlier");
      #endif

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

#ifndef HUMAN_READABLE

        Serial.write(sensor.id);
        Serial.write((sensor.deltaT >> 24));
        Serial.write((sensor.deltaT >> 16));
        Serial.write((sensor.deltaT >> 8));
        Serial.write((sensor.deltaT & 0x00FF));
#endif

#ifdef HUMAN_READABLE
        Serial.print(String(sensor.id) + ", ");
        Serial.print(String(sensor.deltaT) + ", ");
        Serial.println();
#endif

        sawSweep[sensor.id] = true;
        
      }

      

      } else {

        // Sweep pulse registered, but no sync pulse came before it
        // Ignore data
        sawSyncPulse = false;

      }

    }

  }

  // Enable interrupts again
  sei();

}

Pulse PulsePosition::parsePulseType(unsigned long pulseLength) {

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
  Serial.println("outlier, " + String(pulseLength));
#endif

  Pulse pulse(0, 0, 0, Pulse::PulseType::OUTLIER);
  return pulse;

}
