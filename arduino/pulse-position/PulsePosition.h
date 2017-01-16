#ifndef PULSE_POSITION_H
#define PULSE_POSITION_H

#include "LighthouseSensor.h"
#include "Pulse.h"
#include "Adafruit_BLE_UART.h"

class PulsePosition {

  private:

  public:

    // Constructor
    PulsePosition();

    // Definitions

    // Pulse length definitions
    // Formatted as:
    // Skip 0/1, if rotor will skip laser
    // Rotor 0/1, signifies which rotor is performing a sweep
    // Data 0/1, data bit of the OOTX frame
#define CPU_SPEED 96
#define S0R0D0 63 * CPU_SPEED   // 6048
#define S0R0D1 83 * CPU_SPEED   // 7968
#define S0R1D0 73 * CPU_SPEED   // 7008
#define S0R1D1 94 * CPU_SPEED   // 9024
#define S1R0D0 104 * CPU_SPEED  // 9984
#define S1R0D1 125 * CPU_SPEED  // 12000
#define S1R1D0 115 * CPU_SPEED  // 11040
#define S1R1D1 135 * CPU_SPEED  // 12960
#define PULSE_CHANNEL_WIDTH 5 * CPU_SPEED

#define SWEEP_CYCLE_TIME 8333
#define SWEEP_CYCLE_CLOCK_CYCLES SWEEP_CYCLE_TIME * CPU_SPEED

    // Variables
    byte sensorCount;
    byte syncPulseSensor;
    volatile unsigned long pulseLength;
    volatile unsigned long syncPulseStart;
    volatile unsigned long prevSyncPulseStart;
    volatile unsigned long syncPulseTimer;
    volatile bool sawSyncPulse = false;
    volatile bool resetSyncPulseTimer = true;
    volatile byte syncPulseCounter = 0;
    volatile byte meta = 0;
    Pulse syncPulse;
    Pulse prevSyncPulse;

    LighthouseSensor sensors[34];

    volatile bool station;
    volatile bool skip;
    volatile bool rotor;
    volatile bool data;

    // Functions
    void begin(byte _sensorCount, byte _syncPulseSensor);
    Pulse parsePulse(LighthouseSensor& sensor);
    void writePulseTime(LighthouseSensor& sensor);
    Pulse parsePulseType(unsigned long pulseLength);
    void writeData();
    void getOutputBuffer(uint8_t *buffer);

};

#endif
