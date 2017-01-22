#ifndef DATA_DISCUS_H
#define DATA_DISCUS_H

#include <Arduino.h>
#include "NeoPixel.h"
#include "PulsePosition.h"

class DataDiscus {

  private:

    // Functions

  public:

    // Enumerators
    enum State {STATE_DISCONNECTED, STATE_CONNECTED, STATE_PAIRING, STATE_TRACKING, STATE_NONE};

    // Constructors
    DataDiscus();
    DataDiscus(byte _sensorCount, byte _syncPulseSensor, byte _pixelCount, byte _neoPixelPin);


    // Variables
    byte sensorCount;
    byte syncPulseSensor;
    byte pixelCount;
    byte neoPixelPin;
    unsigned long connectionStartTime;
    uint16_t trackingStartDelay = 4000;
    bool shouldStartTracking = false;
    
    NeoPixel ring = NeoPixel(24, 23);
    PulsePosition pulsePosition;

    State state = STATE_NONE;

    // Animation variables
    byte ddWaitingBaseColor[3] = {3, 10, 25};
    byte ddWaitingHighlightColor[3] = {5, 100, 100};
    float ddWaitingLengths[2] = {.8, .8};
    float ddWaitingDurations[2] = {2500, 1954};

    float ddBatteryLevelDurations[2] = {1500, 2000};
    float ddBatteryAnimationDuration = ddBatteryLevelDurations[1] + (2 * ddBatteryLevelDurations[0]);
    bool didShowBatteryLevel = false;

    // Definitions

    // Functions
    void begin();
    void setState(State _state);
    void showBatteryLevel();

    bool isDisconnected();
    bool isConnected();
    bool isPairing();
    bool isTracking();

};

#endif
