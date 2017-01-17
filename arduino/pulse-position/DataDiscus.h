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
    enum State {STATE_DISCONNECTED, STATE_CONNECTED, STATE_PAIRING, STATE_TRACKING, STATE_BATTERY, STATE_NONE};

//    #define STATE_DISCONNECTED
//    #define STATE_CONNECTED
//    #define STATE_PAIRING
//    #define STATE_TRACKING
//    #define STATE_BATTERY
//    #define STATE_NONE
    
    State state = STATE_NONE;

    byte sensorCount;
    byte syncPulseSensor;
    byte pixelCount;
    byte neoPixelPin;

    NeoPixel ring = NeoPixel(24, 23);
    PulsePosition pulsePosition;

    bool isDisconnected();
    bool isConnected();
    bool isPairing();
    bool isTracking();

    byte ddWaitingBaseColor[3] = {3, 10, 25};
    byte ddWaitingHighlightColor[3] = {5, 100, 100};
    float ddWaitingLengths[2] = {.8, .8};
    float ddWaitingDurations[2] = {2500, 1954};

    // Constructors
    DataDiscus();
    DataDiscus(byte _sensorCount, byte _syncPulseSensor, byte _pixelCount, byte _neoPixelPin);


    // Variables

    // Definitions

    // Functions
    void begin();
    void setState(State _state);
    void showBatteryLevel();

};

#endif
