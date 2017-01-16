#include "DataDiscus.h"
#include <Arduino.h>

DataDiscus::DataDiscus() {


}

DataDiscus::DataDiscus(byte _sensorCount, byte _syncPulseSensor, byte _pixelCount, byte _neoPixelPin) {

  sensorCount = _sensorCount;
  syncPulseSensor = _syncPulseSensor;
  pixelCount = _pixelCount;
  neoPixelPin = _neoPixelPin;

//  ring = NeoPixel(pixelCount, neoPixelPin);
  
}

void DataDiscus::begin() {

  pulsePosition.begin(sensorCount, syncPulseSensor);
  
  ring.begin();

  setState(State::STATE_DISCONNECTED);
  
}

void ddByteArrayCopy(byte *arrayOriginal, byte *arrayCopy, byte arraySize) {
  while (arraySize--) *arrayCopy++ = *arrayOriginal++;
}

void ddFloatArrayCopy(float *arrayOriginal, float *arrayCopy, float arraySize) {
  while (arraySize--) *arrayCopy++ = *arrayOriginal++;
}

void DataDiscus::setState(State _state) {

  state = _state;

  switch (state) {

    case State::STATE_DISCONNECTED: {

        ring.isWaiting = false;

        byte _baseColor[] = {3, 10, 25};
        byte _highlightColor[] = {5, 100, 100};
        float _lengths[] = {.65, .65};
        float _durations[] = {2500, 1954};

        ddByteArrayCopy(_baseColor, ddWaitingBaseColor, sizeof(_baseColor));
        ddByteArrayCopy(_highlightColor, ddWaitingHighlightColor, sizeof(_highlightColor));
        ddFloatArrayCopy(_lengths, ddWaitingLengths, sizeof(_lengths));
        ddFloatArrayCopy(_durations, ddWaitingDurations, sizeof(_durations));

        ring.setWaiting(ddWaitingBaseColor, ddWaitingHighlightColor, ddWaitingLengths, ddWaitingDurations, 500, false, true, true);

      }
      break;

    case State::STATE_CONNECTED: {

        ring.isWaiting = false;

        byte _baseColor[] = {0, 10, 0};
        byte _highlightColor[] = {5, 255, 0};
        float _lengths[] = {.5, .5};
        float _durations[] = {2500, -2500};

        ddByteArrayCopy(_baseColor, ddWaitingBaseColor, sizeof(_baseColor));
        ddByteArrayCopy(_highlightColor, ddWaitingHighlightColor, sizeof(_highlightColor));
        ddFloatArrayCopy(_lengths, ddWaitingLengths, sizeof(_lengths));
        ddFloatArrayCopy(_durations, ddWaitingDurations, sizeof(_durations));

        ring.setWaiting(ddWaitingBaseColor, ddWaitingHighlightColor, ddWaitingLengths, ddWaitingDurations, 500, false, true, true);

      }
      break;


    case State::STATE_PAIRING: {

        ring.isWaiting = false;

        byte _baseColor[] = {3, 10, 25};
        byte _highlightColor[] = {5, 100, 255};
        float _lengths[] = {.3, .3};
        float _durations[] = {1000, 800};

        ddByteArrayCopy(_baseColor, ddWaitingBaseColor, sizeof(_baseColor));
        ddByteArrayCopy(_highlightColor, ddWaitingHighlightColor, sizeof(_highlightColor));
        ddFloatArrayCopy(_lengths, ddWaitingLengths, sizeof(_lengths));
        ddFloatArrayCopy(_durations, ddWaitingDurations, sizeof(_durations));

        ring.setWaiting(ddWaitingBaseColor, ddWaitingHighlightColor, ddWaitingLengths, ddWaitingDurations, 500, false, true, true);

      }

      break;

    case State::STATE_TRACKING: {

      }
      break;

    default:
      break;
  }


}

bool DataDiscus::isDisconnected() {

  return (state == State::STATE_DISCONNECTED);
  
}

bool DataDiscus::isConnected() {

  return (state == State::STATE_CONNECTED);
  
}

bool DataDiscus::isPairing() {

  return (state == State::STATE_PAIRING);
  
}

bool DataDiscus::isTracking() {

  return (state == State::STATE_TRACKING);
  
}

