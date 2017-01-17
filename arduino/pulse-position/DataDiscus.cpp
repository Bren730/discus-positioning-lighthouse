#include "DataDiscus.h"
#include <Arduino.h>

DataDiscus::DataDiscus() {


}

DataDiscus::DataDiscus(byte _sensorCount, byte _syncPulseSensor, byte _pixelCount, byte _neoPixelPin) {

  sensorCount = _sensorCount;
  syncPulseSensor = _syncPulseSensor;
  pixelCount = _pixelCount;
  neoPixelPin = _neoPixelPin;

  //  state = STATE_DISCONNECTED;

  //  ring = NeoPixel(pixelCount, neoPixelPin);

}

void DataDiscus::begin() {

  pulsePosition.begin(sensorCount, syncPulseSensor);

  ring.begin();

  setState(STATE_NONE);
  showBatteryLevel();

}

void ddByteArrayCopy(byte *arrayOriginal, byte *arrayCopy, byte arraySize) {
  while (arraySize--) *arrayCopy++ = *arrayOriginal++;
}

void ddFloatArrayCopy(float *arrayOriginal, float *arrayCopy, byte arraySize) {
  arraySize = byte(arraySize / sizeof(float));
  while (arraySize--) *arrayCopy++ = *arrayOriginal++;
}

void DataDiscus::setState(State _state) {

  cli();

  switch (_state) {

    case STATE_DISCONNECTED: {

        if (_state != state) {

          byte _baseColor[] = {3, 10, 25};
          byte _highlightColor[] = {5, 100, 100};
          float _lengths[] = {.65, .65};
          float _durations[] = {2500, 1954};

          ddByteArrayCopy(_baseColor, ddWaitingBaseColor, sizeof(ddWaitingBaseColor));
          ddByteArrayCopy(_highlightColor, ddWaitingHighlightColor, sizeof(ddWaitingHighlightColor));
          ddFloatArrayCopy(_lengths, ddWaitingLengths, sizeof(ddWaitingLengths));
          ddFloatArrayCopy(_durations, ddWaitingDurations, sizeof(ddWaitingDurations));

          ring.setWaiting(ddWaitingBaseColor, ddWaitingHighlightColor, ddWaitingLengths, ddWaitingDurations, 500, false, true, true);
        }
      }
      break;

    case STATE_CONNECTED: {
        if (_state != state) {

          Serial.println();

          byte _baseColor[] = {0, 10, 0};
          byte _highlightColor[] = {5, 255, 0};
          float _lengths[] = {.5, .5};
          float _durations[] = {2500, -2500};

          ddByteArrayCopy(_baseColor, ddWaitingBaseColor, sizeof(ddWaitingBaseColor));
          ddByteArrayCopy(_highlightColor, ddWaitingHighlightColor, sizeof(ddWaitingHighlightColor));
          ddFloatArrayCopy(_lengths, ddWaitingLengths, sizeof(ddWaitingLengths));
          ddFloatArrayCopy(_durations, ddWaitingDurations, sizeof(ddWaitingDurations));

          ring.setWaiting(ddWaitingBaseColor, ddWaitingHighlightColor, ddWaitingLengths, ddWaitingDurations, 500, false, true, true);
        }
      }
      break;


    case STATE_PAIRING: {
        if (_state != state) {

          byte _baseColor[] = {3, 10, 25};
          byte _highlightColor[] = {5, 100, 255};
          float _lengths[] = {.5, .5};
          float _durations[] = {1000, -1000};

          ddByteArrayCopy(_baseColor, ddWaitingBaseColor, sizeof(ddWaitingBaseColor));
          ddByteArrayCopy(_highlightColor, ddWaitingHighlightColor, sizeof(ddWaitingHighlightColor));
          ddFloatArrayCopy(_lengths, ddWaitingLengths, sizeof(ddWaitingLengths));
          ddFloatArrayCopy(_durations, ddWaitingDurations, sizeof(ddWaitingDurations));

          ring.setWaiting(ddWaitingBaseColor, ddWaitingHighlightColor, ddWaitingLengths, ddWaitingDurations, 500, false, true, true);
        }
      }

      break;

    case STATE_TRACKING: {

      }
      break;

    case STATE_NONE: {

      }
      break;

    default:
      break;
  }

  state = _state;

  sei();
}

void DataDiscus::showBatteryLevel() {

  float percentage = 0.9;
  byte r = (1 - percentage) * 255;
  byte g = percentage * 255;

  byte color[] = {r, g, 0};

  ring.didSetPercentageStartTime = false;

  ring.setPercentage(percentage, color, ddBatteryLevelDurations[0], ddBatteryLevelDurations[1]);

}

bool DataDiscus::isDisconnected() {

  return (state == STATE_DISCONNECTED);

}

bool DataDiscus::isConnected() {

  return (state == STATE_CONNECTED);

}

bool DataDiscus::isPairing() {

  return (state == STATE_PAIRING);

}

bool DataDiscus::isTracking() {

  return (state == STATE_TRACKING);

}

