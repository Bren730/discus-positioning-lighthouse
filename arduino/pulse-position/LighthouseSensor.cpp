#include "LighthouseSensor.h"
#include <Arduino.h>

LighthouseSensor::LighthouseSensor(unsigned char _pin) {

  pin = _pin;
  id = _pin;

  pinMode(pin, INPUT);

}

void LighthouseSensor::begin() {

  pinMode(pin, INPUT);

}
