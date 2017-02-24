#include "LighthouseSensor.h"
#include <Arduino.h>

LighthouseSensor::LighthouseSensor() {
  
}

LighthouseSensor::LighthouseSensor(unsigned char _pin, int _id) {

  pin = _pin;
  id = _id;

  pinMode(pin, INPUT);

}

void LighthouseSensor::begin() {

  pinMode(pin, INPUT);

}
