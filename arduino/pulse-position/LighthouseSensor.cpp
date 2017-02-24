#include "LighthouseSensor.h"
#include <Arduino.h>

LighthouseSensor::LighthouseSensor() {
  
}

LighthouseSensor::LighthouseSensor(unsigned char _pin, unsigned int _id, unsigned int _pos) {

  pin = _pin;
  id = _id;
  pos = _pos;

  pinMode(pin, INPUT);

}

void LighthouseSensor::begin() {

  pinMode(pin, INPUT);

}
