#include "LighthouseSensor.h"
#include <Arduino.h>

LighthouseSensor::LighthouseSensor(unsigned char _pin) {

	pin = _pin;
	
}

void LighthouseSensor::begin() {

	pinMode(pin, INPUT);

}