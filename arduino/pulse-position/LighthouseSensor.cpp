#include "LighthouseSensor.h"
#include <Arduino.h>

LighthouseSensor::LighthouseSensor(unsigned char _pin, byte _id) {

	pin = _pin;
 id = _id;
	
}

void LighthouseSensor::begin() {

	pinMode(pin, INPUT);

}
