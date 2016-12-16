#include "Pulse.h"
#include <Arduino.h>

Pulse::Pulse(bool _skip, bool _rotor, bool _data, PulseType _pulseType) {

	skip = _skip;
	rotor = _rotor;
	data = _data;
	pulseType =_pulseType;
	
}