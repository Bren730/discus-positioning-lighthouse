#include <Arduino.h>

class LighthouseSensor {

private:

	// Functions

public:

	// Constructor
	LighthouseSensor(unsigned char _pin);

	// Variables
	unsigned char pin;
	unsigned long pulseStart;
	unsigned long pulseLength;
	unsigned long syncPulseStart;
	uint16_t deltaT; // Max cycle length is 8333us, so we only need a 16 bit integer
	bool sawSyncPulse;

	bool station;
	bool skip;
	bool rotor;
	bool data;

	// Definitions
	#define S0R0D0 63
	#define S0R0D1 83
	#define S0R1D0 73
	#define S0R1D1 94
	#define S1R0D0 104
	#define S1R0D1 125
	#define S1R1D0 115
	#define S1R1D1 135

	// Functions
	void begin();

};
