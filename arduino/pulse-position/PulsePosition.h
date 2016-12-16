#include "LighthouseSensor.h"
#include "Pulse.h"

class PulsePosition {

private:

public:

	// Constructor
	PulsePosition();

	// Definitions
	#define S0R0D0 63
	#define S0R0D1 83
	#define S0R1D0 73
	#define S0R1D1 94
	#define S1R0D0 104
	#define S1R0D1 125
	#define S1R1D0 115
	#define S1R1D1 135

	#define SWEEP_CYCLE_TIME 8333

	// Variables
	unsigned long syncPulseTimer;
	bool resetSyncPulseTimer = true;

	// Functions
	void begin();
	void parsePulse(LighthouseSensor& sensor);
	Pulse parsePulseType(unsigned long pulseLength);

};