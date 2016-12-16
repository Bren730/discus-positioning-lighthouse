
class Pulse {

private:

public:

	enum PulseType {SYNC_PULSE, SWEEP, OUTLIER};

	// Constructor
	Pulse(bool _skip, bool _rotor, bool _data, PulseType _pulseType);

	// Definitions

	// Variables
	bool skip;
	bool rotor;
	bool data;
	bool valid;

	// Enumerators
	PulseType pulseType;

	// Functions

};