#ifndef PULSE_H
#define PULSE_H

class Pulse {

  private:

  public:

    enum PulseType {SYNC_PULSE, SWEEP, OUTLIER, PULSE_START};

    // Constructors
    Pulse();
    Pulse(bool _skip, bool _rotor, bool _data, PulseType _pulseType);

    // Definitions

    // Variables
    bool station;
    bool skip;
    bool rotor;
    bool data;
    bool valid;

    // Enumerators
    PulseType pulseType;

    // Functions

};

#endif
