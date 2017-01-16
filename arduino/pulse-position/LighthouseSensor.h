#ifndef LIGHTHOUSE_SENSOR_H
#define LIGHTHOUSE_SENSOR_H

#include <Arduino.h>

class LighthouseSensor {

  private:

    // Functions

  public:

  // Empty constructor
  LighthouseSensor();
  
    // Constructor with pin
    LighthouseSensor(unsigned char _pin);

    // Variables
    volatile unsigned char pin;
    volatile unsigned long pulseStart;
    volatile unsigned long pulseLength;
    volatile unsigned long deltaT; // Max Lighthouse cycle length is 799968 clock cycles

    volatile byte id;
    volatile bool sawSweep;

    // Definitions

    // Functions
    void begin();

};

#endif
