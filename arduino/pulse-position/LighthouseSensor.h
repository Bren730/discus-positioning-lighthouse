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
    LighthouseSensor(unsigned char _pin, unsigned int _id, unsigned int _pos);

    // Variables
    volatile unsigned int pin;
    volatile unsigned long pulseStart;
    volatile unsigned long pulseLength;
    volatile unsigned long deltaT; // Max Lighthouse cycle length is 799968 clock cycles

    volatile unsigned int id;
    volatile unsigned int pos;
    volatile bool sawSweep;

    // Definitions

    // Functions
    void begin();

};

#endif
