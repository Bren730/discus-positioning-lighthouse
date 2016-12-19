#include <Arduino.h>

class LighthouseSensor {

  private:

    // Functions

  public:

    // Constructor
    LighthouseSensor(unsigned char _pin);

    // Variables
    volatile unsigned char pin;
    volatile unsigned long pulseStart;
    volatile unsigned long pulseLength;
    volatile unsigned long deltaT; // Max Lighthouse cycle length is 799968 clock cycles

    volatile byte id;

    // Definitions

    // Functions
    void begin();

};
