#include <Arduino.h>
#include "Adafruit_NeoPixel.h"

class NeoPixel {

  private:

    // Functions

  public:

    // Constructors
    NeoPixel(int pixelCount, int pin);
    
    // Functions
    void waiting(byte baseColor[], byte highlightColor[], float highlightLength, float highlightLength2, float duration, float duration2, float fadeIn, bool reverse, bool two, bool additive);
    void startWaiting();

    void show();
    
    void setPercentage(double _percentage, byte rgb[], uint16_t fadeIn, uint16_t duration);
    void showPercentage();

    // Variables
    Adafruit_NeoPixel neoPixel;
    
    unsigned long startTimeWaiting;
    bool isWaiting = false;

    bool didSetPercentageStartTime = false;
    unsigned long percentageStartTime;
    double startPercentage = 0;
    double percentage = 0;
    byte percentagRgb[3];
    uint16_t percentageFadeIn;
    uint16_t percentageDuration;
    
};
