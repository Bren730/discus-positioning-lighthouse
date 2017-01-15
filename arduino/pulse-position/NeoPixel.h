#include <Arduino.h>
#include "Adafruit_NeoPixel.h"
#include "Easing.h"

class NeoPixel {

  private:

    // Functions

  public:

    // Constructors
    NeoPixel(int pixelCount, int pin);
    
    // Functions
    void showWaiting();
    void setWaiting(byte baseColor[], byte highlightColor[], float highlightLengths[], float durations[], float fadeIn, bool reverse, bool two, bool additive);

    void show();
    
    void setPercentage(double _percentage, byte rgb[], uint16_t fadeIn, uint16_t duration);
    void showPercentage();

    // Variables
    Adafruit_NeoPixel neoPixel;
    
    unsigned long waitingStartTime;
    bool isWaiting = false;
    byte waitingBaseColor[3];
    byte waitingHighlightColor[3];
    float waitingHighlightLengths[2];
    float waitingDurations[2];
    float waitingFadeIn;
    bool waitingReverse;
    bool waitingTwo;
    bool waitingAdditive;

    bool didSetPercentageStartTime = false;
    unsigned long percentageStartTime;
    double startPercentage = 0;
    double percentage = 0;
    byte percentagRgb[3];
    uint16_t percentageFadeIn;
    uint16_t percentageDuration;
    
};
