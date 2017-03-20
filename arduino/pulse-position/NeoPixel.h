#ifndef NEOPIXEL_H
#define NEOPIXEL_H

#include <Arduino.h>
#include "Adafruit_NeoPixel.h"
#include "Easing.h"

class NeoPixel {

  private:

    // Functions

  public:

  enum Visual {VISUAL_WAITING, VISUAL_PERCENTAGE, VISUAL_NONE};

  Visual visual;

    // Constructors
    NeoPixel();
    NeoPixel(int pixelCount, int pin);
    
    // Functions
    void begin();
    void setVisual(Visual _visual);
    void update();
    void showWaiting();
    void setWaiting(byte baseColor[], byte highlightColor[], float highlightLengths[], float durations[], float fadeIn, bool reverse, bool two, bool additive);
    void setMasterPixelBrightness(float _brightness);

    void show();
    void setPixel(int pixel, byte r, byte g, byte b);
    
    void setPercentage(double _percentage, byte rgb[], uint16_t fadeIn, uint16_t duration);
    void showPercentage();

    // Variables
    Adafruit_NeoPixel neoPixel;
    
    float masterPixelBrightness;
    float tempMasterPixelBrightness;
    
    unsigned long waitingStartTime;
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
    byte percentageRgb[3];
    uint16_t percentageFadeIn;
    uint16_t percentageDuration;
    
};

#endif
