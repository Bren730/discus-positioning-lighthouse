#include "NeoPixel.h"
#include "Adafruit_NeoPixel.h"
#include <Arduino.h>

NeoPixel::NeoPixel(int pixelCount, int pin) {

  neoPixel = Adafruit_NeoPixel(pixelCount, pin, NEO_GRB + NEO_KHZ800);
  neoPixel.begin();
  neoPixel.show();

}

void NeoPixel::startWaiting() {

  if (!isWaiting) {
    startTimeWaiting = millis();
    isWaiting = true;
  }

}

void NeoPixel::waiting(byte baseColor[], byte highlightColor[], float highlightLength, float highlightLength2, float duration, float duration2, float fadeIn, bool reverse, bool two, bool additive) {

  if (isWaiting) {

    float overallIntensity = ((float)millis() - (float)startTimeWaiting) / fadeIn;

    if (overallIntensity > 1) {
      overallIntensity = 1;
    }

    float numWaves = 1 / highlightLength;
    float numWaves2 = 1 / highlightLength2;

    for (byte i = 0; i < neoPixel.numPixels(); i++) {
      double lT;
      double lT2;

      if (reverse) {

        lT = (double)millis() + i * (duration / (double)neoPixel.numPixels());
        lT2 = (double)millis() - i * (duration2 / (double)neoPixel.numPixels());

      } else {

        lT = (double)millis() - i * (duration / (double)neoPixel.numPixels());
        lT2 = (double)millis() + i * (duration2 / (double)neoPixel.numPixels());

      }

      int lCycle = floor(lT / duration);
      int lCycle2 = floor(lT2 / duration2);

      float localPerc = (lT - (lCycle * duration)) / duration;
      float localPerc2 = (lT2 - (lCycle2 * duration2)) / duration2;

      //    if (localPerc < highlightLength) {

      float intensity = overallIntensity * 0.5 * cos((localPerc) * 2 * PI * numWaves + PI) + 0.5;
      float intensity2 = overallIntensity * 0.5 * cos((localPerc2) * 2 * PI * numWaves2 + PI) + 0.5;

      if (localPerc > highlightLength) {
        intensity = 0;
      }

      if (localPerc2 > highlightLength2) {
        intensity2 = 0;
      }

      //      Serial.println(String(i) + " " + String(intensity));

      uint16_t r = overallIntensity * ((highlightColor[0] - baseColor[0]) * intensity + baseColor[0]);
      uint16_t g = overallIntensity * ((highlightColor[1] - baseColor[1]) * intensity + baseColor[1]);
      uint16_t b = overallIntensity * ((highlightColor[2] - baseColor[2]) * intensity + baseColor[2]);

      uint16_t r2 = overallIntensity * ((highlightColor[0] - baseColor[0]) * intensity2 + baseColor[0]);
      uint16_t g2 = overallIntensity * ((highlightColor[1] - baseColor[1]) * intensity2 + baseColor[1]);
      uint16_t b2 = overallIntensity * ((highlightColor[2] - baseColor[2]) * intensity2 + baseColor[2]);

      if (additive) {

        r2 = r2 + r;
        g2 = g2 + g;
        b2 = b2 + b;

        if (r2 > 255) {
          r2 = 255;
        }
        if (g2 > 255) {
          g2 = 255;
        }
        if (b2 > 255) {
          b2 = 255;
        }

      } else {

        r2 = (r2 + r) / 2;
        g2 = (g2 + g) / 2;
        b2 = (b2 + b) / 2;

      }

      neoPixel.setPixelColor(i, r, g, b);

      if (two) {
        neoPixel.setPixelColor(i, r2, g2, b2);
      }

      //    }

    }

    show();
  }
}

void NeoPixel::setPercentage(double _percentage, byte rgb[], uint16_t fadeIn, uint16_t duration) {

  if (!didSetPercentageStartTime) {

    startPercentage = percentage;
    percentage = _percentage;
    percentagRgb[0] = rgb[0];
    percentagRgb[1] = rgb[1];
    percentagRgb[2] = rgb[2];
    percentageFadeIn = fadeIn;
    percentageDuration = duration;

    percentageStartTime = millis();

    didSetPercentageStartTime = true;
  }

}

void NeoPixel::showPercentage() {

  if (didSetPercentageStartTime) {

    //    Serial.println(millis());

    double progress = (millis() - percentageStartTime) / (double)percentageFadeIn;
    
    if (millis() - percentageStartTime < percentageFadeIn) {
      double percentagePerPin = 1.0 / (double)neoPixel.numPixels();
      double percentageDiff = percentage - startPercentage;
      double relativeProgress = progress * percentageDiff;
      double absoluteProgress = startPercentage + relativeProgress;

      byte currentPin = floor((startPercentage + relativeProgress) * neoPixel.numPixels());

      for (byte i = 0; i < neoPixel.numPixels(); i++) {

        if (i == currentPin) {

          float pixelProgress = (absoluteProgress - (i * percentagePerPin)) / percentagePerPin;

          neoPixel.setPixelColor(i, pixelProgress * percentagRgb[0], pixelProgress * percentagRgb[1], pixelProgress * percentagRgb[2]);

        } else if (i < currentPin) {

          neoPixel.setPixelColor(i, percentagRgb[0], percentagRgb[1], percentagRgb[2]);

        } else {
          neoPixel.setPixelColor(i, 0, 0, 0);

        }

      }

    }

    if (millis() - percentageStartTime > (percentageFadeIn + percentageDuration)) {
      //      for (byte i = 0; i < neoPixel.numPixels(); i++) {
      //
      //        neoPixel.setPixelColor(i, 0, 0, 0);
      //
      //      }

      didSetPercentageStartTime = false;

    }

    // Keep at end of function
    show();

  }

}

void NeoPixel::show() {

  neoPixel.show();
  // This delay prevents artefacts
  delay(1);
}

