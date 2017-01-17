#include "NeoPixel.h"
#include "Adafruit_NeoPixel.h"
#include <Arduino.h>

NeoPixel::NeoPixel() {

}

NeoPixel::NeoPixel(int pixelCount, int pin) {

  neoPixel = Adafruit_NeoPixel(pixelCount, pin, NEO_GRB + NEO_KHZ800);

}

void NeoPixel::begin() {

  neoPixel.begin();
  neoPixel.show();

}

void NeoPixel::setVisual(Visual _visual) {

  visual = _visual;

}

void NeoPixel::update() {

  switch (visual) {

    case VISUAL_WAITING: {
        showWaiting();
      }
      break;

    case VISUAL_PERCENTAGE: {
        showPercentage();
      }
      break;

    case VISUAL_NONE: {
      }
      break;

    default:

      break;

  }

}

void byteArrayCopy(byte *arrayOriginal, byte *arrayCopy, byte arraySize) {
  while (arraySize--) *arrayCopy++ = *arrayOriginal++;
}

void floatArrayCopy(float *arrayOriginal, float *arrayCopy, float arraySize) {
  while (arraySize--) *arrayCopy++ = *arrayOriginal++;
}

void NeoPixel::setWaiting(byte baseColor[], byte highlightColor[], float highlightLengths[], float durations[], float fadeIn, bool reverse, bool two, bool additive) {

    byteArrayCopy(baseColor, waitingBaseColor, sizeof(baseColor));
    byteArrayCopy(highlightColor, waitingHighlightColor, sizeof(highlightColor));
    floatArrayCopy(highlightLengths, waitingHighlightLengths, sizeof(highlightLengths));
    floatArrayCopy(durations, waitingDurations, sizeof(durations));
    waitingFadeIn = fadeIn;
    waitingReverse = reverse;
    waitingTwo = two;
    waitingAdditive = additive;

    waitingStartTime = millis();

    setVisual(VISUAL_WAITING);

}

void NeoPixel::showWaiting() {

  float overallIntensity = ((float)millis() - (float)waitingStartTime) / waitingFadeIn;

  if (overallIntensity > 1) {
    overallIntensity = 1;
  }

  float numWaves = 1 / waitingHighlightLengths[0];
  float numWaves2 = 1 / waitingHighlightLengths[1];

  for (byte i = 0; i < neoPixel.numPixels(); i++) {
    double lT;
    double lT2;

    if (waitingReverse) {

      lT = (double)millis() + i * (waitingDurations[0] / (double)neoPixel.numPixels());
      lT2 = (double)millis() - i * (waitingDurations[1] / (double)neoPixel.numPixels());

    } else {

      lT = (double)millis() - i * (waitingDurations[0] / (double)neoPixel.numPixels());
      lT2 = (double)millis() + i * (waitingDurations[1] / (double)neoPixel.numPixels());

    }

    int lCycle = floor(lT / waitingDurations[0]);
    int lCycle2 = floor(lT2 / waitingDurations[1]);

    float localPerc = (lT - (lCycle * waitingDurations[0])) / waitingDurations[0];
    float localPerc2 = (lT2 - (lCycle2 * waitingDurations[1])) / waitingDurations[1];

    //    if (localPerc < highlightLength) {

    float intensity = overallIntensity * 0.5 * cos((localPerc) * 2 * PI * numWaves + PI) + 0.5;
    float intensity2 = overallIntensity * 0.5 * cos((localPerc2) * 2 * PI * numWaves2 + PI) + 0.5;

    if (localPerc > waitingHighlightLengths[0]) {
      intensity = 0;
    }

    if (localPerc2 > waitingHighlightLengths[1]) {
      intensity2 = 0;
    }

    //      Serial.println(String(i) + " " + String(intensity));

    uint16_t r = overallIntensity * ((waitingHighlightColor[0] - waitingBaseColor[0]) * intensity + waitingBaseColor[0]);
    uint16_t g = overallIntensity * ((waitingHighlightColor[1] - waitingBaseColor[1]) * intensity + waitingBaseColor[1]);
    uint16_t b = overallIntensity * ((waitingHighlightColor[2] - waitingBaseColor[2]) * intensity + waitingBaseColor[2]);

    uint16_t r2 = overallIntensity * ((waitingHighlightColor[0] - waitingBaseColor[0]) * intensity2 + waitingBaseColor[0]);
    uint16_t g2 = overallIntensity * ((waitingHighlightColor[1] - waitingBaseColor[1]) * intensity2 + waitingBaseColor[1]);
    uint16_t b2 = overallIntensity * ((waitingHighlightColor[2] - waitingBaseColor[2]) * intensity2 + waitingBaseColor[2]);

    if (waitingAdditive) {

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

    if (waitingTwo) {
      neoPixel.setPixelColor(i, r2, g2, b2);
    }

  }

  show();

}

void NeoPixel::setPercentage(double _percentage, byte rgb[], uint16_t fadeIn, uint16_t duration) {

  if (!didSetPercentageStartTime) {

    startPercentage = percentage;
    percentage = _percentage;
    percentageRgb[0] = rgb[0];
    percentageRgb[1] = rgb[1];
    percentageRgb[2] = rgb[2];
    percentageFadeIn = fadeIn;
    percentageDuration = duration;

    percentageStartTime = millis();

    didSetPercentageStartTime = true;

    setVisual(VISUAL_PERCENTAGE);
  }

}

void NeoPixel::showPercentage() {

  if (didSetPercentageStartTime) {

    //    Serial.println(millis());

    double progress = (millis() - percentageStartTime) / (double)percentageFadeIn;

    if (millis() - percentageStartTime < percentageFadeIn) {

      //      Serial.println(progress);
      progress = QuinticEaseInOut(progress);
      //      Serial.println(progress);
      //      Serial.println();
      delay(16);
      double percentagePerPin = 1.0 / (double)neoPixel.numPixels();
      double percentageDiff = percentage - startPercentage;
      double relativeProgress = progress * percentageDiff;
      double absoluteProgress = startPercentage + relativeProgress;

      byte currentPin = floor((startPercentage + relativeProgress) * neoPixel.numPixels());

      for (byte i = 0; i < neoPixel.numPixels(); i++) {

        if (i == currentPin) {

          float pixelProgress = (absoluteProgress - (i * percentagePerPin)) / percentagePerPin;

          neoPixel.setPixelColor(i, pixelProgress * percentageRgb[0], pixelProgress * percentageRgb[1], pixelProgress * percentageRgb[2]);

        } else if (i < currentPin) {

          neoPixel.setPixelColor(i, percentageRgb[0], percentageRgb[1], percentageRgb[2]);

        } else {
          neoPixel.setPixelColor(i, 0, 0, 0);

        }

      }

    }

    if (millis() - percentageStartTime > (percentageFadeIn + percentageDuration)) {

      // Animate the percentage bar back to 0
      didSetPercentageStartTime = false;

      setPercentage(0, percentageRgb, percentageFadeIn, 0);

    }

    if ((millis() - percentageStartTime) > (unsigned int)(percentageFadeIn + percentageFadeIn + percentageDuration)) {

      setVisual(Visual::VISUAL_NONE);

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



