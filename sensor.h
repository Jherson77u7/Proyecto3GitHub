#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>

class Sensor {
  private:
    int sensorPin;
    unsigned long lastPulseTime;
    unsigned long pulseInterval;
    int currentBPM;
    int validPulseCount;
    bool detectedPulse;

  public:
    Sensor(int pin) : sensorPin(pin), lastPulseTime(0), pulseInterval(0), currentBPM(0), validPulseCount(0), detectedPulse(false) {}

    void initialize() {
        pinMode(sensorPin, INPUT);
    }

    void detectPulse() {
        unsigned long currentTime = millis();
        pulseInterval = currentTime - lastPulseTime;

        if (pulseInterval > 300 && pulseInterval < 2000) {
            currentBPM = 60000 / pulseInterval;

            if (currentBPM >= 40 && currentBPM <= 180) {
                validPulseCount++;
                detectedPulse = true;
            }
        }

        lastPulseTime = currentTime;
    }

    int getCurrentBPM() { return currentBPM; }
    int getValidPulseCount() { return validPulseCount; }
    void resetValidPulseCount() { validPulseCount = 0; }
    bool isPulseDetected() { return detectedPulse; }
    void clearPulseDetection() { detectedPulse = false; }
};

#endif
