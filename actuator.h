#ifndef ACTUATOR_H
#define ACTUATOR_H

#include <Arduino.h>

class Actuator {
  private:
    int pin;

  public:
    Actuator(int pin) : pin(pin) {}

    void initialize() {
        pinMode(pin, OUTPUT);
        deactivate();  // Asegura que el actuador esté desactivado al inicio
    }

    void activate() {
        digitalWrite(pin, HIGH);
    }

    void deactivate() {
        digitalWrite(pin, LOW);
    }
};

#endif
