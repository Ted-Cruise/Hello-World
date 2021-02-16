#ifndef VOLTAGEDIVIDER_H
#define VOLTAGEDIVIDER_H

#include <Arduino.h>

class VoltageDivider {
  public:
    VoltageDivider(const uint8_t pin, const unsigned int r1, const unsigned int r2, const float referenceVoltage);

    float getVoltage();
  
  private:
    uint8_t _pin;
    unsigned int _r1, _r2;
    float _referenceVoltage;
};

#endif
