#include <VoltageDivider.h>

VoltageDivider::VoltageDivider(const uint8_t pin, const unsigned int r1, const unsigned int r2, const float referenceVoltage) {
  _pin = pin;
  _r1 = r1;
  _r2 = r2;
  _referenceVoltage = referenceVoltage;
}

float VoltageDivider::getVoltage() {
  float vout = (analogRead(_pin) * _referenceVoltage) / 1024.0;
  float vin = vout / (_r2 / ((float) _r1 + _r2));
  
  return vin;
}