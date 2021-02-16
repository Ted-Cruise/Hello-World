#include <PID.h>

PID::PID(float proportionalGain, float intergalGain, float derivativeGain) {
  _proportionalGain = proportionalGain;
  _intergalGain = intergalGain;
  _derivativeGain = derivativeGain;
}

void PID::setSetpoint(float setpoint) {
  _setpoint = setpoint;
}

float PID::update(float input) {
  float error = _setpoint - input;

  float timeChange = (micros() - _lastPidUpdate) / 1000000.0;

  _integral += error * timeChange;
  float derivative = (error - _preError) / timeChange;

  float output = 0;
  output += _proportionalGain * error;
  output += _intergalGain * _integral;
  output += _derivativeGain * derivative;

  _preError = error;

  return output;
}