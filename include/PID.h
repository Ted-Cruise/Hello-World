#ifndef PID_H
#define PID_H

#include <Arduino.h>

class PID {
  public:
    PID(float proportionalGain, float intergalGain, float derivativeGain);

    void setSetpoint(float setpoint);

    float update(float input);
  private:
    float _proportionalGain, _intergalGain, _derivativeGain;
    float _setpoint;
    float _preError;
    float _integral;
    unsigned long _lastPidUpdate;
};

#endif