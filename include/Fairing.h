#ifndef FAIRING_H
#define FAIRING_H

#include <Arduino.h>

#include <Servo.h>

class Fairing {
  public:
    Fairing(byte servoPin);

    void setServoProperties(byte closedAngle, byte openAngle);
    
    void begin();

    void deploy();
    void reset();

    bool isDeployed();
  private:
    int _closedAngle;
    int _openAngle;

    Servo _servo;
    byte _servoPin;

    bool _isDeployed;
};

#endif