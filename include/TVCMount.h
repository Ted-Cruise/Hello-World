#ifndef TVCMOUNT_H
#define TVCMOUNT_H

#include <Arduino.h>

#include <Servo.h>

class TVCMount {
  public:
    TVCMount(uint8_t yServoPin, uint8_t zServoPin);

    void setYServoProperties(uint8_t zeroAngle, float scaleFactor);
    void setZServoProperties(uint8_t zeroAngle, float scaleFactor);

    void begin();

    void setAngle(uint8_t yAngle, uint8_t zAngle);
    void center();
  private:
    Servo _yServo;
    uint8_t _yServoPin;
    uint8_t _yServoZeroAngle = 90;
    uint8_t _yServoScaleFactor = 1.0;

    Servo _zServo;
    uint8_t _zServoPin;
    uint8_t _zServoZeroAngle = 90;
    uint8_t _zServoScaleFactor = 1.0;
};

#endif