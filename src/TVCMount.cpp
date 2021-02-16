#include <TVCMount.h>

TVCMount::TVCMount(byte yServoPin, byte zServoPin) {
  _yServoPin = yServoPin;
  _zServoPin = zServoPin;
}

void TVCMount::setYServoProperties(byte zeroAngle, float scaleFactor) {
  _yServoZeroAngle = zeroAngle;
  _yServoScaleFactor = scaleFactor;
}

void TVCMount::setZServoProperties(byte zeroAngle, float scaleFactor) {
  _zServoZeroAngle = zeroAngle;
  _zServoScaleFactor = scaleFactor;
}

void TVCMount::begin() {
  _yServo.attach(_yServoPin);
  _zServo.attach(_zServoPin);
}

void TVCMount::setAngle(byte yAngle, byte zAngle) {
  _yServo.write((yAngle + _yServoZeroAngle) * _yServoScaleFactor);
  _zServo.write((zAngle + _zServoZeroAngle) * _zServoScaleFactor);
}

void TVCMount::center() {
  setAngle(0, 0);
}