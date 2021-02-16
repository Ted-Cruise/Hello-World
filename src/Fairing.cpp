#include <Fairing.h>

Fairing::Fairing(byte servoPin) {
  _servoPin = servoPin;
}

void Fairing::setServoProperties(byte closedAngle, byte openAngle) {
  _closedAngle = closedAngle;
  _openAngle = openAngle;
}

void Fairing::begin() {
  _servo.attach(_servoPin);
}

void Fairing::deploy() {
  _servo.write(_openAngle);
  _isDeployed = true;
}

void Fairing::reset() {
  _servo.write(_closedAngle);
  _isDeployed = false;
}

bool Fairing::isDeployed() {
  return _isDeployed;
}
