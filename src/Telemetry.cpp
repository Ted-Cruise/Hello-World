#include <Telemetry.h>

Telemetry::Telemetry(const uint8_t loraCsPin, const uint8_t loraIntPin, const uint8_t loraRstPin) {
  _lora = new RH_RF95(loraCsPin, loraIntPin);
  _loraRstPin = loraRstPin;

  _loraCypher = new Speck();
  _loraDriver = new RHEncryptedDriver(*_lora, *_loraCypher);
}

void Telemetry::setEncryptionKey(const uint8_t key[], const int len) {
  _loraCypher->setKey(key, len);
}

void Telemetry::setTransmitRate(const int hertz) {
  _transmitRate = hertz;
}

bool Telemetry::initRadio() {
  pinMode(_loraRstPin, OUTPUT);
  digitalWrite(_loraRstPin, HIGH);

  resetRadio();
  
  if (!_lora->init() || !_lora->setFrequency(LORA_FREQ)) {
    return false;
  }

  _lora->setTxPower(LORA_POWER, false);

  return true;
}

bool Telemetry::willTransmitTelemetry() {
  return millis() > _lastTransmittedTelem + (1000 / _transmitRate);
}

bool Telemetry::sendTelemetry(const uint8_t telem[], const uint8_t len) {
  if (!willTransmitTelemetry() || isSendingTelem()) {
    return true;
  }
  _lastTransmittedTelem = millis();

  //Serial.println("tras!");
  if (!_loraDriver->send(telem, len)) {
    return false;
  }

  return true;
}

bool Telemetry::receiveTelemetry(uint8_t telem[], uint8_t len) {
  if (isSendingTelem()) {
    return true;
  }

  if (!_loraDriver->recv(telem, &len)) {
    return false;
  }

  return true;
}

int16_t Telemetry::getSignalStrength() {
  int16_t lastRssi = _loraDriver->lastRssi();
  
  if (lastRssi < -120 || lastRssi > 0) {
    lastRssi = -120;
  }

  return lastRssi;
}

void Telemetry::resetRadio() {
  digitalWrite(_loraRstPin, LOW);
  delay(10);
  digitalWrite(_loraRstPin, HIGH);
  delay(10);
}

bool Telemetry::isSendingTelem() {
  return _loraDriver->mode() == RH_RF95_MODE_TX;
}