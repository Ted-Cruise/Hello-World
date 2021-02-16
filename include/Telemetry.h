#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <Arduino.h>

#include <RH_RF95.h>
#include <RHEncryptedDriver.h>
#include <Speck.h>

// Define YIELD for RadioHead library, for some reason it is not defined on the ESP32 platform, which causes RadioHead to hang
#ifdef YIELD
#undef YIELD
#define YIELD yield()
#endif 

class Telemetry {
  public:
    constexpr static size_t MAX_LEN = 235; // Gotten from _driver.maxMessageLength() minus a few bytes because RadioHead doesn't seem to work if exactly max length

    Telemetry(const uint8_t loraCsPin, const uint8_t loraIntPin, const uint8_t loraRstPin);

    void setEncryptionKey(const uint8_t key[], const int len);
    void setTransmitRate(const int hertz);

    bool initRadio();

    bool willTransmitTelemetry();
    bool sendTelemetry(const uint8_t telem[], const uint8_t len);
    bool receiveTelemetry(uint8_t telem[], uint8_t len);

    // In RSSI
    int16_t getSignalStrength();
  private:
    constexpr static float LORA_FREQ = 915.0; // in MHz
    constexpr static uint8_t LORA_POWER = 23; // in dBm

    int _transmitRate = 5;

    unsigned long _lastTransmittedTelem;

    RH_RF95 * _lora;
    uint8_t _loraRstPin;

    Speck * _loraCypher;
    RHEncryptedDriver * _loraDriver;

    void resetRadio();
    bool isSendingTelem();
};

#endif