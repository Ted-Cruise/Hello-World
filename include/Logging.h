#ifndef LOGGING_H
#define LOGGING_H

#include <Arduino.h>

#include <SPI.h>
#include <SdFat.h>
#include <Adafruit_SPIFlash.h>

class Logging {
  public:
    class SdLog : public SdFile {
      public:
        void printSep(uint8_t val) {
          print(SEPERATOR); print(val);
        } 
        void printSep(int val) {
          print(SEPERATOR); print(val);
        } 
        void printSep(float val) {
          print(SEPERATOR); print(val);
        } 
        void printSep(unsigned long val) {
          print(SEPERATOR); print(val);
        } 
        void printSep(const char * val) {
          print(SEPERATOR); print(val);
        } 
    };

    Logging(const uint8_t sdCsPin);

    void setLoggingRate(const int hertz);

    bool initFlash();

    bool willCreateLog();

    bool doesFlashLogExist(const uint32_t index);
    void readFlashLog(const uint32_t index, uint8_t buffer[256]);
    void writeFlashLog(const uint32_t index, uint8_t buffer[256]);

    bool doFlashLogsExist();
    bool createSdLog(SdLog &sdLog);

    void eraseWrittenSectors();
  private:    
    const constexpr static char * SD_FLIGHT_FILE_PREFIX = "Flight-";
    const constexpr static char * SD_FLIGHT_FILE_EXTENTION = ".csv";

    const constexpr static char * SEPERATOR = ","; // Seperator between logged values

    const int PAGE_SIZE = 256;

    int _loggingRate = 5;

    unsigned long _lastSavedLog;

    Adafruit_FlashTransport_QSPI _flashTransport;
    Adafruit_SPIFlashBase * _flash;

    byte _sdCardCsPin;
    SdFat _sdCard;

    byte getFlightNumber();
};

#endif