#include <Logging.h>

Logging::Logging(const uint8_t sdCardCsPin) {
  _flash = new Adafruit_SPIFlashBase(&_flashTransport);

  _sdCardCsPin = sdCardCsPin;
}

void Logging::setLoggingRate(const int hertz) {
  _loggingRate = hertz;
}

bool Logging::initFlash() {
  const SPIFlash_Device_t SUPPORTED_FLASH_DEVICES[] = {
    S25FL064L
  };
  const size_t SUPPORTED_FLASH_COUNT = sizeof(SUPPORTED_FLASH_DEVICES) / sizeof(SUPPORTED_FLASH_DEVICES[0]);

  if (!_flash->begin(SUPPORTED_FLASH_DEVICES, SUPPORTED_FLASH_COUNT)) {
    return false;
  }

  return true;
}

bool Logging::willCreateLog() {
  if (_loggingRate == 0) return false;

  return millis() > _lastSavedLog + (1000 / _loggingRate);
}

bool Logging::doesFlashLogExist(const uint32_t index) {
  return _flash->read8(index * PAGE_SIZE) != 0xFF; // Is the first byte empty?
}

void Logging::readFlashLog(const uint32_t index, uint8_t buffer[256]) {
  _flash->readBuffer(index * PAGE_SIZE, buffer, PAGE_SIZE);
}

void Logging::writeFlashLog(const uint32_t index, uint8_t buffer[256]) {
  if (!willCreateLog()) return;
  _lastSavedLog = millis();

  _flash->writeBuffer(index * PAGE_SIZE, buffer, PAGE_SIZE);
}

bool Logging::doFlashLogsExist() {
  return doesFlashLogExist(0);
}

bool Logging::createSdLog(SdLog &sdLog) {
  if (!_sdCard.begin(_sdCardCsPin)) {
    return false;
  }

  char fileName[20];
  sprintf(fileName, "%s%d%s", SD_FLIGHT_FILE_PREFIX, getFlightNumber(), SD_FLIGHT_FILE_EXTENTION);

  if (!sdLog.open(fileName, O_RDWR | O_CREAT | O_TRUNC)) {
    _sdCard.errorHalt("Failed to open file");
    return false;
  }

  return true;
}

byte Logging::getFlightNumber() {
  byte flightNumber = 0;
  char fileName[20];
  
  do {
    sprintf(fileName, "%s%d%s", SD_FLIGHT_FILE_PREFIX, ++flightNumber, SD_FLIGHT_FILE_EXTENTION);
  } while (_sdCard.exists(fileName));

  return flightNumber;
}

void Logging::eraseWrittenSectors() {
  // Re-init flash, otherwise erase fails sometimes
  initFlash();

  unsigned int addr = 0;

  while (_flash->read8(addr * SFLASH_SECTOR_SIZE) != 0xFF) { // Is the first byte empty?
    _flash->eraseSector(addr);
    _flash->waitUntilReady();

    addr++;
  }
}