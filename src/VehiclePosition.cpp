#include <VehiclePosition.h>

VehiclePosition::VehiclePosition(byte BMP280Address) {
  _BMP280Address = BMP280Address;
}

bool VehiclePosition::initSensors() {
  if (!_BME280.begin(_BMP280Address)) {
    return false;
  }

  _BME280.setSampling(BME280_MODE,
                      BME280_TEMP_OVERSAMPLING_RATIO,
                      BME280_HUMIDITY_OVERSAMPLING_RATIO,
                      BME280_PRESSURE_OVERSAMPLING_RATIO,
                      BME280_IIR_FILTER_COEFFICIENT,
                      BME280_STANDBY_LENGTH);

  return true;
}

void VehiclePosition::getPosition(Position &pos) {
  readBME280(pos.BME280RawData);

  pos.rawAltitude = pressureToAltitude(pos.BME280RawData.pressure);

  // Calculate Kalman gain
  _kalmanGain = _errorInEstimate / (_errorInEstimate + BME280_PRESSURE_ERROR);

  // Calculate the current estimate
  _preEstimate = _estimate;
  _estimate = _preEstimate + _kalmanGain * (pos.rawAltitude - _preEstimate);

  // Calculate the new estimate error
  _preErrorInEstimate = _errorInEstimate;
  _errorInEstimate = (1 - _kalmanGain) * _preErrorInEstimate;

  pos.altitude = _estimate;
}

void VehiclePosition::readBME280(BME280Reading &BME280Reading) {
  BME280Reading.temperature = _BME280.readTemperature();
  BME280Reading.pressure = _BME280.readHumidity();
  BME280Reading.pressure = _BME280.readPressure();
}

float VehiclePosition::pressureToAltitude(float pressure) {
  pressure /= 100; // convert to hPa

  // Gets altitude above sea level in meters
  return 44330 * (1.0 - pow(pressure / SEA_LEVEL_PRESSURE_HPA, 0.1903));
}

// Converts Pascals to meters air at 0 degrees Celsius
float VehiclePosition::pressureToMetersAir(float pressure) {
  return pressure * 0.078880172892718;
}