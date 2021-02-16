#ifndef VEHICLEPOSITION_H
#define VEHICLEPOSITION_H

#include <Arduino.h>

#include <Adafruit_BME280.h>

struct BME280Reading {
  float temperature;
  float humidity;
  float pressure;
};

struct Position {
  float altitude;
  float rawAltitude;

  BME280Reading BME280RawData;
};

class VehiclePosition {
  public:
    VehiclePosition(byte BMP280Address);

    bool initSensors();

    void getPosition(Position &position);

  private:
    constexpr static Adafruit_BME280::sensor_mode BME280_MODE = Adafruit_BME280::MODE_NORMAL;
    constexpr static Adafruit_BME280::sensor_sampling BME280_TEMP_OVERSAMPLING_RATIO = Adafruit_BME280::SAMPLING_X2;
    constexpr static Adafruit_BME280::sensor_sampling BME280_HUMIDITY_OVERSAMPLING_RATIO = Adafruit_BME280::SAMPLING_X2;
    constexpr static Adafruit_BME280::sensor_sampling BME280_PRESSURE_OVERSAMPLING_RATIO = Adafruit_BME280::SAMPLING_X4;
    constexpr static Adafruit_BME280::sensor_filter BME280_IIR_FILTER_COEFFICIENT = Adafruit_BME280::FILTER_X2; // X2 is essentially a low-pass filter
    constexpr static Adafruit_BME280::standby_duration BME280_STANDBY_LENGTH = Adafruit_BME280::STANDBY_MS_500;
    constexpr static float BME280_PRESSURE_ERROR = 1.2; // Another word for noise, uncertainty, etc. In Pa

    constexpr static float SEA_LEVEL_PRESSURE_HPA = 1013.25;

    Adafruit_BME280 _BME280;
    byte _BMP280Address;

    // Variables related to the altitude Kalman filter
    float _kalmanGain;
    float _estimate;
    float _preEstimate;
    float _errorInMeasurement = pressureToMetersAir(BME280_PRESSURE_ERROR); // As the Kalman filter is on vehicle altitude instead of pressure, convert the pressure error to meters air error
    float _errorInEstimate = _errorInMeasurement; // Just use the measurement error as an initial value
    float _preErrorInEstimate;

    void readBME280(BME280Reading &BME280Reading);

    static float pressureToAltitude(float pressure);
    static float pressureToMetersAir(float pressure);
};

#endif