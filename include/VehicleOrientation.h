#ifndef VEHICLEORIENTATION_H
#define VEHICLEORIENTATION_H

#include <Arduino.h>

#include <BMI088.h>
#include <Adafruit_MLX90393.h>
#include <orientationUtils/OrientationUtils.h>

struct BMI088Reading {
  float accelX;
  float accelY;
  float accelZ;

  float gyroX;
  float gyroY;
  float gyroZ;

  float temperature;
};

struct MLX90393Reading {
  float magX;
  float magY;
  float magZ;
};

struct Orientation {
  BMI088Reading BMI088RawData;
  MLX90393Reading MLX90393RawData;

  Vector<3> absolute;
  uint8_t heading;
};

class VehicleOrientation {
  public:
    VehicleOrientation(byte accelAddress, byte gyroAddress, byte magAddress);

    void calcGyroBias(bool calcGyroBias);

    void enableAccelComp(bool accelCompEnabled);
    void enableMagComp(bool magCompEnabled);

    void setAccelCompGain(float accelCompGain);
    void setMagCompGain(float magCompGain);

    bool initSensors();

    void getOrientation(Orientation &orientation); 

  private:
    // I don't think its worth adding a setter for these
    const static unsigned int READINGS_FOR_BIAS_CALC = 200;
    const static unsigned int READINGS_FOR_INIT_HEADING = 20;

    const static Bmi088Accel::Range ACCEL_RANGE = Bmi088Accel::RANGE_12G;
    const static Bmi088Gyro::Range GYRO_RANGE = Bmi088Gyro::RANGE_250DPS;

    const static mlx90393_resolution MAG_RESOLUTION = MLX90393_RES_19;
    const static mlx90393_gain MAG_GAIN = MLX90393_GAIN_2_5X;
    const static mlx90393_filter MAG_FILTER = MLX90393_FILTER_2; // X2 is essentially a low-pass filter
    const static mlx90393_oversampling MAG_OVERSAMPLING = MLX90393_OSR_2;
    const unsigned long MAG_SENSE_DURATION = mlx90393_tconv[MAG_FILTER][MAG_OVERSAMPLING] + 10; // + 10 to make up for the delay() removed from the lib

    Bmi088Accel * _accel;
    Bmi088Gyro * _gyro;

    Adafruit_MLX90393 _mag;
    byte _magAddress;

    unsigned long _lastMagReadingTime;
    bool _lastMagReadingSuccessful;

    bool _calcGyroBias;
    float _gyroXOffset;
    float _gyroYOffset;
    float _gyroZOffset;

    bool _accelCompEnabled;
    float _accelCompGain;

    bool _magCompEnabled;
    float _magCompGain;
    Quaternion * _initialHeading;

    Quaternion _absOri;
    unsigned long _lastOriUpdate;

    void calculateGyroBias();
    void getInitialHeading();

    void accelCompFilter(Orientation &orientation);
    void magCompFilter(Orientation &orientation);

    void getHeading(Orientation &ori);

    void readBMI088(BMI088Reading &BMI088Reading);
    void readMLX90393(MLX90393Reading &MLX90393Reading);
};

#endif