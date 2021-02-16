#include <VehicleOrientation.h>

VehicleOrientation::VehicleOrientation(byte accelAddress, byte gyroAddress, byte magAddress) {
  _accel = new Bmi088Accel(Wire, accelAddress);
  _gyro = new Bmi088Gyro(Wire, gyroAddress);
  _magAddress = magAddress;
}

void VehicleOrientation::calcGyroBias(bool calcGyroBias) {
  _calcGyroBias = calcGyroBias;
}

void VehicleOrientation::enableAccelComp(bool accelCompEnabled) {
  _accelCompEnabled = accelCompEnabled;
}

void VehicleOrientation::enableMagComp(bool magCompEnabled) {
  _magCompEnabled = magCompEnabled;
}

void VehicleOrientation::setAccelCompGain(float accelCompGain) {
  _accelCompGain = accelCompGain;
}

void VehicleOrientation::setMagCompGain(float magCompGain) {
  _magCompGain = magCompGain;
}

bool VehicleOrientation::initSensors() {
  if (_accel->begin() < 0 || _gyro->begin() < 0) {
    return false;
  }

  if (!_accel->setRange(ACCEL_RANGE) || !_gyro->setRange(GYRO_RANGE)) {
    return false;
  }

  if (!_mag.begin_I2C(_magAddress)) {
    return false;
  }

  if (!_mag.setResolution(MLX90393_X, MAG_RESOLUTION) || !_mag.setResolution(MLX90393_Y, MAG_RESOLUTION) || !_mag.setResolution(MLX90393_Z, MAG_RESOLUTION)) {
    return false;
  }

  if (!_mag.setGain(MAG_GAIN) || !_mag.setFilter(MAG_FILTER) || !_mag.setOversampling(MAG_OVERSAMPLING)) {
    return false;
  }

  //calculateGyroBias();
  //getInitialHeading();

  return true;
}

void VehicleOrientation::calculateGyroBias() {
  if (!_calcGyroBias) return;

  float gyroXOffset = 0;
  float gyroYOffset = 0;
  float gyroZOffset = 0;

  for (int i = 0; i < READINGS_FOR_BIAS_CALC; i++) {
    while(!_gyro->getDrdyStatus());

    BMI088Reading BMI088Reading;
    readBMI088(BMI088Reading);
    
    gyroXOffset += BMI088Reading.gyroX;
    gyroYOffset += BMI088Reading.gyroY;
    gyroZOffset += BMI088Reading.gyroZ;
  }

  gyroXOffset /= READINGS_FOR_BIAS_CALC;
  gyroYOffset /= READINGS_FOR_BIAS_CALC;
  gyroZOffset /= READINGS_FOR_BIAS_CALC;

  _gyroXOffset = gyroXOffset;
  _gyroYOffset = gyroYOffset;
  _gyroZOffset = gyroZOffset;
}

void VehicleOrientation::getInitialHeading() {
  if (!_magCompEnabled) return;

  float magX = 0;
  float magY = 0;
  float magZ = 0;

  for (int i = 0; i < READINGS_FOR_INIT_HEADING; i++) {
    MLX90393Reading MLX90393Reading;
    readMLX90393(MLX90393Reading);

    magX += MLX90393Reading.magX;
    magY += MLX90393Reading.magY;
    magZ += MLX90393Reading.magZ;
  }

  magX /= READINGS_FOR_INIT_HEADING;
  magY /= READINGS_FOR_INIT_HEADING;
  magZ /= READINGS_FOR_INIT_HEADING;

  *_initialHeading = Quaternion(0, magX, magY, magZ);
  _initialHeading->normalize();
}

void VehicleOrientation::getOrientation(Orientation &ori) {
  readMLX90393(ori.MLX90393RawData);

  // Don't update orientation if no new data is available from the gyroscope, the vehicle's primary orientation sensor 
  if (!_gyro->getDrdyStatus()) {
    return;
  }

  readBMI088(ori.BMI088RawData);
  
  Quaternion angularAccelerations(0, ori.BMI088RawData.gyroX, ori.BMI088RawData.gyroY, ori.BMI088RawData.gyroZ);
  Quaternion derivitive = (_absOri * 0.5) * angularAccelerations;

  float timeChange = (micros() - _lastOriUpdate) / 1000000.0;
  
  _absOri = _absOri + derivitive * timeChange;
  _absOri.normalize();

  _lastOriUpdate = micros();

  //accelCompFilter(ori);
  //magCompFilter(ori);

  //getHeading(ori);

  ori.absolute = _absOri.toEuler();
}

void VehicleOrientation::accelCompFilter(Orientation &ori) {
  if (!_accelCompEnabled) return;

  Vector<3> bodyGravity(ori.BMI088RawData.accelX, ori.BMI088RawData.accelY, ori.BMI088RawData.accelZ);
  Vector<3> worldGravity = _absOri.rotateVector(bodyGravity);
  
  Quaternion expectedGravity(1, 0, 0);
  Quaternion correction = Quaternion(worldGravity).normalize().rotationBetweenVectors(expectedGravity);
  correction.conjugate().rotate(correction.fractional(_accelCompGain));
  _absOri = _absOri * correction.normalize();
}

void VehicleOrientation::magCompFilter(Orientation &ori) {
  if (!_magCompEnabled) return;

  Vector<3> bodyHeading(ori.MLX90393RawData.magX, ori.MLX90393RawData.magY, ori.MLX90393RawData.magY);
  Vector<3> worldHeading = _absOri.rotateVector(bodyHeading);

  Quaternion correction = Quaternion(worldHeading).normalize().rotationBetweenVectors(*_initialHeading);
  correction.conjugate().rotate(correction.fractional(_magCompGain));
  _absOri = _absOri * correction.normalize();
}

void VehicleOrientation::getHeading(Orientation &ori) {
  if (ori.MLX90393RawData.magZ > 0) {
    ori.heading = 90 - atan2f(ori.MLX90393RawData.magY, ori.MLX90393RawData.magZ) * RAD_TO_DEG;
  } else if (ori.MLX90393RawData.magZ < 0) {
    ori.heading = 270 - atan2f(ori.MLX90393RawData.magY, ori.MLX90393RawData.magZ) * RAD_TO_DEG;
  } else if (ori.MLX90393RawData.magZ == 0 && ori.MLX90393RawData.magY < 0) {
    ori.heading = 180;
  } else {
    ori.heading = 0;
  }
}

void VehicleOrientation::readBMI088(BMI088Reading &BMI088Reading) {
  _accel->readSensor();
  BMI088Reading.accelX = _accel->getAccelX_mss() * -1;
  BMI088Reading.accelY = _accel->getAccelY_mss() * -1;
  BMI088Reading.accelZ = _accel->getAccelZ_mss() * -1;

  _gyro->readSensor();
  BMI088Reading.gyroX = _gyro->getGyroX_rads() * -1 - _gyroXOffset;
  BMI088Reading.gyroY = _gyro->getGyroY_rads() * -1 - _gyroYOffset;
  BMI088Reading.gyroZ = _gyro->getGyroZ_rads() * -1 - _gyroZOffset;

  BMI088Reading.temperature = _accel->getTemperature_C();
}

void VehicleOrientation::readMLX90393(MLX90393Reading &MLX90393Reading) {
  if (millis() > _lastMagReadingTime + MAG_SENSE_DURATION) {
    _lastMagReadingTime = millis();

    if (_lastMagReadingSuccessful) {
      if (!_mag.readMeasurement(&MLX90393Reading.magY, &MLX90393Reading.magX, &MLX90393Reading.magZ)) {
        Serial.println("Failed on receive dummy");
      }
    }

    _lastMagReadingSuccessful = _mag.startSingleMeasurement();

    if (!_lastMagReadingSuccessful) {
      Serial.println("Stuppid your shit dont work");
    }
  }
}