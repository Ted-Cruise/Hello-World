#include <Arduino.h>

#include <SdFat.h>

#include <credentials.h>

#include <VehicleState.h>
#include <StatusLight.h>
#include <VehiclePosition.h>
#include <VehicleOrientation.h>
#include <MovingAverage.h>
#include <PID.h>
#include <TVCMount.h>
#include <Fairing.h>
#include <VoltageDivider.h>
#include <Telemetry.h>
#include <Logging.h>

const unsigned int STATE_CHANGE_TIME = 100; // How long must the condition required for state change be true to change states, in ms
const float LAUNCH_ACCEL_THRESH = 11.0; // Low side, in m/s^2
const float BURNOUT_ACCEL_THRESH = 2.0; // High side, in m/s^2
const unsigned int ALTITUDE_READ_INTERVAL = 1000; // How often to read the vehicle's altitude for apogee detection, in ms
const float APOGEE_DETECT_ALTITUDE_DIFFERENCE = -0.1; // Difference in altitude from the reading taken ALTITUDE_READ_INTERVAL ms ago for apogee to be detected, in meters
const unsigned int CHUTE_DEPLOY_ALTITUDE = 25; // Altitude to deploy the parachutes, in meters
const unsigned int GYRO_READ_INTERVAL = 500; // How often to read the gyroscopes to determine if the vehicle has landed, in ms
const unsigned int LANDING_GYRO_READINGS = 20; // Number of readings in the gyro moving averages used to determine if the vehicle has landed
const float LANDING_GYRO_MAX_ST_DEV = 2; // Maximum standard deviation of the gyro readings in the moving averages on any of the axis, in degrees/sec

const uint8_t STATUS_LIGHT_PIN = A2;

const uint8_t PRESSURE_ADDR = 0x76;
const uint8_t ACCEL_ADDR = 0x18;
const uint8_t GYRO_ADDR = 0x68;
const uint8_t MAG_ADDR = 0x14;

const uint8_t SERVO_PINS[] = {11, 10, 5, 4, 6, 9};
const uint8_t PYRO_PINS[] = {13, 12, 0, 1};

const bool CALCULATE_GYRO_BIAS = false;
const unsigned int READINGS_FOR_GYRO_BIAS_CALC = 200;
const bool USE_ACCEL_COMP = false;
const bool USE_MAG_COMP = false;

const int VEHICLE_MOMENT_ARM = 30; // In cm // Todo: FIXXXX!
const int VEHICLE_MASS = 200; // In grams // Todo: FIXXXX!

const float TVC_Y_P_GAIN = 0.3; // Todo: FIXXXX!
const float TVC_Y_I_GAIN = 0; // Todo: FIXXXX!
const float TVC_Y_D_GAIN = 0.1; // Todo: FIXXXX!

const float TVC_Z_P_GAIN = 0.3; // Todo: FIXXXX!
const float TVC_Z_I_GAIN = 0; // Todo: FIXXXX!
const float TVC_Z_D_GAIN = 0.1; // Todo: FIXXXX!

const float MAX_Y_TVC_CORRECTION_ANGLE = 5.0;
const float MAX_Z_TVC_CORRECTION_ANGLE = 5.0;

const uint8_t TVC_Y_SERVO_ZERO_ANGLE = 90;
const float TVC_Y_SERVO_SCALE_FACTOR = 1.0;
const uint8_t TVC_Z_SERVO_ZERO_ANGLE = 90;
const float TVC_Z_SERVO_SCALE_FACTOR = 1.0;

const uint8_t FAIRING_SERVO_CLOSED_ANGLE = 90;
const uint8_t FAIRING_SERVO_OPEN_ANGLE = 0;

const uint8_t BAT_V_DIV_PIN = A6;
const unsigned int BAT_V_DIV_R1 = 300000;
const unsigned int BAT_V_DIV_R2 = 100000;
const float BAT_V_DIV_REF_V = 3;

const uint8_t LORA_CS_PIN = A4;
const uint8_t LORA_INT_PIN = 8;
const uint8_t LORA_RST_PIN = A1;

const uint8_t SD_CS_PIN = A5;

const int TELEM_TRANSMIT_RATE = 5; // In hertz
const int IDLE_LOGGING_RATE = 5; // Logging rate when vehicle is on the launch pad, In hertz
const int ACTIVE_LOGGING_RATE = 60; // Logging rate when vehicle is in flight, in hertz

const float ABORT_TILT = 30; // The angle off vertical at which the rocket will call an abort at 

VehicleState state;
StatusLight statusLight(STATUS_LIGHT_PIN, &state);
VehiclePosition vehiclePos(PRESSURE_ADDR);
VehicleOrientation vehicleOri(ACCEL_ADDR, GYRO_ADDR, MAG_ADDR);
MovingAverage gyroXMovingAvg(LANDING_GYRO_READINGS);
MovingAverage gyroYMovingAvg(LANDING_GYRO_READINGS);
MovingAverage gyroZMovingAvg(LANDING_GYRO_READINGS);
PID tvcY(TVC_Y_P_GAIN, TVC_Y_I_GAIN, TVC_Y_D_GAIN);
PID tvcZ(TVC_Z_P_GAIN, TVC_Z_I_GAIN, TVC_Z_D_GAIN);
TVCMount tvcMount(SERVO_PINS[0], SERVO_PINS[1]);
Fairing fairing(SERVO_PINS[2]);
VoltageDivider battery(BAT_V_DIV_PIN, BAT_V_DIV_R1, BAT_V_DIV_R2, BAT_V_DIV_REF_V);
Telemetry telem(LORA_CS_PIN, LORA_INT_PIN, LORA_RST_PIN);
Logging logging(SD_CS_PIN);

unsigned long preLoopTime;

unsigned long stateChangeStartTime;
unsigned int lastAltitudeReading;
unsigned long lastAltitudeReadingTime;
unsigned long lastGyroReadingTime;

Position pos;
Orientation ori;

double motorForce;
double tvcYTorque;
double tvcZTorque;
double tvcYAngle;
double tvcZAngle;

uint32_t logNum;

bool manuallyAborted;

union SentTelemetry {
  struct {
    unsigned long vehicleOnTime;
    unsigned long flightTime;
    uint8_t vehicleState;
    float batteryVoltage;
    float accelX;
    float accelY;
    float accelZ;
    float gyroX;
    float gyroY;
    float gyroZ;
    float altitude;
    float temp;
    float oriAngleX;
    float oriAngleY;
    float oriAngleZ;
    uint8_t heading;
    float tvcYTorque;
    float tvcZTorque;
    float tvcYAngle;
    float tvcZAngle;
  } telem;
  uint8_t serializedTelem[150]; // Approx size of the struct
} sentTelem;

union ReceivedTelemetry {
  struct {
    boolean manuallyAborted;
  } telem;
  uint8_t serializedTelem[Telemetry::MAX_LEN];
} receivedTelem;

const char *FLIGHT_LOG_HEADERS[] = {
  "vehicle_on_time",
  "flight_time",
  "state",
  "batt_voltage",
  "accel_x",
  "accel_y",
  "accel_z",
  "gyro_x",
  "gyro_y",
  "gyro_z",
  "mag_x",
  "mag_y",
  "mag_z",
  "raw_alt",
  "alt",
  "max_alt",
  "pressure",
  "temp",
  "delta_t",
  "ori_angle_x",
  "ori_angle_y",
  "ori_angle_z",
  "heading",
  "motor_force",
  "tvc_y_torque",
  "tvc_z_torque",
  "tvc_y_angle",
  "tvc_z_angle"
};

union FlightLog {
  struct {
    // Several functions look at the first byte of a page to check if the page has been written to
    // If a page hasn't been written to the first byte will be 0xFF, this makes sure this won't be the case
    // if the page has been written to, by setting it to 0x00
    uint8_t writeCheck; 
    
    unsigned long vehicleOnTime;
    unsigned long flightTime;
    uint8_t vehicleState;
    float batteryVoltage;
    float accelX;
    float accelY;
    float accelZ;
    float gyroX;
    float gyroY;
    float gyroZ;
    float magX;
    float magY;
    float magZ;
    float pressure;
    float temp;
    float rawAltitude;
    float altitude;
    float maxAltitude;
    int deltaT;
    float oriAngleX;
    float oriAngleY;
    float oriAngleZ;
    uint8_t heading;
    float motorForce;
    float tvcYTorque;
    float tvcZTorque;
    float tvcYAngle;
    float tvcZAngle;
  } log;
  uint8_t serializedLog[256]; // One flash page
} flightLog;

bool transferLogsToSd() {
  Logging::SdLog sdLog;
  if (!logging.createSdLog(sdLog)) {
    return false;
  }

  unsigned int headerCount = sizeof(FLIGHT_LOG_HEADERS) / sizeof(FLIGHT_LOG_HEADERS[0]);
  for (unsigned int i = 0; i < headerCount; i++) {
    if (i == 0) {
      sdLog.print(FLIGHT_LOG_HEADERS[i]);
    } else {
      sdLog.printSep(FLIGHT_LOG_HEADERS[i]);
    }
  }

  sdLog.println();
  sdLog.flush();

  logNum = 0;
  while (true) {
    if (!logging.doesFlashLogExist(logNum)) break;

    logging.readFlashLog(logNum, flightLog.serializedLog);

    sdLog.print(flightLog.log.vehicleOnTime);
    sdLog.printSep(flightLog.log.flightTime);
    sdLog.printSep(flightLog.log.vehicleState);
    sdLog.printSep(flightLog.log.batteryVoltage);
    sdLog.printSep(flightLog.log.accelX);
    sdLog.printSep(flightLog.log.accelY);
    sdLog.printSep(flightLog.log.accelZ);
    sdLog.printSep(flightLog.log.gyroX);
    sdLog.printSep(flightLog.log.gyroY);
    sdLog.printSep(flightLog.log.gyroZ);
    sdLog.printSep(flightLog.log.magX);
    sdLog.printSep(flightLog.log.magY);
    sdLog.printSep(flightLog.log.magZ);
    sdLog.printSep(flightLog.log.pressure);
    sdLog.printSep(flightLog.log.temp);
    sdLog.printSep(flightLog.log.rawAltitude);
    sdLog.printSep(flightLog.log.altitude);
    sdLog.printSep(flightLog.log.maxAltitude);
    sdLog.printSep(flightLog.log.deltaT);
    sdLog.printSep(flightLog.log.oriAngleX);
    sdLog.printSep(flightLog.log.oriAngleY);
    sdLog.printSep(flightLog.log.oriAngleZ);
    sdLog.printSep(flightLog.log.heading);
    sdLog.printSep(flightLog.log.motorForce);
    sdLog.printSep(flightLog.log.tvcYTorque);
    sdLog.printSep(flightLog.log.tvcZTorque);
    sdLog.printSep(flightLog.log.tvcYAngle);;
    sdLog.printSep(flightLog.log.tvcZAngle);
    
    sdLog.println();
    sdLog.flush();

    logNum++;
  }
  logging.eraseWrittenSectors();

  if (!sdLog.close()) {
    return false;
  }

  return true;
}

void setup() {
  bool bootedNominally = true;

  Serial.begin(115200);
  while (!Serial);

  state.setState(VehicleState::BOOTING_UP);

  statusLight.begin();
  statusLight.updateLight();

  if (!vehiclePos.initSensors()) {
    bootedNominally = false;
    Serial.println("Failed to initialize vehicle position sensors");
  }

  if (!vehicleOri.initSensors()) {
    bootedNominally = false;
    Serial.println("Failed to initialize vehicle orientation sensors");
  }
  vehicleOri.calcGyroBias(CALCULATE_GYRO_BIAS);
  vehicleOri.enableAccelComp(USE_ACCEL_COMP);
  vehicleOri.enableMagComp(USE_MAG_COMP);
  
  if (!telem.initRadio()) {
    bootedNominally = false;
    Serial.println("Failed to initialize telemetry radio");
  }
  telem.setEncryptionKey(TELEM_ENCRYPT_KEY, sizeof(TELEM_ENCRYPT_KEY));
  telem.setTransmitRate(TELEM_TRANSMIT_RATE);

  if (!logging.initFlash()) {
    bootedNominally = false;
    Serial.println("Failed to initialize logging flash");
  }
  logging.setLoggingRate(IDLE_LOGGING_RATE);

  if (logging.doFlashLogsExist()) {
    state.setState(VehicleState::RECOVERING_LOG);
    statusLight.updateLight();

    Serial.println("Logs found on flash, recovering...");

    if (transferLogsToSd()) {
      state.setState(VehicleState::BOOTING_UP);
      statusLight.updateLight();
      
      Serial.println("Logs recovered and saved to SD!");
    } else {
      bootedNominally = false;
      Serial.println("Failed to recover logs from flash");
    }
  }

  tvcMount.setYServoProperties(TVC_Y_SERVO_ZERO_ANGLE, TVC_Y_SERVO_SCALE_FACTOR);
  tvcMount.setZServoProperties(TVC_Z_SERVO_ZERO_ANGLE, TVC_Z_SERVO_SCALE_FACTOR);
  tvcMount.begin();

  fairing.setServoProperties(FAIRING_SERVO_CLOSED_ANGLE, FAIRING_SERVO_OPEN_ANGLE);
  fairing.begin();
  fairing.reset();

  if (bootedNominally) {
    state.setState(VehicleState::PAD_IDLE);
    Serial.println("\nAll sensors initialized, computer booted nominally");
  } else {
    state.setState(VehicleState::BOOT_ERROR);
    Serial.println("\nFailed to initilise all sensors, computer failed to boot");
  }

  pinMode(A0, OUTPUT);
}

bool hasVehicleLanded() {
  if (millis() > lastGyroReadingTime + GYRO_READ_INTERVAL) {
    lastGyroReadingTime = millis();

    gyroXMovingAvg.update(ori.BMI088RawData.gyroX);
    gyroYMovingAvg.update(ori.BMI088RawData.gyroY);
    gyroZMovingAvg.update(ori.BMI088RawData.gyroZ);
  }
  
  // Convert degrees used in the constant to radians, since the gyros use radians
  const float gyroMaxStDev = LANDING_GYRO_MAX_ST_DEV * DEG_TO_RAD;

  // Just need to get the number of recored readings from one of the moving averages as they are all updated at the same time
  if (gyroXMovingAvg.getNumRecordedReadings() == LANDING_GYRO_READINGS) { 
    if (gyroXMovingAvg.getStDev() < gyroMaxStDev && gyroYMovingAvg.getStDev() < gyroMaxStDev && gyroZMovingAvg.getStDev() < gyroMaxStDev) {
      return true;
    }
  }

  return false;
}

unsigned long lastUpdate;

void loop() {
  statusLight.updateLight();
  
  if (state.getState() == VehicleState::BOOT_ERROR) return;

  vehiclePos.getPosition(pos);
  vehicleOri.getOrientation(ori);

  switch (state.getState()) {
    case VehicleState::PAD_IDLE: {
      if (-ori.BMI088RawData.accelX >= LAUNCH_ACCEL_THRESH) {
        if (stateChangeStartTime == 0) {
          stateChangeStartTime = millis();
        }

        if (millis() > stateChangeStartTime + STATE_CHANGE_TIME) {
          state.setState(VehicleState::POWERED_ASCENT);

          // This is required, in case the conditions to change to the next state are already true
          stateChangeStartTime = 0; 
        }
      } else {
        stateChangeStartTime = 0;
      }

      tvcMount.center();

      break;
    } case VehicleState::POWERED_ASCENT: {
      if (-ori.BMI088RawData.accelX <= BURNOUT_ACCEL_THRESH) {
        if (stateChangeStartTime == 0) {
          stateChangeStartTime = millis();
        }

        if (millis() > stateChangeStartTime + STATE_CHANGE_TIME) {
          state.setState(VehicleState::UNPOWERED_ASCENT);

          // This is required, in case the conditions to change to the next state are already true
          stateChangeStartTime = 0; 
        }
      } else {
        stateChangeStartTime = 0;
      }

      vehicleOri.enableAccelComp(false);

      // Calculate force of motor; F = m * a
      motorForce = VEHICLE_MASS * ori.BMI088RawData.accelX;

      // Calculate correctional torque using a PID controller and the vehicle's rotation off vertical 
      tvcYTorque = tvcY.update(ori.absolute.y());
      tvcZTorque = tvcZ.update(ori.absolute.x()); // x and z are flipped, see Vector.h for more details

      // Solve for the angle of the correctional torque, given the torque, current motor force, and vehicle moment arm
      // θ = sin-1(torque / (force * moment arm))
      tvcYAngle = asin(tvcYTorque / (motorForce * VEHICLE_MOMENT_ARM));
      tvcZAngle = asin(tvcZTorque / (motorForce * VEHICLE_MOMENT_ARM));

      // Convert the TVC mount angle to degrees, as the TVC system uses degrees and the navigation system uses radians
      tvcYAngle = tvcYAngle * RAD_TO_DEG;
      tvcZAngle = tvcZAngle * RAD_TO_DEG;

      // Subtract the angle from 180, as the angle returned from the previous equation is between the moment arm and the force, and we need the supplement of that
      tvcYAngle = 180 - tvcYAngle;
      tvcZAngle = 180 - tvcZAngle;

      // Make sure correction angles are not bigger than their maximum values
      if (tvcYAngle > MAX_Y_TVC_CORRECTION_ANGLE) {
        tvcYAngle = MAX_Y_TVC_CORRECTION_ANGLE;
      }
      if (tvcZAngle > MAX_Z_TVC_CORRECTION_ANGLE) {
        tvcZAngle = MAX_Z_TVC_CORRECTION_ANGLE;
      }

      tvcMount.setAngle(tvcYAngle, tvcZAngle);

      logging.setLoggingRate(ACTIVE_LOGGING_RATE);

      break;
    } case VehicleState::UNPOWERED_ASCENT: {
      if (millis() > lastAltitudeReadingTime + ALTITUDE_READ_INTERVAL) {
        if (lastAltitudeReading != 0 && pos.altitude <= lastAltitudeReading + APOGEE_DETECT_ALTITUDE_DIFFERENCE) { // APOGEE_DETECT_ALTITUDE_DIFFERENCE is negative, so addition is used
          state.setState(VehicleState::BALLISTIC_DESCENT);
        }

        lastAltitudeReading = pos.altitude;
        lastAltitudeReadingTime = millis();
      }

      // Reset vars related to TVC, so arbitrary values are not passed to telem/logging
      motorForce = 0;
      tvcYTorque = 0;
      tvcZTorque = 0;
      tvcYAngle = 0;
      tvcZAngle = 0;

      tvcMount.center();

      break;
    } case VehicleState::BALLISTIC_DESCENT: {
      if (pos.altitude <= CHUTE_DEPLOY_ALTITUDE) {
        state.setState(VehicleState::CHUTE_DESCENT);
      }

      break;
    } case VehicleState::ABORT:
      case VehicleState::CHUTE_DESCENT: {
      if (hasVehicleLanded()) {
        state.setState(VehicleState::LANDED_DATA_TRANSFER);
      }

      fairing.deploy();

      break;
    } case VehicleState::LANDED_DATA_TRANSFER: {
      transferLogsToSd();
      logging.setLoggingRate(0);

      state.setState(VehicleState::LANDED_IDLE);

      break;
    } case VehicleState::LANDED_IDLE: {
      break;
    }
  }

  float batteryVoltage = battery.getVoltage();

  if (telem.willTransmitTelemetry()) {
    sentTelem.telem.vehicleOnTime = state.getVehicleOnTime();
    sentTelem.telem.flightTime = state.getFlightTime();
    sentTelem.telem.vehicleState = state.getState();
    sentTelem.telem.batteryVoltage = batteryVoltage;
    sentTelem.telem.accelX = ori.BMI088RawData.accelX;
    sentTelem.telem.accelY = ori.BMI088RawData.accelY;
    sentTelem.telem.accelZ = ori.BMI088RawData.accelZ;
    sentTelem.telem.gyroX = ori.BMI088RawData.gyroX;
    sentTelem.telem.gyroY = ori.BMI088RawData.gyroY;
    sentTelem.telem.gyroZ = ori.BMI088RawData.gyroZ;
    sentTelem.telem.altitude = pos.altitude;
    sentTelem.telem.temp = pos.BME280RawData.temperature;

    // Don't need extra precision of double in telem
    // x and z are flipped, see Vector.h for more details
    sentTelem.telem.oriAngleX = ori.absolute.z() * RAD_TO_DEG; 
    sentTelem.telem.oriAngleY = ori.absolute.y() * RAD_TO_DEG;
    sentTelem.telem.oriAngleZ = ori.absolute.x() * RAD_TO_DEG;
    
    sentTelem.telem.heading = ori.heading;
    sentTelem.telem.tvcYTorque = tvcYTorque;
    sentTelem.telem.tvcZTorque = tvcZTorque;
    sentTelem.telem.tvcYAngle = tvcYAngle;
    sentTelem.telem.tvcZAngle = tvcZAngle;
    telem.sendTelemetry(sentTelem.serializedTelem, sizeof(sentTelem.serializedTelem));
  }
  telem.receiveTelemetry(receivedTelem.serializedTelem, sizeof(sentTelem.serializedTelem));

  if (logging.willCreateLog()) {
    flightLog.log.vehicleOnTime = state.getVehicleOnTime();
    flightLog.log.flightTime = state.getFlightTime();
    flightLog.log.vehicleState = state.getState();
    flightLog.log.batteryVoltage = batteryVoltage;
    flightLog.log.accelX = ori.BMI088RawData.accelX;
    flightLog.log.accelY = ori.BMI088RawData.accelY;
    flightLog.log.accelZ =ori.BMI088RawData.accelZ;
    flightLog.log.gyroX = ori.BMI088RawData.gyroX;
    flightLog.log.gyroY = ori.BMI088RawData.gyroY;
    flightLog.log.gyroZ = ori.BMI088RawData.gyroZ;
    flightLog.log.magX = ori.MLX90393RawData.magX;
    flightLog.log.magY = ori.MLX90393RawData.magY;
    flightLog.log.magZ = ori.MLX90393RawData.magZ;
    flightLog.log.pressure = pos.BME280RawData.pressure;
    flightLog.log.temp = pos.BME280RawData.temperature;
    flightLog.log.rawAltitude = pos.rawAltitude;
    flightLog.log.altitude = pos.altitude;
    //flightLog.log.maxAltitude;
    flightLog.log.deltaT = millis() - preLoopTime;

    // x and z are flipped, see Vector.h for more details
    flightLog.log.oriAngleX = ori.absolute.z() * RAD_TO_DEG;
    flightLog.log.oriAngleY = ori.absolute.y() * RAD_TO_DEG;
    flightLog.log.oriAngleZ = ori.absolute.x() * RAD_TO_DEG;

    flightLog.log.heading = ori.heading;
    flightLog.log.motorForce = motorForce;
    flightLog.log.tvcYTorque = tvcYTorque;
    flightLog.log.tvcZTorque = tvcZTorque;
    flightLog.log.tvcYAngle = tvcYAngle;
    flightLog.log.tvcZAngle = tvcZAngle;

    logging.writeFlashLog(logNum++, flightLog.serializedLog);
  }

  // For debug
  if (digitalRead(A0) == HIGH && state.getState() != VehicleState::LANDED_IDLE) {
    while(digitalRead(A0) == HIGH);
    Serial.println("Transferring logs to SD...");

    state.setState(VehicleState::LANDED_DATA_TRANSFER);
  }

  // For debug
  if (millis() > lastUpdate + 1100) {
    lastUpdate = millis();
    Serial.println("Loop time (ms):");
    Serial.println(millis() - preLoopTime);
    Serial.print("ori x:"); Serial.println(ori.absolute.x() * RAD_TO_DEG);
    Serial.print("ori y:"); Serial.println(ori.absolute.y() * RAD_TO_DEG);
    Serial.print("ori z:"); Serial.println(ori.absolute.z() * RAD_TO_DEG);
    Serial.print("accel x:"); Serial.println(ori.BMI088RawData.accelX);
    Serial.print("accel y:"); Serial.println(ori.BMI088RawData.accelY);
    Serial.print("accel z:"); Serial.println(ori.BMI088RawData.accelZ);
    Serial.print("mag x:"); Serial.println(ori.MLX90393RawData.magX);
    Serial.print("mag y:"); Serial.println(ori.MLX90393RawData.magY);
    Serial.print("mag z:"); Serial.println(ori.MLX90393RawData.magZ);
  }
  preLoopTime = millis();

  switch (state.getState()) {
    case VehicleState::POWERED_ASCENT:
    case VehicleState::UNPOWERED_ASCENT:
      if (ori.absolute.y() > ABORT_TILT || ori.absolute.z() > ABORT_TILT) {
        state.setState(VehicleState::ABORT);
      }

    case VehicleState::BALLISTIC_DESCENT:
      if (receivedTelem.telem.manuallyAborted) {
        state.setState(VehicleState::ABORT);
      }

      break;
  }
}