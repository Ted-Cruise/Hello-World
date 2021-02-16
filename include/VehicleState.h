#ifndef VEHICLESTATE_H
#define VEHICLESTATE_H

#include <Arduino.h>

class VehicleState {
  public:
    enum State {
      BOOTING_UP,
      RECOVERING_LOG,
      BOOT_ERROR,
      PAD_IDLE,
      POWERED_ASCENT,
      UNPOWERED_ASCENT,
      BALLISTIC_DESCENT,
      CHUTE_DESCENT,
      ABORT,
      LANDED_DATA_TRANSFER,
      LANDED_IDLE
    };
    
    void setState(State state);
    State getState();
    
    unsigned long getVehicleOnTime();
    unsigned long getFlightTime();
    
  private:
    State _state;
    
    unsigned long _initFlightTime;
    unsigned long _finalFlightTime;
};

#endif