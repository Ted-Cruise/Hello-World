#include <VehicleState.h>

void VehicleState::setState(VehicleState::State state) {
  _state = state;

  switch(_state) {
    case POWERED_ASCENT:
      _initFlightTime = millis();
      break;
    case LANDED_DATA_TRANSFER:
      _finalFlightTime = millis();
      break;
  }
}
    
VehicleState::State VehicleState::getState() {
  return _state;
}
    
unsigned long VehicleState::getVehicleOnTime() {
  return millis();
}
    
unsigned long VehicleState::getFlightTime() {
  if (_initFlightTime == 0) { // Vehicle not in flight yet
    return 0;
  } else if (_finalFlightTime == 0) {
    return millis() - _initFlightTime;
  } else {
    return _finalFlightTime - _initFlightTime;
  }
}