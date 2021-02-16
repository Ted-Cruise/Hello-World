#include <StatusLight.h>

StatusLight::StatusLight(uint8_t lightPin, VehicleState *state) {
  _statusLight = new Adafruit_NeoPixel_ZeroDMA(1, lightPin, NEO_GRB);
  _state = state;
}

void StatusLight::begin() {
  _statusLight->begin(); // Only returns false if not SAMD51, so return value not checked
  _statusLight->show();
}
    
void StatusLight::updateLight() {
  switch(_state->getState()) {
    case VehicleState::BOOTING_UP:
      setLightColor(BOOTING_COLOR);
      break;
    case VehicleState::RECOVERING_LOG:
    case VehicleState::LANDED_DATA_TRANSFER:
      setLightColor(LOADING_COLOR);
      break;
    case VehicleState::BOOT_ERROR:
    case VehicleState::ABORT:
      setLightColor(ERROR_COLOR);
      break;
    case VehicleState::PAD_IDLE:
    case VehicleState::LANDED_IDLE:
      setLightColor(IDLE_COLOR);
      break;
    case VehicleState::POWERED_ASCENT:
      setLightColor(POWERED_ASCENT_COLOR);
      break;
    case VehicleState::UNPOWERED_ASCENT:
      setLightColor(UNPOWERED_ASCENT_COLOR);
      break;
    case VehicleState::BALLISTIC_DESCENT:
      setLightColor(BALLISTIC_DESCENT_COLOR);
      break;
    case VehicleState::CHUTE_DESCENT:
      setLightColor(CHUTE_DESCENT_COLOR);
      break;
  }
}

void StatusLight::setLightColor(uint32_t color) {
  if (millis() % WARNING_STROBE_DELAY > WARNING_STROBE_DELAY - WARNING_STROBE_LENGTH) {
    _statusLight->setPixelColor(0, WARNING_STROBE_COLOR);
    _statusLight->setBrightness(WARNING_STROBE_BRIGHTNESS);
  } else {
    _statusLight->setPixelColor(0, color);
    _statusLight->setBrightness(STATUS_LIGHT_BRIGHTNESS);
  }
  _statusLight->show();
}
