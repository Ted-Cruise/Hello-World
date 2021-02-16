#ifndef STATUSLIGHT_H
#define STATUSLIGHT_H

#include <Arduino.h>

#include <Adafruit_NeoPixel_ZeroDMA.h>
#include <VehicleState.h>

class StatusLight {
  public:
    StatusLight(uint8_t lightPin, VehicleState *state);

    void begin();
    
    void updateLight();
    
  private:
    constexpr static unsigned int WARNING_STROBE_DELAY = 2000; // Delay between warning strobe light flashes, in ms
    constexpr static unsigned int WARNING_STROBE_LENGTH = 50; // Length of the warning strobe light flashes, in ms

    constexpr static uint8_t WARNING_STROBE_BRIGHTNESS = 255;
    constexpr static uint8_t STATUS_LIGHT_BRIGHTNESS = 75;

    // Can't be declared as constants
    uint32_t WARNING_STROBE_COLOR = Adafruit_NeoPixel_ZeroDMA::Color(255, 255, 255); // White
    uint32_t BOOTING_COLOR = Adafruit_NeoPixel_ZeroDMA::Color(255, 255, 255); // White
    uint32_t ERROR_COLOR = Adafruit_NeoPixel_ZeroDMA::Color(255, 0, 0); // Red
    uint32_t IDLE_COLOR = Adafruit_NeoPixel_ZeroDMA::Color(0, 255, 0); // Green
    uint32_t POWERED_ASCENT_COLOR = Adafruit_NeoPixel_ZeroDMA::Color(255, 165, 0); // Orange
    uint32_t UNPOWERED_ASCENT_COLOR = Adafruit_NeoPixel_ZeroDMA::Color(106, 13, 173); // Purple
    uint32_t BALLISTIC_DESCENT_COLOR = Adafruit_NeoPixel_ZeroDMA::Color(173, 216, 230); // Light blue
    uint32_t CHUTE_DESCENT_COLOR = Adafruit_NeoPixel_ZeroDMA::Color(0, 0, 255); // Dark blue
    uint32_t LOADING_COLOR = Adafruit_NeoPixel_ZeroDMA::Color(128, 128, 0); // Yellow
    
    Adafruit_NeoPixel_ZeroDMA * _statusLight;
    VehicleState * _state;
    
    void setLightColor(uint32_t color);
};

#endif