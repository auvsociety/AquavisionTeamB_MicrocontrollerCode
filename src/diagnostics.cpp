#include "diagnostics.hpp"

const int16_t led_bitmask[9] = {1, 2, 4, 8, 16, 32, 64, 128, 256};

const uint8_t led_pinmap[9] = {PB12, PB13, PB14, PB15, PA5, PA4, PA15, PB4, PB5};

int led_status[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
void initializeLED() {
  for (int pin_index = 0; pin_index < 9; pin_index++) {
    pinMode(led_pinmap[pin_index], OUTPUT);
    digitalWrite(led_pinmap[pin_index], led_status[pin_index]);
  }
}

void setLED(int16_t led_indicator) {
  for (int pin_index = 0; pin_index < 9; pin_index++) {
    if ((led_indicator & led_bitmask[pin_index]) == led_bitmask[pin_index]){
      for(int i = 0; i < 9; i++){
        digitalWrite(led_pinmap[i], HIGH);
      }
      digitalWrite(led_pinmap[pin_index], LOW);
      nh.loginfo("Led Glowing.");
    }
  }
}