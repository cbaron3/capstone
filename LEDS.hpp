#pragma once

#include "Config.hpp"

namespace LEDS {
  inline void init(){
    for(unsigned int i = 0; i < LED_COUNT; i++) {
        pinMode(LED_PINS[i], OUTPUT);
    }
  }

  inline void on(Pin led) {
    digitalWriteFast(led, HIGH);
  }

  inline void off(Pin led) {
    digitalWriteFast(led, LOW);
  }

  inline void sweep(int delay_time) {
    for(unsigned int i = 0; i < LED_COUNT; i++) {
        digitalWrite(LED_PINS[i], HIGH);
        delay(delay_time);
        digitalWrite(LED_PINS[i], LOW);
    }
    
    for(int i = LED_COUNT - 2; i >= 0; i--) {
        digitalWrite(LED_PINS[i], HIGH);
        delay(delay_time);
        digitalWrite(LED_PINS[i], LOW);
    }
  }

  inline void inout(int delay_time) {
    for(unsigned int i = 0; i < LED_COUNT/2; i++) {
        digitalWrite(LED_PINS[i], HIGH);
        digitalWrite(LED_PINS[LED_COUNT-i], HIGH);
        delay(delay_time);
        digitalWrite(LED_PINS[i], LOW);
        digitalWrite(LED_PINS[LED_COUNT-i], LOW);
    }
  }

  inline void flash(int delay_time) {
    for(unsigned int i = 0; i < LED_COUNT; i++) {
        digitalWrite(LED_PINS[i], HIGH);    
    }

    delay(delay_time);
    
    for(unsigned int i = 0; i < LED_COUNT; i++) {
        digitalWrite(LED_PINS[i], LOW);
    }
  }
}
