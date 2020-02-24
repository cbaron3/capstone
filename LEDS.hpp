#pragma once

/**
 * File to encapsulate functions that interface with LEDs
 */

#include "Config.hpp"

/**
 * LED namespace; encapsulates all related functions
 */
namespace LEDS {

  /**
   * @brief Initialize all LEDs as outputs
   * 
   */
  inline void init(){
    for(unsigned int i = 0; i < sizeof(LED_PINS); i++) {
        pinMode(LED_PINS[i], OUTPUT);
    }
  }

  /**
   * @brief Turn on led
   * 
   * @param led pin of led to turn on
   */
  inline void on(Pin led) {
    digitalWriteFast(led, HIGH);
  }

  /**
   * @brief Turn off led
   * 
   * @param led pin of led to turn off
   */
  inline void off(Pin led) {
    digitalWriteFast(led, LOW);
  }

  /**
   * @brief Sweep LEDs
   * 
   * @param delay_time how fast leds will blink; in milliseconds 
   */
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

  /**
   * @brief Turn on LEDs in an IN/OUT fashion
   * 
   * @param delay_time how fast leds will blink; in milliseconds 
   */
  inline void inout(int delay_time) {
    for(unsigned int i = 0; i < LED_COUNT/2; i++) {
        digitalWrite(LED_PINS[i], HIGH);
        digitalWrite(LED_PINS[LED_COUNT-i], HIGH);
        delay(delay_time);
        digitalWrite(LED_PINS[i], LOW);
        digitalWrite(LED_PINS[LED_COUNT-i], LOW);
    }
  }

  /**
   * @brief Flash all leds
   * 
   * @param delay_time how fast leds will blink; in milliseconds 
   */
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
