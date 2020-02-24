#pragma once

/**
 * File to encapsulate functions that interface with receiver interrupt pins
 */

#include "Config.hpp"

namespace RECEIVER {
  
  // Receiver channels
  struct Data {
    unsigned int recv1, recv2, recv3;
  };

  // Anonymous namespace to protect data (make private)
  namespace {
    Data data;
    
    // elapsedMicros is a Teensy type that automatically measures duration in ms precision
    elapsedMicros time_elapsed_us[sizeof(RECEIVERS)];  

    // Resulting pulse widths; volatile because memory accessed in interrupt and can't be optimized out
    volatile unsigned int measured_pwm[sizeof(RECEIVERS)];

    void measure_pulses(unsigned int pin_index) {
      // Read pin state. 
      //    If the pin is high, pulse has started and the timer is reset.
      //    If the pin is low, the pulse has ended and the resulting time difference can be calculated for the pulse
      if(digitalReadFast(RECEIVERS[pin_index])) {
          time_elapsed_us[pin_index] = 0;
      } else {
          // Because the time elapsed is set to zero on every HIGH, the result will be the value stored in time_elapsed which is automatically incremented;
          measured_pwm[pin_index] = time_elapsed_us[pin_index];
      }
    }

    // Interrupt service routine to be called on change of the first receiver pin
    void measure_recv1(void) {
        measure_pulses(0);
    }
    
    // Interrupt service routine to be called on change of the second receiver pin
    void measure_recv2(void) {
        measure_pulses(1);
    }

    // Interrupt service routine to be called on change of the third receiver pin
    void measure_recv3(void) {
        measure_pulses(2);
    }
  }
  
  /**
   * @brief Attach all interrupt pins
   * 
   */
  void start_interrupts(void) {
    
    attachInterrupt(digitalPinToInterrupt(RECEIVERS[0]), measure_recv1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RECEIVERS[1]), measure_recv2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RECEIVERS[2]), measure_recv3, CHANGE);

  }
  
  /**
   * @brief Detach all interrupt pins to minimize CPU impact when not needed
   * 
   */
  void terminate_interrupts(void) {

    for(int i = 0; i < sizeof(RECEIVERS); i++) {
      detachInterrupt(digitalPinToInterrupt(RECEIVERS[i]));
    }

  }

  /**
   * @brief Initialize receiver pins
   * 
   */
  inline void init() {

    for(int i = 0; i < sizeof(RECEIVERS); i++) {
      pinMode(RECEIVERS[i], INPUT);
    }

  }

  /**
   * @brief Read receiver channel data
   * 
   * @return Data receiver pulse widths
=   */
  inline Data read() {
     data.recv1 = measured_pwm[0];
     data.recv2 = measured_pwm[1];
     data.recv3 = measured_pwm[2];

     return data;
  }

  /**
   * @brief Print formatted receiver data
   * 
   * @param data receiver data
   */
  inline void print(const Data& data) {
    DEBUG_PRINT(" Channel 1: "); DEBUG_PRINT(data.recv1);
    DEBUG_PRINT("\t Channel 2: "); DEBUG_PRINT(data.recv2);
    DEBUG_PRINT("\t Channel 3: "); DEBUG_PRINT(data.recv3);
    DEBUG_PRINTLN("");
  }
}
