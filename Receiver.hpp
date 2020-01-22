#pragma once

#include "Config.hpp"

namespace RECEIVER {
  struct ReceiverData {
    unsigned int recv1, recv2;
  };

  namespace {
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
  }
  
  void init_interrupts(void) {
    attachInterrupt(digitalPinToInterrupt(RECEIVERS[0]),measure_recv1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RECEIVERS[1]),measure_recv2, CHANGE);
  }
  
  void terminate_interrupts(void) {
    detachInterrupt(digitalPinToInterrupt(RECEIVERS[0]));
    detachInterrupt(digitalPinToInterrupt(RECEIVERS[1]));
  }

  inline void init() {
    pinMode(RECEIVERS[0],INPUT);
    pinMode(RECEIVERS[1],INPUT);
  }

  inline ReceiverData read_data() {
     ReceiverData data;

     data.recv1 = measured_pwm[0];
     data.recv2 = measured_pwm[1];

     return data;
  }

  inline void print_data(const ReceiverData& data) {
    Serial.print(" Channel 1: "); Serial.print(data.recv1);
    Serial.print("\t Channel 2: "); Serial.print(data.recv2);
    Serial.println("");
  }
}
