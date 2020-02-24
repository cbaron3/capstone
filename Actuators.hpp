#pragma once

/**
 * File to encapsulate functions that interface with the servos
 * Makes use of the Arduino Servo library 
 */
#include <Servo.h>

/**
 * Actuator namespace; encapsulates all related functions
 */
namespace ACTUATORS {

  /**
   * @brief Initializes each servo object by attaching it to the appropriate pin
   * 
   * @param left_elevon servo object reference representing the left elevon
   * @param right_elevon servo object reference representing the right elevon
   * @param rudder servo object reference representing the rudder
   */
  inline void init(Servo& left_elevon, Servo& right_elevon, Servo& rudder) {
    left_elevon.attach(SERVO1);
    right_elevon.attach(SERVO2);
    rudder.attach(SERVO3);
  }

  /**
   * @brief Function to test servo functionality by sweeping servos up and down
   * 
   * @param left left elevon servo
   * @param right right elevon servo
   */
  inline void sweep(Servo& left, Servo& right) {
    static int angle = 0;
    static int delta = 1;
    static bool left_flag = true;

    if(angle >= 180) {
      delta = -1;
    } else if(angle <= 0) {
      delta = 1;
    }

    if(left_flag) {
      left_flag = !left_flag;
      left.write(angle);
    } else {
      left_flag = !left_flag;
      right.write(angle);
    }
    
    angle += delta;
  }

  /**
   * @brief Command a servo to move using pulse width
   * 
   * @param motor object pertaining to which servo you want to move
   * @param ms width of pulse to send to servo
   */
  inline void command_ms(Servo& motor, unsigned long ms) {
    motor.writeMicroseconds(ms);
  }

  /**
   * @brief Command a servo to move using angle
   * 
   * @param motor object pertaining to which servo you want to move
   * @param deg angle to send to servo
   */
  inline void command_deg(Servo& motor, float deg) {
    motor.write(deg);
  }
}
