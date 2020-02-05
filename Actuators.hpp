#pragma once

#include <Servo.h>

namespace ACTUATORS {
  inline void init(Servo& left_elevon, Servo& right_elevon) {
    left_elevon.attach(SERVO1);
    right_elevon.attach(SERVO2);
  }

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
}
