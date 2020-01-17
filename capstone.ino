#include <Servo.h>
#include <RH_RF95.h>  // Note: Must be used as library

#include "src/Adafruit_MPL3115A2_Library/Adafruit_MPL3115A2.h"
#include "src/Arduino-PID-Library/PID_v1.h"
#include "src/MadgwickAHRS/MadgwickAHRS.h"
#include "src/MPU9250/MPU9250.h"
#include "src/TinyGPS/TinyGPS.h"

#include "Config.hpp"

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

void init_receivers(void) {
    pinMode(RECEIVERS[0],INPUT);
    pinMode(RECEIVERS[1],INPUT);
}

void init_interrupts(void) {
    attachInterrupt(digitalPinToInterrupt(RECEIVERS[0]),measure_recv1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RECEIVERS[1]),measure_recv2, CHANGE);
}

void terminate_interrupts(void) {
    detachInterrupt(digitalPinToInterrupt(RECEIVERS[0]));
    detachInterrupt(digitalPinToInterrupt(RECEIVERS[1]));
}

enum class OP_MODE {MANUAL_MODE, AUTO};
OP_MODE mode = OP_MODE::MANUAL_MODE;
OP_MODE last_mode = mode;

Servo left_elevon, right_elevon;

void init_servos(void) {
      left_elevon.attach(SERVO1);
      right_elevon.attach(SERVO2);
    }


void setup_leds(void) {
    for(int i = 0; i < LED_COUNT; i++) {
        pinMode(LEDS[i], OUTPUT);
    }
}


void loop_leds(int delay_time) {
    for(int i = 0; i < LED_COUNT; i++) {
        digitalWrite(LEDS[i], HIGH);
        delay(delay_time);
        digitalWrite(LEDS[i], LOW);
    }
    
    for(int i = LED_COUNT - 2; i > 0; i--) {
        digitalWrite(LEDS[i], HIGH);
        delay(delay_time);
        digitalWrite(LEDS[i], LOW);
    }
}

void setup() {
    if(DEBUG_MODE) {
        Serial.begin(115200);
        while(!Serial) {}
    }
    

    switch(MODE) {
        case BOOT_MODE::LED_TEST:
        case BOOT_MODE::LED_TEST_RANDOM: {
            setup_leds();
        } break;
    }
}


void loop() {
    switch(MODE) {
        case BOOT_MODE::LED_TEST: {
            loop_leds( 250 );
        } break;

        case BOOT_MODE::LED_TEST_RANDOM: {
            loop_leds( random(50, 250) );
        } break;
    }
}


// if(mode == OP_MODE::MANUAL) {
//         // If we enter manual mode and the last mode we were in is auto, reenable interrupts
//         if(last_mode != mode) {
//             last_mode = mode;
//             init_interrupts();
//         }

//         left_elevon.writeMicroseconds(measured_pwm[0]);
//         right_elevon.writeMicroseconds(measured_pwm[1]);

//     } else {
//         // If we enter automatic mode and the last mode we were in is manual, kill interrupts
//         if(last_mode != mode) {
//             last_mode = mode;
//             terminate_interrupts();
//         }
//     }
