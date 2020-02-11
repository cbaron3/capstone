#pragma once

/**
 * File for containing all configuration variables such as pins, constants
 */
 
#include "Arduino.h"
#include "src/aero-cpp-lib/include/Data.hpp"

using Pin = unsigned int;

#define  DEBUG

#ifdef DEBUG
    #define DEBUG_PRINT(x) Serial.print(x)
    #define DEBUG_PRINTLN(x) Serial.println(x)
    #define DEBUG_START(x) Serial.begin(x); delay(1000)
#else
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINTLN(x)
    #define DEBUG_START(x)
#endif

/* SAMPLE TIME */
static constexpr unsigned long SAMPLE_TIME_MS = 20;

/* DEVICE NAMES */
const String NAMES[] = {"GND", "PLANE", "G1", "G2"};

/* CALIBRATION FLAGS */
static constexpr bool CALIBRATE_ACCEL = false;
static constexpr bool CALIBRATE_GYRO = false;
static constexpr bool CALIBRATE_MAG = false;
static constexpr bool CALIBRATE_BARO = false;

/* BOOT MODE */ 
enum class BOOT_MODE {LED_TEST, LED_TEST_RANDOM};
static constexpr BOOT_MODE MODE = BOOT_MODE::LED_TEST_RANDOM;

/* PID */
static constexpr double ROLL_KP = 1.0f, ROLL_KI = 0.00f, ROLL_KD = 0.0f;
static constexpr double PITCH_KP = 10.0f, PITCH_KI = 1.00f, PITCH_KD = 1.0f;

static constexpr double DEFAULT_ROLL_SETPOINT = 0;
static constexpr double ROLL_MIN_LIMIT = -90;
static constexpr double ROLL_MAX_LIMIT = 90;

static constexpr double DEFAULT_PITCH_SETPOINT = 0;
static constexpr double PITCH_MIN_LIMIT = -90;
static constexpr double PITCH_MAX_LIMIT = 90;

static constexpr double DEFAULT_YAW_SETPOINT = 0;
static constexpr double YAW_MIN_LIMIT = -90;
static constexpr double YAW_MAX_LIMIT = 90;

/* BAROMETER */
static constexpr double ALTITUDE_BIAS = 250.0f; // In metres

/* SYSTEM FLAGS */
static constexpr bool DEBUG_MODE = true;  // Block usage of Serial printing if not debugging to speed up system

/* LEDS */
static constexpr Pin LED1 = 8;
static constexpr Pin LED2 = 7;
static constexpr Pin LED3 = 6;
static constexpr Pin LED4 = 5;
static constexpr unsigned int LED_COUNT = 4;
static constexpr Pin LED_PINS[LED_COUNT] = {LED1, LED2, LED3, LED4};

/* FROM LEFT TO RIGHT */
static constexpr Pin POWER_LED = LED1;    // On when board is powered
static constexpr Pin ACTIVITY_LED = LED2; // Toggles on and off to signal that the CPU is running
static constexpr Pin RADIO_LED = LED3;    // Turns on when new message is received
static constexpr Pin MODE_LED = LED4;     // On when in manual mode, else off

/* SERVOS */
static constexpr Pin SERVO1 = 20;
static constexpr Pin SERVO2 = 21;
static constexpr Pin SERVOS[] = {SERVO1, SERVO2};

static constexpr float SERVO_MIN_ANGLE = 0.0f;
static constexpr float SERVO_MAX_ANGLE = 179.0f;
//static constexpr float SERVO_MIN_MS = 800.0f;
//static constexpr float SERVO_MAX_MS = 2000.0f;

static constexpr float SERVO_MIN_MS = 900.0f;
static constexpr float SERVO_MAX_MS = 2100.0f;


// For glider
//static constexpr int DEFAULT_LEFT_ELEVON_ANGLE = 90;
//static constexpr int DEFAULT_RIGHT_ELEVON_ANGLE = 90;

// For fixed wing trainer
static constexpr int DEFAULT_LEFT_ELEVON_ANGLE = 90;
static constexpr int DEFAULT_RIGHT_ELEVON_ANGLE = 90;

/* RECEIVER INPUTS */
static constexpr Pin RECEIVER1 = 22;  // Left elevon
static constexpr Pin RECEIVER2 = 23;  // Right elevon
static constexpr Pin RECEIVERS[] = {RECEIVER1, RECEIVER2};

/* RADIO */
static constexpr Pin RADIO_CS = 14;
static constexpr Pin RADIO_RESET = 15;
static constexpr Pin RADIO_INT = 16;
static constexpr float RADIO_FREQ = 915.0f;
static constexpr int RADIO_POWER = 23;
static constexpr aero::def::ID THIS_DEVICE = aero::def::ID::G1;

template <class X, class M, class N, class O, class Q>
inline X map_generic(X x, M in_min, N in_max, O out_min, Q out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/* GPS */
static constexpr HardwareSerial *GPS_PORT = &Serial2;
static constexpr Pin GPS_FIX =  4;
