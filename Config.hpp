#pragma once

/**
 * File for containing all configuration variables such as pins, constants
 */
 
#include "Arduino.h"
#include "src/aero-cpp-lib/include/Data.hpp"

using Pin = unsigned int;

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
static constexpr String NAMES[] = {"GND", "PLANE", "G1", "G2"};

/* CALIBRATION FLAGS */
static constexpr bool CALIBRATE_ACCEL = false;
static constexpr bool CALIBRATE_GYRO = false;
static constexpr bool CALIBRATE_MAG = false;
static constexpr bool CALIBRATE_BARO = false;

/* BOOT MODE */ 
enum class BOOT_MODE {LED_TEST, LED_TEST_RANDOM};
static constexpr BOOT_MODE MODE = BOOT_MODE::LED_TEST_RANDOM;

/* PID */
static constexpr double KP = 1, KI = 0, KD = 0;

static constexpr double DEFAULT_ROLL_SETPOINT = 0;
static constexpr double ROLL_MIN_LIMIT = -90;
static constexpr double ROLL_MAX_LIMIT = 90;

static constexpr double DEFAULT_PITCH_SETPOINT = 0;
static constexpr double PITCH_MIN_LIMIT = -90;
static constexpr double PITCH_MAX_LIMIT = 90;

static constexpr double DEFAULT_YAW_SETPOINT = 0;
static constexpr double YAW_MIN_LIMIT = -90;
static constexpr double YAW_MAX_LIMIT = 90;

/* SYSTEM FLAGS */
static constexpr bool DEBUG_MODE = true;  // Block usage of Serial printing if not debugging to speed up system

/* LEDS */
static constexpr Pin LED1 = 8;
static constexpr Pin LED2 = 7;
static constexpr Pin LED3 = 6;
static constexpr Pin LED4 = 5;
static constexpr unsigned int LED_COUNT = 4;
static constexpr Pin LED_PINS[LED_COUNT] = {LED1, LED2, LED3, LED4};

static constexpr Pin POWER_LED = LED1;
static constexpr Pin ACTIVITY_LED = LED2;
static constexpr Pin RADIO_LED = LED3;
static constexpr Pin MODE_LED = LED4;


/* SERVOS */
static constexpr Pin SERVO1 = 20;
static constexpr Pin SERVO2 = 21;
static constexpr Pin SERVOS[] = {SERVO1, SERVO2};

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
    
/* GPS */
static constexpr HardwareSerial *GPS_PORT = &Serial2;
static constexpr Pin GPS_FIX =  4;

const aero::def::ID THIS_DEVICE = aero::def::ID::G1;
