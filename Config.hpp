#pragma once

/**
 * File for containing all configuration variables such as pins, constants
 */
 
#include "Arduino.h"
#include "src/aero-cpp-lib/include/Data.hpp"

using Pin = unsigned int;

/* BOOT MODE */ 
enum class BOOT_MODE {LED_TEST, LED_TEST_RANDOM};
static constexpr BOOT_MODE MODE = BOOT_MODE::LED_TEST_RANDOM;

/* SYSTEM FLAGS */
static constexpr bool DEBUG_MODE = true;  // Block usage of Serial printing if not debugging to speed up system

/* LEDS */
static constexpr Pin LED1 = 8;
static constexpr Pin LED2 = 7;
static constexpr Pin LED3 = 6;
static constexpr Pin LED4 = 5;
static constexpr unsigned int LED_COUNT = 4;
static constexpr Pin LED_PINS[LED_COUNT] = {LED1, LED2, LED3, LED4};

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
