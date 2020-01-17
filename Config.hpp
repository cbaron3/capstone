#pragma once

#include "Arduino.h"

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
static constexpr Pin LEDS[LED_COUNT] = {LED1, LED2, LED3, LED4};


/* SERVOS */
static constexpr Pin SERVO1 = 20;
static constexpr Pin SERVO2 = 21;
static constexpr Pin SERVOS[] = {20, 21};

/* RECEIVER INPUTS */
static constexpr Pin RECEIVER1 = 22;
static constexpr Pin RECEIVER2 = 23;
static constexpr Pin RECEIVERS[] = {22, 23};

/* RADIO */
// Uses the only SPI bus available
static constexpr Pin RADIO_CS = 14;
static constexpr Pin RADIO_RESET = 15;
static constexpr Pin RADIO_INT = 16;

/* GPS */
static constexpr HardwareSerial *GPS_PORT = &Serial2;
static constexpr Pin GPS_FIX =  4;
