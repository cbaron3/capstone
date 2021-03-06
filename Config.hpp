#pragma once

/**
 * File for containing all configuration variables such as pins, constants
 */
 
#include "Arduino.h"
#include "src/aero-cpp-lib/include/Data.hpp"


using Pin = unsigned int;

static constexpr float TARGET_LAT = 0.0f;
static constexpr float TARGET_LON = 0.0f;
static constexpr float TARGET_ALT = 0.0f;

enum mode_type {MODE_IDLE = 0, MODE_MANUAL = 1, MODE_AUTO = 2, MODE_SAFETY = 3};
static constexpr mode_type DEFAULT_MODE = MODE_AUTO;

// Commands
static constexpr uint8_t CMD_PITCH_UP_GLIDER1 = 0x00; // Glider 1 Emergency Pitch Up; Only if this device is G1.
static constexpr uint8_t CMD_PITCH_UP_GLIDER2 = 0x01; // Glider 2 Emergency Pitch Up; Only if this device is G2.
static constexpr uint8_t CMD_TOGGLE_ENGAGE    = 0x05;
static constexpr uint8_t CMD_COMMS_TEST       = 0x06; // Glider Communications Check. Can also be used to send back data.
static constexpr uint8_t CMD_MODE_SWAP        = 0x07; // Glider Mode Swap. Auto/Manual mode toggle.

static constexpr uint8_t STATE_GPS_FIX = 0x00; // Bit 0 for GPS fix
static constexpr uint8_t STATE_ENGAGED_MODE = 0x1E; // Bit 30 for engage mode
static constexpr uint8_t STATE_UNDEFINED_MODE = 0x1F; // Bit 31 for undefined mode

static constexpr float ALLOWABLE_AZIMUTH_ERROR = 3.0f;
static constexpr float ALLOWABLE_ELEVATION_ERROR = 3.0f;

#define  DEBUG
//#define  VERBOSE_DEBUG
//#define  TEST_TRAINER
#define FINAL_GLIDER

#ifdef DEBUG
    #define DEBUG_PRINT(x) Serial.print(x)
    #define DEBUG_PRINTLN(x) Serial.println(x)
    #define DEBUG_START(x) Serial.begin(x); delay(1000)
#else
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINTLN(x)
    #define DEBUG_START(x)
#endif

#if defined(FINAL_GLIDER)
    static constexpr unsigned long LEFT_ELEVON_MS_OFFSET = 0;
    static constexpr unsigned long RIGHT_ELEVON_MS_OFFSET = 0;

    static constexpr float SERVO_MIN_MS = 800.0f;
    static constexpr float SERVO_MAX_MS = 2000.0f;

#elif defined(TEST_GLIDER)
    static constexpr unsigned long LEFT_ELEVON_MS_OFFSET = 0;
    static constexpr unsigned long RIGHT_ELEVON_MS_OFFSET = 0;

    static constexpr float SERVO_MIN_MS = 800.0f;
    static constexpr float SERVO_MAX_MS = 2000.0f;

#elif defined(TEST_TRAINER)
    static constexpr unsigned long LEFT_ELEVON_MS_OFFSET = 40;   // Pitch
    static constexpr unsigned long RIGHT_ELEVON_MS_OFFSET = 50; // Roll

    static constexpr float SERVO_MIN_MS = 900.0f;
    static constexpr float SERVO_MAX_MS = 2100.0f;
#else 
    static constexpr unsigned long LEFT_ELEVON_MS_OFFSET = 0;
    static constexpr unsigned long RIGHT_ELEVON_MS_OFFSET = 0;

    static constexpr float SERVO_MIN_MS = 800.0f;
    static constexpr float SERVO_MAX_MS = 2000.0f;
#endif

/* DEVICE NAMES */
const String NAMES[] = {"GND", "PLANE", "G1", "G2"};

/* CALIBRATION FLAGS */
static constexpr bool CALIBRATE_ACCEL = false;
static constexpr bool CALIBRATE_GYRO = true;
static constexpr bool CALIBRATE_MAG = false;
static constexpr bool CALIBRATE_BARO = true;

/* BOOT MODE */ 
enum class BOOT_MODE {LED_TEST, LED_TEST_RANDOM};
static constexpr BOOT_MODE MODE = BOOT_MODE::LED_TEST_RANDOM;

/* PID */
static constexpr double ROLL_KP = 3.0f, ROLL_KI = 0.00f, ROLL_KD = 0.0f;
static constexpr double PITCH_KP = 8.0f, PITCH_KI = 0.00f, PITCH_KD = 0.0f;
static constexpr double YAW_KP = 1.0f, YAW_KI = 0.00f, YAW_KD = 0.0f;

static constexpr double DEFAULT_ROLL_SETPOINT = 0;
static constexpr double ROLL_MIN_LIMIT = -90;
static constexpr double ROLL_MAX_LIMIT = 90;

static constexpr double DEFAULT_PITCH_SETPOINT = 8;
static constexpr double PITCH_MIN_LIMIT = -90;
static constexpr double PITCH_MAX_LIMIT = 90;

static constexpr double DEFAULT_YAW_SETPOINT = 0;
static constexpr double YAW_MIN_LIMIT = -90;
static constexpr double YAW_MAX_LIMIT = 90;

static constexpr bool INVERT_PID = true;

/* BAROMETER */
static constexpr double ALTITUDE_BIAS = 238.0f; // In metres

// Sampling intervals of 0=6 ms , 1=10, 2=18, 3=34, 4=66, 5=130, 6=258, and 7=512
static constexpr unsigned long OVER_SAMPLE_PERIODS_MS[] = {6, 10, 18, 34, 66, 130, 258, 512};
static constexpr int OVER_SAMPLE_RATIO = 7;

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
static constexpr Pin SERVO3 = 3;
static constexpr Pin SERVOS[] = {SERVO1, SERVO2, SERVO3};

static constexpr float SERVO_MIN_ANGLE = 0.0f;
static constexpr float SERVO_MAX_ANGLE = 179.0f;

const int RADIO_ID = 0;
const int GPS_ID = 1;
const int I2C_ID = 2;
const int SETPOINT_ID = 3;
const int AUTO_ID = 4;
const int MANUAL_ID = 5;
const int ACTIVITY_ID = 6;
const int DEBUG_ID = 7;

// For fixed wing trainer
static constexpr int DEFAULT_LEFT_ELEVON_ANGLE = 90;
static constexpr int DEFAULT_RIGHT_ELEVON_ANGLE = 90;
static constexpr int DEFAULT_RUDDER_ANGLE = 90;

static constexpr int SAFETY_LEFT_ELEVON_ANGLE = 180;
static constexpr int SAFETY_RIGHT_ELEVON_ANGLE = 0;
static constexpr int SAFETY_RUDDER_ANGLE = 90;


/* RECEIVER INPUTS */
static constexpr Pin RECEIVER1 = 22;   // Left elevon
static constexpr Pin RECEIVER2 = 23;  // Right elevon
static constexpr Pin RECEIVER3 = 4;   // Rudder
static constexpr Pin RECEIVERS[] = {RECEIVER1, RECEIVER2, RECEIVER3};

/* RADIO */
static constexpr Pin RADIO_CS = 14;
static constexpr Pin RADIO_RESET = 15;
static constexpr Pin RADIO_INT = 16;
static constexpr float RADIO_FREQ = 905.0f;
static constexpr int RADIO_POWER = 23;

static constexpr aero::def::ID THIS_DEVICE = aero::def::ID::G2;

// Glider 1
float g1_axb = 0.0f;
float g1_axs = 0.0f;
float g1_ayb = 0.0f;
float g1_ays = 0.0f;
float g1_azb = 0.0f;
float g1_azs = 0.0f;

float g1_mxb = 0.0f;
float g1_mxs = 0.0f;
float g1_myb = 0.0f;
float g1_mys = 0.0f;
float g1_mzb = 0.0f;
float g1_mzs = 0.0f;

// Glider 2
float g2_axb = 0.0f;
float g2_axs = 0.0f;
float g2_ayb = 0.0f;
float g2_ays = 0.0f;
float g2_azb = 0.0f;
float g2_azs = 0.0f;

float g2_mxb = 0.0f;
float g2_mxs = 0.0f;
float g2_myb = 0.0f;
float g2_mys = 0.0f;
float g2_mzb = 0.0f;
float g2_mzs = 0.0f;


template <class X, class M, class N, class O, class Q>
inline X map_generic(X x, M in_min, N in_max, O out_min, Q out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

template <class A, class B, class C>
inline A constrain_generic(A a, B in_min, C in_max) {
  if(a <= in_max && a >= in_min) {
    return a;
  } else if(a >= in_max) {
    return in_max;
  } else {
    return in_min;
  }
}

/* GPS */
static constexpr HardwareSerial *GPS_PORT = &Serial2;
static constexpr Pin GPS_FIX =  4;

/* DELAYS */
const long GPS_INTERVAL_MS = 1000;
const unsigned long THREAD_DELAY_MS = 1000;
const unsigned long DEBUG_INTERVAL_MS = 250;
const unsigned long IMU_SAMPLE_INTERVAL_MS = 20;
const unsigned long RADIO_INTERVAL_MS = 250;
const unsigned long MANUAL_INTERVAL_MS = 20;    // 50 HZ Servos
const unsigned long AUTO_INTERVAL_MS = 10;      // 50 HZ Servos
static constexpr int BARO_SAMPLE_INTERVAL_MS = OVER_SAMPLE_PERIODS_MS[OVER_SAMPLE_RATIO];
