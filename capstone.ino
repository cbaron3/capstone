// Standard Servo library for arduino
#include <Servo.h>

// PID library
#include "src/PID_v1/PID_v1.h"

// Madgwick filter library
#include "src/MadgwickAHRS/src/MadgwickAHRS.h"

// Use magnometer
// Replace with constant perhaps?
#define USE_MAG

// Mode select
enum class MODES {
    SERVO_TEST,
    ELEVON_MIXING_TEST,
    IMU_TEST,
    PITCH_TEST,
    ROLL_TEST,
    FULL_STABILIZATION
};

// Need to find a way to log PID performance so we can plot it using python. 
    // Either use serial 
    // Or write to SD Card


// Codebase constants
const double MIN_PITCH = -90;
const double MAX_PITCH = 90;

const double MIN_ROLL = -90;
const double MAX_ROLL = 90;

// TODO: Define pins
const int LEFT_ELEVON_PIN = 0;
const int RIGHT_ELEVON_PIN = 0;

// Update rate. Should be sync'd with sensor
const int SAMPLES_PER_SEC = 25;

// Pitch controller
double desiredPitch = 0, actualPitch = 0, outputPitch = 0;
double kpPitch = 1, kiPitch = 1, kdPitch = 1;

PID pitchController( &actualPitch, &outputPitch, &desiredPitch, 
                    kpPitch, kiPitch, kdPitch, DIRECT );

// Roll controller
double desiredRoll = 0, actualRoll = 0, outputRoll = 0;
double kpRoll = 1, kiRoll = 1, kdRoll = 1;

PID rollController( &actualRoll, &outputRoll, &desiredRoll, 
                    kpRoll, kiRoll, kdRoll, DIRECT );

// Madgwick Filter
Madgwick ahrsFilter;

// Servos
Servo leftElevon, rightElevon;

void setup() {
    // Initialize pitch controller
    pitchController.SetOutputLimits(MIN_PITCH, MAX_PITCH);
    pitchController.SetMode(AUTOMATIC);

    // Initialize roll controller
    rollController.SetOutputLimits(MIN_ROLL, MAX_ROLL);
    rollController.SetMode(AUTOMATIC);

    // Initialize IMU
    /* ... */

    // Initialize filter
    ahrsFilter.begin(SAMPLES_PER_SEC);

    // Initialize servos
    leftEvelon.attach(LEFT_ELEVON_PIN);
    rightElevon.attach(RIGHT_ELEVON_PIN);
}

void loop() {
    // Read from IMU...

    // Filter will return roll, pitch angles
    double gx = 0, gy = 0, gz = 0;
    double ax = 0, ay = 0, az = 0;
    double mx = 0, my = 0, mz = 0;

    #ifdef USE_MAG
        filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
    #elif
        filter.update(gx, gy, gz, ax, ay, az);
    #endif

    double roll = 0, pitch  = 0, yaw = 0;
    roll = filter.getRoll();
    pitch = filter.getPitch();
    // Yaw is heading
    yaw = filter.getYaw();
}

// For example
    // Accel
        // raw is the value
        // scale is 2g, 4g, ...
        // range is 32768 cause from -32768, 32767 
float convertRaw(int raw, float scale, float range) {
    float val = (raw * scale) / range;
    return val;
}