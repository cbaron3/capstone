#pragma once

/**
 * File to encapsulate functions that interface with the controllers; PID v1.
 * Makes use of the Arduino PID_v1 library 
 */

#include "src/Arduino-PID-Library/PID_v1.h"
#include "Config.hpp"

/**
 * Controller namespace; encapsulates all related functions
 */
namespace CONTROLLER {
    // Roll values
    double roll_setpoint, roll_input, roll_output;

    // Pitch values
    double pitch_setpoint, pitch_input, pitch_output;

    // Yaw values
    double yaw_setpoint, yaw_input, yaw_output;

    /**
     * @brief Initialize PID controllers
     * 
     * @param roll roll controller
     * @param pitch pitch controller
     * @param yaw yaw controller
     */
    inline void init(PID& roll, PID& pitch, PID& yaw) {

        // Set default setpoints
        roll_setpoint = DEFAULT_ROLL_SETPOINT;
        pitch_setpoint = DEFAULT_PITCH_SETPOINT;
        yaw_setpoint = DEFAULT_YAW_SETPOINT;

        // Output limits
        roll.SetOutputLimits(ROLL_MIN_LIMIT, ROLL_MAX_LIMIT);
        pitch.SetOutputLimits(PITCH_MIN_LIMIT, PITCH_MAX_LIMIT);
        yaw.SetOutputLimits(YAW_MIN_LIMIT, YAW_MAX_LIMIT);

        // PID will be updated every time the IMU is updated; therefore they share sample times
        roll.SetMode(AUTOMATIC);
        roll.SetSampleTime(IMU_SAMPLE_INTERVAL_MS);

        pitch.SetMode(AUTOMATIC);
        pitch.SetSampleTime(IMU_SAMPLE_INTERVAL_MS);

        yaw.SetMode(AUTOMATIC);
        yaw.SetSampleTime(IMU_SAMPLE_INTERVAL_MS);
    }

    /**
     * @brief Compute PID output values. Values are updated by reference
     * 
     * @param roll roll controller
     * @param pitch pitch controller
     * @param yaw yaw controller
     */
    inline void update(PID& roll, PID& pitch, PID& yaw) {
        // PID computations
        roll.Compute();
        pitch.Compute();
        yaw.Compute();

        // Invert values if needed
        if(INVERT_PID == true) {
            roll_output = -1 * roll_output;
            pitch_output = -1 * pitch_output;
            yaw_output = -1 * yaw_output;
        }
        
    }
}
