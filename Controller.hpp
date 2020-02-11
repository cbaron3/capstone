#pragma once

#include "src/Arduino-PID-Library/PID_v1.h"
#include "Config.hpp"

namespace CONTROLLER {
    double roll_setpoint, roll_input, roll_output;
    double pitch_setpoint, pitch_input, pitch_output;
    double yaw_setpoint, yaw_input, yaw_output;

    inline void init(PID& roll, PID& pitch, PID& yaw) {
        // Set default setpoints
        roll_setpoint = DEFAULT_ROLL_SETPOINT;
        pitch_setpoint = DEFAULT_PITCH_SETPOINT;
        yaw_setpoint = DEFAULT_YAW_SETPOINT;

        roll.SetOutputLimits(ROLL_MIN_LIMIT, ROLL_MAX_LIMIT);
        pitch.SetOutputLimits(PITCH_MIN_LIMIT, PITCH_MAX_LIMIT);
        yaw.SetOutputLimits(YAW_MIN_LIMIT, YAW_MAX_LIMIT);

        roll.SetMode(AUTOMATIC);
        roll.SetSampleTime(SAMPLE_TIME_MS);

        pitch.SetMode(AUTOMATIC);
        pitch.SetSampleTime(SAMPLE_TIME_MS);

        yaw.SetMode(AUTOMATIC);
        yaw.SetSampleTime(SAMPLE_TIME_MS);
    }

    inline void update(PID& roll, PID& pitch, PID& yaw) {
        // PID computations
        roll.Compute();
        pitch.Compute();
        yaw.Compute();

        if(INVERT_PID == true) {
            roll_output = -1 * roll_output;
            pitch_output = -1 * pitch_output;
            yaw_output = -1 * yaw_output;
        }
        
    }
}