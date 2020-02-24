#pragma once

/**
 * File to encapsulate functions that interface with the barometer; MPL3115A2.
 * Makes use of the SparkfunMPL3115A2 library 
 */

#include "Config.hpp"
#include "src/MPL3115A2_Breakout/SparkFunMPL3115A2.h"

/**
 * Barometer namespace; encapsulates all related functions
 */
namespace BARO {

  struct Data {
    float altitude;        // In metres
    float delta_altitude;  // In metres, based on calibration. Value that determines how much our altitude has changed from our initial calibration point.
    float biased_altitude; // In metres; altitde based on calibration accounting for the altitude bias
  };

  // Anonymous namespace to protect data (make private)
  namespace {
    
    // Barometer data
    Data data;

    // Starting altitude of glider
    float elevation_offset = 0.0f;

    // Flag to signal if barometer calibration was performed
    bool calibrated = false;

    // Swap MPL3115A2 into barometer mode
    inline void swap_to_baro(MPL3115A2& baro) {
      baro.setModeStandby();
      baro.setModeBarometer();
      baro.setOversampleRate(OVER_SAMPLE_RATIO);
      baro.enableEventFlags();
      baro.setModeActive();
    }

    // Swap MPL3115A2 into altimeter mode
    inline void swap_to_altimeter(MPL3115A2& baro) {
      baro.setModeStandby();
      baro.setModeAltimeter();
      baro.setOversampleRate(OVER_SAMPLE_RATIO);
      baro.enableEventFlags();
      baro.setModeActive();
    }
  }
  
  /**
   * @brief Initialize barometer object
   * 
   * @param baro barometer object
   */
  inline void init(MPL3115A2& baro) {
    // Join the I2C bus
    baro.begin();

    // Set mode to altimeter
    swap_to_altimeter(baro);
  }

  /**
   * @brief Read sensor data
   * 
   * @param baro sensor to read data from 
   * @return Data sensor data
   */
  inline Data read(MPL3115A2& baro) {
    // Weird, looks like read in ft returns in m
    data.altitude = baro.readAltitude();  

    if(calibrated == true) {
      data.delta_altitude = data.altitude - ( elevation_offset);
      data.biased_altitude = data.altitude - ( elevation_offset) + ALTITUDE_BIAS;
    }
    
    return data;
  }

  /**
   * @brief Print formatted data
   * 
   * @param data data to print
   */
  inline void print(const Data& data) {

    DEBUG_PRINT(" Altitude (m): "); DEBUG_PRINT(data.altitude);
    
    if(calibrated == true) {
      DEBUG_PRINT("\t Altitude Change (m): "); DEBUG_PRINT(data.delta_altitude);
      DEBUG_PRINT("\t Altitude (biased) (m): "); DEBUG_PRINT(data.biased_altitude);
    }
    
    DEBUG_PRINTLN("");
  }

  /**
   * @brief Calibrate barometer by calculating starting position
   * 
   * @param baro baro to calibrate
   */
  inline void calibrate(MPL3115A2& baro) {

    // Amount of barometer samples for calibration
    static const int SAMPLES = 10;

    float accumulated_pressure = 0.0f;
    float pressure = 0.0f;

    for(int i = 0; i < SAMPLES; i++) {
      pressure = baro.readAltitude();
      accumulated_pressure += pressure;
      delay(OVER_SAMPLE_PERIODS_MS[OVER_SAMPLE_RATIO]); // Delay for update rate
    }

    elevation_offset = accumulated_pressure / SAMPLES;

    calibrated = true;
  }
 
}
