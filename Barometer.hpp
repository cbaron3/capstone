#pragma once

/**
 * File for functions that interface with the barometer; MPL3115A2.
 * Makes use of the SparkfunMPL3115A2 library 
 */

#include "Config.hpp"
#include "src/MPL3115A2_Breakout/SparkFunMPL3115A2.h"

/**
 * Barometer namespace; encapsulates all related functions
 */
namespace BARO {

  struct BaroData {
    float altitude; // In metres
    float delta_altitude; // In metres, based on calibration. Value that determines how much our altitude has changed from our initial calibration point.
  };

  namespace {
    
    float calc_sea_level_pressure = 0.0f;
    float elevation_offset = 0.0f;

    // Place offsets here
    bool calibrated = false;

     inline void swap_to_baro(MPL3115A2& baro) {
      baro.setModeStandby();
      // Use altimeter mode. The other settings are taken from example programs
      baro.setModeBarometer();
      // Sampling intervals of 0=6 ms , 1=10, 2=18, 3=34, 4=66, 5=130, 6=258, and 7=512
      baro.setOversampleRate(2);
      baro.enableEventFlags();
      baro.setModeActive();
    }

    inline void swap_to_altimeter(MPL3115A2& baro) {
      baro.setModeStandby();
      // Use altimeter mode. The other settings are taken from example programs
      baro.setModeAltimeter();
      // Sampling intervals of 0=6 ms , 1=10, 2=18, 3=34, 4=66, 5=130, 6=258, and 7=512
      baro.setOversampleRate(2);
      baro.enableEventFlags();
      baro.setModeActive();
    }
  }
  

  // Initialize the sensor
  inline void init(MPL3115A2& baro) {
    // Join the I2C bus
    baro.begin();

    swap_to_altimeter(baro);
  }

  // Read data from sensor
  inline BaroData read_data(MPL3115A2& baro) {
    BaroData data;
    // Weird, looks like read in ft returns in m
    data.altitude = baro.readAltitudeFt();  
    
    data.delta_altitude = data.altitude - elevation_offset;
    
    return data;
  }

  // Print the data in a formatted method
  inline void print_data(const BaroData& data) {
    DEBUG_PRINT(" Altitude (m): "); DEBUG_PRINT(data.altitude);
    
    if(calibrated) {
      DEBUG_PRINT("\t Altitude Change (m): "); DEBUG_PRINT(data.delta_altitude);
    }
    
    DEBUG_PRINTLN("");
  }

  inline void calibrate(MPL3115A2& baro, float altitude_bias) {
    // TODO: Do nothing for now, requires current altitude
    swap_to_baro(baro);

    static const int SAMPLES = 10;

    float accumulated_pressure = 0.0f;
    float pressure = 0.0f;

    for(int i = 0; i < SAMPLES; i++) {
      pressure = baro.readPressure();
      accumulated_pressure += pressure;
      delay(10);
    }

    float average_pressure = accumulated_pressure / SAMPLES;

    float pressure_power = pow(1.0-(altitude_bias*0.0000225577), 5.255877);
    calc_sea_level_pressure = average_pressure / pressure_power;
    elevation_offset = 101325.0 - (101325.0 * pressure_power);

    swap_to_altimeter(baro);

    baro.setBarometricInput(calc_sea_level_pressure);
    
    calibrated = true;
  }

 
}
