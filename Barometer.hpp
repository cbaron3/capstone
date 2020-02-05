#pragma once

/**
 * File for functions that interface with the barometer; MPL3115A2.
 * Makes use of the SparkfunMPL3115A2 library 
 */
 
#include <SparkFunMPL3115A2.h>

/**
 * Barometer namespace; encapsulates all related functions
 */
namespace BARO {

  struct BaroData {
    float altitude; // In metres
    float delta_altitude; // In metres, based on calibration. Value that determines how much our altitude has changed from our initial calibration point.
  };

  namespace {
    // Place offsets here
    bool calibrated = false;
  }

  // Initialize the sensor
  inline void init(MPL3115A2& baro) {
    // Join the I2C bus
    baro.begin();

    // Use altimeter mode. The other settings are taken from example programs
    baro.setModeAltimeter();
    // Sampling intervals of 0=6 ms , 1=10, 2=18, 3=34, 4=66, 5=130, 6=258, and 7=512
    baro.setOversampleRate(2);
    baro.enableEventFlags();
  }

  // Read data from sensor
  inline BaroData read_data(MPL3115A2& baro) {
    BaroData data;
    // Weird, looks like read in ft returns in m
    data.altitude = baro.readAltitudeFt();  
    
    // TODO: Calculate height change
    
    return data;
  }

  // Print the data in a formatted method
  inline void print_data(const BaroData& data) {
    Serial.print(" Altitude (m): "); Serial.print(data.altitude);
    
    if(calibrated) {
      Serial.print("\t Altitude Change (m): "); Serial.print(data.delta_altitude);
    }
    
    Serial.println("");
  }

  inline void calibrate(MPL3115A2& baro) {
    // TODO: Do nothing for now, requires current altitude
    calibrated = true;
  }
}
