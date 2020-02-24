#pragma once

/**
 * File to encapsulate functions that interface with the GPS; Adafruit GPS based on MK3339 chipset.
 * Makes use of the Adafruit_GPS library 
 */

#include <Adafruit_GPS.h>
#include "Config.hpp"

#define GPSECHO false

/**
 * GPS namespace; encapsulates all related functions that use Adafruit library
 */
namespace GPS {

  // Time and date data
  struct TimeStamp {
    byte hr;                                     
    byte min;                                   
    byte sec;                                  
    byte msec;                            
    int year;                                     
    byte month;                                    
    byte day;                                      
  };

  struct Coord {
    // In decimal degrees
    double lat;
    double lon;
  };

  struct Data {
    // GPS timestamp, dd/mm/yy hr:min:sec:msec
    TimeStamp timestamp;
    // Coordinate information
    Coord coord;
    // Number of satellites
    unsigned int satellites;
    // Do we have a fix or not
    bool fix;
    // Speed, altitude and course
    float speed, altitude, angle;
  };

  // Anonymous namespace to protect data (make private)
  namespace {
    Data data;
  }
  
  /**
   * @brief Initialize GPS
   * 
   * @param GPS gps to initialize
   */
  inline void init(Adafruit_GPS& GPS) {
    // 9600 NMEA is the default baud rate this module but note that some use 4800
    GPS.begin(9600);
    
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // The data we are requesting
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  
    // Request updates on antenna status, comment out to keep quiet
    GPS.sendCommand(PGCMD_ANTENNA);
  
    // Ask for firmware version
    Serial2.println(PMTK_Q_RELEASE);
  }


  /**
   * @brief Check if there is new data ready to be parsed
   * 
   * @param GPS gps object
   * @return true if new data is available
   * @return false if new data is not available
   */
  inline bool check(Adafruit_GPS& GPS) {
     // Read data from GPS directly
    char c = GPS.read();

    // Used for debugging
    if (GPSECHO)
      if (c) DEBUG_PRINT(c);
      
    // If sentence is received, check the checksum and parse it if valid
    if (GPS.newNMEAreceived()) {
      if (!GPS.parse(GPS.lastNMEA())) // Resets the newNMEAreceived() flag to false
        return false; // If we fail to parse, return false indicating that a new message was received but not parsed
    } else {
      return false; // No new message received;
    }
    
    return true;
  }
  
  /**
   * @brief Returns parsed data from GPS module
   * 
   * @param GPS gps object
   * @return Data gps data
   */
  inline Data read(Adafruit_GPS& GPS) {
    
    // Time and date
    data.timestamp.year = GPS.year;
    data.timestamp.month = GPS.month;
    data.timestamp.day = GPS.day;
    data.timestamp.hr = GPS.hour;
    data.timestamp.min = GPS.minute;
    data.timestamp.sec = GPS.seconds;
    data.timestamp.msec = GPS.milliseconds;

    // Connection quality
    data.fix = (bool)GPS.fix;
    data.satellites = (int)GPS.satellites;
    
    // Latitude, in decimal degrees
    data.coord.lat = GPS.latitudeDegrees;

    // Longitude, in decimal degrees
    data.coord.lon = GPS.longitudeDegrees;

    // Altitude, in metres
    data.altitude = GPS.altitude;

    // Speed; in m/s
    data.speed = GPS.speed / 1.944f;

    // Course, in degrees
    data.angle = GPS.angle;

    return data;
  }

  /**
   * @brief Prints formatted GPS data
   * 
   * @param data gps data to print
   */
  inline void print(const Data& data) {
    DEBUG_PRINT(" Fix: "); DEBUG_PRINT(data.fix);
    DEBUG_PRINT(" \t Satellites: "); DEBUG_PRINT(data.satellites);

    // Print time stamp
    DEBUG_PRINT("\t Date (MM/DD/YY): "); 
    DEBUG_PRINT(data.timestamp.month);
    DEBUG_PRINT("/"); DEBUG_PRINT(data.timestamp.day);
    DEBUG_PRINT("/"); DEBUG_PRINT(data.timestamp.year);

    DEBUG_PRINT("\t Time: (HR:MIN:SEC): ");
    DEBUG_PRINT(data.timestamp.hr);
    DEBUG_PRINT(":"); DEBUG_PRINT(data.timestamp.min);
    DEBUG_PRINT(":"); DEBUG_PRINT(data.timestamp.sec);

    // Print coordinate
    DEBUG_PRINT("\t Lat, Lon :"); 
    DEBUG_PRINT(data.coord.lat);
    DEBUG_PRINT(", "); 
    DEBUG_PRINT(data.coord.lon);

    // Print other data
    DEBUG_PRINT("\t Speed (knots): "); DEBUG_PRINT(data.speed);
    DEBUG_PRINT("\t Altitude (m): "); DEBUG_PRINT(data.altitude);
    DEBUG_PRINT("\t Course (deg): "); DEBUG_PRINT(data.angle);
    DEBUG_PRINTLN("");
  }
}
