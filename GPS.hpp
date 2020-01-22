#pragma once

/**
 * File for functions that interface with the GPS; Adafruit GPS based on MK3339 chipset.
 * Makes use of the Adafruit_GPS library 
 */
#include <Adafruit_GPS.h>

#define GPSECHO false

/**
 * GPS namespace; encapsulates all related functions
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

  struct GPSData {
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

  inline bool read_sensor(Adafruit_GPS& GPS) {
     // Read data from GPS directly
    char c = GPS.read();

    // Used for debugging
    if (GPSECHO)
      if (c) Serial.print(c);
      
    // If sentence is received, check the checksum and parse it if valid
    if (GPS.newNMEAreceived()) {
      if (!GPS.parse(GPS.lastNMEA())) // Resets the newNMEAreceived() flag to false
        return false; // If we fail to parse, return false indicating that a new message was received but not parsed
    } else {
      return false; // No new message received;
    }
    
    return true;
  }
  
  inline GPSData read_data(Adafruit_GPS& GPS) {
    GPSData data;

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

  inline void print_data(const GPSData& data) {
    Serial.print(" Fix: "); Serial.print(data.fix);
    Serial.print(" \t Satellites: "); Serial.print(data.satellites);

    // Print time stamp
    Serial.print("\t Date (MM/DD/YY): "); 
    Serial.print(data.timestamp.month);
    Serial.print("/"); Serial.print(data.timestamp.day);
    Serial.print("/"); Serial.print(data.timestamp.year);

    Serial.print("\t Time: (HR:MIN:SEC): ");
    Serial.print(data.timestamp.hr);
    Serial.print(":"); Serial.print(data.timestamp.min);
    Serial.print(":"); Serial.print(data.timestamp.sec);

    // Print coordinate
    Serial.print("\t Lat, Lon :"); 
    Serial.print(data.coord.lat, 7);
    Serial.print(", "); 
    Serial.print(data.coord.lon, 7);

    // Print other data
    Serial.print("\t Speed (knots): "); Serial.print(data.speed);
    Serial.print("\t Altitude (m): "); Serial.print(data.altitude);
    Serial.print("\t Course (deg): "); Serial.print(data.angle);
    Serial.println("");
  }
  
}
