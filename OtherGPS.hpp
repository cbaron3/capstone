#pragma once 

#include "TinyGPS.h"

namespace OtherGPS {
  struct TimeStamp {
    byte hr;                                     
    byte min;                                   
    byte sec;                                  
    byte msec;                            
    int year;                                     
    byte month;                                    
    byte day;                                      
  };

  struct Stats {
    unsigned long chars;
    unsigned short seqs;
    unsigned short fails;
  };

  struct Coord {
      double lat;
      double lon;
  };
    
  struct GPSData {
    // GPS timestamp, dd/mm/yy hr:min:sec:msec
    TimeStamp timestamp;
    Coord coord;
    // Transmission statistics such as recieved sentences and amount of failed parsing
    Stats stats;
    unsigned int satellites;
    float speed, altitude, angle;
  };

  
  inline void init(HardwareSerial& port) {
    port.begin(9600);
    while(!port){}
  }

  inline GPSData read_data(TinyGPS& gps, HardwareSerial& port) {
    GPSData data;

    float flat, flon;
    unsigned long age, chars = 0;
    unsigned short sentences = 0, failed = 0;

    // Get position data and statistics
    gps.f_get_position(&flat, &flon, &age);
    gps.stats(&chars, &sentences, &failed);

    // Satellites
    data.satellites = gps.satellites();
    
    // Latitude
    data.coord.lat = flat;

    // Longitude
    data.coord.lon = flon;

    // Altitude
    data.altitude = gps.f_altitude();

    // Speed; in m/s
    data.speed = gps.f_speed_kmph()/3.6;

    // Course; not a part of data
    data.angle = gps.f_course();

    gps.crack_datetime(&data.timestamp.year, &data.timestamp.month, &data.timestamp.day, 
                        &data.timestamp.hr, &data.timestamp.min, &data.timestamp.sec, 
                        &data.timestamp.msec, &age);
    
    return data;
  }

  inline void print_data(const GPSData& data) {
    DEBUG_PRINT(" Satellites: "); DEBUG_PRINT(data.satellites);

    // Print time stamp
    DEBUG_PRINT("\t Date (MM/DD/YY): "); 
    DEBUG_PRINT(data.timestamp.month);
    DEBUG_PRINT("/"); DEBUG_PRINT(data.timestamp.day);
    DEBUG_PRINT("/"); DEBUG_PRINT(data.timestamp.year);

    DEBUG_PRINT("/t Time: (HR:MIN:SEC): ");
    DEBUG_PRINT(data.timestamp.hr);
    DEBUG_PRINT(":"); DEBUG_PRINT(data.timestamp.min);
    DEBUG_PRINT(":"); DEBUG_PRINT(data.timestamp.sec);

    // Print coordinate
    DEBUG_PRINTLN("\t Lat, Lon :"); 
    DEBUG_PRINT(data.coord.lat);
    DEBUG_PRINT(", "); 
    DEBUG_PRINT(data.coord.lon);

    // Print other data
    DEBUG_PRINT("\t Speed (m/s): "); DEBUG_PRINT(data.speed);
    DEBUG_PRINT("\t Altitude (m): "); DEBUG_PRINT(data.altitude);
    DEBUG_PRINT("\t Course (deg): "); DEBUG_PRINT(data.angle);
    DEBUG_PRINTLN("");
  }
  
  inline void fill_buffer(TinyGPS& gps, HardwareSerial& port, unsigned long ms) {
    unsigned long start = millis();
    do {
        while (port.available())
        gps.encode(port.read());
    } while (millis() - start < ms);
  }
}
