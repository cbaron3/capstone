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
    Serial.print(" Satellites: "); Serial.print(data.satellites);

    // Print time stamp
    Serial.print("\t Date (MM/DD/YY): "); 
    Serial.print(data.timestamp.month);
    Serial.print("/"); Serial.print(data.timestamp.day);
    Serial.print("/"); Serial.print(data.timestamp.year);

    Serial.print("/t Time: (HR:MIN:SEC): ");
    Serial.print(data.timestamp.hr);
    Serial.print(":"); Serial.print(data.timestamp.min);
    Serial.print(":"); Serial.print(data.timestamp.sec);

    // Print coordinate
    Serial.println("\t Lat, Lon :"); 
    Serial.print(data.coord.lat, 7);
    Serial.print(", "); 
    Serial.print(data.coord.lon, 7);

    // Print other data
    Serial.print("\t Speed (m/s): "); Serial.print(data.speed);
    Serial.print("\t Altitude (m): "); Serial.print(data.altitude);
    Serial.print("\t Course (deg): "); Serial.print(data.angle);
    Serial.println("");
  }
  
  inline void fill_buffer(TinyGPS& gps, HardwareSerial& port, unsigned long ms) {
    unsigned long start = millis();
    do {
        while (port.available())
        gps.encode(port.read());
    } while (millis() - start < ms);
  }
}