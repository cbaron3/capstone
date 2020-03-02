#pragma once

#include "Arduino.h"

class GeodeticConverter
{
 public:
  GeodeticConverter(float home_latitude = 0, float home_longitude = 0, float home_altitude = 0)
  {
    setHome(home_latitude, home_longitude, home_altitude);
  }
  
  void setHome(float home_latitude, float home_longitude, float home_altitude)
  {
    home_latitude_ = home_latitude;
    home_longitude_ = home_longitude;
    home_altitude_ = home_altitude;

    // Save NED origin
    home_latitude_rad_ = deg2Rad(home_latitude);
    home_longitude_rad_ = deg2Rad(home_longitude);

    // Compute ECEF of NED origin
    geodetic2Ecef(home_latitude_, home_longitude_, home_altitude_, &home_ecef_x_, &home_ecef_y_, &home_ecef_z_);

  }

  void getHome(float* latitude, float* longitude, float* altitude)
  {
    *latitude = home_latitude_;
    *longitude = home_longitude_;
    *altitude = home_altitude_;
  }

  void geodetic2Ecef(const float latitude, const float longitude, const float altitude, float* x,
                     float* y, float* z)
  {
    // Convert geodetic coordinates to ECEF.
    // http://code.google.com/p/pysatel/source/browse/trunk/coord.py?r=22
    float lat_rad = deg2Rad(latitude);
    float lon_rad = deg2Rad(longitude);
    float xi = sqrt(1 - kFirstEccentricitySquared * sin(lat_rad) * sin(lat_rad));
    *x = (kSemimajorAxis / xi + altitude) * cos(lat_rad) * cos(lon_rad);
    *y = (kSemimajorAxis / xi + altitude) * cos(lat_rad) * sin(lon_rad);
    *z = (kSemimajorAxis / xi * (1 - kFirstEccentricitySquared) + altitude) * sin(lat_rad);
  }

  void ecef2Geodetic(const float x, const float y, const float z, float* latitude,
                     float* longitude, float* altitude)
  {
    // Convert ECEF coordinates to geodetic coordinates.
    // J. Zhu, "Conversion of Earth-centered Earth-fixed coordinates
    // to geodetic coordinates," IEEE Transactions on Aerospace and
    // Electronic Systems, vol. 30, pp. 957-961, 1994.

    float r = sqrt(x * x + y * y);
    float Esq = kSemimajorAxis * kSemimajorAxis - kSemiminorAxis * kSemiminorAxis;
    float F = 54 * kSemiminorAxis * kSemiminorAxis * z * z;
    float G = r * r + (1 - kFirstEccentricitySquared) * z * z - kFirstEccentricitySquared * Esq;
    float C = (kFirstEccentricitySquared * kFirstEccentricitySquared * F * r * r) / pow(G, 3);
    float S = cbrt(1 + C + sqrt(C * C + 2 * C));
    float P = F / (3 * pow((S + 1 / S + 1), 2) * G * G);
    float Q = sqrt(1 + 2 * kFirstEccentricitySquared * kFirstEccentricitySquared * P);
    float r_0 = -(P * kFirstEccentricitySquared * r) / (1 + Q)
        + sqrt(
            0.5 * kSemimajorAxis * kSemimajorAxis * (1 + 1.0 / Q)
                - P * (1 - kFirstEccentricitySquared) * z * z / (Q * (1 + Q)) - 0.5 * P * r * r);
    float U = sqrt(pow((r - kFirstEccentricitySquared * r_0), 2) + z * z);
    float V = sqrt(
        pow((r - kFirstEccentricitySquared * r_0), 2) + (1 - kFirstEccentricitySquared) * z * z);
    float Z_0 = kSemiminorAxis * kSemiminorAxis * z / (kSemimajorAxis * V);
    *altitude = static_cast<float>(U * (1 - kSemiminorAxis * kSemiminorAxis / (kSemimajorAxis * V)));
    *latitude = rad2Deg(atan((z + kSecondEccentricitySquared * Z_0) / r));
    *longitude = rad2Deg(atan2(y, x));
  }

private:
  // Geodetic system parameters
  static constexpr float kSemimajorAxis = 6378137;
  static constexpr float kSemiminorAxis = 6356752.3142;
  static constexpr float kFirstEccentricitySquared = 6.69437999014 * 0.001;
  static constexpr float kSecondEccentricitySquared = 6.73949674228 * 0.001;
  static constexpr float kFlattening = 1 / 298.257223563;

  inline float rad2Deg(const float radians)
  {
    return (radians / M_PI) * 180.0;
  }

  inline float deg2Rad(const float degrees)
  {
    return (degrees / 180.0) * M_PI;
  }

  float home_latitude_rad_, home_latitude_;
  float home_longitude_rad_, home_longitude_;
  float home_altitude_;

  float home_ecef_x_;
  float home_ecef_y_;
  float home_ecef_z_;

}; // class GeodeticConverter
