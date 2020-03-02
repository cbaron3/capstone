#pragma once

#include "GPSConverter.hpp"

#include "Config.hpp"
namespace SETPOINT {
    // Have our current GPS and ALT

    struct LLA {
      float lat, lon, alt;
    };

    struct ECEF {
      float x,y,z;
    };

    struct Angles {
      float elev, azimuth;
    };

    namespace {
      GeodeticConverter gps_converter;

      Angles plane_angle, target_angle;

      LLA target_geo, curr_geo, last_geo;
      ECEF target_ecef, curr_ecef, last_ecef;

      bool valid = false;
    }

    void init(float home_lat, float home_lon, float home_alt) {
        target_geo.lat = home_lat;
        target_geo.lon = home_lon;
        target_geo.alt = home_alt;

        gps_converter.setHome(home_lat, home_lon, home_alt);

        gps_converter.geodetic2Ecef(target_geo.lat, target_geo.lon, target_geo.alt, &target_ecef.x, &target_ecef.y, &target_ecef.z);
    }

    ECEF calc_delta(const ECEF& start, const ECEF& end) {
      ECEF delta;

      delta.x = end.x - start.x;
      delta.y = end.y - start.y;
      delta.z = end.z - start.z;

      return delta;
    }

    Angles get_plane_angle() {
      return plane_angle;
    }

    Angles get_target_angle() {
      return target_angle;
    }

    Angles calc_angles(const ECEF& start, const ECEF& end) {
      // https://gis.stackexchange.com/questions/58923/calculating-view-angle
      Angles result;

      ECEF delta = calc_delta(start, end);

      // Cosine component of elevation
      float elev_c = (start.x*delta.x + start.y*delta.y + start.z*delta.z) 
                  / sqrt((pow(start.x,2) + pow(start.y,2)+ pow(start.z,2))*(pow(delta.x,2) + pow(delta.y,2) + pow(delta.z,2)));

      // Cosine component of azimuth
      float azi_c = (-start.z*start.x*delta.x - start.z*start.y*delta.y + (pow(start.x,2)+pow(start.y,2))*delta.z) 
                  / sqrt((pow(start.x,2) + pow(start.y,2)) * (pow(start.x,2) + pow(start.y,2) + pow(start.z,2)) * (pow(delta.x,2) + pow(delta.y,2) + pow(delta.z,2)));

      // Sine component of azimuth
      float azi_s = (-start.y*delta.x + start.x*delta.y)
                  / sqrt((pow(start.x,2) + pow(start.y,2)) *(pow(delta.x,2) + pow(delta.y,2) + pow(delta.z,2)));

      // Convert to degrees
      result.azimuth = atan2(azi_s, azi_c) * 57.2958f;
      result.elev = acos(azi_c) * 57.2958f;

      return result;
    }

    bool update(float lat, float lon, float alt) {
        // If this is the first time entering into the update function, do not calculate angle
        if(valid == false) {
          valid = true;

          curr_geo.lat = lat;
          curr_geo.lon = lon;
          curr_geo.alt = alt;

          gps_converter.geodetic2Ecef(curr_geo.lat, curr_geo.lon, curr_geo.alt,
                                      &curr_ecef.x, &curr_ecef.y, &curr_ecef.z);

          return false;
        } else {

          last_geo = curr_geo;
          last_ecef = curr_ecef;

          curr_geo.lat = lat;
          curr_geo.lon = lon;
          curr_geo.alt = alt;

          gps_converter.geodetic2Ecef(curr_geo.lat, curr_geo.lon, curr_geo.alt,
                                      &curr_ecef.x, &curr_ecef.y, &curr_ecef.z);

                

          // Calculate deltas from last to current position and current to target position
          plane_angle = calc_angles(last_ecef, curr_ecef);
          target_angle = calc_angles(curr_ecef, target_ecef);

          return true;
        }
    }
}