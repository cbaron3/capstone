#pragma once

#include "GPSConverter.hpp"


namespace SETPOINT {
    // Have our current GPS and ALT

    struct LLA {
      float lat, lon, alt;
    }

    struct ECEF {
      float x,y,z;
    }

    struct Angles {
      float elev, azimuth;
    }

    namespace {
      GeodeticConverter gps_converter;

      bool valid_ecef = false;

      LLA last_geo, curr_geo, target_geo, delta_geo;
      ECEF last_ecef, curr_ecef, target_ecef, delta_ecef;

      Angles last_to_curr, curr_to_target;

    }

    void set_initial(float home_lat, float home_lon, float home_alt) {
        target_geo.lat = home_lat;
        target_geo.lon = home_lon;
        target_geo.alt = home_alt;

        gps_converter.setHome(home_lat, home_lon, home_alt);

        gps_converter.geodetic2Ecef(target_geo.lat, target_geo.lon, target_geo.alt,
                                      target_ecef.x, target_ecef.y, target_ecef.z);      
    }

    // zero based on target coordinates

    // angle of descent is based on Y and Z plane. so need altitude

    // heading is y/x plane
        // Current and last
        // Current and target

    void calculate_deltas(void) {
      delta_ecef.x = curr_ecef.x - last_ecef.x;
      delta_ecef.y = curr_ecef.y - last_ecef.y;
      delta_ecef.z = curr_ecef.z - last_ecef.z;
    }

    void calculate_angles(void) {
      calculate_deltas();

      // Calculate last to curr
      elev_c = (x*dx + y*dy + z*dz) / Sqrt((x^2+y^2+z^2)*(dx^2+dy^2+dz^2))
      azi_c = (-z*x*dx - z*y*dy + (x^2+y^2)*dz) / Sqrt((x^2+y^2)(x^2+y^2+z^2)(dx^2+dy^2+dz^2))
      azi_s = (-y*dx + x*dy) / Sqrt((x^2+y^2)(dx^2+dy^2+dz^2))

      angles.azimuth = atan2(azi_s, azi_c);
      angles.elev = acos(azi_c);

      // Calculate curr to target
    }

    Angles angles() {
      return angles;
    }

    bool update(float lat, float lon, float alt) {
        if(valid == false) {
          valid = true;

          curr_geo.lat = lat;
          curr_geo.lon = lon;
          curr_geo.alt = alt;

          gps_converter.geodetic2Ecef(curr_geo.lat, curr_geo.lon, curr_geo.alt,
                                      curr_ecef.x, curr_ecef.y, curr_ecef.z);

          return false;
        } else {

          last_geo = curr_geo;
          last_ecef = curr_ecef;

          curr_geo.lat = lat;
          curr_geo.lon = lon;
          curr_geo.alt = alt;

          gps_converter.geodetic2Ecef(curr_geo.lat, curr_geo.lon, curr_geo.alt,
                                      curr_ecef.x, curr_ecef.y, curr_ecef.z);

          calcutate_angles();

          return true;
        }




        // Track current and last position

        // Actual angle of descent
            // Current and last
        // Desired angle of descent
            // Current and target
    }
}