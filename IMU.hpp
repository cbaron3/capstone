#pragma once

/**
 * File for functions that interface with the IMU; MPU9250.
 * Makes use of the MPU9250 Bolderflight library 
 */
#include "MPU9250.h"
#include "LEDS.hpp"
#include <EEPROM.h>

/**
 * GPS namespace; encapsulates all related functions
 */
namespace IMU {
  namespace {
    uint8_t EepromBuffer[48];
    int MAG_OFFSET = 24;

    inline void calibrate_accel(MPU9250& imu) {
      Serial.println("Starting Accelerometer Calibration");
      for(int j = 0; j < 5; j++) { LEDS::flash(500); }
      delay(2500);
      
      for(int i = 0; i < 6; i++) {
        imu.calibrateAccel();
        Serial.println("Switch");
        for(int j = 0; j < 5; j++) { LEDS::flash(500); }
        delay(2500);
      }

      float value;
      value = imu.getAccelBiasX_mss();
      memcpy(EepromBuffer,&value,4);
      value = imu.getAccelScaleFactorX();
      memcpy(EepromBuffer+4,&value,4);
      value = imu.getAccelBiasY_mss();
      memcpy(EepromBuffer+8,&value,4);
      value = imu.getAccelScaleFactorY();
      memcpy(EepromBuffer+12,&value,4);
      value = imu.getAccelBiasZ_mss();
      memcpy(EepromBuffer+16,&value,4);
      value = imu.getAccelScaleFactorZ();
      memcpy(EepromBuffer+20,&value,4);

      for (size_t i=0; i < MAG_OFFSET; i++) {
        EEPROM.write(i,EepromBuffer[i]);
      }
      
      Serial.println("Accelerometer Calibration Done");
    }
    
    inline void calibrate_gyro(MPU9250& imu) {
      Serial.println("Starting Gyroscope Calibration");
      
      // Do nothing, dont need to pass anything in
      for(int j = 0; j < 5; j++) { LEDS::inout(250); }

      imu.calibrateGyro();
      delay(1000);
      
      for(int j = 0; j < 5; j++) { LEDS::inout(250); }

      Serial.println("Gyro Calibration Done");
      
    }
    
    inline void calibrate_mag(MPU9250& imu) {
      // Do nothing, need to pass in EEPROM. Figure 8 motion.
      Serial.println("Starting Magnometer Calibration");
      for(int j = 0; j < 6; j++) { LEDS::sweep(100); }

      imu.calibrateMag();

      for(int j = 0; j < 6; j++) { LEDS::sweep(100); }

      float value;
      value = imu.getMagBiasX_uT();
      memcpy(EepromBuffer+24,&value,4);
      value = imu.getMagScaleFactorX();
      memcpy(EepromBuffer+28,&value,4);
      value = imu.getMagBiasY_uT();
      memcpy(EepromBuffer+32,&value,4);
      value = imu.getMagScaleFactorY();
      memcpy(EepromBuffer+36,&value,4);
      value = imu.getMagBiasZ_uT();
      memcpy(EepromBuffer+40,&value,4);
      value = imu.getMagScaleFactorZ();
      memcpy(EepromBuffer+44,&value,4);
  
      for (size_t i=MAG_OFFSET; i < sizeof(EepromBuffer); i++) {
        EEPROM.write(i,EepromBuffer[i]);
      }

      Serial.println("Magnometer Calibration Done");
    }
  
  }

  struct MPU9250Data {
    float ax, ay, az; // meters/s^s
    float gx, gy, gz; // rad/s
    float mx, my, mz; // uT
  };

  enum SENSOR { ACCEL = 1, GYRO = 2, MAG = 4 };
  
  inline void init(MPU9250& imu) {
    // start communication with IMU 
    int status = imu.begin();
    if (status < 0) {
      Serial.println("IMU initialization unsuccessful");
      Serial.println("Check IMU wiring or try cycling power");
      Serial.print("Status: ");
      Serial.println(status);
      while(1) {}
    }

    // load and set accel and mag bias and scale
    // factors from CalibrateMPU9250.ino 
    for (size_t i=0; i < sizeof(EepromBuffer); i++) {
      EepromBuffer[i] = EEPROM.read(i);
    }

    float axb, axs, ayb, ays, azb, azs;
    float hxb, hxs, hyb, hys, hzb, hzs;

    memcpy(&axb,EepromBuffer+0,4);
    memcpy(&axs,EepromBuffer+4,4);
    memcpy(&ayb,EepromBuffer+8,4);
    memcpy(&ays,EepromBuffer+12,4);
    memcpy(&azb,EepromBuffer+16,4);
    memcpy(&azs,EepromBuffer+20,4);
    memcpy(&hxb,EepromBuffer+24,4);
    memcpy(&hxs,EepromBuffer+28,4);
    memcpy(&hyb,EepromBuffer+32,4);
    memcpy(&hys,EepromBuffer+36,4);
    memcpy(&hzb,EepromBuffer+40,4);
    memcpy(&hzs,EepromBuffer+44,4);
  
    imu.setAccelCalX(axb,axs);
    imu.setAccelCalY(ayb,ays);
    imu.setAccelCalZ(azb,azs);
  
    imu.setMagCalX(hxb,hxs);
    imu.setMagCalY(hyb,hys);
    imu.setMagCalZ(hzb,hzs);
  
    // setting the accelerometer full scale range to +/- 2G
    imu.setAccelRange(MPU9250::ACCEL_RANGE_2G);
    // setting the gyroscope full scale range to +/-500 deg/s
    imu.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
    // setting DLPF bandwidth to 20 Hz
    imu.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_41HZ);
    // setting SRD to 19 for a 50 Hz update rate
    imu.setSrd(19);
  }

  inline MPU9250Data read_data(MPU9250& imu) {
    imu.readSensor();
    
    MPU9250Data data;

    data.ax = imu.getAccelX_mss();
    data.ay = imu.getAccelY_mss();
    data.az = imu.getAccelZ_mss();

    data.gx = imu.getGyroX_rads();
    data.gy = imu.getGyroY_rads();
    data.gz = imu.getGyroZ_rads();

    data.mx = imu.getMagX_uT();
    data.my = imu.getMagY_uT();
    data.mz = imu.getMagZ_uT();
    
    return data;
  }

  inline void print_data(const MPU9250Data& data) {
    // display the data
    Serial.print(" AX (m/s^s): "); Serial.print(data.ax,4);
    Serial.print("\t AY (m/s^s): "); Serial.print(data.ay,4);
    Serial.print("\t AZ (m/s^s): "); Serial.print(data.az,4);
    
    Serial.print("\t GX (rad/s): "); Serial.print(data.gx,4);
    Serial.print("\t GY (rad/s): "); Serial.print(data.gy,4);
    Serial.print("\t GZ (rad/s): "); Serial.print(data.gz,4);

    Serial.print("\t MX (uT): "); Serial.print(data.mx,4);
    Serial.print("\t MY (uT): "); Serial.print(data.my,4);
    Serial.print("\t MZ (uT): "); Serial.print(data.mz,4);
    Serial.println("");
  }

  inline void calibrate(MPU9250& imu, SENSOR sensor) {
    int sensor_mask = (int) sensor;

    if(sensor == ACCEL) {
      calibrate_accel(imu);
    } else if (sensor == GYRO) {
      calibrate_gyro(imu);
    } else if (sensor == MAG) {
      calibrate_mag(imu);
    }
  }
}
