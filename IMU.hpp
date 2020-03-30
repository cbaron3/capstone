#pragma once

/**
 * File to encapsulate functions that interface with the imu; MPU9250.
 * Makes use of the Bolderflight MPU9250 library 
 */

#include "src/SparkFun_MPU-9250-DMP_Arduino_Library/SparkFunMPU9250-DMP.h"
#include "src/uNavAHRS/uNavAHRS.h"
#include <MPU9250.h>
#include "LEDS.hpp"
#include <EEPROM.h>
#include <Arduino.h>
#include "Config.hpp"

/**
 * IMU namespace; encapsulates all related functions
 */
namespace IMU {

  enum SENSOR { ACCEL = 1, GYRO = 2, MAG = 4 };

  struct Data {
    float ax, ay, az; // meters/s^s
    float gx, gy, gz; // rad/s
    float mx, my, mz; // uT
  };
  
  namespace {
    
    // EEPROM values
    uint8_t eeprom_buffer[48];
    const unsigned int MAG_OFFSET = 24;

    Data data;

    // Calibrate accelerometer routine
    inline void calibrate_accel(MPU9250& imu) {
      DEBUG_PRINTLN("Starting Accelerometer Calibration");
      for(int j = 0; j < 5; j++) { LEDS::flash(500); }
      delay(2500);
      
      // Delay to allow user to swap XYZ plane; flash LEDs to notify user to swap
      for(int i = 0; i < 6; i++) {
        imu.calibrateAccel();
        DEBUG_PRINTLN("Switch");
        for(int j = 0; j < 5; j++) { LEDS::flash(500); }
        delay(2500);
      }

      // Load biases and sensitivites into eeprom
      float value;
      value = imu.getAccelBiasX_mss();
      DEBUG_PRINT("AXB: "); DEBUG_PRINTLN(value);
      memcpy(eeprom_buffer,&value,4);
      
      value = imu.getAccelScaleFactorX();
      DEBUG_PRINT("AXS: "); DEBUG_PRINTLN(value);
      memcpy(eeprom_buffer+4,&value,4);
      
      value = imu.getAccelBiasY_mss();
      DEBUG_PRINT("AYB: "); DEBUG_PRINTLN(value);
      memcpy(eeprom_buffer+8,&value,4);
      
      value = imu.getAccelScaleFactorY();
      DEBUG_PRINT("AYS: "); DEBUG_PRINTLN(value);
      memcpy(eeprom_buffer+12,&value,4);
      
      value = imu.getAccelBiasZ_mss();
      DEBUG_PRINT("AZB: "); DEBUG_PRINTLN(value);
      memcpy(eeprom_buffer+16,&value,4);
      
      value = imu.getAccelScaleFactorZ();
      DEBUG_PRINT("AZS: "); DEBUG_PRINTLN(value);
      memcpy(eeprom_buffer+20,&value,4);

      for (size_t i=0; i < MAG_OFFSET; i++) {
        EEPROM.write(i+100,eeprom_buffer[i]);
      }
      
      DEBUG_PRINTLN("Accelerometer Calibration Done");
    }
    
    // Calibrate gyorscope routine
    inline void calibrate_gyro(MPU9250& imu) {
      DEBUG_PRINTLN("Starting Gyroscope Calibration");
      
      // Do nothing, dont need to pass anything in
      for(int j = 0; j < 5; j++) { LEDS::inout(250); }

      imu.calibrateGyro();
      delay(1000);
      
      for(int j = 0; j < 5; j++) { LEDS::inout(250); }

      DEBUG_PRINTLN("Gyro Calibration Done");
      
    }
    
    // Calibrate magnetometer routine
    inline void calibrate_mag(MPU9250& imu) {
      // Figure 8 motion.
      DEBUG_PRINTLN("Starting Magnometer Calibration");
      
      // LEDs sweep to indicate user should start figure 8 motion
      for(int j = 0; j < 6; j++) { LEDS::sweep(100); }

      imu.calibrateMag();

      // LEDs sweep again to indicate user should stop figure 8 motion
      for(int j = 0; j < 6; j++) { LEDS::sweep(100); }

      // Load biases and sensitivites into eeprom
      float value;
      value = imu.getMagBiasX_uT();
      DEBUG_PRINT("MXB: "); DEBUG_PRINTLN(value);
      memcpy(eeprom_buffer+24,&value,4);
      
      value = imu.getMagScaleFactorX();
      DEBUG_PRINT("MXS: "); DEBUG_PRINTLN(value);
      memcpy(eeprom_buffer+28,&value,4);
      
      value = imu.getMagBiasY_uT();
      DEBUG_PRINT("MYB: "); DEBUG_PRINTLN(value);
      memcpy(eeprom_buffer+32,&value,4);
      
      value = imu.getMagScaleFactorY();
      DEBUG_PRINT("MYS: "); DEBUG_PRINTLN(value);
      memcpy(eeprom_buffer+36,&value,4);
      
      value = imu.getMagBiasZ_uT();
      DEBUG_PRINT("MZB: "); DEBUG_PRINTLN(value);
      memcpy(eeprom_buffer+40,&value,4);
      
      value = imu.getMagScaleFactorZ();
      DEBUG_PRINT("MZS: "); DEBUG_PRINTLN(value);
      memcpy(eeprom_buffer+44,&value,4);
  
      for (size_t i=MAG_OFFSET; i < sizeof(eeprom_buffer); i++) {
        EEPROM.write(i+100,eeprom_buffer[i]);
      }

      DEBUG_PRINTLN("Magnometer Calibration Done");
    }
  
  }

  /**
   * @brief Initialize imu 
   * 
   * @param imu imu object to initialize
   */
  inline void init(MPU9250& imu) {
    // start communication with IMU 
    int status = imu.begin();

    if (status < 0) {
      DEBUG_PRINTLN("IMU initialization unsuccessful");
      DEBUG_PRINTLN("Check IMU wiring or try cycling power");
      DEBUG_PRINT("Status: ");
      DEBUG_PRINTLN(status);
      while(1) {}
    }

    // load and set accel and mag bias and scale factors
    for (size_t i=100; i < sizeof(eeprom_buffer); i++) {
      eeprom_buffer[i] = EEPROM.read(i);
    }

    float axb, axs, ayb, ays, azb, azs;
    float hxb, hxs, hyb, hys, hzb, hzs;

    memcpy(&axb,eeprom_buffer+0,4);
    memcpy(&axs,eeprom_buffer+4,4);
    memcpy(&ayb,eeprom_buffer+8,4);
    memcpy(&ays,eeprom_buffer+12,4);
    memcpy(&azb,eeprom_buffer+16,4);
    memcpy(&azs,eeprom_buffer+20,4);
    memcpy(&hxb,eeprom_buffer+24,4);
    memcpy(&hxs,eeprom_buffer+28,4);
    memcpy(&hyb,eeprom_buffer+32,4);
    memcpy(&hys,eeprom_buffer+36,4);
    memcpy(&hzb,eeprom_buffer+40,4);
    memcpy(&hzs,eeprom_buffer+44,4);


//    Serial.println(axb);
//    Serial.println(axs);
//    Serial.println(ayb);
//    Serial.println(ayb);
//    Serial.println(azb);
//    Serial.println(axs);
//    
//    imu.setAccelCalX(axb,axs);
//    imu.setAccelCalY(ayb,ays);
//    imu.setAccelCalZ(azb,azs);
//
//    Serial.println(hxb);
//    Serial.println(hxs);
//    
//    Serial.println(hyb);
//    Serial.println(hys);
//    
//    Serial.println(hzb);
//    Serial.println(hzs);
//    
//    imu.setMagCalX(hxb,hxs);
//    imu.setMagCalY(hyb,hys);
//    imu.setMagCalZ(hzb,hzs);
  
    // setting the accelerometer full scale range to +/- 2G
    imu.setAccelRange(MPU9250::ACCEL_RANGE_2G);
    // setting the gyroscope full scale range to +/-500 deg/s
    imu.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
    // setting DLPF bandwidth to 20 Hz
    imu.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_41HZ);
    // setting SRD to 19 for a 50 Hz update rate
    imu.setSrd(19);

    
  }

  /**
   * @brief Read data from sensor
   * 
   * @param imu imu to read data from
   * @return Data resulting data
   */
  inline Data read(MPU9250& imu) {
    imu.readSensor();

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

  /**
   * @brief Print formatted IMU data
   * 
   * @param data data to print
   */
  inline void print(const Data& data) {

    DEBUG_PRINT(" AX (m/s^s): "); DEBUG_PRINT(data.ax);
    DEBUG_PRINT("\t AY (m/s^s): "); DEBUG_PRINT(data.ay);
    DEBUG_PRINT("\t AZ (m/s^s): "); DEBUG_PRINT(data.az);
    
    DEBUG_PRINT("\t GX (rad/s): "); DEBUG_PRINT(data.gx);
    DEBUG_PRINT("\t GY (rad/s): "); DEBUG_PRINT(data.gy);
    DEBUG_PRINT("\t GZ (rad/s): "); DEBUG_PRINT(data.gz);

    DEBUG_PRINT("\t MX (uT): "); DEBUG_PRINT(data.mx);
    DEBUG_PRINT("\t MY (uT): "); DEBUG_PRINT(data.my);
    DEBUG_PRINT("\t MZ (uT): "); DEBUG_PRINT(data.mz);
    DEBUG_PRINTLN("");
  }

  inline void calibrate(MPU9250& imu, SENSOR sensor) {
    if(sensor == ACCEL) {
      calibrate_accel(imu);
    } else if (sensor == GYRO) {
      calibrate_gyro(imu);
    } else if (sensor == MAG) {
      calibrate_mag(imu);
    }
  }
}

/* OLD TEST CODE TO CHECK */
// For some reason this dynamic polymorphism isnt working
//class IMU {
//
//public:
//
//    struct Data {
//        float yaw = 0.0f;
//        float pitch = 0.0f;
//        float roll = 0.0f;
//    };
//
//    enum Sensor { 
//        ACCEL = 1, 
//        GYRO = 2, 
//        MAG = 4 
//    };
//
//    virtual bool init(void) = 0;
//
//    virtual bool update(void) = 0;
//
//    virtual void calibrate(Sensor sensor) = 0;
//
//    void print(void) {
//        DEBUG_PRINT("\t Pitch (deg): "); DEBUG_PRINT(m_data.pitch);
//        DEBUG_PRINT("\t Roll (deg): "); DEBUG_PRINT(m_data.roll);
//        DEBUG_PRINT("\t Yaw (deg): "); DEBUG_PRINT(m_data.yaw);
//        DEBUG_PRINTLN("");
//    }
//    
//    Data get_data(void)  {
//        return m_data;
//    }
//
//    float get_yaw(void) {
//        return (m_data.yaw);
//    }
//
//    float get_pitch(void)  {
//        return (m_data.pitch);
//    }
//
//    float get_roll(void)  {
//        return (m_data.roll);
//    }
//
//    virtual ~IMU() {}
//
//protected:
//    Data m_data;
//
//};
//
//class MPU9250_AHRS : public IMU {
//
//public:
//    MPU9250_AHRS() {}
//
//    bool init(void) override {
//        DEBUG_PRINTLN("Init IMU...");
//        
//        // start communication with IMU 
//        int status = imu.begin();
//        
//        if (status < 0) {
//            return false; 
//        }
//
//        // load and set accel and mag bias and scale
//        // factors from CalibrateMPU9250.ino 
//        for (size_t i=0; i < sizeof(m_eeprom_buf); i++) {
//          m_eeprom_buf[i] = EEPROM.read(i);
//        }
//
//        float axb, axs, ayb, ays, azb, azs;
//        float hxb, hxs, hyb, hys, hzb, hzs;
//
//        memcpy(&axb,m_eeprom_buf+0,4);
//        memcpy(&axs,m_eeprom_buf+4,4);
//        memcpy(&ayb,m_eeprom_buf+8,4);
//        memcpy(&ays,m_eeprom_buf+12,4);
//        memcpy(&azb,m_eeprom_buf+16,4);
//        memcpy(&azs,m_eeprom_buf+20,4);
//        memcpy(&hxb,m_eeprom_buf+24,4);
//        memcpy(&hxs,m_eeprom_buf+28,4);
//        memcpy(&hyb,m_eeprom_buf+32,4);
//        memcpy(&hys,m_eeprom_buf+36,4);
//        memcpy(&hzb,m_eeprom_buf+40,4);
//        memcpy(&hzs,m_eeprom_buf+44,4);
//
//        
//        
//        imu.setAccelCalX(axb,axs);
//        imu.setAccelCalY(ayb,ays);
//        imu.setAccelCalZ(azb,azs);
//    
//        imu.setMagCalX(hxb,hxs);
//        imu.setMagCalY(hyb,hys);
//        imu.setMagCalZ(hzb,hzs);
//
//
//        DEBUG_PRINTLN("Init IMU...");
//        
//        // setting the accelerometer full scale range to +/- 2G
//        //imu.setAccelRange(MPU9250::ACCEL_RANGE_2G);
//
//        DEBUG_PRINTLN("Init IMU...");
//        // setting the gyroscope full scale range to +/-500 deg/s
//        //imu.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
//        // setting DLPF bandwidth to 20 Hz
//        //imu.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_41HZ);
//        // setting SRD to 19 for a 50 Hz update rate
//        //imu.setSrd(19);
//
//        
//        DEBUG_PRINTLN("Init IMU...");
//        return true;
//    }
//
//    bool update(void) override {
//        imu.readSensor();
//
//        // NOTE: This will take two minute for good results!
//        filter.update(imu.getAccelX_mss(), imu.getAccelY_mss(), imu.getAccelZ_mss(), 
//                    imu.getGyroX_rads(), imu.getGyroY_rads(), imu.getGyroZ_rads(),
//                    imu.getMagX_uT(), imu.getMagY_uT(), imu.getMagZ_uT());
//
//        m_data.pitch = filter.getPitch_rad()*RAD_TO_DEG;
//        m_data.roll =  filter.getRoll_rad()*RAD_TO_DEG;
//        m_data.yaw =   filter.getYaw_rad()*RAD_TO_DEG;
//    }
//
//    void calibrate(Sensor sensor) override {
//        if(sensor == ACCEL) {
//            calibrate_accel();
//        } else if (sensor == GYRO) {
//            calibrate_gyro();
//        } else if (sensor == MAG) {
//            calibrate_mag();
//        }
//    }
//    
//
//
//private:
//    int m_eeprom_buf[48] = {};
//    int mag_offset = 24;
//    MPU9250 imu{Wire, 0x68};
//    uNavAHRS filter;
//
//    void calibrate_accel() {
//      DEBUG_PRINTLN("Starting Accelerometer Calibration");
//      for(int j = 0; j < 5; j++) { LEDS::flash(500); }
//      delay(2500);
//      
//      for(int i = 0; i < 6; i++) {
//        imu.calibrateAccel();
//        DEBUG_PRINTLN("Switch");
//        for(int j = 0; j < 5; j++) { LEDS::flash(500); }
//        delay(2500);
//      }
//
//      float value;
//      value = imu.getAccelBiasX_mss();
//      memcpy(m_eeprom_buf,&value,4);
//      value = imu.getAccelScaleFactorX();
//      memcpy(m_eeprom_buf+4,&value,4);
//      value = imu.getAccelBiasY_mss();
//      memcpy(m_eeprom_buf+8,&value,4);
//      value = imu.getAccelScaleFactorY();
//      memcpy(m_eeprom_buf+12,&value,4);
//      value = imu.getAccelBiasZ_mss();
//      memcpy(m_eeprom_buf+16,&value,4);
//      value = imu.getAccelScaleFactorZ();
//      memcpy(m_eeprom_buf+20,&value,4);
//
//      for (size_t i=0; i < mag_offset; i++) {
//        EEPROM.write(i,m_eeprom_buf[i]);
//      }
//      
//      DEBUG_PRINTLN("Accelerometer Calibration Done");
//    }
//    
//    void calibrate_gyro() {
//      DEBUG_PRINTLN("Starting Gyroscope Calibration");
//      
//      // Do nothing, dont need to pass anything in
//      for(int j = 0; j < 5; j++) { LEDS::inout(250); }
//
//      imu.calibrateGyro();
//      delay(1000);
//      
//      for(int j = 0; j < 5; j++) { LEDS::inout(250); }
//
//      DEBUG_PRINTLN("Gyro Calibration Done");
//      
//    }
//    
//    void calibrate_mag() {
//      // Do nothing, need to pass in EEPROM. Figure 8 motion.
//      DEBUG_PRINTLN("Starting Magnometer Calibration");
//      for(int j = 0; j < 6; j++) { LEDS::sweep(100); }
//
//      imu.calibrateMag();
//
//      for(int j = 0; j < 6; j++) { LEDS::sweep(100); }
//
//      float value;
//      value = imu.getMagBiasX_uT();
//      memcpy(m_eeprom_buf+24,&value,4);
//      value = imu.getMagScaleFactorX();
//      memcpy(m_eeprom_buf+28,&value,4);
//      value = imu.getMagBiasY_uT();
//      memcpy(m_eeprom_buf+32,&value,4);
//      value = imu.getMagScaleFactorY();
//      memcpy(m_eeprom_buf+36,&value,4);
//      value = imu.getMagBiasZ_uT();
//      memcpy(m_eeprom_buf+40,&value,4);
//      value = imu.getMagScaleFactorZ();
//      memcpy(m_eeprom_buf+44,&value,4);
//  
//      for (size_t i=mag_offset; i < sizeof(m_eeprom_buf); i++) {
//        EEPROM.write(i,m_eeprom_buf[i]);
//      }
//
//      DEBUG_PRINTLN("Magnometer Calibration Done");
//    }
//};
//
//
//
//
//
//
//class MPU9250_DMP_EWMA : public IMU {
//
//public:
//    MPU9250_DMP_EWMA() {
//
//    }
//
//    bool init(void) override {
//        bool success = true;
//
//        // Verify communication and initialize
//        inv_error_t  err_flag;
//
//        err_flag = imu.begin();
//        
//        if(err_flag != INV_SUCCESS) {
//            DEBUG_PRINTLN("Failed to begin");
//            success = false;
//        }
//
//        // Calibration routine takes 8 seconds; keep still
//        err_flag = imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
//                     DMP_FEATURE_GYRO_CAL, // Use gyro calibration
//                     1000 / IMU_SAMPLE_INTERVAL_MS); // Set DMP FIFO rate to 10 Hz
//
//        if(err_flag != INV_SUCCESS) {
//            DEBUG_PRINTLN("Failed to DMP");
//            success = false;
//        }
//
//        return success;
//    }
//
//    bool update(void) override {
//        // Check for new data in the FIFO
//        if (imu.fifoAvailable()) {
//            // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
//            if (imu.dmpUpdateFifo() == INV_SUCCESS) {
//                // computeEulerAngles can be used -- after updating the
//                // quaternion values -- to estimate roll, pitch, and yaw
//                imu.computeEulerAngles();
//
//                m_data.yaw = (imu.yaw);
//                m_data.pitch = (imu.pitch);
//                m_data.roll = (imu.roll);
//
//                if(m_data.roll < 180) m_data.roll += 360;
//                if(m_data.pitch < 180) m_data.pitch += 360;
//                
//                m_data.roll = map_generic(m_data.roll, 270, 450, -90, 90);
//                m_data.pitch = map_generic(m_data.pitch, 200, 520, -90, 90);
//
//                // Swap R and P
//                m_data.pitch = constrain_generic(m_data.pitch, -90.0f, 90.0f);
//                m_data.roll = constrain_generic(m_data.roll, -90.0f, 90.0f);
//
//                //float t = m_data.pitch;
//                //m_data.pitch = m_data.roll;
//                //m_data.roll = t;
//
//                //m_data.yaw = yaw_filter.filter(m_data.yaw);
//                //m_data.pitch = pitch_filter.filter(m_data.pitch);
//                //m_data.roll = roll_filter.filter(m_data.roll);
//                
//
//            } else {
//                return false;
//            }
//        } else {
//            return false;
//        }
//
//        return true;
//    }
//
//    void calibrate(Sensor sensor) override {
//        return;
//    }
//
//private:
//    MPU9250_DMP imu;
//    
//    const float YAW_FILTER_COEF = 0.8f;
//    const float ROLL_FILTER_COEF = 0.8f;
//    const float PITCH_FILTER_COEF = 0.8f;
//
//    Ewma yaw_filter{YAW_FILTER_COEF};
//    Ewma roll_filter{ROLL_FILTER_COEF};
//    Ewma pitch_filter{PITCH_FILTER_COEF};
//};
