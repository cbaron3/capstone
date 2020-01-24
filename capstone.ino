// System configuration including pins, debugging, and system mode
#include "Config.hpp"

// IMU functionality based on the MPU9250
#include "IMU.hpp"
MPU9250 mpu9250(Wire, 0x68);

// GPS functionalty based on Adafruit GPS using AdafruitGPS interrupt library
#include "GPS.hpp"
Adafruit_GPS adafruit_gps(GPS_PORT);

// Other GPS implementation using TinyGPS instead. Not ideal cause requires you to spend time waiting for buffer to fill
#include "OtherGPS.hpp"

// Barometer functionality based on the MPL3115AS
#include "Barometer.hpp"
MPL3115A2 mpl3115;

// Servo functionality
#include "Actuators.hpp"
PWMServo left_elevon, right_elevon;

// LED usage
#include "LEDS.hpp"

// Pins reading PWM from the RC receiver
#include "Receiver.hpp"

#include "src/uNavAHRS/uNavAHRS.h"
uNavAHRS filter;

#include "src/Arduino-PID-Library/PID_v1.h"

//     Roll PID stuff
double roll, pitch, heading;
int leftOutput, rightOutput; //Output for flaps

double rollSetpoint, rollInput, rollOutput;
static constexpr double KP = 0.65, KI = 0, KD = 0.05;
PID rollPID(&rollInput, &rollOutput, &rollSetpoint, KP, KI, KD, DIRECT);

//   PitchPID Stuff
double pitchSetpoint, pitchInput, pitchOutput;
PID pitchPID(&pitchInput, &pitchOutput, &pitchSetpoint, KP, KI, KD, DIRECT);

// Data timer
unsigned long prev = 0;
const long interval = 2000;

// Servo timer
unsigned long servo_prev = 0;
const long servo_interval = 50;

void setup() {
  if(DEBUG_MODE) {
    Serial.begin(115200);
    delay(2500);
  }

  // Initialize sensors
  IMU::init(mpu9250);
  GPS::init(adafruit_gps);
  BARO::init(mpl3115);

  // Initialize servos and receiver input
  ACTUATORS::init(left_elevon, right_elevon);
  RECEIVER::init();
  RECEIVER::init_interrupts();
  
  // Initialize leds with simple animation
  LEDS::init();
  LEDS::sweep(250);

  // TODO: Calibration routes only if specific mode. Need to calibrate gyro and barometer everytime
  IMU::calibrate(mpu9250, IMU::GYRO);
  //IMU::calibrate(mpu9250, IMU::MAG);
  //IMU::calibrate(mpu9250, IMU::ACCEL);
  // BARO::calibrate();

  rollSetpoint = 0;
  pitchSetpoint = 0;

  rollPID.SetOutputLimits(-90, 90);
  pitchPID.SetOutputLimits(-90, 90);

  rollPID.SetMode(AUTOMATIC);
  pitchPID.SetMode(AUTOMATIC);
}

void loop() {
  // Only read data from GPS if valid NMEA received
//  if(GPS::read_sensor(adafruit_gps) == true) {
//    unsigned long curr = millis();
//    if(curr - prev >= interval) {
//      prev = curr;

      // Read data
//      GPS::GPSData gps_data = GPS::read_data(adafruit_gps);
//      IMU::MPU9250Data imu_data = IMU::read_data(mpu9250);
//      BARO::BaroData baro_data = BARO::read_data(mpl3115);
//      RECEIVER::ReceiverData recv_data = RECEIVER::read_data();
//
//      filter.update(imu_data.gx,imu_data.gy, imu_data.gz, 
//                    imu_data.ax, imu_data.ay, imu_data.az,
//                    imu_data.mx, imu_data.my, imu_data.mz);
//        
      // Print data
      // GPS::print_data(gps_data);
      // IMU::print_data(imu_data);
      // BARO::print_data(baro_data);
      // RECEIVER::print_data(recv_data);
//      Serial.println("");
//      
//    }
//  }

//  unsigned long servo_curr = millis();
//  if(servo_curr - servo_prev >= servo_interval) {
//    servo_prev = servo_curr;
//    // ACTUATORS::sweep(left_elevon, right_elevon);
//  }

  IMU::MPU9250Data imu_data = IMU::read_data(mpu9250);

  filter.update(imu_data.gx,imu_data.gy, imu_data.gz, 
              imu_data.ax, imu_data.ay, imu_data.az,
              imu_data.mx, imu_data.my, imu_data.mz);

  rollInput = filter.getPitch_rad()*180.0f/PI;
  pitchInput = filter.getRoll_rad()*180.0f/PI;
  float yaw = filter.getYaw_rad()*180.0f/PI;

//  Serial.print(rollInput);
//  Serial.print("\t");
//  Serial.print(pitchInput);
//  Serial.print("\t");
//  Serial.println(yaw);
  
  rollPID.Compute();
  pitchPID.Compute();

//  Serial.print(rollOutput);
//  Serial.print("\t");
//  Serial.print(pitchOutput);
//  Serial.print("\t");
//  Serial.println(yaw);
//  Serial.println("**********");

  Serial.print(pitchInput);
  Serial.print(",");
  Serial.print(-1*pitchOutput);
  Serial.print(",");
  Serial.print(rollInput);
  Serial.print(",");
  Serial.println(-1*rollOutput);
  
  leftOutput = ((rollOutput + pitchOutput) / 2) + 90;
  rightOutput = ((rollOutput - pitchOutput) / 2) + 90;

  left_elevon.write(leftOutput);
  right_elevon.write(rightOutput);

  delay(5);
}
