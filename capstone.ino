#include "src/aero-cpp-lib/include/Utility.hpp"

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

// Radio usage
#include "Radio.hpp"
RH_RF95 radio {RADIO_CS, RADIO_INT};

// LED usage
#include "LEDS.hpp"

// Pins reading PWM from the RC receiver
#include "Receiver.hpp"

#include "src/uNavAHRS/uNavAHRS.h"
volatile uNavAHRS filter;

#include "src/Arduino-PID-Library/PID_v1.h"

//     Roll PID stuff
int leftOutput, rightOutput; //Output for flaps

double rollSetpoint, rollInput, rollOutput;
static constexpr double KP = 0.65, KI = 0, KD = 0.05;
PID rollPID(&rollInput, &rollOutput, &rollSetpoint, KP, KI, KD, DIRECT);

//   PitchPID Stuff
double pitchSetpoint, pitchInput, pitchOutput;
PID pitchPID(&pitchInput, &pitchOutput, &pitchSetpoint, KP, KI, KD, DIRECT);

//   PitchPID Stuff
double headingSetpoint, headingInput, headingOutput;
PID headingPID(&headingInput, &headingOutput, &headingSetpoint, KP, KI, KD, DIRECT);

// Data timer
unsigned long prev = 0;
const long interval = 1000;

// Servo timer
unsigned long servo_prev = 0;
const long servo_interval = 50;

volatile aero::def::ParsedMessage_t* msg;
aero::Message message_handler;

// Thread
#include <TeensyThreads.h>

volatile bool new_msg = false;

unsigned long last_time = 0;

volatile float new_yaw, new_pitch, new_roll;

void thread_radio() {
  while(true) {
    if( RADIO::receive(radio, msg) == true && new_msg == false) {
      if(msg->m_to == THIS_DEVICE) {
        aero::def::RawMessage_t server_response = message_handler.build(aero::def::ID::G1, aero::def::ID::Plane);
        bool sent = RADIO::respond(radio, server_response);
        new_msg = sent;
      } 
    } 
    
    threads.yield();
    
  }
}

GPS::GPSData gps_data; 
volatile bool new_gps = false;

void thread_gps() {
  while(true) {
    if(GPS::read_sensor(adafruit_gps) == true && new_gps == false) {
      unsigned long curr = millis();
      if(curr - prev >= interval) {
        
        prev = curr;
  
        // Read data
        gps_data = GPS::read_data(adafruit_gps);

        new_gps = true;

      } 
    }
    
    threads.yield(); 
    
  }
}

IMU::MPU9250Data imu_data;
BARO::BaroData baro_data;
volatile bool new_i2c = false;
volatile unsigned long prev_i2c = 0;
const unsigned long interval_i2c = 200;
volatile unsigned long curr_i2c = 0;

void thread_i2c() {
  /* Read I2C Sensors (IMU, Barometer) */ 
  while(true) {
    if(new_i2c == false) {
      curr_i2c = millis();
      
      if(curr_i2c - prev_i2c >= interval_i2c) {
        prev_i2c = curr_i2c;
        imu_data = IMU::read_data(mpu9250);
        baro_data = BARO::read_data(mpl3115);
        
        new_i2c = true;
      }
    } 
  }

  threads.yield();
}

RECEIVER::ReceiverData recv_data;
void thread_manual() {
  while(true) {
    /* If manual mode, rewrite to servos */
    recv_data = RECEIVER::read_data();

    // Get channel data and simply write that to the servos
    // servo.writeMicroseconds or something
  }
}

// This
void thread_auto() {
  /* If auto mode, compute PID and write to servos; every new ahrs */
  while(true) {
    if(new_i2c == true) {
      pitchInput = new_pitch;
      rollInput = new_roll;
      headingInput = new_yaw;

      pitchPID.Compute();
      rollPID.Compute();
      headingPID.Compute();

  //  Serial.print(rollOutput);
  //  Serial.print("\t");
  //  Serial.print(pitchOutput);
  //  Serial.print("\t");
  //  Serial.println(yaw);
  //  Serial.println("**********");
  
  //  Serial.print(pitchInput);
  //  Serial.print(",");
  //  Serial.print(-1*pitchOutput);
  //  Serial.print(",");
  //  Serial.print(rollInput);
  //  Serial.print(",");
  //  Serial.println(-1*rollOutput);
    
  //  leftOutput = ((rollOutput + pitchOutput) / 2) + 90;
  //  rightOutput = ((rollOutput - pitchOutput) / 2) + 90;
  //
  //  left_elevon.write(leftOutput);
  //  right_elevon.write(rightOutput);
      
      new_i2c = false;
    }
    
  }
}

void thread_setpoint() {
  while(true) {
    if(new_gps == true) {
      //GPS::print_data(gps_data);
      //new_gps = false;
    }
  }
}

void setup() {
  msg = new aero::def::ParsedMessage_t();
  
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
  //IMU::calibrate(mpu9250, IMU::GYRO);
  //IMU::calibrate(mpu9250, IMU::MAG);
  //IMU::calibrate(mpu9250, IMU::ACCEL);
  // BARO::calibrate();

  RADIO::init(radio, RADIO_RESET);

  rollSetpoint = 0;
  pitchSetpoint = 0;
  headingSetpoint = 0;

  rollPID.SetOutputLimits(-90, 90);
  pitchPID.SetOutputLimits(-90, 90);
  headingPID.SetOutputLimits(-90, 90);

  rollPID.SetMode(AUTOMATIC);
  pitchPID.SetMode(AUTOMATIC);
  headingPID.SetMode(AUTOMATIC);
  

  // Add, returns an integer id
  // Suspend (id)
  // Restart (id)
  threads.setMicroTimer(1);

  // Always keep radio, gps, and i2c thread alive
  threads.addThread(thread_radio);
  threads.addThread(thread_gps);
  threads.addThread(thread_i2c);

  // Automatic mode; turn on setpoint and automatic mode threads
  threads.addThread(thread_setpoint);
  threads.addThread(thread_auto);

  // Manual mode; turn on manual thread
  threads.addThread(thread_manual);
  
  Serial.println("Setup complete");
}

void parse_cmds() {
   Serial.println("Received new message");

  if(msg->cmds() != NULL) {
    if(aero::bit::read(msg->cmds()->pitch, 0)) {
      Serial.println("Pitch command");
    }

    if(aero::bit::read(msg->cmds()->pitch, 7)) {
      Serial.println("Mode swap command");

      // set flags to false; then terminate auto/setpoint or manual thread. Keep sensor threads alive
    }
  }
}

void update_ypr() {
    filter.update(imu_data.gx,imu_data.gy, imu_data.gz, 
                  imu_data.ax, imu_data.ay, imu_data.az,
                  imu_data.mx, imu_data.my, imu_data.mz);

    new_pitch = filter.getPitch_rad()*180.0f/PI;
    new_roll = filter.getRoll_rad()*180.0f/PI;
    new_yaw = filter.getYaw_rad()*180.0f/PI;
    Serial.print(new_pitch); Serial.print(" "); Serial.print(new_roll); Serial.print(" "); Serial.println(new_yaw);
}

void loop() {
  // Check for new radio commands
  if(new_msg == true) {
      parse_cmds();
      new_msg = false;
  }

  // For some reason, cannot but filter in thread
  if(new_i2c == true) {
    update_ypr();
    new_i2c = false;
  }
}
