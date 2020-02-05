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
Servo left_elevon, right_elevon;

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
static constexpr double KP = 1, KI = 0, KD = 0;
PID rollPID(&rollInput, &rollOutput, &rollSetpoint, KP, KI, KD, DIRECT);

//   PitchPID Stuff
double pitchSetpoint, pitchInput, pitchOutput;
PID pitchPID(&pitchInput, &pitchOutput, &pitchSetpoint, KP, KI, KD, DIRECT);

//   PitchPID Stuff
double headingSetpoint, headingInput, headingOutput;
PID headingPID(&headingInput, &headingOutput, &headingSetpoint, KP, KI, KD, DIRECT);

// Threads
#include "Threads.hpp"

volatile float new_yaw = 0.0f, new_pitch = 0.0f, new_roll = 0.0f;

enum mode_type {MODE_MANUAL = 0, MODE_AUTO = 1};
mode_type mode = MODE_MANUAL;

const unsigned long THREAD_DELAY_MS = 1000;

/********************************************/
/* Thread for handling radio communications */ 
/********************************************/

volatile aero::def::ParsedMessage_t* msg;
volatile bool new_msg = false;

void thread_radio() {
  // Wait so thread does not start right away
  delay(THREAD_DELAY_MS);

  // Message handler for building a response
  aero::Message message_handler;
  
  // Thread loop
  while(true) {
    
    // Keep checking for a new radio message
    if( RADIO::receive(radio, msg) == true) {
      
      // Only accept message if it was addressed to this grlider
      if(msg->m_to == THIS_DEVICE) {

        // Build response and set flag if response was succesfully sent
        aero::def::RawMessage_t server_response = message_handler.build(aero::def::ID::G1, aero::def::ID::Gnd);
        bool sent = RADIO::respond(radio, server_response);
        new_msg = sent;
      } 
    } 
    
    threads.yield();
  }
}

/******************************/
/* Thread for polling the GPS */ 
/******************************/

GPS::GPSData gps_data; 
volatile bool new_gps = false;

const long GPS_INTERVAL_MS = 1000;

void thread_gps() {
  // Wait so thread does not start right away
  delay(THREAD_DELAY_MS);
  unsigned long prev = 0, curr = 0;
  
  // Thread loop
  while(true) {

    // Check if new gps data is available to parse
    if(GPS::read_sensor(adafruit_gps) == true && new_gps == false) {
      
      // Limit update rate for GPS to make sure parsing is not affected
      curr = millis();
      if(curr - prev >= GPS_INTERVAL_MS) {
        
        prev = curr;
  
        // Read data
        gps_data = GPS::read_data(adafruit_gps);

        new_gps = true;

      } 
    }
    
    threads.yield(); 
  }
}

/*************************************/
/* Thread for polling the I2C sensor */ 
/*************************************/

IMU::MPU9250Data imu_data;
BARO::BaroData baro_data;

volatile bool new_i2c = false;
// use 100
const unsigned long I2C_INTERVAL_MS = 20;

void thread_i2c() {
  // Wait so thread does not start right away
  delay(THREAD_DELAY_MS);

  unsigned long prev = 0, curr = 0;

  // Thread loop
  while(true) {
    
    if(new_i2c == false) {

      // Keep update rate for sensors at 20 Hz
      curr = millis();
      if(curr - prev >= I2C_INTERVAL_MS) {
        prev = curr;

        // Read data
        imu_data = IMU::read_data(mpu9250);
        baro_data = BARO::read_data(mpl3115);
        
        new_i2c = true;
      }
    }

   threads.yield();
  }
}

/********************************************/
/* Thread for handling manual servo control */ 
/********************************************/

const unsigned long MANUAL_INTERVAL_MS = 10;

RECEIVER::ReceiverData recv_data;

void thread_manual() {
  // Wait so thread does not start right away
  delay(THREAD_DELAY_MS);

  // Only used for debugging
  int servo_ms = 1000;
    
  unsigned long prev = 0, curr = 0;
  
  // Thread loop
  while(true) {

    // Read data
    recv_data = RECEIVER::read_data();
      
    curr = millis();
    if(curr - prev >= MANUAL_INTERVAL_MS) {
      prev = curr;
      
//      // Could use write microseconds or analog write
//      left_elevon.writeMicroseconds(recv_data.recv1);
//      right_elevon.writeMicroseconds(recv_data.recv2);

      if(!DEBUG_MANUAL_MODE) {
        left_elevon.writeMicroseconds(recv_data.recv1);
        right_elevon.writeMicroseconds(recv_data.recv2);
      } else {
        // Debugging manual mode
        left_elevon.writeMicroseconds(servo_ms);
        right_elevon.writeMicroseconds(servo_ms);
  
        servo_ms += 1;
        if(servo_ms >= 2000) {
          servo_ms = 1000;
        }
      }

    } else {
      threads.yield();
    }

    // Don't think I should yield this THREADS, it is important
    // threads.yield();
  }
}

/***********************************************/
/* Thread for automatic control; stabilization */ 
/***********************************************/

volatile bool new_ypr = false;
const unsigned long AUTO_INTERVAL_MS = 5;

void thread_auto() {
  // Wait so thread does not start right away
  delay(THREAD_DELAY_MS);

  unsigned long prev = 0, curr = 0;

  int left_angle = 0;
  int right_angle  = 0;

  // Thread loop; update rate dependent on the update rate of I2C sensors
  while(true) {
    if(new_ypr == true) {
      // Set PID inputs
      pitchInput = new_pitch;
      rollInput = new_roll;
      headingInput = new_yaw;

      // PID computations
      pitchPID.Compute();
      rollPID.Compute();
      headingPID.Compute();

      rollOutput = -1 * rollOutput;
      pitchOutput = -1 * pitchOutput;
      headingOutput = -1 * headingOutput;

      //Serial.println(rollOutput);
      //Serial.println(pitchOutput);

      // Basic elevon mixing; needs to be improveed
      leftOutput = ((rollOutput + pitchOutput) / 2) + 90;
      rightOutput = ((rollOutput - pitchOutput) / 2) + 90;

      

      if(left_angle > leftOutput) {
        left_angle -= 1;
      } else if(left_angle < leftOutput) {
        left_angle += 1;
      }

      if(right_angle > rightOutput) {
        right_angle -= 1;
      } else if(right_angle < rightOutput) {
        right_angle += 1;
      }

      Serial.print("L -- "); Serial.print(leftOutput); Serial.print(" "); Serial.println(left_angle);
      Serial.print("R -- "); Serial.print(rightOutput); Serial.print(" "); Serial.println(right_angle);
      
      left_elevon.write(left_angle);
      right_elevon.write(right_angle);

      //delay(5);
      
      new_ypr = false;
    }

    // Don't think I should yield this THREADS, it is important
    // threads.yield();
  }
}

void thread_setpoint() {
   // Wait so thread does not start right away
  delay(THREAD_DELAY_MS);

  // Thread loop
  while(true) {
    if(new_gps == true) {
      /* Compute setpoint here */
      new_gps = false;
    }

    threads.yield();
  }
}

void parse_cmds() {
   Serial.println("Received new message");

  if(msg->cmds() != NULL) {
    if(aero::bit::read(msg->cmds()->pitch, 0)) {
      Serial.println("Pitch command");
    }

    if(aero::bit::read(msg->cmds()->pitch, 7)) {
      Serial.println("Mode swap command");

      if(mode == MODE_MANUAL) {
        Serial.println("Starting auto mode...");
        // If we were in manual mode, stop the manual thread and start the setpoint and auto thread
        THREADS::suspend(MANUAL_ID);
        RECEIVER::terminate_interrupts();
        
        THREADS::restart(AUTO_ID);
        THREADS::restart(SETPOINT_ID);
        
        mode = MODE_AUTO;
      } else {
        Serial.println("Starting manual mode...");
        // if we were in auto mode, stop the auto and setpoint thread and start the manual mode thread
        THREADS::suspend(AUTO_ID);
        THREADS::suspend(SETPOINT_ID);

        RECEIVER::start_interrupts();
        THREADS::restart(MANUAL_ID);
        mode = MODE_MANUAL;
      }
    }
  }
}

void update_ypr() {
    // NOTE: This will take two minute for good results!
    filter.update(imu_data.gx,imu_data.gy, imu_data.gz, 
                  imu_data.ax, imu_data.ay, imu_data.az,
                  imu_data.mx, imu_data.my, imu_data.mz);

    new_pitch = filter.getPitch_rad()*180.0f/PI;
    new_roll = filter.getRoll_rad()*180.0f/PI;
    new_yaw = filter.getYaw_rad()*180.0f/PI;
    
    new_ypr = true;
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
  RECEIVER::start_interrupts();
  
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
  rollPID.SetSampleTime(I2C_INTERVAL_MS);
  pitchPID.SetMode(AUTOMATIC);
  pitchPID.SetSampleTime(I2C_INTERVAL_MS);
  headingPID.SetMode(AUTOMATIC);
  headingPID.SetSampleTime(I2C_INTERVAL_MS);
  
  threads.setMicroTimer(1);

  // Always keep radio, gps, and i2c thread alive
  THREADS::ids[RADIO_ID]  = threads.addThread(thread_radio);
  THREADS::ids[GPS_ID]    = threads.addThread(thread_gps);
  THREADS::ids[I2C_ID]    = threads.addThread(thread_i2c);
  
  // Automatic mode; turn on setpoint and automatic mode threads
  THREADS::ids[SETPOINT_ID] = threads.addThread(thread_setpoint);
  THREADS::ids[AUTO_ID]     = threads.addThread(thread_auto);

  // Manual mode; turn on manual thread
  THREADS::ids[MANUAL_ID] = threads.addThread(thread_manual);

  // TODO: MAKE A THREAD THAT MONITORS STACK USAGE

  if(mode == MODE_MANUAL) {
    THREADS::suspend(AUTO_ID);
    THREADS::suspend(SETPOINT_ID);
    RECEIVER::terminate_interrupts();
  } else {
    THREADS::suspend(MANUAL_ID);
  }
  
  Serial.println("Setup complete");
}

void loop() {
  // Check for new radio commands
  if(new_msg == true) {
      parse_cmds();
      new_msg = false;
  }

  // For some reason, cannot put filter in thread
  if(new_i2c == true) {
    update_ypr();
    new_i2c = false;
  }
}
