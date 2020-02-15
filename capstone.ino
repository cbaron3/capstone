#include "src/aero-cpp-lib/include/Utility.hpp"

// System configuration including pins, debugging, and system mode
#include "Config.hpp"

// IMU functionality based on the MPU9250
#include "IMU.hpp"
IMU* imu;


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

#include "Controller.hpp"

// Controller for pitch
PID roll_control(&CONTROLLER::roll_input, 
                 &CONTROLLER::roll_output, 
                 &CONTROLLER::roll_setpoint, 
                 ROLL_KP, ROLL_KI, ROLL_KD, DIRECT);

// Controller for roll
PID pitch_control(&CONTROLLER::pitch_input, 
                 &CONTROLLER::pitch_output, 
                 &CONTROLLER::pitch_setpoint, 
                 PITCH_KP, PITCH_KI, PITCH_KD, DIRECT);

// Controller for yaw
PID yaw_control(&CONTROLLER::yaw_input, 
                 &CONTROLLER::yaw_output, 
                 &CONTROLLER::yaw_setpoint, 
                 YAW_KP, YAW_KI, YAW_KD, DIRECT);

// Threads
#include "Threads.hpp"

volatile float new_yaw = 0.0f, new_pitch = 0.0f, new_roll = 0.0f;

mode_type mode = DEFAULT_MODE;

/********************************************/
/* Thread for handling radio communications */ 
/********************************************/

volatile aero::def::ParsedMessage_t* msg;
volatile bool new_msg = false;

void thread_radio() {
  // Wait so thread does not start right away
  delay(THREAD_DELAY_MS);
  DEBUG_PRINTLN("Radio thread started");

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
        LEDS::on(RADIO_LED);
        bool sent = RADIO::respond(radio, server_response);
        LEDS::off(RADIO_LED);
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
IMU::Data imu_data;
BARO::BaroData baro_data;

volatile bool new_imu = false;
volatile bool new_baro = false;

void thread_i2c() {
  // Wait so thread does not start right away
  delay(THREAD_DELAY_MS);

  unsigned long prev_imu = 0, curr_imu = 0;

  unsigned long prev_baro = 0, curr_baro = 0;

  // Thread loop
  while(true) {
    
    if(new_imu == false) {
      // Keep update rate for sensors at 20 Hz
      curr_imu = millis();
      if(curr_imu - prev_imu >= IMU_SAMPLE_INTERVAL_MS) {
        prev_imu = curr_imu;

        // Read data
        imu->update();
        
        new_imu = true;
      }
    } 
    
    curr_baro = millis();
    if(curr_baro - prev_baro >= BARO_SAMPLE_INTERVAL_MS) {
      prev_baro = curr_baro;
      baro_data = BARO::read_data(mpl3115);
      
    }

    threads.yield();

  }
}

/********************************************/
/* Thread for handling manual servo control */ 
/********************************************/

void thread_manual() {
  // Wait so thread does not start right away
  delay(THREAD_DELAY_MS);

  unsigned long prev = 0, curr = 0;
  RECEIVER::ReceiverData recv_data;
  
  // Thread loop
  while(true) {

    curr = millis();
    if(curr - prev >= MANUAL_INTERVAL_MS) {
      prev = curr;
      recv_data = RECEIVER::read_data();
      left_elevon.writeMicroseconds(recv_data.recv1);
      right_elevon.writeMicroseconds(recv_data.recv2);

    } 
    // Don't think I should yield this THREADS, it is important
    threads.yield();
  }
}

/***********************************************/
/* Thread for automatic control; stabilization */ 
/***********************************************/

void thread_auto() {
  // Wait so thread does not start right away
  delay(THREAD_DELAY_MS);

  unsigned long curr = 0, prev = 0;

  float left_output = 0.0f, right_output = 0.0f;
  
  while(true) {
    curr = millis();
    if(curr - prev >= AUTO_INTERVAL_MS) {
      prev = curr;

      // Apply mixing if needed
      #if defined(TEST_TRAINER)
        left_output = (-1*CONTROLLER::pitch_output) + 90.0f;
        right_output = (-1*CONTROLLER::roll_output) + 90.0f;
      #else
        leftOutput = ((CONTROLLER::roll_output + CONTROLLER::pitch_output) / 2.0f) + 90.0f;
        rightOutput = ((CONTROLLER::roll_output - CONTROLLER::pitch_output) / 2.0f) + 90.0f;
      #endif
      
      unsigned long left_signal = (unsigned long) map_generic(left_output, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE, SERVO_MIN_MS, SERVO_MAX_MS) - LEFT_ELEVON_MS_OFFSET;
      unsigned long right_signal = (unsigned long) map_generic(right_output, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE, SERVO_MIN_MS, SERVO_MAX_MS) - RIGHT_ELEVON_MS_OFFSET;

      left_elevon.writeMicroseconds(left_signal);
      right_elevon.writeMicroseconds(right_signal);

    } 
    
    threads.yield();

  }  
}

/*************************************/
/* Thread for calculating setpoints */ 
/************************************/

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

/**************************************/
/* Thread for reporting CPU activity */ 
/*************************************/
void thread_activity() {
  while(true) {
    LEDS::on(ACTIVITY_LED);
    threads.delay(500);
    LEDS::off(ACTIVITY_LED);
    threads.delay(500);
  }
}

/**
 * @brief Parse a message for commands and take the appropriate action
 */
void parse_cmds(void) {
  int msg_id = static_cast<int>(msg->m_from);
  DEBUG_PRINT("Received radio message from: "); DEBUG_PRINTLN(NAMES[msg_id]);

  // If we received command information
  if(msg->cmds() != NULL) {

    // Check for the pitch bit
    if(aero::bit::read(msg->cmds()->pitch, 0)) {
      DEBUG_PRINTLN("Pitch Command");
    }

    if(aero::bit::read(msg->cmds()->pitch, 7)) {
      DEBUG_PRINT("Mode Swap Command: ");

      if(mode == MODE_MANUAL) {
        // Device is currently in manual mode therefore we will swap to auto
        DEBUG_PRINTLN("Changing from manual to auto");

        // If we were in manual mode, stop the manual thread and interrupts; start the setpoint and auto thread
        THREADS::suspend(MANUAL_ID);
        RECEIVER::terminate_interrupts();
        
        THREADS::restart(AUTO_ID);
        THREADS::restart(SETPOINT_ID);
        
        // Assign mode
        mode = MODE_AUTO;
        LEDS::off(MODE_LED);
      } else {
        // Device is currently in auto mode therefore we will swap to manual
        DEBUG_PRINTLN("Changing from auto to manual");

        // If we were in auto mode, stop the auto and setpoint thread; start the manual mode thread and interrupts
        THREADS::suspend(AUTO_ID);
        THREADS::suspend(SETPOINT_ID);

        RECEIVER::start_interrupts();
        THREADS::restart(MANUAL_ID);

        // Assign mode
        mode = MODE_MANUAL;
        LEDS::on(MODE_LED);
      }
    }
  }
}

/**
 * @brief Update the kalman filter values
 * @details Results in a flag being set that lets the auto thread know that new data has been received
 */
void update_controller(void) {
    
    
    // Set PID inputs
    CONTROLLER::pitch_input = imu->get_pitch();
    CONTROLLER::roll_input  = imu->get_roll();
    CONTROLLER::yaw_input   = imu->get_yaw();

    // TODO: Add moving average to the roll, pitch, and yaw values
    DEBUG_PRINT("PID Input: ");
    DEBUG_PRINT("\tRoll: "); DEBUG_PRINT(CONTROLLER::roll_input);
    DEBUG_PRINT("\tPitch: "); DEBUG_PRINT(CONTROLLER::pitch_input);
    DEBUG_PRINT("\tYaw: "); DEBUG_PRINT(CONTROLLER::yaw_input);

    // Update controller values
    CONTROLLER::update(roll_control, pitch_control, yaw_control);
    
    DEBUG_PRINT("PID Output: ");
    DEBUG_PRINT("\tRoll: "); DEBUG_PRINT(CONTROLLER::roll_output);
    DEBUG_PRINT("\tPitch: "); DEBUG_PRINT(CONTROLLER::pitch_output);
    DEBUG_PRINT("\tYaw: "); DEBUG_PRINT(CONTROLLER::yaw_output);
}

void setup() {
  // Initialize serial
  DEBUG_START(115200);

  imu = new MPU9250_AHRS();

  // Initialize sensors
  imu->init();
  GPS::init(adafruit_gps);
  BARO::init(mpl3115);

  // Initialize servos and receiver input
  ACTUATORS::init(left_elevon, right_elevon);
  RECEIVER::init();
  RECEIVER::start_interrupts();
  
  // Initialize radio and msg that contains last received radio message
  RADIO::init(radio, RADIO_RESET);
  msg = new aero::def::ParsedMessage_t();

  // Call calibration routines if necessary
  // NOTE: Accel and Mag save results in EEPROM
  if(CALIBRATE_ACCEL) imu->calibrate(IMU::ACCEL);
  if(CALIBRATE_GYRO)  imu->calibrate(IMU::GYRO);
  if(CALIBRATE_MAG)   imu->calibrate(IMU::MAG);
  if(CALIBRATE_BARO)  BARO::calibrate(mpl3115, ALTITUDE_BIAS);

  CONTROLLER::init(roll_control, pitch_control, yaw_control);

  // Initialization LEDs
  LEDS::init();
  LEDS::on(POWER_LED);

  // Initialize threads; thread time slice at 1 ms
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

  // Handle boot mode
  if(mode == MODE_MANUAL) {
    // If we boot in manual mode, immediately suspend the automatic and setpoint threads
    THREADS::suspend(AUTO_ID);
    THREADS::suspend(SETPOINT_ID);

    LEDS::on(MODE_LED);
  } else {
    // If we boot in auto mode, immediately suspend the manual thread and terminate interrupts
    THREADS::suspend(MANUAL_ID);
    RECEIVER::terminate_interrupts();

    LEDS::off(MODE_LED);
  }
  
  // Thread to monitor CPU activity
  THREADS::ids[ACTIVITY_ID] = threads.addThread(thread_activity);
  
  DEBUG_PRINTLN("Setup Complete");
}

void loop() {
  // Check for new radio commands
  if(new_msg == true) {
      parse_cmds();
      new_msg = false;
  }

  // For some reason, cannot put filter in thread
  if(new_imu == true) {
    update_controller();
    new_imu = false;
  }
}
