#include "src/aero-cpp-lib/include/Utility.hpp"

// System configuration including pins, debugging, and system mode
#include "Config.hpp"

// Testing out the performance of adding an exponential averaging filter
#include "src/EWMA/Ewma.h"
Ewma roll_filter(1.0), pitch_filter(1.0), yaw_filter(1.0), left_filter(0.9), right_filter(0.9);

// IMU functionality based on the MPU9250
#include "IMU.hpp"
MPU9250 mpu9250{Wire, 0x68};
uNavAHRS filter;  // Kalman filter for IMU data

// GPS functionalty based on Adafruit GPS using AdafruitGPS interrupt library
#include "GPS.hpp"
Adafruit_GPS adafruit_gps(GPS_PORT);

// Other GPS implementation using TinyGPS instead. Not ideal cause requires you to spend time waiting for buffer to fill
#include "OtherGPS.hpp"

// Barometer functionality based on the MPL3115AS
#include "Barometer.hpp"
MPL3115A2 mpl3115;

// Pins reading PWM from the RC receiver
#include "Receiver.hpp"

// Servo functionality
#include "Actuators.hpp"
Servo left_elevon, right_elevon, rudder;

// Radio usage
#include "Radio.hpp"
aero::def::Status_t status;
RH_RF95 radio {RADIO_CS, RADIO_INT};

// LED usage
#include "LEDS.hpp"

// PID controller usage
#include "Controller.hpp"

// Setpoint calculator
#include "Setpoint.hpp"

// Controller input
volatile float new_yaw = 0.0f, new_pitch = 0.0f, new_roll = 0.0f;

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

// System mode; auto or manual
mode_type mode = DEFAULT_MODE;

/********************************************/
/* Thread for handling radio communications */ 
/********************************************/

volatile aero::def::ParsedMessage_t* msg; // Poitner to last received message
volatile bool new_msg = false;            // Flag to notify that a new message has been arrived

// Message handler for building a response
volatile aero::Message message_handler;


void thread_radio() {
  // Wait so thread does not start right away
  delay(THREAD_DELAY_MS);
  DEBUG_PRINTLN("Radio thread started");

  
  unsigned long prev = 0, curr = 0;
  
  // Thread loop
  while(true) {
    
    // Check radio on interval
    curr = millis();
    if(curr - prev >= RADIO_INTERVAL_MS) { 
      prev = curr;

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
    }

    // Yield current thread if not executing
    threads.yield(); 
    
  }
}

/******************************/
/* Thread for polling the GPS */ 
/******************************/

GPS::Data gps_data; 
volatile bool new_gps = false;

void thread_gps() {
  // Wait so thread does not start right away
  delay(THREAD_DELAY_MS);
  DEBUG_PRINTLN("GPS thread started");

  unsigned long prev = 0, curr = 0;
  
  // Thread loop
  while(true) {

    // Check if new gps data is available to parse
    if(GPS::check(adafruit_gps) == true && new_gps == false) {
      
      // Limit update rate for GPS to make sure parsing is not affected
      curr = millis();
      if(curr - prev >= GPS_INTERVAL_MS) {
        prev = curr;
  
        // Read data
        gps_data = GPS::read(adafruit_gps);

        new_gps = true;

      } 
    }

    // Yield current thread if not executing
    threads.yield(); 
    
  }
}

/*************************************/
/* Thread for polling the I2C sensor */ 
/*************************************/
IMU::Data imu_data;
BARO::Data baro_data;

// Flags to indicate when there is new barometer and imu data
volatile bool new_imu = false;
volatile bool new_baro = false;

void thread_i2c() {
  // Wait so thread does not start right away
  delay(THREAD_DELAY_MS);
  DEBUG_PRINTLN("I2C thread started");

  unsigned long prev_imu = 0, curr_imu = 0;
  unsigned long prev_baro = 0, curr_baro = 0;

  // Thread loop
  while(true) {
    
    // Only update IMU when data has been processed by filter; hence the flag
    if(new_imu == false) {

      // Keep fixed update rate
      curr_imu = millis();
      if(curr_imu - prev_imu >= IMU_SAMPLE_INTERVAL_MS) {
        prev_imu = curr_imu;

        // Read data
        imu_data = IMU::read(mpu9250);
        
        new_imu = true;
      }
    } 
    
    // Keep fixed update rate
    curr_baro = millis();
    if(curr_baro - prev_baro >= BARO_SAMPLE_INTERVAL_MS) {
      prev_baro = curr_baro;

      baro_data = BARO::read(mpl3115);
    }

    // Yield current thread if not executing
    threads.yield();

  }
}

/********************************************/
/* Thread for handling manual servo control */ 
/********************************************/

void thread_manual() {
  // Wait so thread does not start right away
  delay(THREAD_DELAY_MS);
  DEBUG_PRINTLN("Manual thread started");

  unsigned long prev = 0, curr = 0;
  RECEIVER::Data recv_data;
  
  // Thread loop
  while(true) {

    curr = millis();
    if(curr - prev >= MANUAL_INTERVAL_MS) {
      prev = curr;

      // Read data from the receiver
      recv_data = RECEIVER::read();

      ACTUATORS::command_ms(left_elevon, recv_data.recv1);
      ACTUATORS::command_ms(right_elevon, recv_data.recv2);
      ACTUATORS::command_ms(rudder, recv_data.recv3);
    } 
  }
}

/************************************************************************************/
/* Thread for automatic control; stabilization (CURRENTLY ONLY TESTING PITCH, ROLL) */ 
/************************************************************************************/

// Flag to track if the filter has produced updated yaw, pitch, and roll values
volatile bool updated = false;

// Initial position of the elevon motors
float left_output = 90.0f, right_output = 90.0f, rudder_output = 0.0f;

void thread_auto() {
  // Wait so thread does not start right away
  delay(THREAD_DELAY_MS);
  DEBUG_PRINTLN("Auto thread started");

  unsigned long curr = 0, prev = 0;

  while(true) {

    curr = millis();
    if(curr - prev >= AUTO_INTERVAL_MS && updated == false) {
      prev = curr;
  
      float left_target = 0.0f, right_target = 0.0f, rudder_target = 0.0f;

      // Apply mixing if needed
      #if defined(TEST_TRAINER)
        left_target = (-1*CONTROLLER::pitch_output) + 90.0f;
        // left_target = left_filter.filter(left_target);
        right_target = (-1*CONTROLLER::roll_output) + 90.0f;
        // right_target = right_filter.filter(right_target);
        rudder_target = (-1*CONTROLLER::yaw_output);
      #else
        // Elevon mixing
        left_target = ((CONTROLLER::roll_output + CONTROLLER::pitch_output) / 2.0f) + 90.0f;
        right_target = ((CONTROLLER::roll_output - CONTROLLER::pitch_output) / 2.0f) + 90.0f;
        rudder_target = (-1*CONTROLLER::yaw_output);
      #endif

      // Ramp servo output towards target signal instead of writing the target directly to minimize servos resetting
      if(left_target > left_output) {
        left_output += 2.0f;
      } else if (left_target < left_output) {
        left_output -= 2.0f;
      }

      if(right_target > right_output) {
        right_output += 2.0f;
      } else if (right_target < right_output) {
        right_output -= 2.0f;
      }

      if(rudder_target > rudder_output) {
        rudder_output += 2.0f;
      } else if (rudder_target < rudder_output) {
        rudder_output -= 2.0f;
      }

      // Map output angle to servo millisecond value
      unsigned long left_signal = (unsigned long) map_generic(left_output, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE, SERVO_MIN_MS, SERVO_MAX_MS) - LEFT_ELEVON_MS_OFFSET;
      unsigned long right_signal = (unsigned long) map_generic(right_output, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE, SERVO_MIN_MS, SERVO_MAX_MS) - RIGHT_ELEVON_MS_OFFSET;
      unsigned long rudder_signal = (unsigned long) map_generic(rudder_output, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE, SERVO_MIN_MS, SERVO_MAX_MS);

      ACTUATORS::command_ms(left_elevon, left_signal);
      ACTUATORS::command_ms(right_elevon, left_signal);
      ACTUATORS::command_ms(rudder, left_signal);

      updated = true;
    } 
  }  
}

/*************************************/
/* Thread for calculating setpoints */ 
/************************************/

void thread_setpoint() {
  // Wait so thread does not start right away
  delay(THREAD_DELAY_MS);
  DEBUG_PRINTLN("Setpoint thread started");

  bool updated = false;

  SETPOINT::Angles plane_angle, to_target_angle;

  // Thread loop
  while(true) {
    if(new_gps == true) {
      /* Compute setpoint here */
      
      if(gps_data.fix == true) {
         updated = SETPOINT::update(gps_data.coord.lat, gps_data.coord.lon, baro_data.delta_altitude);

         if(updated = true){
            // Get target and plane angles
            plane_angle = SETPOINT::get_plane_angle();
            to_target_angle = SETPOINT::get_target_angle();

            // CONTROLLER::roll_setpoint = ...;
            // CONTROLLER::pitch_setpoint = ...;
            // CONTROLLER::yaw_setpoint = ...;
         }
      }
      
      new_gps = false;
    }

    // Yield current thread if not executing
    threads.yield();
  }
}

/**************************************/
/* Thread for signallng CPU activity */ 
/*************************************/
void thread_activity() {
  DEBUG_PRINTLN("Activity thread started");

  while(true) {
    LEDS::on(ACTIVITY_LED);
    threads.delay(500); // Calls yield for 500 ms
    LEDS::off(ACTIVITY_LED);
    threads.delay(500);
  }
}

void thread_debug() {
  // Wait so thread does not start right away
  delay(THREAD_DELAY_MS);
  DEBUG_PRINTLN("Debug thread started");

  unsigned long curr = 0, prev = 0;
  while(true) {
    curr = millis();
    if(curr - prev >= DEBUG_INTERVAL_MS) {

      IMU::print(imu_data);
      BARO::print(baro_data);
      GPS::print(gps_data);
      
    }

    // Yield current thread if not executing
    threads.yield();
  }
}

void elevons_up() {
  // TODO: 
  left_elevon.write(SAFETY_LEFT_ELEVON_ANGLE);
  right_elevon.write(SAFETY_RIGHT_ELEVON_ANGLE);
  rudder.write(SAFETY_RUDDER_ANGLE);
}

void elevons_level() {
  // TODO: 
  left_elevon.write(DEFAULT_LEFT_ELEVON_ANGLE);
  right_elevon.write(DEFAULT_RIGHT_ELEVON_ANGLE);
  rudder.write(DEFAULT_RUDDER_ANGLE);
}

void to_safety_mode(void) {
  // Cancel all motor threads
  THREADS::suspend(AUTO_ID);
  THREADS::suspend(SETPOINT_ID);
  
  THREADS::suspend(MANUAL_ID);
  RECEIVER::terminate_interrupts();

  elevons_up();

  // Update mode
  mode = MODE_SAFETY;
  LEDS::off(MODE_LED);
}

void to_auto_mode(void) {
  THREADS::restart(AUTO_ID);
  THREADS::restart(SETPOINT_ID);

  THREADS::suspend(MANUAL_ID);
  RECEIVER::terminate_interrupts();

  mode = MODE_AUTO;
  LEDS::off(MODE_LED);
}

void to_manual_mode(void) {
  THREADS::suspend(AUTO_ID);
  THREADS::suspend(SETPOINT_ID);

  RECEIVER::start_interrupts();
  THREADS::restart(MANUAL_ID);

  mode = MODE_MANUAL;
  LEDS::on(MODE_LED);
}

void to_idle_mode(void) {
  THREADS::suspend(AUTO_ID);
  THREADS::suspend(SETPOINT_ID);

  THREADS::suspend(MANUAL_ID);
  RECEIVER::terminate_interrupts();

  elevons_level();

  mode = MODE_IDLE;
  LEDS::off(MODE_LED);
}

void glider_state_machine(uint8_t command) {
  using namespace aero;
  using namespace aero::def;

  switch(mode) {
    // In IDLE mode, sensors are being polled. Can move into AUTO state when Pitch:5 (ENGAGE/DISENGAGE)
    case MODE_IDLE: {
      if(command == CMD_TOGGLE_ENGAGE) {
        bit::set(status.state, STATE_ENGAGED_MODE);
        message_handler.add_status(status);
        to_auto_mode();
      }
    } break;

    // In AUTO mode, sensors are being polled and elevons react to orientation.
    // Can move into
      // IDLE when Pitch:5 (ENGAGE/DISENGAGE). Stop auto thread, elevons level
      // MANUAL when Pitch:7 (MODE SWAP). Stop auto thread, start interrupts, start manual thread
      // SAFETY when Pitch:0/1 dependeing on device. Stop auto thread, elevons up
    case MODE_AUTO: {
      if(command == CMD_TOGGLE_ENGAGE) {
        to_idle_mode();
      } else if(command == CMD_MODE_SWAP) {
        to_manual_mode();
      } else if( (command == CMD_PITCH_UP_GLIDER1 && THIS_DEVICE == ID::G1) || 
                 (command == CMD_PITCH_UP_GLIDER2 && THIS_DEVICE == ID::G2) ) {
        to_safety_mode();
      }

    } break;

    // In MANUAL mode, sensors are being polled but elevons react to receiver inputs.
    // Can move into
      // AUTO when Pitch:7 (MODE SWAP). Stop manual thread, stop interrupts, start auto thread
      // SAFETY when Pitch:0/1 depending on device. Stop manual thread, stop interrupts, elevons up
    case MODE_MANUAL: {
      if(command == CMD_MODE_SWAP) {
        to_auto_mode();
      } else if( (command == CMD_PITCH_UP_GLIDER1 && THIS_DEVICE == ID::G1) || 
                 (command == CMD_PITCH_UP_GLIDER2 && THIS_DEVICE == ID::G2) ) {
        to_safety_mode();
      }

    } break;

    // In SAFETY mode, sensors are being polled. Elevons are up and will not move. 
    case MODE_SAFETY: {
      // Do nothing
    } break;

    // Unrecognized mode, state machine should never reach here
    default: {
      bit::set(status.state, STATE_UNDEFINED_MODE);
      message_handler.add_status(status);
    } break;
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

    /*
     * PITCH:0 - Glider 1 Pitch Up. Only if THIS_DEVICE == aero::def::G1
     * PITCH:1 - Glider 2 Pitch Up. Only if THIS_DEVICE == aero::def::G2
     * PITCH:6 - Test Comms; Send back empty message aka do nothing on parse. Could add data
     * PITCH:7 - Auto/Manual Mode Swap. 
     */
    
    using namespace aero;
    using namespace aero::def;

    // Extract bit fields
    uint8_t pitch_field = msg->cmds()->pitch;

    // If the command is to initiate a comms test, ignore state. Else pass along command to state machine
    if(pitch_field == CMD_COMMS_TEST) {
        DEBUG_PRINTLN("Comms Test Received");

        if(gps_data.fix == true) {
          bit::set(status.state, STATE_GPS_FIX);
        } else {
          bit::clear(status.state, STATE_GPS_FIX);
        }

        message_handler.add_status(status);
    } else {
      glider_state_machine(pitch_field);
    }
  } else {
    // No glider commands
  }
}

/**
 * @brief Update the kalman filter values
 * @details Results in a flag being set that lets the auto thread know that new data has been received
 */
void update_controller(void) {
    if(updated == true) {
    
      filter.update(imu_data.gx,imu_data.gy, imu_data.gz, 
                    imu_data.ax, imu_data.ay, imu_data.az,
                    imu_data.mx, imu_data.my, imu_data.mz);
  
      new_pitch = filter.getPitch_rad()*RAD_TO_DEG;
      new_roll =  filter.getRoll_rad()*RAD_TO_DEG;
      new_yaw =   filter.getYaw_rad()*RAD_TO_DEG;
      
      // Set PID inputs
      CONTROLLER::pitch_input = new_pitch;
      CONTROLLER::roll_input  = new_roll;
      CONTROLLER::yaw_input   = new_yaw;
  
      // TODO: Add moving average to the roll, pitch, and yaw values
      DEBUG_PRINT("PID Input: ");
      DEBUG_PRINT("\tRoll: "); DEBUG_PRINT(CONTROLLER::roll_input);
      DEBUG_PRINT("\tPitch: "); DEBUG_PRINT(CONTROLLER::pitch_input);
      DEBUG_PRINT("\tYaw: "); DEBUG_PRINTLN(CONTROLLER::yaw_input);
  
      // Update controller values
      CONTROLLER::update(roll_control, pitch_control, yaw_control);
  
      CONTROLLER::roll_output = roll_filter.filter(CONTROLLER::roll_output);
      CONTROLLER::pitch_output = pitch_filter.filter(CONTROLLER::pitch_output);
      CONTROLLER::yaw_output = yaw_filter.filter(CONTROLLER::yaw_output);
      
      DEBUG_PRINT("PID Output: ");
      DEBUG_PRINT("\tRoll: "); DEBUG_PRINT(CONTROLLER::roll_output);
      DEBUG_PRINT("\tPitch: "); DEBUG_PRINT(CONTROLLER::pitch_output);
      DEBUG_PRINT("\tYaw: "); DEBUG_PRINTLN(CONTROLLER::yaw_output);

      updated  = false;
      
    }
}

void setup() {
  // Initialize serial monitor if required
  DEBUG_START(115200);
  DEBUG_PRINTLN("Setup Starting");

  // Initialization LEDs
  LEDS::init();
  LEDS::on(POWER_LED);
  
  // Initialize sensors
  IMU::init(mpu9250);
  GPS::init(adafruit_gps);
  BARO::init(mpl3115);

  // Initialize servos and receiver input
  ACTUATORS::init(left_elevon, right_elevon, rudder);
  RECEIVER::init();
  RECEIVER::start_interrupts();
  
  // Initialize radio and msg that contains last received radio message
  RADIO::init(radio, RADIO_RESET);
  msg = new aero::def::ParsedMessage_t();

  // Call calibration routines if necessary
  // NOTE: Accel and Mag save results in EEPROM
  if(CALIBRATE_ACCEL) IMU::calibrate(mpu9250, IMU::ACCEL);
  if(CALIBRATE_GYRO)  IMU::calibrate(mpu9250, IMU::GYRO);
  if(CALIBRATE_MAG)   IMU::calibrate(mpu9250, IMU::MAG);
  if(CALIBRATE_BARO)  BARO::calibrate(mpl3115);

  // Initialize PID controllers
  CONTROLLER::init(roll_control, pitch_control, yaw_control);

  // Initialize Setpoint calculator
  SETPOINT::init(TARGET_LAT, TARGET_LON, TARGET_ALT);

  // Initialize threads and add threads
  THREADS::init();

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
  // NOTE: threads need to be added for suspend/restart to work
  if(mode == MODE_MANUAL) {
    // If we boot in manual mode, immediately suspend the automatic and setpoint threads
    to_manual_mode();
  } else if(mode == MODE_AUTO) {
    // If we boot in auto mode, immediately suspend the manual thread and terminate interrupts
    to_auto_mode();
  } else {
    to_idle_mode();
  }
  
  // Thread to monitor CPU activity
  THREADS::ids[ACTIVITY_ID] = threads.addThread(thread_activity);

  // Thread to monitor all data; verbose mode
  #if defined(VERBOSE_DEBUG)
    THREADS::ids[DEBUG_ID] = threads.addThread(thread_debug);
  #endif 
  
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
