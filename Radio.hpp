#pragma once

/**
 * File to encapsulate functions that interface with the radio; RFM95W.
 * Makes use of the RadioHead library and the Aero Design's message protocol 
 */
#include <RH_RF95.h>
#include "Config.hpp"
#include "src/aero-cpp-lib/include/Message.hpp"

/**
 * Barometer namespace; encapsulates all related functions
 */
namespace RADIO {
    
    // Anonymous namespace to protect data (make private)
    namespace {
      // Message handler that can build and parse messages
      aero::Message message_handler;

      // Store last message that was received
      aero::def::ParsedMessage_t last_recv;

      // Message buffer
      uint8_t inc_data[RH_RF95_MAX_MESSAGE_LEN] = {0};
      uint8_t inc_data_len = sizeof(inc_data);
    }

    /**
     * @brief Initialize radio object
     * 
     * @param radio radio object
     * @param rst radio reset pin
     * @return true if radio successfully initialized
     * @return false if radio failed to initialize
     */
    inline bool init(RH_RF95& radio, Pin rst) {
        // Toggle the reset pin
        pinMode(rst, OUTPUT);
        digitalWrite(rst, LOW);
        delay(10);
        digitalWrite(rst, HIGH);
        delay(10);

        // Initialize the radio
        if (!radio.init()) {
        return false;
        }

        // Set radio frequency
        if (!radio.setFrequency(RADIO_FREQ)) {
        return false;
        }

        // Set radio power
        radio.setTxPower(RADIO_POWER, false);

        return true;
    }
    
    /**
     * @brief Check radio for new message
     * 
     * @param radio radio object
     * @param msg message to store what is received by the radio, if something is received
     * @return true if radio received a new message
     * @return false if radio did not receive a new message
     */
    bool receive(RH_RF95& radio, aero::def::ParsedMessage_t* msg) {
        if (radio.available()) {
            if (radio.recv(inc_data, &inc_data_len)) {
                // Parse response and return it
                last_recv = message_handler.parse(inc_data);
                *msg = last_recv;
                return true;
            } else {
                return false;
            }
        } else {
            return false;
        }
  }

  /**
   * @brief Respond to a radio message from the ground station
   * 
   * @param radio radio object
   * @param response rseponse to send back to the ground station
   * @return true if radio successfully responded
   * @return false if radio failed to respond
   */
  bool respond(RH_RF95& radio, aero::def::RawMessage_t response) {
        // Send a reply
        bool valid = radio.send((char *)&response, sizeof(response));
        radio.waitPacketSent();
        return valid;
  }

  
}
