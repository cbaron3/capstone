#pragma once

#include <RH_RF95.h>
#include "Config.hpp"

#include "src/aero-cpp-lib/include/Message.hpp"

namespace RADIO {
    namespace {
      aero::Message message_handler;
      aero::def::ParsedMessage_t last_recv;
      uint8_t inc_data[RH_RF95_MAX_MESSAGE_LEN] = {0};
      uint8_t inc_data_len = sizeof(inc_data);
    }
    inline bool init(RH_RF95& radio, Pin rst) {
        // Set the reset pin
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

  bool respond(RH_RF95& radio, aero::def::RawMessage_t response) {
        // Send a reply
        bool valid = radio.send((char *)&response, sizeof(response));
        radio.waitPacketSent();
        return valid;
  }

  
}
