#pragma once

#include <RH_RF95.h>
#include "Config.hpp"

namespace RADIO {
    inline bool init(RH_RF95& radio, Pin rst) {
        pinMode(rst, OUTPUT);

        // Manually reset the radio
        digitalWrite(rst, LOW);
        delay(10);
        digitalWrite(rst, HIGH);
        delay(10);

        if(!radio.init()) {
            return false;
        }

        if(!radio.setFrequency(RADIO_FREQUENCY)) {
            return false;
        }

        radio.setTxPower(RADIO_POWER, false);

        return true;
    }

    
}