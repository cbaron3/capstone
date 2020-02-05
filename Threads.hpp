#pragma once

#include <TeensyThreads.h>

const int RADIO_ID = 0;
const int GPS_ID = 1;
const int I2C_ID = 2;
const int SETPOINT_ID = 3;
const int AUTO_ID = 4;
const int MANUAL_ID = 5;

namespace THREADS {

  // Default state is -1; after initialization it will be populated with valid thread IDs
  int ids[6] = {-1};

  void init() {
    
  }
  
  inline int suspend(int index) {
    if(index > (sizeof(ids)/sizeof(ids[0])) || index < 0) {
      return -1;
    }
    
    if(ids[index] == -1) {
        return -1;
    }
  
    int id = ids[index];
    
    if(threads.getState(id) == Threads::RUNNING) {
      return threads.suspend(id);
    }
  }

  inline int restart(int index) {
    if(index > (sizeof(ids)/sizeof(ids[0])) || index < 0) {
      return -1;
    }
    
    if(ids[index] == -1) {
        return -1;
    }
  
    int id = ids[index];
    
    if(threads.getState(id) == Threads::SUSPENDED) {
      return threads.restart(id);
    }
  }
}
