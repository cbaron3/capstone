#pragma once

/**
 * File to encapsulate functions that interface with the multithreading library.
 * Makes use of the TeensyThreads library 
 */
#include "Config.hpp"
#include <TeensyThreads.h>

/**
 * Threading namespace; encapsulates all related functions
 */
namespace THREADS {

  // Default state is -1; after initialization it will be populated with valid thread IDs
  int ids[8] = {-1};

  /**
   * @brief Initialize threading
   * 
   */
  void init() {
    // Initialize threads; thread time slice at 10 ms
    threads.setMicroTimer(10);
  }
  
  /**
   * @brief Suspend thread if the ID exists
   * 
   * @param index index of thread ID
   * @return int status; -1 if thread does not exist or is already suspended
   */
  inline int suspend(unsigned int index) {
    if(index > (sizeof(ids)/sizeof(ids[0]))) {
      return -1;
    }
    
    if(ids[index] == -1) {
        return -1;
    }
  
    int id = ids[index];
    
    if(threads.getState(id) == Threads::RUNNING) {
      return threads.suspend(id);
    }

    return -1;
  }

  /**
   * @brief Restart thread if the ID exists
   * 
   * @param index index of thread ID
   * @return int status; -1 if thread does not exist or is already running
   */
  inline int restart(unsigned int index) {
    if(index > (sizeof(ids)/sizeof(ids[0]))) {
      return -1;
    }
    
    if(ids[index] == -1) {
        return -1;
    }
  
    int id = ids[index];
    
    if(threads.getState(id) == Threads::SUSPENDED) {
      return threads.restart(id);
    }

    return -1;
  }
}
