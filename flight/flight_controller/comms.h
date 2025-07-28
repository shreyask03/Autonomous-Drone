#ifndef COMMS_H
#define COMMS_H

#include <Arduino.h>
#include "pid.h"
#include "receiver.h"

class Comms{
  public:
    // for the messages that provide multiple related numerical data
    // These would include (setting PID gains, setting positions,etc)
    struct Msg{
      float x = 0;
      float y = 0;
      float z = 0;
    };  

  private:
    // holds gains for a specified axis
    Msg gains;
    
    PID* roll;
    PID* pitch;
    PID* yaw;
    PID* aPitch;
    PID* aRoll;
    Receiver* rx;  // to check armed state

  public:
    Comms(PID* p,PID* r,PID* y,PID* ap, PID* ar, Receiver* rx);
    void init();

    // for PID tuning
    void parseGains(char* message);

    // sending current tune
    void sendTune();

    // for general messages
    void parseMessage(char* message);
  
};

#endif