#ifndef RECEIVER_H
#define RECEIVER_H

#include <Arduino.h>

class Receiver{
  private:
    static const int NUM_CHANNELS = 9;
    volatile uint16_t ch[NUM_CHANNELS];
    volatile uint32_t t[NUM_CHANNELS + 1];
    volatile uint8_t pulse = 0;

    int mapChannel(uint8_t channel, int outMin, int outMax);

    // volatile unsigned long lastFrameTime = 0;


  public:
    void init(); // initializes pin change interrupt
    void handleInterrupt(); // ISR handler
    int getYawRate(); // microseconds mapped to [-90,90] deg/s
    int getPitchRate(); // microseconds mapped to [-90,90] deg/s
    int getRollRate(); // microseconds mapped to [-90,90] deg/s
    int getThrottle(); // raw microseconds
    bool isArmed(); // arming procedure
    bool isAngle(); // angle mode check
    bool isConnected(); // checks for Rx connected
    void waitForConnect(); // blocking function waits for Rx to connect
    void printSignals(); // debug method to ensure proper signals being sent

};

#endif