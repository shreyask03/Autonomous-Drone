#ifndef RECEIVER_H
#define RECEIVER_H

#include <Arduino.h>

class Receiver{
  private:
    static const int NUM_CHANNELS = 9;
    volatile uint16_t ch[NUM_CHANNELS];
    volatile uint32_t t[NUM_CHANNELS + 1];
    volatile uint8_t pulse = 0;

    // channel data set limits -> corresponds to pwm limits directly
    static constexpr int MINPULSEUS = 1000;
    static constexpr int MAXPULSEUS = 2000;

  public:
    // initializes rx to begin sending data
    void init();
    // ISR handler, measures pulses sent from rx to get ch data
    void handleInterrupt();
    int getYawRate(); // microseconds mapped to [-90,90] deg/s
    int getPitchRate(); // microseconds mapped to [-90,90] deg/s
    int getRollRate(); // microseconds mapped to [-90,90] deg/s
    // map channel data from raw microseconds to [-limit,limit] deg/s
    int getRate(uint8_t channel, int limit);
    // map channel data from raw microseconds to [-limit,limit] deg
    int getAngle(uint8_t channel, int limit);

    // raw microseconds
    int getThrottle();

    // armed state check
    bool isArmed();
    // flight state = angle check
    bool isAngle();
    // checks for valid throttle signal from rx
    bool isConnected();
    // blocking function waits for Rx to connect
    void waitForConnect();
    // COMMENT OUT: DEBUG TOOL ONLY | Visualizes received signals
    // void printSignals();

};

#endif