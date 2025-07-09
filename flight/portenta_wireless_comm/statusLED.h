#ifndef STATUSLED_H
#define STATUSLED_H

#include <Arduino.h>

class statusLED{

  private:
    uint8_t rLED = LEDR;
    uint8_t gLED = LEDG;
    uint8_t bLED = LEDB;

  public:
    // initialize LED pins
    void init();
    
    // run when connecting to WiFi
    void connectingWiFi();

    // run when connecting to server
    void connectingServer();

    // triggers a timed flash to indicate data transfer
    void dataTransfer();
};

#endif  
