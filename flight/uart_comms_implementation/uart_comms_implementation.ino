#include "receiver.h"   
#include "pid.h"
#include "comms.h"

static constexpr int RATE_LIMIT = 60; // deg/s

PID roll(4,4,0.005); // try same as pitch values
PID pitch(5,3,0.04); // P=0.75, D = 0.0001 pretty good // P=0.5 slightly better // overall relatively sluggish response, may fix later wih post flight tuning
PID yaw(3,1.5,0.02);

// initialize new PID objects for angle control
PID rollAngle(0.1,0,0,RATE_LIMIT);
PID pitchAngle(0,0,0,RATE_LIMIT); // P = 0.8 , I = 0.01, D = 0.015

Receiver rx;


// initialize communication object
Comms comms(&pitch,&roll,&yaw,&pitchAngle,&rollAngle,&rx);

ISR(PCINT0_vect){
  rx.handleInterrupt();
}

void setup() {
  // put your setup code here, to run once:
  // initialize receiver
  rx.init();

  // wait for rx and tx to establish connection
  // rx.waitForConnect();

  // reset PID values
  roll.reset();
  pitch.reset();
  yaw.reset();

  comms.init();
  if (Serial.availableForWrite()) {
    delay(500); // Wait a moment for Serial1 to connect
    comms.sendTune(); // Send the tune once to Portenta
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  if(!rx.isArmed()){

    // rx.waitForConnect();

    if (Serial.available()) {
      static char buffer[100];
      size_t len = Serial.readBytesUntil('\n', buffer, sizeof(buffer) - 1);
      buffer[len] = '\0';
      comms.parseGains(buffer);
    }
  }
}
