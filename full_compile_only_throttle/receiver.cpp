#include "receiver.h"

void Receiver::init(){
  // setup pin change interrupt
  pinMode(8, INPUT);
  PCICR |= (1 << PCIE0);       // Enable PCINT0
  PCMSK0 |= (1 << PCINT0);     // Enable PCINT0 on PB0 (D8)

  // set up LED pin for visual status information
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}


void Receiver::handleInterrupt() {
  // ISR handler
  // if (PINB & B00000001) {
  //   t[pulse] = micros();

  //   if (pulse > 0 && pulse < NUM_CHANNELS) {
  //     uint16_t pulseWidth = t[pulse] - t[pulse - 1];

  //     // Detect sync pulse
  //     if (pulseWidth > 3000) {
  //       t[0] = t[pulse];
  //       pulse = 1;
  //       return;
  //     }

  //     // Clamp valid signal
  //     if (pulseWidth < 1000) pulseWidth = 1000;
  //     else if (pulseWidth > 2000) pulseWidth = 2000;

  //     ch[pulse] = pulseWidth;
  //   }

  //   pulse++;
  //   if (pulse >= NUM_CHANNELS + 1) pulse = 1;
  // }
  if (PINB & B00000001){
    t[pulse] = micros();
    switch(pulse){
      case 1: // if pulse == 1
      ch[1] = t[1] - t[0]; // channel 1 is the time difference in micros between the rise/fall ( depending on choice) of the first pulse and the second pulse
      if (ch[1] < 1000){ ch[1] = 1000;}
      else if (ch[1] > 2000){ ch[1] = 2000;}
      pulse++;
      if ((t[1]-t[0]) > 3000){ // deals with synchronization issues due to arduino boot up sequence
        t[0] = t[1]; // update time 
        pulse = 1; // pulse becomes starting pulse
        // lastFrameTime = micros(); // mark last frame time
      }
      break;
      case 2: // if pulse == 2
      ch[2] = t[2] - t[1];
      if (ch[2] < 1000){ ch[2] = 1000;}
      else if (ch[2] > 2000){ ch[2] = 2000;}
      pulse++;
      if ((t[2]-t[1]) > 3000){
        t[0] = t[2];
        pulse = 1;
      }
      break;
      case 3: // if pulse == 3
      ch[3] = t[3] - t[2];
      if (ch[3] < 1000){ ch[3] = 1000;}
      else if (ch[3] > 2000){ ch[3] = 2000;}
      pulse++;
      if ((t[3]-t[2]) > 3000){
        t[0] = t[3];
        pulse = 1;
      }
      break;
      case 4:
      ch[4] = t[4] - t[3];
      if (ch[4] < 1000){ ch[4] = 1000;}
      else if (ch[4] > 2000){ ch[4] = 2000;}
      pulse++;
      if ((t[4]-t[3]) > 3000){
        t[0] = t[4];
        pulse = 1;
      }
      break;
      case 5:
      ch[5] = t[5] - t[4];
      if (ch[5] < 1000){ ch[5] = 1000;}
      else if (ch[5] > 2000){ ch[5] = 2000;}
      pulse++;
      if ((t[5]-t[4]) > 3000){
        t[0] = t[5];
        pulse = 1;
      }
      break;
      case 6:
      ch[6] = t[6] - t[5];
      if (ch[6] < 1000){ ch[6] = 1000;}
      else if (ch[6] > 2000){ ch[6] = 2000;}
      pulse++;
      if ((t[6]-t[5]) > 3000){
        t[0] = t[6];
        pulse = 1;
      }
      break;
      case 7:
      ch[7] = t[7] - t[6];
      if (ch[7] < 1000){ ch[7] = 1000;}
      else if (ch[7] > 2000){ ch[7] = 2000;}
      pulse++;
      if ((t[7]-t[6]) > 3000){
        t[0] = t[7];
        pulse = 1;
      }
      break;
      case 8:
      ch[8] = t[8] - t[7];
      if (ch[8] < 1000){ ch[8] = 1000;}
      else if (ch[8] > 2000){ ch[8] = 2000;}
      pulse++;
      if ((t[8]-t[7]) > 3000){
        t[0] = t[8];
        pulse = 1;
      }
      break;
      case 9:
      ch[0] = t[9] - t[8]; // sync pulse
      pulse++;
      if ((t[9]-t[8]) > 3000){
        t[0] = t[9];
        pulse = 1;
      }
      break;
      default:
      pulse++;
      break;
    }
  }
}


int Receiver::getYawRate(){
  // microseconds mapped to [-90,90] deg/s
  return mapChannel(4,-90,90);
}

int Receiver::getPitchRate(){
  // microseconds mapped to [-90,90] deg/s
   return mapChannel(2,-90,90);
}

int Receiver::getRollRate(){
  // microseconds mapped to [-90,90] deg/s
   return mapChannel(1,-90,90);
}

int Receiver::getThrottle(){
  // raw microsecond value
  noInterrupts(); // pause isr updating to prevent value tearing mid assigning
  int val = ch[3]; // update value
  interrupts(); // resume interrupts
  return val;
}

bool Receiver::isArmed() {
  noInterrupts();
  int armSwitch = ch[5]; // Assume channel 5 is arm switch
  interrupts();
  return (armSwitch > 1950);
}

int Receiver::mapChannel(uint8_t channel, int outMin, int outMax) {
  noInterrupts();
  int val = ch[channel];
  interrupts();
  return map(val, 1000, 2000, outMin, outMax);
}
bool Receiver::isConnected(){
  int t = this->getThrottle();
  return (t >= 900 && t <= 2100);
}
void Receiver::waitForConnect(){
  while(!isConnected()){
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // blink LED for status update
    delay(100);
  }
}

// bool Receiver::isSignalLost(unsigned long timeoutUs){
//   return (micros() - lastFrameTime > timeoutUs);
// }