#include "receiver.h"
#include "mpu.h"
#include "pid.h"
#include "motor.h"


Receiver rx;
MPU mpu;

PID pidRoll(0.5,0.0,0.01);
PID pidPitch(0.5,0.0,0.01);
PID pidYaw(0.5,0.0,0.01);

Motor motorFL(11); // Front left
Motor motorFR(3); // Front right
Motor motorBL(10); // Back left
Motor motorBR(9); // Back right

unsigned long prevLoopUs = 0;
const unsigned long LOOP_PERIOD_US = 4000; // 250 Hz


/* reads in and stores channel data for yaw,pitch,roll,throttle, and the arm switch*/
ISR(PCINT0_vect) {
  rx.handleInterrupt();
}

void setup() {
  // put your setup code here, to run once:

  // initialize receiver
  rx.init();

  // wait for rx connection
  rx.waitForConnect(); // now safe to initialize motors

  // initialize gyro
  mpu.init();

  // reset PID values
  pidRoll.reset();
  pidPitch.reset();
  pidYaw.reset();

  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,LOW);
}

void loop() {
  // put your main code here, to run repeatedly:

  unsigned long now = micros();
  static unsigned long lastBlink = 0;
  static bool wasArmed = false;


  if(now - prevLoopUs >= LOOP_PERIOD_US){
    prevLoopUs = now;


    if(!rx.isArmed()){
      if(now - lastBlink >= 100000){ // 100 ms
        digitalWrite(LED_BUILTIN,!digitalRead(LED_BUILTIN));
        lastBlink = now;
      }

      motorFL.setPulse(1000);
      motorFR.setPulse(1000);
      motorBL.setPulse(1000);
      motorBR.setPulse(1000);

      motorFL.writePulse();
      motorFR.writePulse();
      motorBL.writePulse();
      motorBR.writePulse();

      wasArmed = false;
      return;
    }

    if(!wasArmed){
      // initialize motors only once
      motorFL.init();
      motorFR.init();
      motorBL.init();
      motorBR.init();
      wasArmed = true;
    }    


    // Read IMU
    mpu.readRawData();
    float rollRate = mpu.convertRoll();
    float pitchRate = mpu.convertPitch();
    float yawRate = mpu.convertYaw();

    // Read Pilot inputs
    int throttle = rx.getThrottle();
    int targetRoll = rx.getRollRate();
    int targetPitch = rx.getPitchRate();
    int targetYaw = rx.getYawRate();

    // Compute PID error correction
    float rollTerm = pidRoll.compute(targetRoll,rollRate);
    float pitchTerm = pidPitch.compute(targetPitch,pitchRate);
    float yawTerm = pidYaw.compute(targetYaw, yawRate);
      
    // Motor channel mixing
    int baseThrottle = throttle;
    motorFL.setPulse(constrain(baseThrottle + pitchTerm - rollTerm - yawTerm, 1000 , 2000));
    motorFR.setPulse(constrain(baseThrottle + pitchTerm + rollTerm + yawTerm, 1000, 2000));
    motorBL.setPulse(constrain(baseThrottle - pitchTerm - rollTerm + yawTerm, 1000, 2000));
    motorBR.setPulse(constrain(baseThrottle - pitchTerm + rollTerm - yawTerm, 1000, 2000));
      
    // write PWM signals to ESC
    motorFL.writePulse();
    motorFR.writePulse();
    motorBL.writePulse();
    motorBR.writePulse();
  }

}
