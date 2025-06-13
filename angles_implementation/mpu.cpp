#include "mpu.h"


#define OUTPUT_READABLE_YAWPITCHROLL

#define INTERRUPT_PIN 2

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}




void MPU::init(){
  // initialize MPU
  Wire.begin();
  Wire.beginTransmission(this->MPU_ADDR);
  Wire.write(PWR_MGMT1);
  Wire.write(0);
  Wire.endTransmission(true);
}

void MPU::updateGyro(){
  Wire.beginTransmission(this->MPU_ADDR);
  Wire.write(0x43); // starting address GYRO_X high byte
  Wire.endTransmission(false); // restart for reading

  Wire.requestFrom(this->MPU_ADDR,6); // get 16 bytes for the 3 axes in gyro data sent as 2 8 byte packets each
  int16_t rawGx = (Wire.read() << 8) | Wire.read(); // gyro x high byte logic OR with low byte to combine into 1 raw data value
  int16_t rawGy = (Wire.read() << 8) | Wire.read(); // gyro y ""
  int16_t rawGz = (Wire.read() << 8) | Wire.read(); // gyro z ""


  gyro.x = rawGx / this->GY250_SENSE; // divide by scale factor to get to deg/s
  gyro.y = rawGy / this->GY250_SENSE;
  gyro.z = rawGz / this->GY250_SENSE;
}

void MPU::updateAccel(){
  Wire.beginTransmission(this->MPU_ADDR);
  Wire.write(0x3B); // starting address ACCEL_X high byte
  Wire.endTransmission(false); // restart for reading

  Wire.requestFrom(this->MPU_ADDR, 6); // get 16 bytes for 3 axes in accel data sent as 2 8 byte packets each
  int16_t rawAx = (Wire.read() << 8) | Wire.read(); // accel x high byte logic OR with low byte to combine into 1 raw data value
  int16_t rawAy = (Wire.read() << 8) | Wire.read(); // accel y ""
  int16_t rawAz = (Wire.read() << 8) | Wire.read(); // accel z ""
  accel.x = rawAx / this->ACCEL_SCALE;
  accel.y = rawAy / this->ACCEL_SCALE;
  accel.z = rawAz / this->ACCEL_SCALE;
}

MPU::Vector3 MPU::getGyro() const{
  return gyro;
}

MPU::Vector3 MPU::getAccel() const{
  return accel;
}


MPU::Vector3 MPU::computeAngles(float dt){
  // read gyro and accel data
  updateGyro();
  updateAccel();

  // accel angle estimate
  float accel_roll = atan2(accel.y, accel.z) * 180.0/PI;
  float accel_pitch = atan2(-accel.x, sqrt(accel.y*accel.y + accel.z*accel.z)) * 180.0/PI;

  // gyro angle estimate
  // unsigned long now = millis();

  // if(last_upd == 0){
  //     last_upd = millis();
  //     return Vector3(); // skip first angle calculation
  // }

  // float dt = (now - last_upd)/1000.0f; // elapsed time in seconds
  // last_upd = now;   

  float gyro_roll = prev_roll + gyro.x*dt;
  float gyro_pitch = prev_pitch + gyro.y*dt;
  
  prev_roll = gyro_roll;
  prev_pitch = gyro_pitch;

  // complementary filter
  float alpha = 0.95;

  // applies a "high pass filter" to the gyro to correct long term noise and "low pass filter" to the accel to correct short term noise
  float filtered_roll = alpha*gyro_roll + (1 - alpha)*accel_roll;
  float filtered_pitch = alpha*gyro_pitch + (1 - alpha)*accel_pitch;
  
  // filtered_roll = normalizeAngle(filtered_roll);
  // filtered_pitch = normalizeAngle(filtered_pitch);

  MPU::Vector3 angles;
  angles.x = filtered_roll;
  angles.y = filtered_pitch;
  // angles.x = gyro_roll;
  // angles.y = gyro_pitch;
  // angles.z = gyro.z;
  return angles;

}

void MPU::calibrateGyro(int samples = 500){
  long sumX = 0, sumY = 0, sumZ = 0;
  for (int i = 0; i < samples; i++){
    updateGyro();
    sumX += gyro.x;
    sumY += gyro.y;
    sumZ += gyro.z;
    delay(3); //small delay between samples
  }

  gyro_bias.x = sumX / (float)samples;
  gyro_bias.y = sumY / (float)samples;
  gyro_bias.z = sumZ / (float)samples;
}

void MPU::setupForDMP(){
  mpu.initialize();
  pinMode(INTERRUPT_PIN,INPUT);

  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(-16);
  mpu.setYGyroOffset(-12);
  mpu.setZGyroOffset(-39);
  mpu.setZAccelOffset(1911); // 1688 factory default for my test chip

   // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }

}

MPU::Vector3 MPU::getDMPAngles(){
  MPU::Vector3 angles;
    
  // if programming failed, don't try to do anything
  if (!dmpReady) return angles;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // convert angles from radians to degrees
    // sign convention (roll right (right wing down) = +, pitch forward (nose down) = +, yaw right (nose right) = +)
    angles.x = ypr[2] * 180/M_PI; //  roll
    angles.y = -ypr[1] * 180/M_PI; // pitch
    angles.z = ypr[0] * 180/M_PI; // yaw
  }
  return angles;
}