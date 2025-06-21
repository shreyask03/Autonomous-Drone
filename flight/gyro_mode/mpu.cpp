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


// interrupt detection routin
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}




void MPU::init(){
  // initialize MPU
  Wire.begin();
  Wire.beginTransmission(this->MPU_ADDR); // begin writing to MPU
  Wire.write(PWR_MGMT1); // access register
  Wire.write(0); // wake MPU
  Wire.endTransmission(true);
}

void MPU::updateGyro(){
  Wire.beginTransmission(this->MPU_ADDR); // begin writing to MPU
  Wire.write(0x43); // starting address GYRO_X high byte register
  Wire.endTransmission(false); // restart transmission for reading

  Wire.requestFrom(this->MPU_ADDR,6); // get 16 bytes for the 3 axes in gyro data sent as 2 8 byte packets each
  int16_t rawGx = (Wire.read() << 8) | Wire.read(); // gyro x high byte logic OR with low byte to combine into 1 raw data value
  int16_t rawGy = (Wire.read() << 8) | Wire.read(); // gyro y ""
  int16_t rawGz = (Wire.read() << 8) | Wire.read(); // gyro z ""


  gyro.x = rawGx / this->GY250_SENSE; // divide by scale factor to get to deg/s, store in vector
  gyro.y = rawGy / this->GY250_SENSE;
  gyro.z = rawGz / this->GY250_SENSE;
}

MPU::Vector3 MPU::getGyro() const{ // getter function
  return gyro;
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
        // mpu.PrintActiveOffsets();
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