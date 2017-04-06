/*
VectorNav.cpp
Brian R Taylor
brian.taylor@bolderflight.com
2017-03-31

Copyright (c) 2017 Bolder Flight Systems

Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or 
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

// Teensy 3.0 || Teensy 3.1/3.2 || Teensy 3.5 || Teensy 3.6 || Teensy LC 
#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || \
  defined(__MK66FX1M0__) || defined(__MKL26Z64__)

#include "Arduino.h"
#include "VectorNav.h"
#include "SPI.h" // SPI Library

/* VectorNav object, input the SPI CS Pin */
VectorNav::VectorNav(uint8_t csPin){
  _csPin = csPin; // SPI CS Pin
  _mosiPin = MOSI_PIN_11; // SPI MOSI Pin, set to default
  _useSPI = true; // set to use SPI instead of I2C
}

/* VectorNav object, input the SPI CS Pin and MOSI Pin */
VectorNav::VectorNav(uint8_t csPin, spi_mosi_pin pin){
  _csPin = csPin; // SPI CS Pin
  _mosiPin = pin; // SPI MOSI Pin
  _useSPI = true; // set to use SPI instead of I2C
}

/* Starts communication and sets up the VN100 */
int VectorNav::begin(){

  if( _useSPI ){ // using SPI for communication

    // setting CS pin to output
    pinMode(_csPin,OUTPUT);

    // setting CS pin high
    digitalWriteFast(_csPin,HIGH);

    // Teensy 3.0 || Teensy 3.1/3.2
    #if defined(__MK20DX128__) || defined(__MK20DX256__)
      // configure and begin the SPI
      switch( _mosiPin ){
        case MOSI_PIN_7:  // SPI bus 0 alternate 1
          SPI.setMOSI(7);
          SPI.setMISO(8);
          SPI.setSCK(14);
          SPI.begin();
          break;
        case MOSI_PIN_11: // SPI bus 0 default
          SPI.setMOSI(11);
          SPI.setMISO(12);
          SPI.setSCK(13);
          SPI.begin();
          break;
      }
    #endif

    // Teensy 3.5 || Teensy 3.6 
    #if defined(__MK64FX512__) || defined(__MK66FX1M0__)
      // configure and begin the SPI
      switch( _mosiPin ){
        case MOSI_PIN_0:  // SPI bus 1 default
          SPI1.setMOSI(0);
          SPI1.setMISO(1);
          SPI1.setSCK(32);
          SPI1.begin();
          break;
        case MOSI_PIN_7:  // SPI bus 0 alternate 1
          SPI.setMOSI(7);
          SPI.setMISO(8);
          SPI.setSCK(14);
          SPI.begin();
          break;
        case MOSI_PIN_11: // SPI bus 0 default
          SPI.setMOSI(11);
          SPI.setMISO(12);
          SPI.setSCK(13);
          SPI.begin();
          break;
        case MOSI_PIN_21: // SPI bus 1 alternate
          SPI1.setMOSI(21);
          SPI1.setMISO(5);
          SPI1.setSCK(20);
          SPI1.begin();
          break;
        case MOSI_PIN_28: // SPI bus 0 alternate 2
          SPI.setMOSI(28);
          SPI.setMISO(39);
          SPI.setSCK(27);
          SPI.begin();
          break;
        case MOSI_PIN_44: // SPI bus 2 default
          SPI2.setMOSI(44);
          SPI2.setMISO(45);
          SPI2.setSCK(46);
          SPI2.begin();
          break;
        case MOSI_PIN_52: // SPI bus 2 alternate
          SPI2.setMOSI(52);
          SPI2.setMISO(51);
          SPI2.setSCK(53);
          SPI2.begin();
          break;
      }
    #endif

    // Teensy LC 
    #if defined(__MKL26Z64__)
      // configure and begin the SPI
      switch( _mosiPin ){
        case MOSI_PIN_0:  // SPI bus 1 default
          SPI1.setMOSI(0);
          SPI1.setMISO(1);
          SPI1.setSCK(20);
          SPI1.begin();
          break;
        case MOSI_PIN_7:  // SPI bus 0 alternate 1
          SPI.setMOSI(7);
          SPI.setMISO(8);
          SPI.setSCK(14);
          SPI.begin();
          break;
        case MOSI_PIN_11: // SPI bus 0 default
          SPI.setMOSI(11);
          SPI.setMISO(12);
          SPI.setSCK(13);
          SPI.begin();
          break;
        case MOSI_PIN_21: // SPI bus 1 alternate
          SPI1.setMOSI(21);
          SPI1.setMISO(5);
          SPI1.setSCK(20);
          SPI1.begin();
          break;
      }
    #endif
  }
  else{ // using I2C for communication
    // SERIAL
  }

  // setting the time since last TX to arbitrarily large
  timeSinceTX = 100;

  // successful init, return 0
  return 0;
}

/* Enables interrupts given a sample rate divider and pulse width (ns). */
/* Return 0 on success or the VN error code on error. */
int VectorNav::enableInterrupt(uint16_t SRD, uint32_t pulseWidth) {
  uint8_t buffer[SYNC_CONTROL_REG[1]];
  memset(buffer,0,SYNC_CONTROL_REG[1]);

  // sync out mode, trigger start of IMU sampling 800 Hz
  buffer[8] = 1;

  // pulse polarity, positive
  buffer[9] = 1;

  // sync out skip factor
  memcpy(buffer+10,&SRD,sizeof(SRD));

  // sync out pulse width
  memcpy(buffer+12,&pulseWidth,sizeof(pulseWidth));

  // write registers
  int errType = writeRegisters(SYNC_CONTROL_REG[0],SYNC_CONTROL_REG[1],buffer);
  if (errType < 0){return errType;}
  else{return 0;}
}

/* Sets up DLPF given window sizes for the sensors. */
/* Return 0 on success or the VN error code on error. */
int VectorNav::setDLPF(uint16_t magWindowSize, uint16_t accelWindowSize, uint16_t gyroWindowSize, uint16_t temperatureWindowSize, uint16_t pressureWindowSize) {
  uint8_t magFilterMode,accelFilterMode,gyroFilterMode,tempFilterMode,pressFilterMode;
  uint8_t buffer[IMU_FILTER_CONFIG_REG[1]];
  memset(buffer,0,IMU_FILTER_CONFIG_REG[1]);

  // magnetometer, accel, gyro, temperature, and pressure window sizes
  memcpy(buffer,&magWindowSize,sizeof(magWindowSize));
  memcpy(buffer+2,&accelWindowSize,sizeof(accelWindowSize));
  memcpy(buffer+4,&gyroWindowSize,sizeof(gyroWindowSize));
  memcpy(buffer+6,&temperatureWindowSize,sizeof(temperatureWindowSize));
  memcpy(buffer+8,&pressureWindowSize,sizeof(pressureWindowSize));

  // enable or disable filtering
  if(magWindowSize == 0) {
    magFilterMode = 0;
  } else {
    magFilterMode = 3;
  }
  memcpy(buffer+10,&magFilterMode,sizeof(magFilterMode));
  if(accelWindowSize == 0) {
    accelFilterMode = 0;
  } else {
    accelFilterMode = 3;
  }
  memcpy(buffer+11,&accelFilterMode,sizeof(accelFilterMode));
  if(gyroWindowSize == 0) {
    gyroFilterMode = 0;
  } else {
    gyroFilterMode = 3;
  }
  memcpy(buffer+12,&gyroFilterMode,sizeof(gyroFilterMode));
  if(temperatureWindowSize == 0) {
    tempFilterMode = 0;
  } else {
    tempFilterMode = 3;
  }
  memcpy(buffer+13,&tempFilterMode,sizeof(tempFilterMode));
  if(pressureWindowSize == 0) {
    pressFilterMode = 0;
  } else {
    pressFilterMode = 3;
  }
  memcpy(buffer+14,&pressFilterMode,sizeof(pressFilterMode));

  // write registers
  int errType = writeRegisters(IMU_FILTER_CONFIG_REG[0],IMU_FILTER_CONFIG_REG[1],buffer);
  if (errType < 0){return errType;}
  else{return 0;}
}

/* Rotates the IMU reference frame given a 3x3 matrix of coefficients. */
/* Return 0 on success or the VN error code on error. */
int VectorNav::setReferenceFrameRotation(float T[3][3]) {
  uint8_t buffer[REF_FRAME_ROTATION_REG[1]];
  memset(buffer,0,REF_FRAME_ROTATION_REG[1]);
  float C[9];
  uint8_t k = 0;
  for(uint8_t i = 0; i < 3; i++) {
    for(uint8_t j = 0; j < 3; j++) {
      C[k] = T[i][j];
      k++;
    }
  }
  memcpy(buffer,&C,REF_FRAME_ROTATION_REG[1]);

  // write registers
  int errType = writeRegisters(REF_FRAME_ROTATION_REG[0],REF_FRAME_ROTATION_REG[1],buffer);
  if (errType < 0){return errType;}

  // write settings to NVM
  writeSettings();

  // reset sensor
  resetSensor();
  return 0;
}

/* Get accelerometer data given pointers to store the three values. */
/* Return 0 on success or the VN error code on error. */
int VectorNav::getAccel(float* ax, float* ay, float* az) {
  uint8_t data[ACCEL_COMP_REG[1]];
  int errType = readRegisters(ACCEL_COMP_REG[0],ACCEL_COMP_REG[1],data);
  memcpy(ax,data,4);
  memcpy(ay,data+4,4);
  memcpy(az,data+8,4);

  if (errType < 0){return errType;}
  else{return 0;}
}

/* Get gyro data given pointers to store the three values. */
/* Return 0 on success or the VN error code on error. */
int VectorNav::getGyro(float* gx, float* gy, float* gz) {
  uint8_t data[GYRO_COMP_REG[1]];
  int errType = readRegisters(GYRO_COMP_REG[0],GYRO_COMP_REG[1],data);
  memcpy(gx,data,4);
  memcpy(gy,data+4,4);
  memcpy(gz,data+8,4);

  if (errType < 0){return errType;}
  else{return 0;}
}

/* Get magnetometer data given pointers to store the three values. */
/* Return 0 on success or the VN error code on error. */
int VectorNav::getMag(float* hx, float* hy, float* hz) {
  uint8_t data[MAG_COMP_REG[1]];
  float hx_g, hy_g, hz_g;
  int errType = readRegisters(MAG_COMP_REG[0],MAG_COMP_REG[1],data);
  memcpy(&hx_g,data,4);
  memcpy(&hy_g,data+4,4);
  memcpy(&hz_g,data+8,4);

  *hx = hx_g * G2uT;
  *hy = hy_g * G2uT;
  *hz = hz_g * G2uT;

  if (errType < 0){return errType;}
  else{return 0;}
}

/* Get pressure data given pointer to store the value. */
/* Return 0 on success or the VN error code on error. */
int VectorNav::getPressure(float* pressure) {
  uint8_t data[IMU_MEAS_REG[1]];
  float press_kPa;
  int errType = readRegisters(IMU_MEAS_REG[0],IMU_MEAS_REG[1],data);
  memcpy(&press_kPa,data+40,4);
  *pressure = press_kPa * kPa2Pa;

  if (errType < 0){return errType;}
  else{return 0;}
}

/* Get temperature data given pointer to store the value. */
/* Return 0 on success or the VN error code on error. */
int VectorNav::getTemperature(float* temperature) {
  uint8_t data[IMU_MEAS_REG[1]];
  int errType = readRegisters(IMU_MEAS_REG[0],IMU_MEAS_REG[1],data);
  memcpy(temperature,data+36,4);

  if (errType < 0){return errType;}
  else{return 0;}
}

/* Get motion6 data given pointers to store the values. */
/* Return 0 on success or the VN error code on error. */
int VectorNav::getMotion6(float* ax, float* ay, float* az, float* gx, float* gy, float* gz) {
  uint8_t data[IMU_COMP_REG[1]];
  int errType = readRegisters(IMU_COMP_REG[0],IMU_COMP_REG[1],data);

  memcpy(ax,data+12,4);
  memcpy(ay,data+16,4);
  memcpy(az,data+20,4);

  memcpy(gx,data+24,4);
  memcpy(gy,data+28,4);
  memcpy(gz,data+32,4);

  if (errType < 0){return errType;}
  else{return 0;}
}

/* Get motion9 data given pointers to store the values. */
/* Return 0 on success or the VN error code on error. */
int VectorNav::getMotion9(float* ax, float* ay, float* az, float* gx, float* gy, float* gz, float* hx, float* hy, float* hz) {
  uint8_t data[IMU_COMP_REG[1]];
  float hx_g, hy_g, hz_g;
  int errType = readRegisters(IMU_COMP_REG[0],IMU_COMP_REG[1],data);
  memcpy(&hx_g,data,4);
  memcpy(&hy_g,data+4,4);
  memcpy(&hz_g,data+8,4);

  *hx = hx_g * G2uT;
  *hy = hy_g * G2uT;
  *hz = hz_g * G2uT;

  memcpy(ax,data+12,4);
  memcpy(ay,data+16,4);
  memcpy(az,data+20,4);

  memcpy(gx,data+24,4);
  memcpy(gy,data+28,4);
  memcpy(gz,data+32,4);

  if (errType < 0){return errType;}
  else{return 0;}
}

/* Get Euler data given pointers to store the values. */
/* Return 0 on success or the VN error code on error. */
int VectorNav::getEuler(float* yaw, float* pitch, float* roll) {
  uint8_t data[EULER_REG[1]];
  float yaw_d, pitch_d, roll_d;
  int errType = readRegisters(EULER_REG[0],EULER_REG[1],data);
  memcpy(&yaw_d,data,4);
  memcpy(&pitch_d,data+4,4);
  memcpy(&roll_d,data+8,4);

  *yaw = yaw_d * deg2rad;
  *pitch = pitch_d * deg2rad;
  *roll = roll_d * deg2rad;

  if (errType < 0){return errType;}
  else{return 0;}
}

/* Get Euler and IMU data given pointers to store the values. */
/* Return 0 on success or the VN error code on error. */
int VectorNav::getEulerIMU(float* yaw, float* pitch, float* roll, float* ax, float* ay, float* az, float* gx, float* gy, float* gz, float* hx, float* hy, float* hz) {
  uint8_t data[EULER_IMU_REG[1]];
  float yaw_d, pitch_d, roll_d;
  float hx_g, hy_g, hz_g;
  int errType = readRegisters(EULER_IMU_REG[0],EULER_IMU_REG[1],data);

  memcpy(&yaw_d,data,4);
  memcpy(&pitch_d,data+4,4);
  memcpy(&roll_d,data+8,4);

  *yaw = yaw_d * deg2rad;
  *pitch = pitch_d * deg2rad;
  *roll = roll_d * deg2rad;

  memcpy(&hx_g,data+12,4);
  memcpy(&hy_g,data+16,4);
  memcpy(&hz_g,data+20,4);

  *hx = hx_g * G2uT;
  *hy = hy_g * G2uT;
  *hz = hz_g * G2uT;

  memcpy(ax,data+24,4);
  memcpy(ay,data+28,4);
  memcpy(az,data+32,4);

  memcpy(gx,data+36,4);
  memcpy(gy,data+40,4);
  memcpy(gz,data+44,4);

  if (errType < 0){return errType;}
  else{return 0;}
}

/* Get Quaternion data given pointers to store the values. */
/* Return 0 on success or the VN error code on error. */
int VectorNav::getQuat(float* quat[4]) {
  uint8_t data[QUATERNION_REG[1]];
  int errType = readRegisters(QUATERNION_REG[0],QUATERNION_REG[1],data);
  memcpy(quat[0],data,4);
  memcpy(quat[1],data+4,4);
  memcpy(quat[2],data+8,4);
  memcpy(quat[3],data+12,4);

  if (errType < 0){return errType;}
  else{return 0;}
}

/* Get Quaternion and IMU data given pointers to store the values. */
/* Return 0 on success or the VN error code on error. */
int VectorNav::getQuatIMU(float* quat[4], float* ax, float* ay, float* az, float* gx, float* gy, float* gz, float* hx, float* hy, float* hz) {
  uint8_t data[QUATERNION_IMU_REG[1]];
  float hx_g, hy_g, hz_g;
  int errType = readRegisters(QUATERNION_IMU_REG[0],QUATERNION_IMU_REG[1],data);

  memcpy(quat[0],data,4);
  memcpy(quat[1],data+4,4);
  memcpy(quat[2],data+8,4);
  memcpy(quat[3],data+12,4);

  memcpy(&hx_g,data+16,4);
  memcpy(&hy_g,data+20,4);
  memcpy(&hz_g,data+24,4);

  *hx = hx_g * G2uT;
  *hy = hy_g * G2uT;
  *hz = hz_g * G2uT;

  memcpy(ax,data+28,4);
  memcpy(ay,data+32,4);
  memcpy(az,data+36,4);

  memcpy(gx,data+40,4);
  memcpy(gy,data+44,4);
  memcpy(gz,data+48,4);

  if (errType < 0){return errType;}
  else{return 0;}
}

/* Writes the current register settings to non-volatile memory */
void VectorNav::writeSettings() {
  if( _useSPI ){

    if(timeSinceTX >= 50) {

    } else {
      delayMicroseconds(50 - timeSinceTX);
    }

    // Teensy 3.0 || Teensy 3.1/3.2
    #if defined(__MK20DX128__) || defined(__MK20DX256__)
      if((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)){
        // begin the transaction
        SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
        digitalWriteFast(_csPin,LOW); // select the VN100
        SPI.transfer(CMD_FLASH); // specify command is a Flash
        SPI.transfer(0x00); // 3 bytes of zeros sent in header
        SPI.transfer(0x00); // 3 bytes of zeros sent in header
        SPI.transfer(0x00); // 3 bytes of zeros sent in header
        digitalWriteFast(_csPin,HIGH); // deselect the VN100
        timeSinceTX = 0;

        delay(1000); // writing to non-volatile memory takes about 500 ms to complete
      }
    #endif

    // // Teensy 3.5 || Teensy 3.6 
    // #if defined(__MK64FX512__) || defined(__MK66FX1M0__)
    //   if((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)||(_mosiPin == MOSI_PIN_28)){
    //     // begin the transaction
    //     SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
    //     digitalWriteFast(_csPin,LOW); // select the VN100
    //     SPI.transfer(subAddress | SPI_READ); // specify the starting register address

    //     for(uint8_t i = 0; i < count; i++){
    //       dest[i] = SPI.transfer(0x00); // read the data
    //     }

    //     digitalWriteFast(_csPin,HIGH); // deselect the VN100
    //     SPI.endTransaction(); // end the transaction
    //   }
    //   else if((_mosiPin == MOSI_PIN_0)||(_mosiPin == MOSI_PIN_21)){
    //     // begin the transaction
    //     SPI1.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
    //     digitalWriteFast(_csPin,LOW); // select the VN100
    //     SPI1.transfer(subAddress | SPI_READ); // specify the starting register address

    //     for(uint8_t i = 0; i < count; i++){
    //       dest[i] = SPI1.transfer(0x00); // read the data
    //     }

    //     digitalWriteFast(_csPin,HIGH); // deselect the VN100
    //     SPI1.endTransaction(); // end the transaction
    //   }
    //   else if((_mosiPin == MOSI_PIN_44)||(_mosiPin == MOSI_PIN_52)){
    //     // begin the transaction
    //     SPI2.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
    //     digitalWriteFast(_csPin,LOW); // select the VN100
    //     SPI2.transfer(subAddress | SPI_READ); // specify the starting register address

    //     for(uint8_t i = 0; i < count; i++){
    //       dest[i] = SPI2.transfer(0x00); // read the data
    //     }

    //     digitalWriteFast(_csPin,HIGH); // deselect the VN100
    //     SPI2.endTransaction(); // end the transaction
    //   }
    // #endif

    // // Teensy LC 
    // #if defined(__MKL26Z64__)
    //   if((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)){
    //     // begin the transaction
    //     SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
    //     digitalWriteFast(_csPin,LOW); // select the VN100
    //     SPI.transfer(subAddress | SPI_READ); // specify the starting register address

    //     for(uint8_t i = 0; i < count; i++){
    //       dest[i] = SPI.transfer(0x00); // read the data
    //     }

    //     digitalWriteFast(_csPin,HIGH); // deselect the VN100
    //     SPI.endTransaction(); // end the transaction
    //   }
    //   else if((_mosiPin == MOSI_PIN_0)||(_mosiPin == MOSI_PIN_21)){
    //     // begin the transaction
    //     SPI1.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
    //     digitalWriteFast(_csPin,LOW); // select the VN100
    //     SPI1.transfer(subAddress | SPI_READ); // specify the starting register address

    //     for(uint8_t i = 0; i < count; i++){
    //       dest[i] = SPI1.transfer(0x00); // read the data
    //     }

    //     digitalWriteFast(_csPin,HIGH); // deselect the VN100
    //     SPI1.endTransaction(); // end the transaction
    //   }
    // #endif
  }
  else{
    // SERIAL
  }
}

/* Restores the sensor to factory defaults */
void VectorNav::restoreSettings() {
  if( _useSPI ){

    if(timeSinceTX >= 50) {

    } else {
      delayMicroseconds(50 - timeSinceTX);
    }

    // Teensy 3.0 || Teensy 3.1/3.2
    #if defined(__MK20DX128__) || defined(__MK20DX256__)
      if((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)){
        // begin the transaction
        SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
        digitalWriteFast(_csPin,LOW); // select the VN100
        SPI.transfer(CMD_RESTORE); // specify command is a Flash
        SPI.transfer(0x00); // 3 bytes of zeros sent in header
        SPI.transfer(0x00); // 3 bytes of zeros sent in header
        SPI.transfer(0x00); // 3 bytes of zeros sent in header
        digitalWriteFast(_csPin,HIGH); // deselect the VN100
        timeSinceTX = 0;

        delay(5000); // takes a few seconds for the sensor to come back up and converge on a solution
      }
    #endif

    // // Teensy 3.5 || Teensy 3.6 
    // #if defined(__MK64FX512__) || defined(__MK66FX1M0__)
    //   if((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)||(_mosiPin == MOSI_PIN_28)){
    //     // begin the transaction
    //     SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
    //     digitalWriteFast(_csPin,LOW); // select the VN100
    //     SPI.transfer(subAddress | SPI_READ); // specify the starting register address

    //     for(uint8_t i = 0; i < count; i++){
    //       dest[i] = SPI.transfer(0x00); // read the data
    //     }

    //     digitalWriteFast(_csPin,HIGH); // deselect the VN100
    //     SPI.endTransaction(); // end the transaction
    //   }
    //   else if((_mosiPin == MOSI_PIN_0)||(_mosiPin == MOSI_PIN_21)){
    //     // begin the transaction
    //     SPI1.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
    //     digitalWriteFast(_csPin,LOW); // select the VN100
    //     SPI1.transfer(subAddress | SPI_READ); // specify the starting register address

    //     for(uint8_t i = 0; i < count; i++){
    //       dest[i] = SPI1.transfer(0x00); // read the data
    //     }

    //     digitalWriteFast(_csPin,HIGH); // deselect the VN100
    //     SPI1.endTransaction(); // end the transaction
    //   }
    //   else if((_mosiPin == MOSI_PIN_44)||(_mosiPin == MOSI_PIN_52)){
    //     // begin the transaction
    //     SPI2.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
    //     digitalWriteFast(_csPin,LOW); // select the VN100
    //     SPI2.transfer(subAddress | SPI_READ); // specify the starting register address

    //     for(uint8_t i = 0; i < count; i++){
    //       dest[i] = SPI2.transfer(0x00); // read the data
    //     }

    //     digitalWriteFast(_csPin,HIGH); // deselect the VN100
    //     SPI2.endTransaction(); // end the transaction
    //   }
    // #endif

    // // Teensy LC 
    // #if defined(__MKL26Z64__)
    //   if((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)){
    //     // begin the transaction
    //     SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
    //     digitalWriteFast(_csPin,LOW); // select the VN100
    //     SPI.transfer(subAddress | SPI_READ); // specify the starting register address

    //     for(uint8_t i = 0; i < count; i++){
    //       dest[i] = SPI.transfer(0x00); // read the data
    //     }

    //     digitalWriteFast(_csPin,HIGH); // deselect the VN100
    //     SPI.endTransaction(); // end the transaction
    //   }
    //   else if((_mosiPin == MOSI_PIN_0)||(_mosiPin == MOSI_PIN_21)){
    //     // begin the transaction
    //     SPI1.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
    //     digitalWriteFast(_csPin,LOW); // select the VN100
    //     SPI1.transfer(subAddress | SPI_READ); // specify the starting register address

    //     for(uint8_t i = 0; i < count; i++){
    //       dest[i] = SPI1.transfer(0x00); // read the data
    //     }

    //     digitalWriteFast(_csPin,HIGH); // deselect the VN100
    //     SPI1.endTransaction(); // end the transaction
    //   }
    // #endif
  }
  else{
    // SERIAL
  }
}

/* Resets the sensors */
/* Return 0 on success or the VN error code on error. */
void VectorNav::resetSensor() {
  if( _useSPI ){

    if(timeSinceTX >= 50) {

    } else {
      delayMicroseconds(50 - timeSinceTX);
    }

    // Teensy 3.0 || Teensy 3.1/3.2
    #if defined(__MK20DX128__) || defined(__MK20DX256__)
      if((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)){
        // begin the transaction
        SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
        digitalWriteFast(_csPin,LOW); // select the VN100
        SPI.transfer(CMD_RESET); // specify command is a Reset
        SPI.transfer(0x00); // 3 bytes of zeros sent in header
        SPI.transfer(0x00); // 3 bytes of zeros sent in header
        SPI.transfer(0x00); // 3 bytes of zeros sent in header
        digitalWriteFast(_csPin,HIGH); // deselect the VN100
        timeSinceTX = 0;

        delay(5000); // takes a few seconds for the sensor to come back up and converge on a solution
      }
    #endif

    // // Teensy 3.5 || Teensy 3.6 
    // #if defined(__MK64FX512__) || defined(__MK66FX1M0__)
    //   if((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)||(_mosiPin == MOSI_PIN_28)){
    //     // begin the transaction
    //     SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
    //     digitalWriteFast(_csPin,LOW); // select the VN100
    //     SPI.transfer(subAddress | SPI_READ); // specify the starting register address

    //     for(uint8_t i = 0; i < count; i++){
    //       dest[i] = SPI.transfer(0x00); // read the data
    //     }

    //     digitalWriteFast(_csPin,HIGH); // deselect the VN100
    //     SPI.endTransaction(); // end the transaction
    //   }
    //   else if((_mosiPin == MOSI_PIN_0)||(_mosiPin == MOSI_PIN_21)){
    //     // begin the transaction
    //     SPI1.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
    //     digitalWriteFast(_csPin,LOW); // select the VN100
    //     SPI1.transfer(subAddress | SPI_READ); // specify the starting register address

    //     for(uint8_t i = 0; i < count; i++){
    //       dest[i] = SPI1.transfer(0x00); // read the data
    //     }

    //     digitalWriteFast(_csPin,HIGH); // deselect the VN100
    //     SPI1.endTransaction(); // end the transaction
    //   }
    //   else if((_mosiPin == MOSI_PIN_44)||(_mosiPin == MOSI_PIN_52)){
    //     // begin the transaction
    //     SPI2.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
    //     digitalWriteFast(_csPin,LOW); // select the VN100
    //     SPI2.transfer(subAddress | SPI_READ); // specify the starting register address

    //     for(uint8_t i = 0; i < count; i++){
    //       dest[i] = SPI2.transfer(0x00); // read the data
    //     }

    //     digitalWriteFast(_csPin,HIGH); // deselect the VN100
    //     SPI2.endTransaction(); // end the transaction
    //   }
    // #endif

    // // Teensy LC 
    // #if defined(__MKL26Z64__)
    //   if((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)){
    //     // begin the transaction
    //     SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
    //     digitalWriteFast(_csPin,LOW); // select the VN100
    //     SPI.transfer(subAddress | SPI_READ); // specify the starting register address

    //     for(uint8_t i = 0; i < count; i++){
    //       dest[i] = SPI.transfer(0x00); // read the data
    //     }

    //     digitalWriteFast(_csPin,HIGH); // deselect the VN100
    //     SPI.endTransaction(); // end the transaction
    //   }
    //   else if((_mosiPin == MOSI_PIN_0)||(_mosiPin == MOSI_PIN_21)){
    //     // begin the transaction
    //     SPI1.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
    //     digitalWriteFast(_csPin,LOW); // select the VN100
    //     SPI1.transfer(subAddress | SPI_READ); // specify the starting register address

    //     for(uint8_t i = 0; i < count; i++){
    //       dest[i] = SPI1.transfer(0x00); // read the data
    //     }

    //     digitalWriteFast(_csPin,HIGH); // deselect the VN100
    //     SPI1.endTransaction(); // end the transaction
    //   }
    // #endif
  }
  else{
    // SERIAL
  }
}

/* Reads registers from VN100 given a starting register address, number of bytes, and a pointer to store data. */
/* Returns the number of bytes read on success or the VN error code on error. */
int VectorNav::readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest){
  uint8_t buffer[HEADER_LENGTH];

  if( _useSPI ){

    if(timeSinceTX >= 50) {

    } else {
      delayMicroseconds(50 - timeSinceTX);
    }

    // Teensy 3.0 || Teensy 3.1/3.2
    #if defined(__MK20DX128__) || defined(__MK20DX256__)
      if((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)){
        // begin the transaction
        SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
        digitalWriteFast(_csPin,LOW); // select the VN100
        SPI.transfer(CMD_READ); // specify command is a read
        SPI.transfer(subAddress); // specify the starting register address
        SPI.transfer(0x00); // 2 bytes of zeros sent in header
        SPI.transfer(0x00); // 2 bytes of zeros sent in header
        digitalWriteFast(_csPin,HIGH); // deselect the VN100
        delayMicroseconds(50); // wait at least 50 us for response buffer to fill
        digitalWriteFast(_csPin,LOW); // select the VN100
        for(uint8_t i = 0; i <  HEADER_LENGTH; i++){
          buffer[i] = SPI.transfer(0x00); // read the header
        }

        // check the response header
        if(buffer[3] != 0) {
          // end communication
          digitalWriteFast(_csPin,HIGH); // deselect the VN100
          SPI.endTransaction(); // end the transaction
          timeSinceTX = 0;
          return -1*buffer[3];
        } else {
          for(uint8_t i = 0; i <  count; i++){
            dest[i] = SPI.transfer(0x00); // read the data
          }
          // end communication
          digitalWriteFast(_csPin,HIGH); // deselect the VN100
          SPI.endTransaction(); // end the transaction
          timeSinceTX = 0;
        }
      }
    #endif

    // // Teensy 3.5 || Teensy 3.6 
    // #if defined(__MK64FX512__) || defined(__MK66FX1M0__)
    //   if((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)||(_mosiPin == MOSI_PIN_28)){
    //     // begin the transaction
    //     SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
    //     digitalWriteFast(_csPin,LOW); // select the VN100
    //     SPI.transfer(subAddress | SPI_READ); // specify the starting register address

    //     for(uint8_t i = 0; i < count; i++){
    //       dest[i] = SPI.transfer(0x00); // read the data
    //     }

    //     digitalWriteFast(_csPin,HIGH); // deselect the VN100
    //     SPI.endTransaction(); // end the transaction
    //   }
    //   else if((_mosiPin == MOSI_PIN_0)||(_mosiPin == MOSI_PIN_21)){
    //     // begin the transaction
    //     SPI1.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
    //     digitalWriteFast(_csPin,LOW); // select the VN100
    //     SPI1.transfer(subAddress | SPI_READ); // specify the starting register address

    //     for(uint8_t i = 0; i < count; i++){
    //       dest[i] = SPI1.transfer(0x00); // read the data
    //     }

    //     digitalWriteFast(_csPin,HIGH); // deselect the VN100
    //     SPI1.endTransaction(); // end the transaction
    //   }
    //   else if((_mosiPin == MOSI_PIN_44)||(_mosiPin == MOSI_PIN_52)){
    //     // begin the transaction
    //     SPI2.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
    //     digitalWriteFast(_csPin,LOW); // select the VN100
    //     SPI2.transfer(subAddress | SPI_READ); // specify the starting register address

    //     for(uint8_t i = 0; i < count; i++){
    //       dest[i] = SPI2.transfer(0x00); // read the data
    //     }

    //     digitalWriteFast(_csPin,HIGH); // deselect the VN100
    //     SPI2.endTransaction(); // end the transaction
    //   }
    // #endif

    // // Teensy LC 
    // #if defined(__MKL26Z64__)
    //   if((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)){
    //     // begin the transaction
    //     SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
    //     digitalWriteFast(_csPin,LOW); // select the VN100
    //     SPI.transfer(subAddress | SPI_READ); // specify the starting register address

    //     for(uint8_t i = 0; i < count; i++){
    //       dest[i] = SPI.transfer(0x00); // read the data
    //     }

    //     digitalWriteFast(_csPin,HIGH); // deselect the VN100
    //     SPI.endTransaction(); // end the transaction
    //   }
    //   else if((_mosiPin == MOSI_PIN_0)||(_mosiPin == MOSI_PIN_21)){
    //     // begin the transaction
    //     SPI1.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
    //     digitalWriteFast(_csPin,LOW); // select the VN100
    //     SPI1.transfer(subAddress | SPI_READ); // specify the starting register address

    //     for(uint8_t i = 0; i < count; i++){
    //       dest[i] = SPI1.transfer(0x00); // read the data
    //     }

    //     digitalWriteFast(_csPin,HIGH); // deselect the VN100
    //     SPI1.endTransaction(); // end the transaction
    //   }
    // #endif
  }
  else{
    // SERIAL
  }

  return count;
}

/* Writes registers to the VN100 given a starting register address, number of bytes, and a pointer to a buffer of data. */
/* Returns the number of bytes written on success or the VN error code on error. */
int VectorNav::writeRegisters(uint8_t subAddress, uint8_t count, uint8_t* buffer){
  uint8_t headerBuffer[HEADER_LENGTH];

  if( _useSPI ){

    if(timeSinceTX >= 50) {

    } else {
      delayMicroseconds(50 - timeSinceTX);
    }

    // Teensy 3.0 || Teensy 3.1/3.2
    #if defined(__MK20DX128__) || defined(__MK20DX256__)
      if((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)){
        // begin the transaction
        SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
        digitalWriteFast(_csPin,LOW); // select the VN100
        SPI.transfer(CMD_WRITE); // specify command is a write
        SPI.transfer(subAddress); // specify the starting register address
        SPI.transfer(0x00); // 2 bytes of zeros sent in header
        SPI.transfer(0x00); // 2 bytes of zeros sent in header
        for(uint8_t i = 0; i < count; i++){
          SPI.transfer(buffer[i]);
        }
        digitalWriteFast(_csPin,HIGH); // deselect the VN100
        delayMicroseconds(50); // wait at least 50 us for response buffer to fill
        digitalWriteFast(_csPin,LOW); // select the VN100
        for(uint8_t i = 0; i <  HEADER_LENGTH; i++){
          headerBuffer[i] = SPI.transfer(0x00); // read the header
        }
        // end communication
        digitalWriteFast(_csPin,HIGH); // deselect the VN100
        SPI.endTransaction(); // end the transaction
        timeSinceTX = 0;

        // check the response header
        if(headerBuffer[3] != 0) {
          return -1*headerBuffer[3];
        }
      }
    #endif

    // // Teensy 3.5 || Teensy 3.6 
    // #if defined(__MK64FX512__) || defined(__MK66FX1M0__)
    //   if((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)||(_mosiPin == MOSI_PIN_28)){
    //     // begin the transaction
    //     SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
    //     digitalWriteFast(_csPin,LOW); // select the VN100
    //     SPI.transfer(subAddress | SPI_READ); // specify the starting register address

    //     for(uint8_t i = 0; i < count; i++){
    //       dest[i] = SPI.transfer(0x00); // read the data
    //     }

    //     digitalWriteFast(_csPin,HIGH); // deselect the VN100
    //     SPI.endTransaction(); // end the transaction
    //   }
    //   else if((_mosiPin == MOSI_PIN_0)||(_mosiPin == MOSI_PIN_21)){
    //     // begin the transaction
    //     SPI1.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
    //     digitalWriteFast(_csPin,LOW); // select the VN100
    //     SPI1.transfer(subAddress | SPI_READ); // specify the starting register address

    //     for(uint8_t i = 0; i < count; i++){
    //       dest[i] = SPI1.transfer(0x00); // read the data
    //     }

    //     digitalWriteFast(_csPin,HIGH); // deselect the VN100
    //     SPI1.endTransaction(); // end the transaction
    //   }
    //   else if((_mosiPin == MOSI_PIN_44)||(_mosiPin == MOSI_PIN_52)){
    //     // begin the transaction
    //     SPI2.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
    //     digitalWriteFast(_csPin,LOW); // select the VN100
    //     SPI2.transfer(subAddress | SPI_READ); // specify the starting register address

    //     for(uint8_t i = 0; i < count; i++){
    //       dest[i] = SPI2.transfer(0x00); // read the data
    //     }

    //     digitalWriteFast(_csPin,HIGH); // deselect the VN100
    //     SPI2.endTransaction(); // end the transaction
    //   }
    // #endif

    // // Teensy LC 
    // #if defined(__MKL26Z64__)
    //   if((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)){
    //     // begin the transaction
    //     SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
    //     digitalWriteFast(_csPin,LOW); // select the VN100
    //     SPI.transfer(subAddress | SPI_READ); // specify the starting register address

    //     for(uint8_t i = 0; i < count; i++){
    //       dest[i] = SPI.transfer(0x00); // read the data
    //     }

    //     digitalWriteFast(_csPin,HIGH); // deselect the VN100
    //     SPI.endTransaction(); // end the transaction
    //   }
    //   else if((_mosiPin == MOSI_PIN_0)||(_mosiPin == MOSI_PIN_21)){
    //     // begin the transaction
    //     SPI1.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
    //     digitalWriteFast(_csPin,LOW); // select the VN100
    //     SPI1.transfer(subAddress | SPI_READ); // specify the starting register address

    //     for(uint8_t i = 0; i < count; i++){
    //       dest[i] = SPI1.transfer(0x00); // read the data
    //     }

    //     digitalWriteFast(_csPin,HIGH); // deselect the VN100
    //     SPI1.endTransaction(); // end the transaction
    //   }
    // #endif
  }
  else{
    // SERIAL
  }

  return count;
}

/* Inputs the inertial velocity in the sensor frame (x, y, z) in m/s
for compensating the onboard EKF. Should be updated at a rate of at least 5 Hz */
/* Return 0 on success or the VN error code on error. */
int VN100::velocityCompensation(float U, float V, float W) {
  uint8_t buffer[VEL_COMPENSATION_MEAS_REG[1]];
  memset(buffer,0,VEL_COMPENSATION_MEAS_REG[1]);

  // magnetometer, accel, gyro, temperature, and pressure window sizes
  memcpy(buffer,&U,sizeof(U));
  memcpy(buffer+4,&V,sizeof(V));
  memcpy(buffer+8,&W,sizeof(W));

  // write registers
  int errType = writeRegisters(VEL_COMPENSATION_MEAS_REG[0],VEL_COMPENSATION_MEAS_REG[1],buffer);
  if (errType < 0){return errType;}
  else{return 0;}
}

/* Commands the VN100 to tare attitude at the current orientation. */
void VN100::tareAttitude() {
  if( _useSPI ){

    if(timeSinceTX >= 50) {

    } else {
      delayMicroseconds(50 - timeSinceTX);
    }

    // Teensy 3.0 || Teensy 3.1/3.2
    #if defined(__MK20DX128__) || defined(__MK20DX256__)
      if((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)){
        // begin the transaction
        SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
        digitalWriteFast(_csPin,LOW); // select the VN100
        SPI.transfer(CMD_TARE); // specify command is a Tare
        SPI.transfer(0x00); // 3 bytes of zeros sent in header
        SPI.transfer(0x00); // 3 bytes of zeros sent in header
        SPI.transfer(0x00); // 3 bytes of zeros sent in header
        digitalWriteFast(_csPin,HIGH); // deselect the VN100
        timeSinceTX = 0;

        delay(5000); // takes a few seconds for the sensor to come back up and converge on a solution
      }
    #endif

    // // Teensy 3.5 || Teensy 3.6 
    // #if defined(__MK64FX512__) || defined(__MK66FX1M0__)
    //   if((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)||(_mosiPin == MOSI_PIN_28)){
    //     // begin the transaction
    //     SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
    //     digitalWriteFast(_csPin,LOW); // select the VN100
    //     SPI.transfer(subAddress | SPI_READ); // specify the starting register address

    //     for(uint8_t i = 0; i < count; i++){
    //       dest[i] = SPI.transfer(0x00); // read the data
    //     }

    //     digitalWriteFast(_csPin,HIGH); // deselect the VN100
    //     SPI.endTransaction(); // end the transaction
    //   }
    //   else if((_mosiPin == MOSI_PIN_0)||(_mosiPin == MOSI_PIN_21)){
    //     // begin the transaction
    //     SPI1.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
    //     digitalWriteFast(_csPin,LOW); // select the VN100
    //     SPI1.transfer(subAddress | SPI_READ); // specify the starting register address

    //     for(uint8_t i = 0; i < count; i++){
    //       dest[i] = SPI1.transfer(0x00); // read the data
    //     }

    //     digitalWriteFast(_csPin,HIGH); // deselect the VN100
    //     SPI1.endTransaction(); // end the transaction
    //   }
    //   else if((_mosiPin == MOSI_PIN_44)||(_mosiPin == MOSI_PIN_52)){
    //     // begin the transaction
    //     SPI2.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
    //     digitalWriteFast(_csPin,LOW); // select the VN100
    //     SPI2.transfer(subAddress | SPI_READ); // specify the starting register address

    //     for(uint8_t i = 0; i < count; i++){
    //       dest[i] = SPI2.transfer(0x00); // read the data
    //     }

    //     digitalWriteFast(_csPin,HIGH); // deselect the VN100
    //     SPI2.endTransaction(); // end the transaction
    //   }
    // #endif

    // // Teensy LC 
    // #if defined(__MKL26Z64__)
    //   if((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)){
    //     // begin the transaction
    //     SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
    //     digitalWriteFast(_csPin,LOW); // select the VN100
    //     SPI.transfer(subAddress | SPI_READ); // specify the starting register address

    //     for(uint8_t i = 0; i < count; i++){
    //       dest[i] = SPI.transfer(0x00); // read the data
    //     }

    //     digitalWriteFast(_csPin,HIGH); // deselect the VN100
    //     SPI.endTransaction(); // end the transaction
    //   }
    //   else if((_mosiPin == MOSI_PIN_0)||(_mosiPin == MOSI_PIN_21)){
    //     // begin the transaction
    //     SPI1.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
    //     digitalWriteFast(_csPin,LOW); // select the VN100
    //     SPI1.transfer(subAddress | SPI_READ); // specify the starting register address

    //     for(uint8_t i = 0; i < count; i++){
    //       dest[i] = SPI1.transfer(0x00); // read the data
    //     }

    //     digitalWriteFast(_csPin,HIGH); // deselect the VN100
    //     SPI1.endTransaction(); // end the transaction
    //   }
    // #endif
  }
  else{
    // SERIAL
  }
}

/* Sets the antenna offset from the VN200. Position X, Y, Z are the relative position
from the VN200 to its antenna in the VN200 frame with units of meters. */
/* Returns 0 on success or the VN200 error code on error. */
int VN200::setAntennaOffset(float positionX, float positionY, float positionZ) {
  uint8_t buffer[GPS_ANT_OFFSET_REG[1]];
  memset(buffer,0,GPS_ANT_OFFSET_REG[1]);

  // antenna offset
  memcpy(buffer,&positionX,sizeof(positionX));
  memcpy(buffer+4,&positionY,sizeof(positionY));
  memcpy(buffer+8,&positionZ,sizeof(positionZ));

  // write registers
  int errType = writeRegisters(GPS_ANT_OFFSET_REG[0],GPS_ANT_OFFSET_REG[1],buffer);
  if (errType < 0){return errType;}
  else{return 0;}
}

/* Copies the current bias estimates into register 74 and writes them to NVM. The VN-200
will use these bias estimates as the initial state at startup. */
void VN200::setFilterBias() {
  if( _useSPI ){

    if(timeSinceTX >= 50) {

    } else {
      delayMicroseconds(50 - timeSinceTX);
    }

    // Teensy 3.0 || Teensy 3.1/3.2
    #if defined(__MK20DX128__) || defined(__MK20DX256__)
      if((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)){
        // begin the transaction
        SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
        digitalWriteFast(_csPin,LOW); // select the VN100
        SPI.transfer(CMD_SET_FILTER_BIAS); // specify command is a Tare
        SPI.transfer(0x00); // 3 bytes of zeros sent in header
        SPI.transfer(0x00); // 3 bytes of zeros sent in header
        SPI.transfer(0x00); // 3 bytes of zeros sent in header
        digitalWriteFast(_csPin,HIGH); // deselect the VN100
        timeSinceTX = 0;

        delay(1000); 

        writeSettings(); // write the settings to NVM
      }
    #endif

    // // Teensy 3.5 || Teensy 3.6 
    // #if defined(__MK64FX512__) || defined(__MK66FX1M0__)
    //   if((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)||(_mosiPin == MOSI_PIN_28)){
    //     // begin the transaction
    //     SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
    //     digitalWriteFast(_csPin,LOW); // select the VN100
    //     SPI.transfer(subAddress | SPI_READ); // specify the starting register address

    //     for(uint8_t i = 0; i < count; i++){
    //       dest[i] = SPI.transfer(0x00); // read the data
    //     }

    //     digitalWriteFast(_csPin,HIGH); // deselect the VN100
    //     SPI.endTransaction(); // end the transaction
    //   }
    //   else if((_mosiPin == MOSI_PIN_0)||(_mosiPin == MOSI_PIN_21)){
    //     // begin the transaction
    //     SPI1.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
    //     digitalWriteFast(_csPin,LOW); // select the VN100
    //     SPI1.transfer(subAddress | SPI_READ); // specify the starting register address

    //     for(uint8_t i = 0; i < count; i++){
    //       dest[i] = SPI1.transfer(0x00); // read the data
    //     }

    //     digitalWriteFast(_csPin,HIGH); // deselect the VN100
    //     SPI1.endTransaction(); // end the transaction
    //   }
    //   else if((_mosiPin == MOSI_PIN_44)||(_mosiPin == MOSI_PIN_52)){
    //     // begin the transaction
    //     SPI2.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
    //     digitalWriteFast(_csPin,LOW); // select the VN100
    //     SPI2.transfer(subAddress | SPI_READ); // specify the starting register address

    //     for(uint8_t i = 0; i < count; i++){
    //       dest[i] = SPI2.transfer(0x00); // read the data
    //     }

    //     digitalWriteFast(_csPin,HIGH); // deselect the VN100
    //     SPI2.endTransaction(); // end the transaction
    //   }
    // #endif

    // // Teensy LC 
    // #if defined(__MKL26Z64__)
    //   if((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)){
    //     // begin the transaction
    //     SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
    //     digitalWriteFast(_csPin,LOW); // select the VN100
    //     SPI.transfer(subAddress | SPI_READ); // specify the starting register address

    //     for(uint8_t i = 0; i < count; i++){
    //       dest[i] = SPI.transfer(0x00); // read the data
    //     }

    //     digitalWriteFast(_csPin,HIGH); // deselect the VN100
    //     SPI.endTransaction(); // end the transaction
    //   }
    //   else if((_mosiPin == MOSI_PIN_0)||(_mosiPin == MOSI_PIN_21)){
    //     // begin the transaction
    //     SPI1.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
    //     digitalWriteFast(_csPin,LOW); // select the VN100
    //     SPI1.transfer(subAddress | SPI_READ); // specify the starting register address

    //     for(uint8_t i = 0; i < count; i++){
    //       dest[i] = SPI1.transfer(0x00); // read the data
    //     }

    //     digitalWriteFast(_csPin,HIGH); // deselect the VN100
    //     SPI1.endTransaction(); // end the transaction
    //   }
    // #endif
  }
  else{
    // SERIAL
  }  
}

int VN200::getGpsLla(double* tow, uint16_t* week, uint8_t* FixType, uint8_t* NumSV, double* latitude, double* longitude, double* altitude, float* NEDVelX, 
  float* NEDVelY, float* NEDVelZ, float* NorthAcc, float* EastAcc, float* VertAcc, float* SpeedAcc, float* TimeAcc) {
  uint8_t data[GPS_LLA_MEAS_REG[1]];
  float latitude_d, longitude_d;
  int errType = readRegisters(GPS_LLA_MEAS_REG[0],GPS_LLA_MEAS_REG[1],data);
  memcpy(tow,data,8);
  memcpy(week,data+8,2);
  memcpy(FixType,data+10,1);
  memcpy(NumSV,data+11,1);
  memcpy(&latitude_d,data+16,8);
  memcpy(&longitude_d,data+24,8);
  memcpy(altitude,data+32,8);
  memcpy(NEDVelX,data+40,4);
  memcpy(NEDVelY,data+44,4);
  memcpy(NEDVelZ,data+48,4);
  memcpy(NorthAcc,data+52,4);
  memcpy(EastAcc,data+56,4);
  memcpy(VertAcc,data+60,4);
  memcpy(SpeedAcc,data+64,4);
  memcpy(TimeAcc,data+68,4);

  *latitude = latitude_d * deg2radL;
  *longitude = longitude_d * deg2radL;

  if (errType < 0){return errType;}
  else{return 0;}
}

int VN200::getGpsEcef(double* tow, uint16_t* week, uint8_t* FixType, uint8_t* NumSV, double* PosX, double* PosY, double* PosZ, float* VelX, float* VelY, 
  float* VelZ, float* XAcc, float* YAcc, float* ZAcc, float* SpeedAcc, float* TimeAcc) {
  uint8_t data[GPS_ECEF_MEAS_REG[1]];
  int errType = readRegisters(GPS_ECEF_MEAS_REG[0],GPS_ECEF_MEAS_REG[1],data);
  memcpy(tow,data,8);
  memcpy(week,data+8,2);
  memcpy(FixType,data+10,1);
  memcpy(NumSV,data+11,1);
  memcpy(PosX,data+16,8);
  memcpy(PosY,data+24,8);
  memcpy(PosZ,data+32,8);
  memcpy(VelX,data+40,4);
  memcpy(VelY,data+44,4);
  memcpy(VelZ,data+48,4);
  memcpy(XAcc,data+52,4);
  memcpy(YAcc,data+56,4);
  memcpy(ZAcc,data+60,4);
  memcpy(SpeedAcc,data+64,4);
  memcpy(TimeAcc,data+68,4);

  if (errType < 0){return errType;}
  else{return 0;}
}

int VN200::getInsLla(double* tow, uint16_t* week, uint8_t* mode, uint8_t* Fix, uint8_t* ErrorType, float* yaw, float* pitch, float* roll, double* latitude, double* longitude, 
  double* altitude, float* NEDVelX, float* NEDVelY, float* NEDVelZ, float* AttAcc, float* PosAcc, float* VelAcc) {
  uint8_t data[INS_LLA_SOL_MEAS_REG[1]];
  uint16_t status;
  float yaw_d, pitch_d, roll_d, AttAcc_d;
  float latitude_d, longitude_d;
  int errType = readRegisters(INS_LLA_SOL_MEAS_REG[0],INS_LLA_SOL_MEAS_REG[1],data);
  memcpy(tow,data,8);
  memcpy(week,data+8,2);
  memcpy(&status,data+10,2);
  memcpy(&yaw_d,data+12,4);
  memcpy(&pitch_d,data+16,4);
  memcpy(&roll_d,data+20,4);
  memcpy(&latitude_d,data+24,8);
  memcpy(&longitude_d,data+32,8);
  memcpy(altitude,data+40,8);
  memcpy(NEDVelX,data+48,4);
  memcpy(NEDVelY,data+52,4);
  memcpy(NEDVelZ,data+56,4);
  memcpy(&AttAcc_d,data+60,4);
  memcpy(PosAcc,data+64,4);
  memcpy(VelAcc,data+68,4);

  *latitude = latitude_d * deg2radL;
  *longitude = longitude_d * deg2radL;

  *yaw = yaw_d * deg2rad;
  *pitch = pitch_d * deg2rad;
  *roll = roll_d * deg2rad;
  *AttAcc = AttAcc_d * deg2rad;

  if ((status & 0x03) == (0x00)) {
    *mode = 0;
  } else if ((status & 0x03) == (0x01)) {
    *mode = 1;
  } else if ((status & 0x03) == (0x02)) {
    *mode = 2;
  }

  if ((status & 0x04) == (0x04)) {
    *Fix = 1;
  } else {
    *Fix = 0;
  }

  if ((status & 0x78) == (0x08)) {
    *ErrorType = 0;
  } else if ((status & 0x78) == (0x10)) {
    *ErrorType = 1;
  } else if ((status & 0x78) == (0x20)) {
    *ErrorType = 2;
  } else if ((status & 0x78) == (0x40)) {
    *ErrorType = 3;
  }

  if (errType < 0){return errType;}
  else{return 0;}
}

int VN200::getInsEcef(double* tow, uint16_t* week, uint8_t* mode, uint8_t* Fix, uint8_t* ErrorType, float* yaw, float* pitch, float* roll, double* PosX, double* PosY, double* PosZ, 
  float* VelX, float* VelY, float* VelZ, float* AttAcc, float* PosAcc, float* VelAcc) {
  uint8_t data[INS_ECEF_SOL_MEAS_REG[1]];
  uint16_t status;
  float yaw_d, pitch_d, roll_d, AttAcc_d;
  int errType = readRegisters(INS_ECEF_SOL_MEAS_REG[0],INS_ECEF_SOL_MEAS_REG[1],data);
  memcpy(tow,data,8);
  memcpy(week,data+8,2);
  memcpy(&status,data+10,2);
  memcpy(&yaw_d,data+12,4);
  memcpy(&pitch_d,data+16,4);
  memcpy(&roll_d,data+20,4);
  memcpy(PosX,data+24,8);
  memcpy(PosY,data+32,8);
  memcpy(PosZ,data+40,8);
  memcpy(VelX,data+48,4);
  memcpy(VelY,data+52,4);
  memcpy(VelZ,data+56,4);
  memcpy(&AttAcc_d,data+60,4);
  memcpy(PosAcc,data+64,4);
  memcpy(VelAcc,data+68,4);

  *yaw = yaw_d * deg2rad;
  *pitch = pitch_d * deg2rad;
  *roll = roll_d * deg2rad;
  *AttAcc = AttAcc_d * deg2rad;

  if ((status & 0x03) == (0x00)) {
    *mode = 0;
  } else if ((status & 0x03) == (0x01)) {
    *mode = 1;
  } else if ((status & 0x03) == (0x02)) {
    *mode = 2;
  }

  if ((status & 0x04) == (0x04)) {
    *Fix = 1;
  } else {
    *Fix = 0;
  }

  if ((status & 0x78) == (0x08)) {
    *ErrorType = 0;
  } else if ((status & 0x78) == (0x10)) {
    *ErrorType = 1;
  } else if ((status & 0x78) == (0x20)) {
    *ErrorType = 2;
  } else if ((status & 0x78) == (0x40)) {
    *ErrorType = 3;
  }

  if (errType < 0){return errType;}
  else{return 0;}
}

int VN200::getNavLla(float* yaw, float* pitch, float* roll, double* latitude, double* longitude, double* altitude, float* NEDVelX, float* NEDVelY, float* NEDVelZ, 
  float* ax, float* ay, float* az, float* gx, float* gy, float* gz) {

  uint8_t data[INS_LLA_STATE_MEAS_REG[1]];
  float yaw_d, pitch_d, roll_d;
  float latitude_d, longitude_d;
  int errType = readRegisters(INS_LLA_STATE_MEAS_REG[0],INS_LLA_STATE_MEAS_REG[1],data);


  memcpy(&yaw_d,data,4);
  memcpy(&pitch_d,data+4,4);
  memcpy(&roll_d,data+8,4);
  memcpy(&latitude_d,data+12,8);
  memcpy(&longitude_d,data+20,8);
  memcpy(altitude,data+28,8);

  memcpy(NEDVelX,data+36,4);
  memcpy(NEDVelY,data+40,4);
  memcpy(NEDVelZ,data+44,4);

  memcpy(ax,data+48,4);
  memcpy(ay,data+52,4);
  memcpy(az,data+56,4);

  memcpy(gx,data+60,4);
  memcpy(gy,data+64,4);
  memcpy(gz,data+68,4);

  *latitude = latitude_d * deg2radL;
  *longitude = longitude_d * deg2radL;

  *yaw = yaw_d * deg2rad;
  *pitch = pitch_d * deg2rad;
  *roll = roll_d * deg2rad;

  if (errType < 0){return errType;}
  else{return 0;}
}

int VN200::getNavEcef(float* yaw, float* pitch, float* roll, double* PosX, double* PosY, double* PosZ, float* VelX, float* VelY, float* VelZ, float* ax, float* ay, 
  float* az, float* gx, float* gy, float* gz) {
  uint8_t data[INS_ECEF_STATE_MEAS_REG[1]];
  float yaw_d, pitch_d, roll_d;
  int errType = readRegisters(INS_ECEF_STATE_MEAS_REG[0],INS_ECEF_STATE_MEAS_REG[1],data);

  memcpy(&yaw_d,data,4);
  memcpy(&pitch_d,data+4,4);
  memcpy(&roll_d,data+8,4);
  memcpy(PosX,data+12,8);
  memcpy(PosY,data+20,8);
  memcpy(PosZ,data+28,8);

  memcpy(VelX,data+36,4);
  memcpy(VelY,data+40,4);
  memcpy(VelZ,data+44,4);

  memcpy(ax,data+48,4);
  memcpy(ay,data+52,4);
  memcpy(az,data+56,4);

  memcpy(gx,data+60,4);
  memcpy(gy,data+64,4);
  memcpy(gz,data+68,4);

  *yaw = yaw_d * deg2rad;
  *pitch = pitch_d * deg2rad;
  *roll = roll_d * deg2rad;

  if (errType < 0){return errType;}
  else{return 0;}
}

#endif
