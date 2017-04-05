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

/* VN100 object, input the SPI CS Pin */
VN100::VN100(uint8_t csPin){
  _csPin = csPin; // SPI CS Pin
  _mosiPin = MOSI_PIN_11; // SPI MOSI Pin, set to default
  _useSPI = true; // set to use SPI instead of I2C
}

/* VN100 object, input the SPI CS Pin and MOSI Pin */
VN100::VN100(uint8_t csPin, spi_mosi_pin pin){
  _csPin = csPin; // SPI CS Pin
  _mosiPin = pin; // SPI MOSI Pin
  _useSPI = true; // set to use SPI instead of I2C
}

/* Starts communication and sets up the VN100 */
int VN100::begin(){

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

/* Get accelerometer data given pointers to store the three values. */
/* Return 0 on success or the VN error code on error. */
int VN100::getAccel(float* ax, float* ay, float* az) {
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
int VN100::getGyro(float* gx, float* gy, float* gz) {
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
int VN100::getMag(float* hx, float* hy, float* hz) {
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
int VN100::getPressure(float* pressure) {
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
int VN100::getTemperature(float* temperature) {
  uint8_t data[IMU_MEAS_REG[1]];
  int errType = readRegisters(IMU_MEAS_REG[0],IMU_MEAS_REG[1],data);
  memcpy(temperature,data+36,4);

  if (errType < 0){return errType;}
  else{return 0;}
}

/* Get motion6 data given pointers to store the values. */
/* Return 0 on success or the VN error code on error. */
int VN100::getMotion6(float* ax, float* ay, float* az, float* gx, float* gy, float* gz) {
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
int VN100::getMotion9(float* ax, float* ay, float* az, float* gx, float* gy, float* gz, float* hx, float* hy, float* hz) {
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
int VN100::getEuler(float* yaw, float* pitch, float* roll) {
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
int VN100::getEulerIMU(float* yaw, float* pitch, float* roll, float* ax, float* ay, float* az, float* gx, float* gy, float* gz, float* hx, float* hy, float* hz) {
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

/* Reads registers from VN100 given a starting register address, number of bytes, and a pointer to store data. */
/* Returns the number of bytes read on success or the VN error code on error. */
int VN100::readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest){
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
int VN100::writeRegisters(uint8_t subAddress, uint8_t count, uint8_t* buffer){
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

#endif
