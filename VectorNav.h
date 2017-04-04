/*
VectorNav.h
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

#ifndef VECTORNAV_h
#define VECTORNAV_h

#include "Arduino.h"

#ifndef SPI_MOSI_PIN
#define SPI_MOSI_PIN
  // Teensy 3.0 || Teensy 3.1/3.2
  #if defined(__MK20DX128__) || defined(__MK20DX256__)
  enum spi_mosi_pin
  {
    MOSI_PIN_7,
    MOSI_PIN_11
  };
  #endif
  // Teensy 3.5 || Teensy 3.6
  #if defined(__MK64FX512__) || defined(__MK66FX1M0__)
  enum spi_mosi_pin
  {
    MOSI_PIN_0,
    MOSI_PIN_7,
    MOSI_PIN_11,
    MOSI_PIN_21,
    MOSI_PIN_28,
    MOSI_PIN_44,
    MOSI_PIN_52
  };
  #endif
  // Teensy LC 
  #if defined(__MKL26Z64__)
  enum spi_mosi_pin
  {
    MOSI_PIN_0,
    MOSI_PIN_7,
    MOSI_PIN_11,
    MOSI_PIN_21
  };
  #endif
#endif

class VN100{
  public:
    VN100(uint8_t cspin);
    VN100(uint8_t csPin, spi_mosi_pin pin);
    int begin();
    int getAccel(float* ax, float* ay, float* az);
    int getGyro(float* gx, float* gy, float* gz);
    int getMag(float* hx, float* hy, float* hz);
    int getPressure(float* pressure);
    int getTemperature(float* temperature);
    int getMotion6(float* ax, float* ay, float* az, float* gx, float* gy, float* gz);
    int getMotion9(float* ax, float* ay, float* az, float* gx, float* gy, float* gz, float* hx, float* hy, float* hz);
    int getEuler(float* yaw, float* pitch, float* roll);
    int getEulerIMU(float* yaw, float* pitch, float* roll, float* ax, float* ay, float* az, float* gx, float* gy, float* gz, float* hx, float* hy, float* hz);
    int readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest);
  private:
    // spi
    uint8_t _csPin;
    spi_mosi_pin _mosiPin;
    bool _useSPI;
    const uint32_t SPI_CLOCK = 16000000; // 16 MHz

    // timing, used to ensure at least 50us between communication with VN
    elapsedMicros timeSinceTX;

    // conversions
    const float G2uT = 100.0f;
    const float kPa2Pa = 1000.0f;
    const float deg2rad = PI/180.0f;

    // VectorNav communication header length
    const uint8_t HEADER_LENGTH = 4;

    // commands
    const uint8_t CMD_READ = 0x01;
    const uint8_t CMD_WRITE = 0x02;
    const uint8_t CMD_FLASH = 0x03;
    const uint8_t CMD_RESTORE = 0x04;
    const uint8_t CMD_TARE = 0x05;
    const uint8_t CMD_RESET = 0x06;
    const uint8_t CMD_MAG_DISTURBANCE = 0x08;
    const uint8_t CMD_ACCEL_DISTURBANCE = 0x09;
    const uint8_t CMD_SET_GYRO_BIAS     = 0x0C;

    /* registers and sizes (register number, number of bytes) */

    // Device configuration registers
    const uint8_t USER_TAG_REG[2]                   = {0,   20};
    const uint8_t MODEL_NUMBER_REG[2]               = {1,   24};
    const uint8_t HARDWARE_REV_REG[2]               = {2,   4};
    const uint8_t SERIAL_NUM_REG[2]                 = {3,   4};
    const uint8_t FIRMWARE_VERSION_REG[2]           = {4,   4};
    const uint8_t SERIAL_BAUDRATE_REG[2]            = {5,   4};
    const uint8_t ASYNC_DATA_OUTPUT_TYPE_REG[2]     = {6,   4};
    const uint8_t ASYNC_DATA_OUTPUT_FREQ_REG[2]     = {7,   4};
    const uint8_t SYNC_CONTROL_REG[2]               = {32,  20};
    const uint8_t COMM_PROTOCOL_CNTRL_REG[2]        = {30,  7};
    const uint8_t BIN_OUTPUT_1_REG[2]               = {75,  22};
    const uint8_t BIN_OUTPUT_2_REG[2]               = {76,  22};
    const uint8_t BIN_OUTPUT_3_REG[2]               = {77,  22};

    // IMU configuration
    const uint8_t MAG_COMPENSATION_REG[2]           = {23,  48};     
    const uint8_t ACCEL_COMPENSATION_REG[2]         = {25,  48};
    const uint8_t GYRO_COMPENSATION_REG[2]          = {84,  48};
    const uint8_t REF_FRAME_ROTATION_REG[2]         = {26,  36};
    const uint8_t IMU_FILTER_CONFIG_REG[2]          = {85,  15};
    const uint8_t DELTA_THETA_VEL_CONFIG_REG[2]     = {82,  6};

    // Attitude configuration
    const uint8_t VPE_BASIC_CNTRL_REG[2]            = {35,  4};
    const uint8_t VPE_MAG_TUNING_REG[2]             = {36,  36};
    const uint8_t VPE_ACCEL_TUNING_REG[2]           = {38,  36};

    // Hard/soft iron estimator configuration
    const uint8_t MAG_HSI_CNTRL_REG[2]              = {44,  4};

    // Velocity compensation configuration
    const uint8_t VEL_COMPENSATION_CNTRL_REG[2]     = {51,  8};

    // World magnetic and gravity model configuration
    const uint8_t MAG_GRAV_REFERENCE_REG[2]         = {21,  24};
    const uint8_t REF_VECTOR_CONFIGURATION_REG[2]   = {83,  32};

    // Hard/soft iron estimator status
    const uint8_t CALC_HSI_REG[2]                   = {47,  48};    

    // Device status registers
    const uint8_t SYNC_STATUS_REG[2]                = {33,  12};

    // IMU measurements
    const uint8_t IMU_MEAS_REG[2]                   = {54,  44};
    const uint8_t DELTA_THETA_VEL_REG[2]            = {80,  28};

    // Attitude measurements
    const uint8_t EULER_REG[2]                      = {8,   12};
    const uint8_t QUATERNION_REG[2]                 = {9,   16};
    const uint8_t EULER_IMU_REG[2]                  = {27,  48};
    const uint8_t QUATERNION_IMU_REG[2]             = {15,  52};
    const uint8_t MAG_COMP_REG[2]                   = {17,  12};
    const uint8_t ACCEL_COMP_REG[2]                 = {18,  12};
    const uint8_t GYRO_COMP_REG[2]                  = {19,  12};
    const uint8_t IMU_COMP_REG[2]                   = {20,  36};
    const uint8_t EULER_GYRO_BODY_ACCEL_REG[2]      = {239, 36};
    const uint8_t EULER_GYRO_INERTIAL_ACCEL_REG[2]  = {240, 36};

    // Velocity compensation input
    const uint8_t VEL_COMPENSATION_MEAS_REG[2]      = {50,  12};
};

class VN200: public VN100{
  public:
    using VN100::VN100;
  private:
    // commands
    const uint8_t CMD_SET_FILTER_BIAS = 0x11;

    // GPS configuration
    const uint8_t GPS_CONFIG_REG[2]                 = {55,  5};
    const uint8_t GPS_ANT_OFFSET_REG[2]             = {57,  12};

    // GPS measurements
    const uint8_t GPS_LLA_MEAS_REG[2]               = {58,  72};
    const uint8_t GPS_ECEF_MEAS_REG[2]              = {59,  72};

    // INS configuration
    const uint8_t INS_CONFIG_REG[2]                 = {67,  4};
    const uint8_t INS_STARTUP_FILTER_BIAS_REG[2]    = {74,  28};

    // INS measurements
    const uint8_t INS_LLA_SOL_MEAS_REG[2]           = {63,  72};
    const uint8_t INS_ECEF_SOL_MEAS_REG[2]          = {64,  72};
    const uint8_t INS_LLA_STATE_MEAS_REG[2]         = {72,  72};
    const uint8_t INS_ECEF_STATE_MEAS_REG[2]        = {73,  72};
};

/* VN-300 firmware still in beta without SPI support. Below is an
education guess at the additional commands and registers needed.
class VN300: public VN200{
  public:
    using VN200::VN200;
  private:

    // commands
    const uint8_t CMD_SET_INITIAL_HEADING = 0x12;

    // GPS configuration
    const uint8_t GPS_COMPASS_BASELINE_CONFIG_REG[2]= {93,  24};
    const uint8_t GPS_COMPASS_ESTIMATED_BASELINE_REG[2]   = {97,  28};
};
*/

#endif
