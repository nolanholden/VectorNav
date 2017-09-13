# VectorNav
Library for communicating with the VectorNav [VN-100](https://www.vectornav.com/products/vn-100) Inertial Measurement Unit and Attitude Heading Reference System (IMU/AHRS) and [VN-200](https://www.vectornav.com/products/vn-200) Inertial Navigation Systems (INS) using Teensy 3.x and LC devices.

# Description
The VN-100 is a miniature, high performance Inertial Measurement Unit and Attitude Heading Reference System (IMU/AHRS). The VN-100 combines a three-axis accelerometer, three-axis gyroscope, three-axis magnetometer, and a barometric pressure sensor with a 32-bit processor. In addition to calibrated and filtered sensor measurements, the VN-100 computes and outputs real-time orientation solutions. Sensor measurements are available at rates of up to 800 Hz and orientation outputs at rates of up to 400 Hz.

The VN-200 adds a uBlox GPS module to the sensors available on the VN-100. Using GPS aiding, the VN-200 provides accurate estimates of position, velocity, and orientation.

The VN-100 and VN-200 are temperature calibrated at 25C. Thermal calibration over the temperature range of -40C to +85C is available as an option.

# Usage
This library currently only supports SPI communication with the VN-100 and VN-200. The VN-300 firmware currently does not support SPI communication and is not supported by this library. Additionally, SPI communication with the VN-100 IMU/AHRS and VN-200 INS are only available on the surface mount device (SMD) packages.

## Installation
Simply clone or download this library into your Arduino/libraries folder. [Teensyduino](http://pjrc.com/teensy/td_download.html) version 1.37 or newer is required.

## Function Descriptions
This library supports both the VN-100 IMU/AHRS and VN-200 INS. The object declaration is different depending on the sensor used. Additionally, while most functions are available on both sensors, some functions are unique given the sensors required (for example, functions using GPS on the VN-200). Functions are listed below for object declaration, setup, and data collection. In each of these categories functions are given for the VN-100, VN-200, and functions common to both.

## Return Types
FILL

### Object Declaration 

#### VN-100
**VN100(uint8_t csPin)**
A VN100 object should be declared, specifying the Teensy chip select pin used. Multiple VN-100 or other SPI objects could be used on the same SPI bus, each with their own chip select pin. SPI Bus 0 is used with the default MOSI, MISO, and SCK pins. The chip select pin can be any available digital pin. For example, the following code declares a VN100 object called *IMU* with a VN-100 sensor located on chip select pin 10.

```C++
VN100 IMU(10);
```

**VN100(uint8_t csPin, SPIClass &ast;SPI)**
Optionally, the SPI bus can be specified. This allows selecting SPI buses other than SPI Bus 0. For example, the following code declares a VN100 object called *IMU* with a VN-100 sensor located on chip select pin 10 and SPI bus 2.

```C++
VN100 IMU(10, &SPI2);
```

**setMOSI, setMISO, and setSCK**
*setMOSI*, *setMISO*, and *setSCK* can be used after the SPI object declaration and before calling begin to set alternate MOSI, MISO, and SCK pins. For example, the code below uses SPI Bus 0, but pin 14 for SCK instead of pin 13.

```C++
VN100 IMU(10);
void setup() {
   SPI.setSCK(14);
   IMU.begin();
}
```

#### VN-200
**VN200(uint8_t csPin)**
A VN200 object should be declared, specifying the Teensy chip select pin used. Multiple VN-200 or other SPI objects could be used on the same SPI bus, each with their own chip select pin. SPI Bus 0 is used with the default MOSI, MISO, and SCK pins. The chip select pin can be any available digital pin. For example, the following code declares a VN200 object called *IMU* with a VN-200 sensor located on chip select pin 10.

```C++
VN200 IMU(10);
```

**VN200(uint8_t csPin, SPIClass &ast;SPI)**
Optionally, the SPI bus can be specified. This allows selecting SPI buses other than SPI Bus 0. For example, the following code declares a VN200 object called *IMU* with a VN-200 sensor located on chip select pin 10 and SPI bus 2.

```C++
VN200 IMU(10, &SPI2);
```

**setMOSI, setMISO, and setSCK**
*setMOSI*, *setMISO*, and *setSCK* can be used after the SPI object declaration and before calling begin to set alternate MOSI, MISO, and SCK pins. For example, the code below uses SPI Bus 0, but pin 14 for SCK instead of pin 13.

```C++
VN200 IMU(10);
void setup() {
   SPI.setSCK(14);
   IMU.begin();
}
```

### Setup Functions
The following functions are used to setup the VN-100 or VN-200. These should be called once before data collection, typically this is done in the Arduino *void setup()* function. The *begin* function should always be used and the rest provide optional functionality.

#### Common
**int begin()**
This should be called in your setup function. It initializes communication with the VN-100 or VN-200.

```C++
VN200 IMU(10);
void setup() {
   IMU.begin();
}
```

**(optional) int enableInterrupt(uint16_t SRD, uint32_t pulseWidth)**
This is an optional function to enable and configure VN-100 or VN-200 generated interrupts. Interrupts are triggered at the start of the VectorNav IMU frame and provide a low jitter interrupt source. You can configure the interrupt period by setting a sample rate divider (SRD) off the 800 Hz IMU base rate. Additionally the pulse width can be configured and is set in units of nano seconds. For example, the following code enables interrupts at 100 Hz with a 0.5 ms (500000 ns) pulse width.

```C++
VN200 IMU(10);
void setup() {
   IMU.begin();
   IMU.enableInterrupt(4,500000);
}
```

**(optional) int setDLPF(uint16_t magWindowSize, uint16_t accelWindowSize, uint16_t gyroWindowSize, uint16_t temperatureWindowSize, uint16_t pressureWindowSize)**
This is an optional function to configure digital low pass filtering on the VN-100 or VN-200. This FIR filter is a uniformly-weighted moving window (boxcar) filter of configurable size. The WindowSize parameter sets the number of samples at the IMU rate (800 Hz) which will be averaged for each output measurement. The filtering does not affect the values used internally to perform the orientation estimate; only the output values are affected. The filtering can be used to down-sample the output IMU measurements to ensure that information is not lost when the IMU measurements are sampled at a lower rate than the internal IMU rate (800 Hz). For example, the following code sets the filter window size to 16 for the magnetometer and accelerometer output, 8 for the gyroscope output, and 4 for the temperature and pressure output.

```C++
VN200 IMU(10);
void setup() {
   IMU.begin();
   IMU.setDLPF(16,16,8,4,4);
}
```

**(optional) int setReferenceFrameRotation(float T[3][3])**
This is an optional function to input a transformation matrix to rotate from the VN-100 or VN-200 internal frame to the vehicle body frame. This transformation matrix rotates the IMU measurements and the orientation estimation, enabling the VN-100 or VN-200 sensor to be placed in any orientation and output sensor measurements and estimates in the vehicle body frame. The transformation matrix is written to non-volatile memory and the sensor is reset, so this function takes about 6 seconds to complete. For example, the following code rotates the sensor 90 degrees in the X-Y plane.

```C++
VN200 IMU(10);
void setup() {
   IMU.begin();
   float T[3][3];
   T[0][0] = 0.0f;		T[0][1] = -1.0f;	T[0][2] = 0.0f;
   T[1][0] = 1.0f;		T[1][1] = 0.0f;		T[1][2] = 0.0f;
   T[2][0] = 0.0f;		T[2][1] = 0.0f;		T[2][2] = 1.0f;		
   IMU.setReferenceFrameRotation(T);
}
```

**(optional) void writeSettings()**
This optional function writes the current VN-100 or VN-200 settings to non-volatile memory. It takes about 1 second to complete this command.

**(optional) void restoreSettings()**
This optional function restores the VN-100 or VN-200 to its factory default settings. It takes about 5 seconds to complete this command.

**(optional) void resetSensor()**
This optional function commands the VN-100 or VN-200 to reboot. It takes about 5 seconds to complete this command.

#### VN-100

**(optional) void tareAttitude()**
This is an optional function to tare (zero) the attitude at the current orientation. For example, the following code tares the attitude.

```C++
VN200 IMU(10);
void setup() {
   IMU.begin();
   IMU.tareAttitude();
}
```
   
**(optional) int velocityCompensation(float U, float V, float W)**
This is an optional function to provide velocity information to aid the VN-100 AHRS. Without any form of external compensation an AHRS does not have by itself any means of knowing how it is moving relative to the fixed Earth. Since the accelerometer measures the effect of both gravity and the acceleration due to motion, the standard AHRS algorithm has to make the assumption that the long term acceleration due to motion is zero. This is not strictly true in all cases; the classic example is an aircraft in a prolonged steady and coordinated turn. In this case, the accelerometer measures a long term acceleration due to the centripetal force created by traveling along a curved path. If the VN-100 is uncompensated during this maneuver, it would incorrectly estimate zero roll angle. By providing velocity information, the VN-100 AHRS can correct for these effects and provide more accurate attitude estimates.

Velocity is input in the X, Y, and Z axis of the sensor frame. If a scalar measurement of velocity is used, such as that measured by an airspeed sensor, the X velocity can be set leaving the Y and Z velocity at zero. The best corrections would be from inertial velocity measurements, such as GPS. While airspeed could be a good source in calm or low wind conditions, at higher wind speeds increased error would be present in the velocity aiding. Velocity compensation should be updated at a rate of at least 1 Hz with the best results for update rates of 10 Hz or higher. Velocities are given in the sensor body-frame in units of m/s. For example, the following code provides a velocity compensation update for a vehicle traveling at 20 m/s as measured by an airspeed sensor.

```C++
IMU.velocityCompensation(20,0,0);
```

#### VN-200

**(optional) int setAntennaOffset(float positionX, float positionY, float positionZ)**
This is an optional function to set the GPS antenna position relative to the VN-200 in the sensor coordinate frame with the origin as the VN-200 and the X, Y, and Z distances given in units of meters.

**(optional) void setFilterBias()**
This is an optional function to save the current filter bias estimates to non-volatile memory. These saved bias estimates will be used as the initial state at startup.

### Data Collection Functions

## Example List

# Wiring

# Performance
