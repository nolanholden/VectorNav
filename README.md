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
All functions below that return an int, return 0 on success. On error the VectorNav error code is returned as a negative number (i.e. VectorNav error code 1 is returned as -1). 

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
The following functions are used to setup the VN-100 or VN-200. All of these other than *velocityCompensation* should be called once before data collection, typically this is done in the Arduino *void setup()* function. The *begin* function should always be used and the rest provide optional functionality. If the functionality of *velocityCompensation* is desired, it should be called at rates above 1 Hz, with 10 Hz or greater providing the best accuracy.

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
This is an optional function to tare (zero) the attitude at the current orientation. It takes about 5 seconds to complete this command. For example, the following code tares the attitude.

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
This is an optional function to save the current filter bias estimates to non-volatile memory. These saved bias estimates will be used as the initial state at the next startup. It takes about 2 seconds to complete this command.

### Data Collection Functions
The functions below are used to collect data from the VN-100 or VN-200. All of the data returned by the function were collected from the VN-100 or VN-200 at the same time, so it is preferable to use the function which returns all of the desired data rather than two separate function calls in order to eliminate potential time skews in your results. For example, it would be preferable to use *getMotion6* to get both gyroscope and accelerometer data rather than call *getAccel* followed by *getGyro*. This preference is because the gyroscope and accelerometer data returned by *getMotion6* were all sampled simultaneously whereas using *getAccel* followed by *getGyro* could possibly introduce a time skew between the accelerometer and gyroscope data. Units of the returned data are given in the detailed description for each function.

#### Common

**int getAccel(float&ast; ax, float&ast; ay, float&ast; az)**
*getAccel(float&ast; ax, float&ast; ay, float&ast; az)* samples the VN-100 or VN-200 and returns the three-axis accelerometer data as floats in m/s/s.

```C++
float ax, ay, az;
IMU.getAccel(&ax, &ay, &az);
```

**int getGyro(float&ast; gx, float&ast; gy, float&ast; gz)**
*getGyro(float&ast; gx, float&ast; gy, float&ast; gz)* samples the VN-100 or VN-200 and returns the three-axis gyroscope data as floats in rad/s.

```C++
float gx, gy, gz;
IMU.getGyro(&gx, &gy, &gz);
```

**int getMag(float&ast; hx, float&ast; hy, float&ast; hz)**
*getMag(float&ast; hx, float&ast; hy, float&ast; hz)* samples the VN-100 or VN-200 and returns the three-axis magnetometer data as floats in uT.

```C++
float hx, hy, hz;
IMU.getMag(&hx, &hy, &hz);
```

**int getPressure(float&ast; pressure)**
*getPressure(float&ast; pressure)* samples the VN-100 or VN-200 and returns the static pressure as a float in Pa.

```C++
float pressure;
IMU.getPressure(&pressure);
```

**int getTemperature(float&ast; t)**
*getTemperature(float&ast; t)* samples the VN-100 or VN-200 and returns the die temperature as a float in C.

```C++
float t;
IMU.getTemperature(&t);
```

**int getMotion6(float&ast; ax, float&ast; ay, float&ast; az, float&ast; gx, float&ast; gy, float&ast; gz)**
*getMotion6(float&ast; ax, float&ast; ay, float&ast; az, float&ast; gx, float&ast; gy, float&ast; gz)* samples the VN-100 or VN-200 and returns the three-axis accelerometer data as floats in m/s/s and the three-axis gyroscope data as floats in rad/s.

```C++
float ax, ay, az, gx, gy, gz;
IMU.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
```

**int getMotion9(float&ast; ax, float&ast; ay, float&ast; az, float&ast; gx, float&ast; gy, float&ast; gz, float&ast; hx, float&ast; hy, float&ast; hz)**
*getMotion9(float&ast; ax, float&ast; ay, float&ast; az, float&ast; gx, float&ast; gy, float&ast; gz, float&ast; hx, float&ast; hy, float&ast; hz)* samples the VN-100 or VN-200 and returns the three-axis accelerometer data as floats in m/s/s, the three-axis gyroscope data as floats in rad/s, and the three-axis magnetometer data as floats in uT.

```C++
float ax, ay, az, gx, gy, gz, hx, hy, hz;
IMU.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &hx, &hy, &hz);
```

**int getEuler(float&ast; yaw, float&ast; pitch, float&ast; roll)**
*getEuler(float&ast; yaw, float&ast; pitch, float&ast; roll)* samples the VN-100 or VN-200 and returns the three-axis orientation estimate as floats in rad.

```C++
float yaw, pitch, roll;
IMU.getEuler(&yaw, &pitch, &roll);
```

**int getEulerIMU(float&ast; yaw, float&ast; pitch, float&ast; roll, float&ast; ax, float&ast; ay, float&ast; az, float&ast; gx, float&ast; gy, float&ast; gz, float&ast; hx, float&ast; hy, float&ast; hz)**
*getEulerIMU(float&ast; yaw, float&ast; pitch, float&ast; roll, float&ast; ax, float&ast; ay, float&ast; az, float&ast; gx, float&ast; gy, float&ast; gz, float&ast; hx, float&ast; hy, float&ast; hz)* samples the VN-100 or VN-200 and returns the three-axis orientation estimate and IMU data. Yaw, pitch, and roll orentation are given as floats in rad, the three-axis accelerometer data as floats in m/s/s, the three-axis gyroscope data as floats in rad/s, and the three-axis magnetometer data as floats in uT.    
    
```C++
float yaw, pitch, roll, ax, ay, az, gx, gy, gz, hx, hy, hz;
IMU.getEulerIMU(&yaw, &pitch, &roll, &ax, &ay, &az, &gx, &gy, &gz, &hx, &hy, &hz);
```

**int getQuat(float&ast; quat[4])**
*int getQuat(float&ast; quat[4])* samples the VN-100 or VN-200 and returns the orientation estimate as  quaternion vector with the fourth value as the scalar terma.

```C++
float quat[4];
IMU.getQuat(&quat);
```

**int getQuatIMU(float&ast; quat[4], float&ast; ax, float&ast; ay, float&ast; az, float&ast; gx, float&ast; gy, float&ast; gz, float&ast; hx, float&ast; hy, float&ast; hz)**
*getQuatIMU(float&ast; quat[4], float&ast; ax, float&ast; ay, float&ast; az, float&ast; gx, float&ast; gy, float&ast; gz, float&ast; hx, float&ast; hy, float&ast; hz)* samples the VN-100 or VN-200 and returns the orientation estimate and IMU data. The orientation estimate is given as a quaternion vector with the fourth value as the scalar term, the three-axis accelerometer data as floats in m/s/s, the three-axis gyroscope data as floats in rad/s, and the three-axis magnetometer data as floats in uT.

```C++
float quat[4], ax, ay, az, gx, gy, gz, hx, hy, hz;
IMU.getQuatIMU(quat, &ax, &ay, &az, &gx, &gy, &gz, &hx, &hy, &hz);
```

#### VN-200

**int getGpsLla(double&ast; tow, uint16_t&ast; week, uint8_t&ast; FixType, uint8_t&ast; NumSV, double&ast; latitude, double&ast; longitude, double&ast; altitude, float&ast; NEDVelX, float&ast; NEDVelY, float&ast; NEDVelZ, float&ast; NorthAcc, float&ast; EastAcc, float&ast; VertAcc, float&ast; SpeedAcc, float&ast; TimeAcc)**
*getGpsLla(double&ast; tow, uint16_t&ast; week, uint8_t&ast; FixType, uint8_t&ast; NumSV, double&ast; latitude, double&ast; longitude, double&ast; altitude, float&ast; NEDVelX, float&ast; NEDVelY, float&ast; NEDVelZ, float&ast; NorthAcc, float&ast; EastAcc, float&ast; VertAcc, float&ast; SpeedAcc, float&ast; TimeAcc)* samples the VN-200 and gives GPS data in Latitude, Longitude, Altitude (LLA) format. GPS time of week is given in seconds, followed by the GPS week number, the fix type, the number of satellites used in the solution, latitude and longitude in rad, altitude in m, NED velocity in m/s, NED position accuracy in m, speed accuracy in m/s, and time accuracy in seconds.

The fix type is described by:

| Value | Description |
| ----- | ----------- |
| 0     | No Fix      |
| 1     | Time Only   |
| 2     | 2D Fix      |
| 3     | 3D Fix      |

```C++
double tow, latitude, longitude, altitude;
float NEDVelX, NEDVelY, NEDVelZ, NorthAcc, EastAcc, VertAcc, SpeedAcc, TimeAcc;
uint16_t week;
uint8_t FixType, NumSV;
IMU.getGpsLla(&tow, &week, &FixType, &NumSV, &latitude, &longitude, &altitude, &NEDVelX, &NEDVelY, &NEDVelZ, &NorthAcc, &EastAcc, &VertAcc, &SpeedAcc, &TimeAcc);
```

**int getGpsEcef(double&ast; tow, uint16_t&ast; week, uint8_t&ast; FixType, uint8_t&ast; NumSV, double&ast; PosX, double&ast; PosY, double&ast; PosZ, float&ast; VelX, float&ast; VelY, float&ast; VelZ, float&ast; XAcc, float&ast; YAcc, float&ast; ZAcc, float&ast; SpeedAcc, float&ast; TimeAcc)**
*getGpsEcef(double&ast; tow, uint16_t&ast; week, uint8_t&ast; FixType, uint8_t&ast; NumSV, double&ast; PosX, double&ast; PosY, double&ast; PosZ, float&ast; VelX, float&ast; VelY, float&ast; VelZ, float&ast; XAcc, float&ast; YAcc, float&ast; ZAcc, float&ast; SpeedAcc, float&ast; TimeAcc)* samples the VN-200 and gives GPS data in Earth Centered Earth Fixed (ECEF) format. GPS time of week is given in seconds, followed by the GPS week number, the fix type, the number of satellites used in the solution, ECEF X, Y, and Z coordinates in m, ECEF velocity in m/s, ECEF position accuracy in m, speed accuracy in m/s, and time accuracy in seconds.

The fix type is described by:

| Value | Description |
| ----- | ----------- |
| 0     | No Fix      |
| 1     | Time Only   |
| 2     | 2D Fix      |
| 3     | 3D Fix      |

```C++
double tow, PosX, PosY, PosZ;
float VelX, VelY, VelZ, XAcc, YAcc, ZAcc, SpeedAcc, TimeAcc;
uint16_t week;
uint8_t FixType, NumSV;
IMU.getGpsEcef(&tow, &week, &FixType, &NumSV, &PosX, &PosY, &PosZ, &VelX, &VelY, &VelZ, &XAcc, &YAcc, &ZAcc, &SpeedAcc, &TimeAcc);
```

**int getInsLla(double&ast; tow, uint16_t&ast; week, uint8_t&ast; mode, uint8_t&ast; Fix, uint8_t&ast; ErrorType, float&ast; yaw, float&ast; pitch, float&ast; roll, double&ast; latitude, double&ast; longitude, double&ast; altitude, float&ast; NEDVelX, float&ast; NEDVelY, float&ast; NEDVelZ, float&ast; AttAcc, float&ast; PosAcc, float&ast; VelAcc)**
*getInsLla(double&ast; tow, uint16_t&ast; week, uint8_t&ast; mode, uint8_t&ast; Fix, uint8_t&ast; ErrorType, float&ast; yaw, float&ast; pitch, float&ast; roll, double&ast; latitude, double&ast; longitude, double&ast; altitude, float&ast; NEDVelX, float&ast; NEDVelY, float&ast; NEDVelZ, float&ast; AttAcc, float&ast; PosAcc, float&ast; VelAcc)* samples the VN-200 and gives GPS data in Latitude, Longitude, Altitude (LLA) format along with the orientation estimate. GPS time of week is given in seconds, followed by the GPS week number, the INS mode (more detailed information below), GPS Fix (bool, 1 for a proper fix), INS error type (more detailed information below), yaw, pitch, and roll Euler angles in rad, latitude and longitude in rad, altitude in m, NED velocity in m/s, attitude uncertainty in rad, position uncertainty in m, and velocity uncertainty in m/s.

The mode is described by:

| Value | Description |
| ----- | ----------- |
| 0     | Not tracking. Insufficient dynamic motion to estimate attitude       |
| 1     | Sufficient dynamic motion, but solution not within performance specs |
| 2     | INS is tracking and operating within specifications                  |

The error type is described by:

| Value | Description |
| ----- | ----------- |
| 1     | Time Error, INS filter loop exceeded 5 ms                         |
| 2     | IMU Error, IMU communication error is detected                    |
| 3     | Mag/Pres Error, Magnetometer or Pressure sensor error is detected |
| 4     | GPS Error, GPS communication error is detected                    |

```C++
double tow, latitude, longitude, altitude;
float yaw, pitch, roll, NEDVelX, NEDVelY, NEDVelZ, AttAcc, PosAcc, VelAcc;
uint16_t week;
uint8_t mode, Fix, ErrorType;
IMU.getInsLla(&tow, &week, &mode, &Fix, &ErrorType, &yaw, &pitch, &roll, &latitude, &longitude, &altitude, &NEDVelX, &NEDVelY, &NEDVelZ, &AttAcc, &PosAcc, &VelAcc);
```
    
**int getInsEcef(double&ast; tow, uint16_t&ast; week, uint8_t&ast; mode, uint8_t&ast; Fix, uint8_t&ast; ErrorType, float&ast; yaw, float&ast; pitch, float&ast; roll, double&ast; PosX, double&ast; PosY, double&ast; PosZ, float&ast; VelX, float&ast; VelY, float&ast; VelZ, float&ast; AttAcc, float&ast; PosAcc, float&ast; VelAcc)**
*getInsEcef(double&ast; tow, uint16_t&ast; week, uint8_t&ast; mode, uint8_t&ast; Fix, uint8_t&ast; ErrorType, float&ast; yaw, float&ast; pitch, float&ast; roll, double&ast; PosX, double&ast; PosY, double&ast; PosZ, float&ast; VelX, float&ast; VelY, float&ast; VelZ, float&ast; AttAcc, float&ast; PosAcc, float&ast; VelAcc)* samples the VN-200 and gives GPS data in Earth Centered Earth Fixed (ECEF) format along with the orientation estimate. GPS time of week is given in seconds, followed by the GPS week number, the INS mode (more detailed information below), GPS Fix (bool, 1 for a proper fix), INS error type (more detailed information below), yaw, pitch, and roll Euler angles in rad,  ECEF X, Y, and Z coordinates in m, ECEF velocity in m/s, attitude uncertainty in rad, position uncertainty in m, and velocity uncertainty in m/s.

The mode is described by:

| Value | Description |
| ----- | ----------- |
| 0     | Not tracking. Insufficient dynamic motion to estimate attitude       |
| 1     | Sufficient dynamic motion, but solution not within performance specs |
| 2     | INS is tracking and operating within specifications                  |

The error type is described by:

| Value | Description |
| ----- | ----------- |
| 1     | Time Error, INS filter loop exceeded 5 ms                         |
| 2     | IMU Error, IMU communication error is detected                    |
| 3     | Mag/Pres Error, Magnetometer or Pressure sensor error is detected |
| 4     | GPS Error, GPS communication error is detected                    |

```C++
double tow, PosX, PosY, PosZ;
float yaw, pitch, roll, VelX, VelY, VelZ, AttAcc, PosAcc, VelAcc;
uint16_t week;
uint8_t mode, Fix, ErrorType;
IMU.getInsEcef(&tow, &week, &mode, &Fix, &ErrorType, &yaw, &pitch, &roll, &PosX, &PosY, &PosZ, &VelX, &VelY, &VelZ, &AttAcc, &PosAcc, &VelAcc);
```

**int getNavLla(float&ast; yaw, float&ast; pitch, float&ast; roll, double&ast; latitude, double&ast; longitude, double&ast; altitude, float&ast; NEDVelX, float&ast; NEDVelY, float&ast; NEDVelZ, float&ast; ax, float&ast; ay, float&ast; az, float&ast; gx, float&ast; gy, float&ast; gz)**
*int getNavLla(float&ast; yaw, float&ast; pitch, float&ast; roll, double&ast; latitude, double&ast; longitude, double&ast; altitude, float&ast; NEDVelX, float&ast; NEDVelY, float&ast; NEDVelZ, float&ast; ax, float&ast; ay, float&ast; az, float&ast; gx, float&ast; gy, float&ast; gz)* samples the VN-200 and gives the INS state with position in Latitude, Longitude, and Altitude (LLA) format. Orientation is given as yaw, roll, and pitch in rad, latitude and longitude in rad, altitude in m, NED velocity in m/s, body frame acceleration in m/s/s, and body frame angular rates in rad/s.
    
```C++
double latitude, longitude, altitude;
float yaw, pitch, roll, NEDVelX, NEDVelY, NEDVelZ, ax, ay, az, gx, gy, gz;
IMU.getNavLla(&yaw, &pitch, &roll, &latitude, &longitude, &altitude, &NEDVelX, &NEDVelY, &NEDVelZ, &ax, &ay, &az, &gx, &gy, &gz);
```

**int getNavEcef(float&ast; yaw, float&ast; pitch, float&ast; roll, double&ast; PosX, double&ast; PosY, double&ast; PosZ, float&ast; VelX, float&ast; VelY, float&ast; VelZ, float&ast; ax, float&ast; ay, float&ast; az, float&ast; gx, float&ast; gy, float&ast; gz)**
*getNavEcef(float&ast; yaw, float&ast; pitch, float&ast; roll, double&ast; PosX, double&ast; PosY, double&ast; PosZ, float&ast; VelX, float&ast; VelY, float&ast; VelZ, float&ast; ax, float&ast; ay, float&ast; az, float&ast; gx, float&ast; gy, float&ast; gz)* samples the VN-200 and gives the INS state with position in Earth Centered Earth Fixed (ECEF) format. Orientation is given as yaw, roll, and pitch in rad, ECEF X, Y, and Z coordinates in m, ECEF velocity in m/s, body frame acceleration in m/s/s, and body frame angular rates in rad/s.

```C++
double PosX, PosY, PosZ;
float yaw, pitch, roll, VelX, VelY, VelZ, ax, ay, az, gx, gy, gz;
IMU.getNavEcef(&yaw, &pitch, &roll, &PosX, &PosY, &PosZ, &VelX, &VelY, &VelZ, &ax, &ay, &az, &gx, &gy, &gz);
```

## Example List

# Wiring

# Performance
