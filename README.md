# VectorNav
Library for communicating with the VectorNav [VN-100](https://www.vectornav.com/products/vn-100) Inertial Measurement Unit and Attitude Heading Reference System (IMU/AHRS) and [VN-200](https://www.vectornav.com/products/vn-200) Inertial Navigation Systems (INS) using Teensy 3.x and LC devices.

# Description
The VN-100 is a miniature, high performance Inertial Measurement Unit and Attitude Heading Reference System (IMU/AHRS). The VN-100 combines a three-axis accelerometer, three-axis gyroscope, three-axis magnetometer, and a barometric pressure sensor with a 32-bit processor. In addition to calibrated and filtered sensor measurements, the VN-100 computes and outputs real-time orientation solutions. Sensor measurements are available at rates of up to 800 Hz and orientation outputs at rates of up to 400 Hz.

The VN-200 adds a uBlox GPS module to the sensors available on the VN-200. Using GPS aiding, the VN-200 provides accurate estimates of position, velocity, and orientation.

The VN-100 and VN-200 are temperature calibrated at 25C. Optionally, thermal calibration over the temperature range of -40C to +85C is available as an option.

# Usage
This library currently only supports SPI communication with the VN-100 and VN-200 INS. The VN-300 firmware currently does not support SPI communication. Additionally, SPI communication with the VN-100 IMU/AHRS and VN-200 INS are only available on the surface mount device (SMD) packages.

# Installation
Simply clone or download this library into your Arduino/libraries folder. [Teensyduino](http://pjrc.com/teensy/td_download.html) version 1.37 or newer is required.

## Function Descriptions
This library supports both the VN-100 IMU/AHRS and VN-200 INS. The object declaration is different depending on the sensor used. Additionally, while most functions are available on both sensors, some functions are unique given the sensors required (for example, functions using GPS on the VN-200). Functions are listed below for object declaration, setup, and data collection. In each of these categories functions are given for the VN-100, VN-200, and functions common to both.

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

### Data Collection Functions

## Example List

# Wiring

# Performance
