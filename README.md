## Overview
The IMU_GY91-ESP32_BLE project demonstrates how to capture, process, and transmit real-time inertial and environmental sensor data using an ESP32 microcontroller and the GY-91 sensor module.</b>
The system reads data from the MPU-9250 (accelerometer, gyroscope, magnetometer) and BMP280 (barometer) sensors, calculates orientation and heading, and broadcasts this data wirelessly via Bluetooth Low Energy (BLE).</b>
This setup can be used for motion tracking, orientation estimation, environmental sensing, robotics, and IoT applications.</b>

---

## Hardware Components
### 1. ESP32 Development Board
The **ESP32** is a versatile microcontroller used as the brain of this system. Its key features include:
- **Dual-core Processor:** Allows running multiple tasks simultaneously, such as reading sensor data, performing computations, and handling BLE communication without delays.
- **Integrated Wi-Fi and Bluetooth (BLE):** Provides wireless connectivity for sending sensor data to smartphones, tablets, or computers in real-time.
- **I²C Compatibility:** The ESP32 supports 3.3V I²C communication, which is required for interfacing with the GY-91 sensor module.
- **Low Power Modes:** Suitable for battery-operated applications, such as portable motion-tracking devices.
- **GPIO Pins:** Allows additional sensors or actuators to be connected for expanding the system.

### 2. GY-91 Sensor Module
The **GY-91** is a compact sensor module that combines two important sensors for motion and environmental data:
- **a) MPU-9250 (9-axis IMU)**
  - **Accelerometer (3-axis):** Measures linear acceleration in X, Y, Z axes, useful for detecting movement and orientation changes.
  - **Gyroscope (3-axis):** Measures angular velocity along X, Y, Z axes, allowing detection of rotation or angular changes.
  - **Magnetometer (3-axis):** Measures magnetic fields along X, Y, Z axes, which is essential for calculating compass heading.
Together, these sensors form a 9-axis IMU (Inertial Measurement Unit), which provides comprehensive motion and orientation information.
- **b) BMP280 (Barometer)**
  - Measures **atmospheric pressure** and **temperature**.
  - Computes **altitude** based on pressure readings using the barometric formula.
  - Enables environmental monitoring and enhances navigation systems by providing vertical positioning data.

### 3. Connections
To interface the GY-91 with the ESP32:
- **I²C Interface:**
  -  SCL (Serial Clock Line)
  -  SDA (Serial Data Line)
  -  Required for both MPU-9250 and BMP280 communication with the ESP32.
- **Power Pins:**
  - VCC: Provides 3.3V or 5V power to the GY-91 module.
  - GND: Common ground between ESP32 and the sensor module.
- **Optional Pins:** Some modules may have additional interrupt pins for advanced features like motion detection or calibration.

---

## Software Requirements
To program the ESP32 and manage sensor data, the following software components are needed:
- **1. Arduino IDE**
  - A widely used development environment for microcontrollers.
  - Provides a simple interface for writing, compiling, and uploading code to the ESP32.
  - Supports libraries and extensions for BLE, sensor communication, and real-time processing.

- **2. ESP32 Board Support Package**
  - Required to add ESP32 support to Arduino IDE.
  - Includes board definitions, libraries, and tools to compile code specifically for ESP32 microcontrollers.
  - Enables access to ESP32-specific features such as BLE, Wi-Fi, and dual-core task management.

- **3. Libraries**
  - **MPU9250 Library:**
    - Provides functions to read accelerometer, gyroscope, and magnetometer data.
    - Computes quaternions and Euler angles for orientation estimation.
  - **Adafruit_BMP280 Library:**
    - Allows easy communication with BMP280 sensor over I²C.
    - Reads pressure, temperature, and calculates altitude.
  - **BLE Libraries:**
    - Enable wireless transmission of sensor data to BLE-enabled devices.
    - Supports creating BLE services and characteristics for structured data communication.

### System Variables
The System Variables section provides a detailed overview of all the global variables used in this project to represent real-time sensor data, processed values, and orientation information. These variables are essential for capturing motion, orientation, heading, and environmental parameters from the GY-91 sensor module.

The variables are categorized into two main groups:
**MPU9250 Variables**
| Variable | Type | Description | Units / Notes |
|----------|------|-------------|---------------|
| `ax` | float | Accelerometer reading along X-axis | m/s², measures linear acceleration including gravity |
| `ay` | float | Accelerometer reading along Y-axis | m/s² |
| `az` | float | Accelerometer reading along Z-axis | m/s² |
| `gx` | float | Gyroscope reading along X-axis | °/s, measures angular velocity |
| `gy` | float | Gyroscope reading along Y-axis | °/s |
| `gz` | float | Gyroscope reading along Z-axis | °/s |
| `mx` | float | Magnetometer reading along X-axis | µT (microtesla), used for heading computation |
| `my` | float | Magnetometer reading along Y-axis | µT |
| `mz` | float | Magnetometer reading along Z-axis | µT |
| `la_x` | float | Linear acceleration along X-axis (gravity removed) | m/s², useful for motion detection |
| `la_y` | float | Linear acceleration along Y-axis | m/s² |
| `la_z` | float | Linear acceleration along Z-axis | m/s² |
| `h` | float | Heading computed from sensor fusion | Degrees, calculated using magnetometer + gyro |
| `qx` | float | Orientation quaternion X component | Unitless, used for 3D orientation |
| `qy` | float | Orientation quaternion Y component | Unitless |
| `qz` | float | Orientation quaternion Z component | Unitless |
| `qw` | float | Orientation quaternion W component | Unitless |
| `ex` | float | Euler angle error / correction along X | Degrees, used in sensor fusion |
| `ey` | float | Euler angle error / correction along Y | Degrees |
| `ez` | float | Euler angle error / correction along Z | Degrees |
| `yaw` | float | Yaw angle | Degrees, rotation around Z-axis |
| `pitch` | float | Pitch angle | Degrees, rotation around Y-axis |
| `roll` | float | Roll angle | Degrees, rotation around X-axis |
| `t` | float | Timestamp of sensor reading | ms, used for timing & synchronization |
| `hD` | int | Heading in degrees | Degrees, user-friendly compass heading |

**BMP280 Variables**
| Variable | Type | Description | Units / Notes |
|----------|------|-------------|---------------|
| `bmp` | Object | BMP280 sensor object | Used to access temperature, pressure, and altitude |
| `ta` | float | Temperature from BMP280 | °C |
| `p` | float | Atmospheric pressure | Pa or hPa |
| `a` | float | Altitude calculated from pressure | Meters above sea level, using barometric formula |

---

## System Functionality
The IMU_GY91-ESP32_BLE system is designed to **capture, process, and transmit real-time motion and environmental data**. Its functionality can be broken down into several key operations:

### 1. Real-Time IMU and Barometer Data Acquisition
- The system continuously reads data from the MPU-9250 sensor, including:
  - **Accelerometer** (X, Y, Z axes) – measures linear acceleration including gravity.
  - **Gyroscope** (X, Y, Z axes) – measures angular velocity.
  - **Magnetometer** (X, Y, Z axes) – measures magnetic field strength for heading calculations.
- Simultaneously, the BMP280 sensor provides:
- **Temperature** (°C)
- **Atmospheric** pressure (Pa or hPa)
- **Altitude** (meters above sea level, calculated from pressure)
### 2. Orientation Computation (Quaternions & Euler Angles)
- Raw IMU data is processed using **sensor fusion algorithms** to compute the device’s orientation.
- **Quaternions** (`qx, qy, qz, qw`) provide a robust representation of 3D rotation without gimbal lock.
- **Euler angles** (`yaw, pitch, roll`) are derived from quaternions, giving intuitive rotational angles around X, Y, and Z axes.
### 3. Heading Calculation
- The system calculates the **heading** (`hD`) in degrees using fused data from the magnetometer and gyroscope.
- Heading provides the orientation relative to the Earth’s magnetic field, essential for navigation and motion tracking applications.
### 4. Altitude Estimation
- The BMP280 barometer measures atmospheric pressure, which is converted to altitude (`a`) using the barometric formula.
- This allows the system to monitor vertical positioning in applications like drones or wearable devices.
### 5. Wireless Data Transmission via BLE
- Processed sensor data is encoded into BLE packets and transmitted to BLE-enabled devices (smartphones, tablets, or computers).
- This enables real-time visualization, logging, or further processing of motion and environmental data in external applications.
### 6. Applications
- Motion tracking & orientation monitoring for drones, robots, or wearables.
- Navigation and heading detection in GPS-independent systems.
- Environmental monitoring, including temperature, pressure, and altitude changes.
- Data logging and analysis for research, engineering, or IoT projects.

---

### What is this system capturing?
The IMU_GY91-ESP32_BLE system captures real-time sensor data from the GY-91 module, providing comprehensive information about motion, orientation, and the surrounding environment. The captured data includes:
#### Accelerometer Data (ax, ay, az)
Measures the acceleration forces acting on the device along three orthogonal axes (X, Y, Z).
  - Useful for detecting motion, orientation changes, vibrations, and impact forces.
  - Can also be used to compute linear acceleration by removing the effect of gravity (la_x, la_y, la_z).
#### Gyroscope Data (gx, gy, gz)
Measures angular velocity around the X, Y, and Z axes.
- Allows detection of rotational motion and angular changes.
- Integral to calculating device orientation when combined with accelerometer and magnetometer data.
#### Magnetometer Data (mx, my, mz)
Measures the strength and direction of the magnetic field along three axes.
- Essential for determining compass heading and correcting drift in orientation calculations.
- Helps the system determine its heading relative to Earth’s magnetic field.
#### Barometer Data (ta, p, a)
Measures atmospheric pressure and temperature.
- Altitude (a) is estimated using the barometric formula.
- Useful for environmental monitoring, weather analysis, and elevation tracking.

### Why is this data important?
This data is crucial for various applications:
- **Motion Tracking: Monitoring movement and orientation in devices like drones, wearables, and robotics.
- **Environmental Monitoring: Assessing atmospheric pressure and temperature for weather-related studies.
- **Navigation Systems: Providing orientation and positioning data for GPS-independent navigation.
What technical skills are applied?

### The project involves:
Embedded Systems Programming: Writing code for the ESP32 to interface with the GY-91 sensor and manage BLE communication.
Sensor Integration: Configuring and reading data from the GY-91 sensor module using the I²C protocol.
Wireless Communication: Implementing BLE protocols to transmit sensor data to external devices.
Data Processing: Handling and formatting sensor data for transmission and analysis.

Getting Started

Hardware Requirements:

ESP32 development board.

GY-91 sensor module.

Jumper wires for connections.

Software Setup:

Install the Arduino IDE.

Add ESP32 board support via the Arduino Board Manager.

Install necessary libraries for I²C communication and BLE.

Connections:

Connect the GY-91 module to the ESP32 using the I²C interface (SCL, SDA, GND, VCC).

Upload Code:

Load the provided Arduino sketch to the ESP32.

Monitor Data:

Use a BLE-compatible app on your smartphone or computer to connect to the ESP32 and view the transmitted sensor data.

Future Enhancements

Data Logging: Implement functionality to log sensor data to an SD card or cloud service.

Advanced Processing: Apply algorithms for sensor fusion to improve data accuracy.

User Interface: Develop a mobile or web application to visualize and analyze the sensor data in real-time.
