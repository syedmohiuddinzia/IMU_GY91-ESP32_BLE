## Overview
The IMU_GY91-ESP32_BLE project demonstrates how to capture, process, and transmit real-time inertial and environmental sensor data using an ESP32 microcontroller and the GY-91 sensor module.</b>
The system reads data from the MPU-9250 (accelerometer, gyroscope, magnetometer) and BMP280 (barometer) sensors, calculates orientation and heading, and broadcasts this data wirelessly via Bluetooth Low Energy (BLE).</b>
This setup can be used for motion tracking, orientation estimation, environmental sensing, robotics, and IoT applications.</b>

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
**a) MPU-9250 (9-axis IMU)**
- **Accelerometer (3-axis):** Measures linear acceleration in X, Y, Z axes, useful for detecting movement and orientation changes.
- **Gyroscope (3-axis):** Measures angular velocity along X, Y, Z axes, allowing detection of rotation or angular changes.
- **Magnetometer (3-axis):** Measures magnetic fields along X, Y, Z axes, which is essential for calculating compass heading.
Together, these sensors form a 9-axis IMU (Inertial Measurement Unit), which provides comprehensive motion and orientation information.

**b) BMP280 (Barometer)**
- Measures **atmospheric pressure** and **temperature**.
- Computes **altitude** based on pressure readings using the barometric formula.
- Enables environmental monitoring and enhances navigation systems by providing vertical positioning data.

3. Connections
To interface the GY-91 with the ESP32:
I²C Interface:
SCL (Serial Clock Line)
SDA (Serial Data Line)
Required for both MPU-9250 and BMP280 communication with the ESP32.

Power Pins:
VCC: Provides 3.3V or 5V power to the GY-91 module.
GND: Common ground between ESP32 and the sensor module.
Optional Pins: Some modules may have additional interrupt pins for advanced features like motion detection or calibration.

Software Requirements
To program the ESP32 and manage sensor data, the following software components are needed:

1. Arduino IDE
A widely used development environment for microcontrollers.
Provides a simple interface for writing, compiling, and uploading code to the ESP32.
Supports libraries and extensions for BLE, sensor communication, and real-time processing.

2. ESP32 Board Support Package
Required to add ESP32 support to Arduino IDE.
Includes board definitions, libraries, and tools to compile code specifically for ESP32 microcontrollers.
Enables access to ESP32-specific features such as BLE, Wi-Fi, and dual-core task management.

3. Libraries
MPU9250 Library:
Provides functions to read accelerometer, gyroscope, and magnetometer data.
Computes quaternions and Euler angles for orientation estimation.

Adafruit_BMP280 Library:
Allows easy communication with BMP280 sensor over I²C.
Reads pressure, temperature, and calculates altitude.

BLE Libraries:
Enable wireless transmission of sensor data to BLE-enabled devices.
Supports creating BLE services and characteristics for structured data communication.

## System Varaiables

### MPu9250 Variables 
### BMP280 Variables

## System Functionality
- Captures real-time IMU and barometer data.
- Computes orientation in Euler angles and quaternions.
- Calculates heading in degrees.
- Estimates altitude from pressure.
- Sends data over BLE for visualization, logging, or further processing.

---

### What is this system capturing?
The system captures real-time sensor data from the GY-91 module, including:
- **Accelerometer Data:** Measures acceleration forces in three axes.
- **Gyroscope Data:** Measures angular velocity in three axes.
- **Magnetometer Data:** Measures magnetic field strength in three axes.
- **Barometer Data:** Measures atmospheric pressure and temperature.

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
