## Overview
The **IMU_GY91-ESP32_BLE** project demonstrates how to capture, process, and transmit real-time motion and environmental sensor data using an ESP32 and the GY-91 sensor module.
The system reads data from the **MPU-9250** (accelerometer, gyroscope, magnetometer) and **BMP280** (barometer), computes orientation and heading, and broadcasts this data via **Bluetooth Low Energy (BLE)**.
It is suitable for applications in **motion tracking, orientation estimation, robotics, environmental monitoring, and IoT**.

---

## Hardware Components
### 1. ESP32 Development Board
- **Dual-core Processor:** Handles multiple tasks such as sensor reading, computation, and BLE transmission simultaneously.
- **Integrated Wi-Fi & BLE:** Wireless connectivity for real-time data visualization.
- **I²C Compatible (3.3V):** Required for sensor communication.
- **Low Power Modes & GPIOs:** Expandable for portable and sensor-rich applications.

### 2. GY-91 Sensor Module
- **MPU-9250 (9-axis IMU):** Provides accelerometer, gyroscope, and magnetometer readings for motion and orientation tracking.
- **BMP280 (Barometer):** Measures temperature, pressure, and computes altitude for environmental and navigation purposes.

### 3. Connections
- **I²C Interface:** SDA → GPIO 21, SCL → GPIO 22
- **Power:** VCC → 3.3V/5V, GND → GND

---

## Software Requirements
- **Arduino IDE** for code development and uploading.
- **ESP32 Board Support Package** for ESP32 compatibility.
- **Libraries:**
  - MPU9250 for IMU data and orientation calculations.
  - Adafruit_BMP280 for pressure, temperature, and altitude.
  - BLE libraries for wireless data transmission.

### System Variables
**Detailed variables for MPU9250 and BMP280 sensors are documented below.** These include accelerometer, gyroscope, magnetometer readings, linear acceleration, quaternions, Euler angles, heading, and barometric data such as temperature, pressure, and altitude.

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
The system captures, processes, and transmits real-time sensor data:
- **IMU & Barometer Data Acquisition:** Reads accelerometer, gyroscope, magnetometer, temperature, pressure, and altitude.
- **Orientation Computation:** Computes quaternions and Euler angles using sensor fusion.
- **Heading Calculation:** Determines heading in degrees from magnetometer and gyroscope.
- **Altitude Estimation:** Converts pressure readings to altitude using the barometric formula.
- **BLE Transmission:** Sends processed data to smartphones or computers for visualization, logging, or further analysis.
**Applications:** Motion tracking, GPS-independent navigation, environmental monitoring, and IoT research.

---

### What is this system capturing?
The IMU_GY91-ESP32_BLE system captures real-time sensor data from the GY-91 module, providing comprehensive information about motion, orientation, and the surrounding environment. The captured data includes:
#### Accelerometer Data (`ax, ay, az`)
Measures the acceleration forces acting on the device along three orthogonal axes (X, Y, Z).
  - Useful for detecting motion, orientation changes, vibrations, and impact forces.
  - Can also be used to compute linear acceleration by removing the effect of gravity (`la_x, la_y, la_z`).
#### Gyroscope Data (`gx, gy, gz`)
Measures angular velocity around the X, Y, and Z axes.
- Allows detection of rotational motion and angular changes.
- Integral to calculating device orientation when combined with accelerometer and magnetometer data.
#### Magnetometer Data (`mx, my, mz`)
Measures the strength and direction of the magnetic field along three axes.
- Essential for determining compass heading and correcting drift in orientation calculations.
- Helps the system determine its heading relative to Earth’s magnetic field.
#### Barometer Data (`ta, p, a`)
Measures atmospheric pressure and temperature.
- Altitude (`a`) is estimated using the barometric formula.
- Useful for environmental monitoring, weather analysis, and elevation tracking.

### Why is this data important?
Capturing this combination of motion and environmental data is critical for a wide range of applications:
#### Motion Tracking
- Monitors movement and orientation in devices such as **drones, robots, wearable devices, and AR/VR systems**.
- Enables precise control, stabilization, and navigation based on real-time movement data.
#### Environmental Monitoring
- Measures atmospheric pressure and temperature for **weather-related studies, environmental sensing, or altitude estimation**.
- Can support research in meteorology or IoT-based smart systems.
#### Navigation Systems
- Provides **orientation, heading, and positional data** independent of GPS.
- Critical for applications like drone autopilot, robotics navigation, and portable devices operating in GPS-denied environments.

### Technical Skills Applied
This project combines several technical skills, demonstrating a practical application of embedded systems and sensor integration:
#### Embedded Systems Programming
- Writing efficient code for the **ESP32 microcontroller** to interface with sensors, process data, and manage BLE communication.
#### Sensor Integration
- Configuring and reading data from the **GY-91 sensor module** over the **I²C protocol**.
- Understanding accelerometer, gyroscope, magnetometer, and barometer data and how to fuse them for accurate orientation and motion tracking.
#### Wireless Communication
- Implementing **BLE (Bluetooth Low Energy)** to transmit sensor data to external devices for visualization, logging, or further processing.
#### Data Processing & Sensor Fusion
- Handling, filtering, and formatting real-time sensor data.
- Computing quaternions, Euler angles, heading, linear acceleration, and altitude to provide actionable insights.
This combination of hardware interfacing, data processing, and wireless communication makes the system capable of **real-time motion analysis, orientation tracking, and environmental monitoring**.

---

## Getting Started
Follow these steps to quickly set up and run the **IMU_GY91-ESP32_BLE** system:

### Hardware Setup
- 1- Connect the **GY-91 module** to the **ESP32** using the I²C interface:**
  - **SDA → GPIO 21, SCL → GPIO 22**
  - **VCC → 3.3V/5V, GND → GND**
- 2- Ensure all connections are secure.**

### Software Setup
- 1. Install the **Arduino IDE** and add **ESP32 board support** via the Boards Manager.**
- 2. Install the required libraries:**
  - MPU9250 (for IMU data)
  - Adafruit_BMP280 (for barometric data)
  - BLE libraries (for wireless communication)

### Uploading the Code
- 1- Open the provided Arduino sketch in Arduino IDE.
- 2- Select your ESP32 board and port under **Tools → Board & Port**.
- 3- Click **Upload** to flash the code to the ESP32.

### Monitoring Data
- Use a BLE-compatible app (e.g., **nRF Connect, LightBlue**) on your smartphone or computer.
- Connect to the ESP32 and visualize real-time sensor data including:
  - Acceleration, angular velocity, and magnetic field
  - Orientation (Euler angles & quaternions)
  - Heading, temperature, pressure, and altitude
 
  ---
  
## Future Enhancements
- **1. Data Logging**
  - Log sensor data to an SD card or cloud service for long-term analysis.
- **2. Advanced Processing**
  - Implement sensor fusion algorithms like Madgwick or Kalman filters to improve orientation accuracy.
- User Interface
  - Develop mobile or web applications for real-time visualization, graphs, and alerts.
- Extended Hardware Integration
  - Add additional sensors such as GPS or environmental sensors to expand system capabilities.

Advanced Processing: Apply algorithms for sensor fusion to improve data accuracy.

User Interface: Develop a mobile or web application to visualize and analyze the sensor data in real-time.
