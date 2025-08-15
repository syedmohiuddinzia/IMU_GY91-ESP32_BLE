#ifndef GY91FUNCTIONS_H
#define GY91FUNCTIONS_H

#include <Arduino.h>
#include "MPU9250.h"
#include <Adafruit_BMP280.h>  // Include BMP280 library

// MPU9250 externs
extern MPU9250 mpu;
extern float ax, ay, az, gx, gy, gz, mx, my, mz, la_x, la_y, la_z;
extern float h, qx, qy, qz, qw, ex, ey, ez, yaw, pitch, roll, t;
extern float declinationAngle, heading;
extern int hD;

// BMP280 externs
extern Adafruit_BMP280 bmp;
extern float ta, p, a;


void sendDataBLE(uint8_t* buffer, size_t& length);
void printDataHex();

// MPU functions
void mpu9250();
void accelerometer();
void gyrometer();
void magnetometer();
void linear_acc();
void quaternion();
void euler();
void roll_pitch_yaw();
void temperature();
void mag_heading();
void print_calibration();

// BMP280 functions
void bmp280_setup();
void read_bmp280();

#endif
