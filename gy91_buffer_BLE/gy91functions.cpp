#include "gy91functions.h"

void mpu9250() { 
    if (mpu.update()) {
        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + 25) {
            accelerometer(); gyrometer(); magnetometer(); linear_acc(); // currently only calling this
            quaternion(); euler(); roll_pitch_yaw(); mag_heading(); temperature(); read_bmp280();
            prev_ms = millis();
        }
    }
}

void printDataHex() {
  float data[] = {ax,ay,az,gx,gy,gz,mx,my,mz,la_x,la_y,la_z,qw,qx,qy,qz,ex,ey,ez,yaw,pitch,roll,t,hD,ta,p,a};
  const int dataCount=sizeof(data)/sizeof(data[0]); uint8_t buffer[dataCount*sizeof(float)];
  memcpy(buffer, data, sizeof(buffer)); // Copy float data into byte array
  for (int i = 0; i < sizeof(buffer); i++) {if (buffer[i]<0x10)Serial.print("0");Serial.print(buffer[i],HEX);Serial.print(" ");}Serial.println();}   // Print in hex format

void sendDataBLE(uint8_t* buffer, size_t& length) {
  float data[] = {ax,ay,az,gx,gy,gz,mx,my,mz,la_x,la_y,la_z,qw,qx,qy,qz,ex,ey,ez,yaw,pitch,roll,t,hD,ta,p,a};
  const int dataCount = sizeof(data) / sizeof(data[0]);
  length = dataCount * sizeof(float);
  memcpy(buffer, data, length);
}

void accelerometer() {
    ax = mpu.getAccX();
    ay = mpu.getAccY();
    az = mpu.getAccZ();
//  Serial.println("AccX: " + String(ax) + " AccY: " + String(ay) + " AccZ: " + String(az));
}

void gyrometer() {
    gx = mpu.getGyroX();
    gy = mpu.getGyroY();
    gz = mpu.getGyroZ();
//  Serial.println("GyroX: " + String(gx) + " GyroY: " + String(gy) + " GyroZ: " + String(gz));
}

void magnetometer() {
    mpu.update_mag();
    mx = mpu.getMagX();
    my = mpu.getMagY();
    mz = mpu.getMagZ();
}

void linear_acc() {
    la_x = mpu.getLinearAccX();
    la_y = mpu.getLinearAccY();
    la_z = mpu.getLinearAccZ();
//  Serial.println("LinAccX: " + String(la_x) + " LinAccY: " + String(la_y) + " LinAccZ: " + String(la_z));
}

void quaternion() {
    qx = mpu.getQuaternionX();
    qy = mpu.getQuaternionY();
    qz = mpu.getQuaternionZ();
    qw = mpu.getQuaternionW();
//   Serial.println("QuatX: " + String(qx) + " QuatY: " + String(qy) + " QuatZ: " + String(qz) + " QuatW: " + String(qw));
}

void euler() {
    ex = mpu.getEulerX();
    ey = mpu.getEulerY();
    ez = mpu.getEulerZ();
//  Serial.println("EulerX: " + String(ex) + " EulerY: " + String(ey) + " EulerZ: " + String(ez));
}

void roll_pitch_yaw() {
    yaw = mpu.getYaw() + 46;
    pitch = mpu.getPitch();
    roll = mpu.getRoll();
//  Serial.println("Yaw: " + String(yaw) + " Pitch: " + String(pitch) + " Roll: " + String(roll));
}

void temperature(){
    t = mpu.getTemperature();
//  Serial.println("Temperature: " + String(t));
}

void mag_heading(){
    hD = atan2(mpu.getMagY(), mpu.getMagX()) / 0.0174532925;
    if (hD <= 0) {
        hD += 360;
    }
//  Serial.println("Heading: " + String(hD));
}

// ----- BMP280 setup -----
void bmp280_setup() {
    if (!bmp.begin(0x76)) {  // Change address to 0x77 if needed
        Serial.println("BMP280 not found!");
        while (1);
    }
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,    /* Operating Mode */
                    Adafruit_BMP280::SAMPLING_X2,  /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16, /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,   /* Filtering */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time */
}

// ----- BMP280 read -----
void read_bmp280() {
    ta = bmp.readTemperature();  // °C
    p = bmp.readPressure() / 100.0F;   // hPa
    a = bmp.readAltitude(1013.25);     // m, using standard sea-level pressure
}

void print_calibration() {
    Serial.println("< calibration parameters >");
    Serial.println("accel bias [g]: ");
    Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.println();
    Serial.println("gyro bias [deg/s]: ");
    Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.println();
    Serial.println("mag bias [mG]: ");
    Serial.print(mpu.getMagBiasX());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasY());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasZ());
    Serial.println();
    Serial.println("mag scale []: ");
    Serial.print(mpu.getMagScaleX());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleY());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleZ());
    Serial.println();
}
