#include "MPU9250.h"
#include "eeprom_utils.h"
#include "gy91functions.h"  // our new header
#include "ble.h"

// MPU9250 instance
MPU9250 mpu;

// BMP280 instance
Adafruit_BMP280 bmp;

// Data variables
float ax, ay, az, gx, gy, gz, mx, my, mz, la_x, la_y, la_z;
float h, qx, qy, qz, qw, ex, ey, ez, yaw, pitch, roll, t;
float declinationAngle = 0.02234021, heading;
int hD;

// BMP280 data
float ta, p, a;

void setup() {
    Serial.begin(115200);
    initBLE();
    Wire.begin();
    delay(2000);

    if (!mpu.setup(0x68)) {
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }
    bmp280_setup(); // Initialize BMP280
    printCalibration();
    loadCalibration();
}

void loop() {
    handleBLE();
    mpu9250();
    printDataHex();
    Serial.print(String(ax)+","+String(ay)+","+String(az)+","+String(gx)+","+String(gy)+","+String(gz)+",");
    Serial.print(String(mx)+","+String(my)+","+String(mz)+","+String(la_x)+","+String(la_y)+","+String(la_z))+",";
    Serial.print(String(qw)+","+String(qx)+","+String(qy)+","+String(qz)+","+String(ex)+","+String(ey)+","+String(ez)+",");
    Serial.println(String(yaw)+","+String(pitch)+","+String(roll)+","+String(t)+","+String(hD)+",");
    Serial.println(String(ta)+","+String(p)+","+String(a));
}
