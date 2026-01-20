# Test Sensor

## Test: Sensors Runtime + Reference Zero (test_sensors.cpp)

### Purpose
Validates runtime sensor data and the “stand-up then zero IMU” workflow.

This test confirms:
- IMU produces live accel/gyro data (non-zero, changing)
- roll/pitch estimate updates smoothly
- pressing 'Z' zeros the reference and makes rollRel/pitchRel near 0
- ToF distance updates continuously
- PCA9685 remains reachable on Wire1 (ACK=YES)

Important note:
Your device at 0x68 returns WHO_AM_I = 0x70, so the Adafruit_MPU6050 library refuses it.
This test uses a register-level IMU driver compatible with MPU6050/MPU6500-class devices.

### Hardware assumptions
- I²C0 (Wire): SDA=GP20, SCL=GP21
  - IMU at 0x68
  - VL53L0X at 0x29
- I²C1 (Wire1): SDA=GP2, SCL=GP3
  - PCA9685 at 0x40
- Logic power 3.3 V, common grounds

### How to run
1. In `platformio.ini`, select only this file:
   ```ini
   build_src_filter =
     -<*>
     +<test_sensors.cpp>
