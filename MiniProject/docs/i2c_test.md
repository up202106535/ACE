# I2C test

## Test: Dual I²C Presence (test_i2c_presence.cpp)

### Purpose
Confirms that each I²C device is physically present on the correct bus and pins.

This test is the first step after wiring changes. It verifies:
- I²C0 (Wire) on GP20/21 sees IMU + ToF
- I²C1 (Wire1) on GP2/3 sees PCA9685

### Hardware assumptions
- MCU: RP2040 (Earle Philhower Arduino core)
- I²C0 (Wire): SDA=GP20, SCL=GP21
  - VL53L0X ToF expected at 0x29
  - IMU expected at 0x68 or 0x69 (yours is at 0x68)
- I²C1 (Wire1): SDA=GP2, SCL=GP3
  - PCA9685 expected at 0x40
  - You may also see 0x70 (often PCA all-call behaviour)

Power:
- All logic at 3.3 V
- All grounds common
- PCA V+ (servo supply) not required for scanning (only VCC logic is required)

### How to run
1. In `platformio.ini`, select only this file:
   ```ini
   build_src_filter =
     -<*>
     +<test_i2c_presence.cpp>
