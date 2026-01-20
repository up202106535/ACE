#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

// I2C0 -> Wire : GP20 SDA, GP21 SCL (IMU + ToF)
static constexpr uint8_t I2C0_SDA = 20;
static constexpr uint8_t I2C0_SCL = 21;

// I2C1 -> Wire1: GP2 SDA, GP3 SCL (PCA9685)
static constexpr uint8_t I2C1_SDA = 2;
static constexpr uint8_t I2C1_SCL = 3;

static constexpr uint8_t IMU_ADDR = 0x68;
static constexpr uint8_t PCA_ADDR = 0x40;

// VL53L0X
VL53L0X tof;
bool tofOk = false;

// IMU state
bool imuOk = false;
float roll = 0.0f, pitch = 0.0f;
float roll0 = 0.0f, pitch0 = 0.0f;
bool referenceSet = false;
uint32_t lastMs = 0;

static bool ack(TwoWire& bus, uint8_t addr) {
  bus.beginTransmission(addr);
  return (bus.endTransmission() == 0);
}

static bool writeReg8(TwoWire& bus, uint8_t addr, uint8_t reg, uint8_t val) {
  bus.beginTransmission(addr);
  bus.write(reg);
  bus.write(val);
  return (bus.endTransmission() == 0);
}

static bool readBytes(TwoWire& bus, uint8_t addr, uint8_t startReg, uint8_t* buf, size_t n) {
  bus.beginTransmission(addr);
  bus.write(startReg);
  if (bus.endTransmission(false) != 0) return false; // repeated start
  int got = bus.requestFrom((int)addr, (int)n);
  if (got != (int)n) return false;
  for (size_t i = 0; i < n; i++) buf[i] = (uint8_t)bus.read();
  return true;
}

static bool readReg8(TwoWire& bus, uint8_t addr, uint8_t reg, uint8_t& out) {
  return readBytes(bus, addr, reg, &out, 1);
}

// Basic init compatible with MPU6050/MPU6500 family for accel/gyro
static bool imuInitBasic() {
  if (!ack(Wire, IMU_ADDR)) return false;

  uint8_t who = 0xFF;
  if (!readReg8(Wire, IMU_ADDR, 0x75, who)) return false;

  Serial.print("[IMU] WHO_AM_I = 0x");
  Serial.println(who, HEX);

  // Wake up: PWR_MGMT_1 (0x6B) = 0x00
  if (!writeReg8(Wire, IMU_ADDR, 0x6B, 0x00)) return false;
  delay(50);

  // Optional: set clock source PLL (PWR_MGMT_1 = 0x01)
  writeReg8(Wire, IMU_ADDR, 0x6B, 0x01);

  // Gyro config: GYRO_CONFIG (0x1B) FS_SEL=1 => ±500 dps
  writeReg8(Wire, IMU_ADDR, 0x1B, 0x08);

  // Accel config: ACCEL_CONFIG (0x1C) AFS_SEL=1 => ±4g
  writeReg8(Wire, IMU_ADDR, 0x1C, 0x08);

  // DLPF config: CONFIG (0x1A) ~ 21Hz (value 4) is typical
  writeReg8(Wire, IMU_ADDR, 0x1A, 0x04);

  return true;
}

static bool imuReadRaw(int16_t& ax, int16_t& ay, int16_t& az, int16_t& gx, int16_t& gy, int16_t& gz) {
  uint8_t buf[14];
  // ACCEL_XOUT_H starts at 0x3B, reads accel(6) + temp(2) + gyro(6)
  if (!readBytes(Wire, IMU_ADDR, 0x3B, buf, sizeof(buf))) return false;

  ax = (int16_t)((buf[0] << 8) | buf[1]);
  ay = (int16_t)((buf[2] << 8) | buf[3]);
  az = (int16_t)((buf[4] << 8) | buf[5]);
  // temp buf[6..7] ignored
  gx = (int16_t)((buf[8] << 8) | buf[9]);
  gy = (int16_t)((buf[10] << 8) | buf[11]);
  gz = (int16_t)((buf[12] << 8) | buf[13]);

  return true;
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0 < 3000)) delay(10);

  Serial.println("\n=== Sensor Test (Register-level IMU) ===");
  Serial.println("Press 'Z' to set reference (standing pose).");

  // I2C0
  Wire.setSDA(I2C0_SDA);
  Wire.setSCL(I2C0_SCL);
  Wire.begin();
  Wire.setClock(50000);

  // I2C1
  Wire1.setSDA(I2C1_SDA);
  Wire1.setSCL(I2C1_SCL);
  Wire1.begin();
  Wire1.setClock(100000);

  // IMU
  Serial.print("[IMU] ACK 0x68: ");
  Serial.println(ack(Wire, IMU_ADDR) ? "YES" : "NO");

  imuOk = imuInitBasic();
  Serial.print("[IMU] init: ");
  Serial.println(imuOk ? "OK" : "FAIL");

  // ToF
  tof.setBus(&Wire); // if compile fails, remove
  tof.setTimeout(200);
  tofOk = tof.init();
  Serial.print("[ToF] init: ");
  Serial.println(tofOk ? "OK" : "FAIL");
  if (tofOk) tof.startContinuous();

  Serial.print("[PCA] ACK 0x40 on Wire1: ");
  Serial.println(ack(Wire1, PCA_ADDR) ? "YES" : "NO");

  Serial.println("\nFormat:");
  Serial.println("roll pitch | rollRel pitchRel | rawAx rawAy rawAz | rawGx rawGy rawGz | ToFmm | PCA_ACK | ref");
  Serial.println("------------------------------------------------------------------------------------------");

  lastMs = millis();
}

void loop() {
  // Manual reference set
  if (Serial.available()) {
    char c = (char)Serial.read();
    if (c == 'z' || c == 'Z') {
      roll0 = roll;
      pitch0 = pitch;
      referenceSet = true;
      Serial.println("[IMU] Reference zeroed.");
    }
  }

  // dt
  uint32_t now = millis();
  float dt = (now - lastMs) * 1e-3f;
  if (dt <= 0.0f || dt > 0.5f) dt = 0.01f;
  lastMs = now;

  // Read IMU raw
  int16_t rax=0, ray=0, raz=0, rgx=0, rgy=0, rgz=0;
  bool imuReadOk = false;
  if (imuOk) {
    imuReadOk = imuReadRaw(rax, ray, raz, rgx, rgy, rgz);
  }

  // Compute roll/pitch from accel raw (no scaling needed for angles)
  if (imuReadOk) {
    float ax = (float)rax;
    float ay = (float)ray;
    float az = (float)raz;

    float rollAcc  = atan2f(ay, az) * 180.0f / PI;
    float pitchAcc = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / PI;

    // gyro raw to deg/s needs scaling; for ±500 dps sensitivity is 65.5 LSB/(deg/s)
    float gx = (float)rgx / 65.5f;
    float gy = (float)rgy / 65.5f;
    // gz ignored for now

    roll  = 0.98f * (roll  + gx * dt) + 0.02f * rollAcc;
    pitch = 0.98f * (pitch + gy * dt) + 0.02f * pitchAcc;
  }

  // ToF
  uint16_t mm = 0;
  bool tofValid = false;
  if (tofOk) {
    mm = tof.readRangeContinuousMillimeters();
    tofValid = !tof.timeoutOccurred();
  }

  // PCA ACK
  bool pcaAck = ack(Wire1, PCA_ADDR);

  // Print at ~10Hz
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint < 100) return;
  lastPrint = millis();

  float rollRel  = roll  - roll0;
  float pitchRel = pitch - pitch0;

  Serial.print(roll, 2); Serial.print(" ");
  Serial.print(pitch, 2); Serial.print(" | ");
  Serial.print(rollRel, 2); Serial.print(" ");
  Serial.print(pitchRel, 2); Serial.print(" | ");

  if (imuReadOk) {
    Serial.print(rax); Serial.print(" ");
    Serial.print(ray); Serial.print(" ");
    Serial.print(raz); Serial.print(" | ");
    Serial.print(rgx); Serial.print(" ");
    Serial.print(rgy); Serial.print(" ");
    Serial.print(rgz);
  } else {
    Serial.print("NA NA NA | NA NA NA");
  }

  Serial.print(" | ");
  if (tofOk && tofValid) Serial.print(mm);
  else if (tofOk) Serial.print("timeout");
  else Serial.print("NA");

  Serial.print(" | ");
  Serial.print(pcaAck ? "YES" : "NO");

  Serial.print(" | ");
  Serial.println(referenceSet ? "REF" : "NOREF");
}
