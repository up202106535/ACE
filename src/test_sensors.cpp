#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <VL53L0X.h>

// I2C0 -> Wire : GP20 SDA, GP21 SCL (IMU + ToF)
static constexpr uint8_t I2C0_SDA = 20;
static constexpr uint8_t I2C0_SCL = 21;

// I2C1 -> Wire1: GP2 SDA, GP3 SCL (PCA9685)
static constexpr uint8_t I2C1_SDA = 2;
static constexpr uint8_t I2C1_SCL = 3;

static constexpr uint8_t IMU_ADDR1 = 0x68;
static constexpr uint8_t IMU_ADDR2 = 0x69;
static constexpr uint8_t PCA_ADDR  = 0x40;

Adafruit_MPU6050 mpu;
VL53L0X tof;

bool imuOk = false;
bool tofOk = false;
uint8_t imuAddrUsed = 0;

static bool ack(TwoWire& bus, uint8_t addr) {
  bus.beginTransmission(addr);
  return (bus.endTransmission() == 0);
}

static bool readReg8(TwoWire& bus, uint8_t addr, uint8_t reg, uint8_t& out) {
  bus.beginTransmission(addr);
  bus.write(reg);
  if (bus.endTransmission(false) != 0) return false; // repeated start
  if (bus.requestFrom((int)addr, 1) != 1) return false;
  out = bus.read();
  return true;
}

static void printIMUStatus() {
  bool ack68 = ack(Wire, IMU_ADDR1);
  bool ack69 = ack(Wire, IMU_ADDR2);

  Serial.println("\n[IMU STATUS]");
  Serial.print("ACK 0x68: "); Serial.println(ack68 ? "YES" : "NO");
  Serial.print("ACK 0x69: "); Serial.println(ack69 ? "YES" : "NO");
  Serial.print("imuOk: ");    Serial.println(imuOk ? "TRUE" : "FALSE");
  Serial.print("imuAddrUsed: 0x");
  if (imuAddrUsed < 16) Serial.print("0");
  Serial.println(imuAddrUsed, HEX);

  if (ack68) {
    uint8_t who = 0xFF;
    bool ok = readReg8(Wire, 0x68, 0x75, who);
    Serial.print("WHO_AM_I @0x68: ");
    if (ok) { Serial.print("0x"); Serial.println(who, HEX); }
    else Serial.println("READ FAIL");
  }
  if (ack69) {
    uint8_t who = 0xFF;
    bool ok = readReg8(Wire, 0x69, 0x75, who);
    Serial.print("WHO_AM_I @0x69: ");
    if (ok) { Serial.print("0x"); Serial.println(who, HEX); }
    else Serial.println("READ FAIL");
  }
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0 < 3000)) delay(10);

  Serial.println("\n=== IMU Diagnostic + ToF + PCA ACK ===");

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

  // IMU init retries
  imuOk = false;
  imuAddrUsed = 0;

  for (int i = 0; i < 6 && !imuOk; i++) {
    if (mpu.begin(0x68, &Wire)) { imuOk = true; imuAddrUsed = 0x68; break; }
    if (mpu.begin(0x69, &Wire)) { imuOk = true; imuAddrUsed = 0x69; break; }
    delay(120);
  }

  if (imuOk) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }

  // ToF
  tof.setBus(&Wire); // comment out if your lib doesn't support it
  tof.setTimeout(200);
  tofOk = tof.init();
  if (tofOk) tof.startContinuous();

  // Print initial status
  printIMUStatus();

  Serial.println("\nFormat:");
  Serial.println("ax ay az | gx gy gz | ToFmm | PCA_ACK");
  Serial.println("-------------------------------------");
}

void loop() {
  // Re-print IMU status every 2 seconds
  static uint32_t lastStatus = 0;
  if (millis() - lastStatus >= 2000) {
    lastStatus = millis();
    printIMUStatus();
  }

  // Print data at ~10 Hz
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint < 100) return;
  lastPrint = millis();

  float ax = 0, ay = 0, az = 0;
  float gx = 0, gy = 0, gz = 0;

  if (imuOk) {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);
    ax = a.acceleration.x;
    ay = a.acceleration.y;
    az = a.acceleration.z;

    gx = g.gyro.x * 180.0f / PI;
    gy = g.gyro.y * 180.0f / PI;
    gz = g.gyro.z * 180.0f / PI;
  }

  Serial.print(ax, 3); Serial.print(" ");
  Serial.print(ay, 3); Serial.print(" ");
  Serial.print(az, 3); Serial.print(" | ");
  Serial.print(gx, 2); Serial.print(" ");
  Serial.print(gy, 2); Serial.print(" ");
  Serial.print(gz, 2); Serial.print(" | ");

  if (tofOk) {
    uint16_t mm = tof.readRangeContinuousMillimeters();
    if (tof.timeoutOccurred()) Serial.print("timeout");
    else Serial.print(mm);
  } else {
    Serial.print("NA");
  }

  Serial.print(" | ");
  Serial.println(ack(Wire1, PCA_ADDR) ? "YES" : "NO");
}
