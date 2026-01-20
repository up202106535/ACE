#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <VL53L0X.h>
#include <Adafruit_PWMServoDriver.h>



static constexpr uint8_t I2C0_SDA = 20;
static constexpr uint8_t I2C0_SCL = 21;

static constexpr uint8_t I2C1_SDA = 2;
static constexpr uint8_t I2C1_SCL = 3;

static bool ack(TwoWire& bus, uint8_t addr) {
  bus.beginTransmission(addr);
  return (bus.endTransmission() == 0);
}

static void scan(TwoWire& bus, const char* name) {
  Serial.print("\n=== Scan "); Serial.print(name); Serial.println(" ===");
  int n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    bus.beginTransmission(a);
    if (bus.endTransmission() == 0) {
      Serial.print("  0x");
      if (a < 16) Serial.print("0");
      Serial.println(a, HEX);
      n++;
    }
    delay(2);
  }
  if (n == 0) Serial.println("  (none)");
}

void setup() {
  Serial.begin(115200);
  delay(800);

  // I2C0: Wire (GP20/21)
  Wire.setSDA(I2C0_SDA);
  Wire.setSCL(I2C0_SCL);
  Wire.begin();
  Wire.setClock(100000);

  // I2C1: Wire1 (GP2/3)
  Wire1.setSDA(I2C1_SDA);
  Wire1.setSCL(I2C1_SCL);
  Wire1.begin();
  Wire1.setClock(100000);

  Serial.println("\n=== Dual I2C presence check ===");
  scan(Wire,  "Wire  (I2C0 GP20/21)");
  scan(Wire1, "Wire1 (I2C1 GP2/3)");

  Serial.println("\nACK summary:");
  Serial.print("  MPU 0x68 on Wire  : "); Serial.println(ack(Wire, 0x68) ? "YES" : "NO");
  Serial.print("  MPU 0x69 on Wire  : "); Serial.println(ack(Wire, 0x69) ? "YES" : "NO");
  Serial.print("  ToF 0x29 on Wire  : "); Serial.println(ack(Wire, 0x29) ? "YES" : "NO");
  Serial.print("  PCA 0x40 on Wire1 : "); Serial.println(ack(Wire1, 0x40) ? "YES" : "NO");
}

void loop() {
  delay(2000);
}

// ============================
// Addresses (expected defaults)
// ============================
static constexpr uint8_t IMU_ADDR1 = 0x68;
static constexpr uint8_t IMU_ADDR2 = 0x69;
static constexpr uint8_t TOF_ADDR  = 0x29;
static constexpr uint8_t PCA_ADDR  = 0x40;

// ============================
// I2C pin mapping (your wiring)
// ============================
// I2C0 -> Wire : GP20 SDA, GP21 SCL
static constexpr uint8_t I2C0_SDA = 20;
static constexpr uint8_t I2C0_SCL = 21;

// I2C1 -> Wire1: GP2 SDA, GP3 SCL
static constexpr uint8_t I2C1_SDA = 2;
static constexpr uint8_t I2C1_SCL = 3;

// ============================
// Devices
// ============================
// Bus: Wire (I2C0)
Adafruit_MPU6050 mpu;
VL53L0X tof;

// Bus: Wire1 (I2C1)
Adafruit_PWMServoDriver pca(PCA_ADDR, Wire1);

// ============================
// State for IMU Euler estimate
// ============================
float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
uint32_t lastMs = 0;

// ============================
// Helpers
// ============================
static void scanBus(TwoWire &bus, const char *name) {
  Serial.print("\n=== I2C Scan: ");
  Serial.print(name);
  Serial.println(" ===");

  uint8_t found = 0;

  for (uint8_t addr = 1; addr < 127; addr++) {
    bus.beginTransmission(addr);
    uint8_t err = bus.endTransmission();

    if (err == 0) {
      Serial.print("  Device at 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      found++;
    }
    delay(2);
  }

  if (found == 0) {
    Serial.println("  No devices found.");
  } else {
    Serial.print("  Total devices: ");
    Serial.println(found);
  }
}

static bool ackAddress(TwoWire &bus, uint8_t addr) {
  bus.beginTransmission(addr);
  return (bus.endTransmission() == 0);
}

// Minimal PCA9685 “register write proof” without servo power:
// write MODE1 = 0x00 (normal), then read back via library begin() path.
// Additionally, we do a raw ACK check before library calls.
static void testPCA9685Presence() {
  Serial.println("\n=== PCA9685 Test (Wire1 / I2C1) ===");

  if (!ackAddress(Wire1, PCA_ADDR)) {
    Serial.println("  ERROR: PCA9685 not ACKing at 0x40 on Wire1.");
    Serial.println("  Check: SDA=GP2, SCL=GP3, VCC=3.3V, GND common, address pins.");
    return;
  }
  Serial.println("  OK: PCA9685 ACKs at 0x40 (Wire1).");

  // Now let the Adafruit library talk on Wire1.
  if (!pca.begin()) {
    Serial.println("  ERROR: pca.begin() failed (despite ACK).");
    Serial.println("  Usually wiring marginal or wrong chip.");
    return;
  }
  Serial.println("  OK: pca.begin() succeeded on Wire1.");

  // Set frequency: even without V+ this exercises register writes.
  pca.setPWMFreq(50);
  Serial.println("  OK: setPWMFreq(50) done (register write).");

  // Write a single channel OFF (again, register write only).
  pca.setPWM(0, 0, 0);
  Serial.println("  OK: setPWM(ch0=OFF) done (register write).");
}

static bool initMPU6050() {
  Serial.println("\n=== MPU6050 Init (Wire / I2C0) ===");

  bool ok = mpu.begin(IMU_ADDR1, &Wire);
  if (!ok) ok = mpu.begin(IMU_ADDR2, &Wire);

  if (!ok) {
    Serial.println("  ERROR: MPU6050 begin() failed at 0x68 and 0x69 on Wire.");
    return false;
  }

  Serial.println("  OK: MPU6050 begin() succeeded on Wire.");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Quick first read
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);
  Serial.print("  First accel z (m/s^2): ");
  Serial.println(a.acceleration.z, 3);

  return true;
}

static bool initVL53L0X() {
  Serial.println("\n=== VL53L0X Init (Wire / I2C0) ===");

  // Pololu VL53L0X library supports selecting bus in most versions via setBus().
  // If your installed version lacks setBus(), comment the next line and ensure
  // the library uses global Wire by default (which it typically does).
  tof.setBus(&Wire);

  tof.setTimeout(500);

  if (!tof.init()) {
    Serial.println("  ERROR: VL53L0X init() failed on Wire.");
    return false;
  }

  Serial.println("  OK: VL53L0X init() succeeded on Wire.");
  tof.startContinuous();
  return true;
}

static void updateEulerFromMPU(float dt) {
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  // accel (m/s^2)
  const float ax = a.acceleration.x;
  const float ay = a.acceleration.y;
  const float az = a.acceleration.z;

  // gyro (rad/s -> deg/s)
  const float gx = g.gyro.x * 180.0f / PI;
  const float gy = g.gyro.y * 180.0f / PI;
  const float gz = g.gyro.z * 180.0f / PI;

  // accel-only angles
  const float accel_roll  = atan2f(ay, az) * 180.0f / PI;
  const float accel_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / PI;

  // integrate gyro
  roll  += gx * dt;
  pitch += gy * dt;
  yaw   += gz * dt;

  // complementary filter
  const float alpha = 0.98f;
  roll  = alpha * roll  + (1.0f - alpha) * accel_roll;
  pitch = alpha * pitch + (1.0f - alpha) * accel_pitch;
}

// ============================
// Setup / Loop
// ============================
bool imuOk = false;
bool tofOk = false;

void setup() {
  Serial.begin(115200);
  delay(800);

  Serial.println("\n=== Dual I2C Bring-Up (Philhower RP2040) ===");

  // ----------------------------
  // I2C0: Wire on GP20/GP21
  // ----------------------------
  Serial.println("\nInit I2C0: Wire (SDA=GP20, SCL=GP21)");
  Wire.setSDA(I2C0_SDA);
  Wire.setSCL(I2C0_SCL);
  Wire.begin();
  Wire.setClock(100000); // robust for breadboards

  // ----------------------------
  // I2C1: Wire1 on GP2/GP3
  // ----------------------------
  Serial.println("Init I2C1: Wire1 (SDA=GP2, SCL=GP3)");
  Wire1.setSDA(I2C1_SDA);
  Wire1.setSCL(I2C1_SCL);
  Wire1.begin();
  Wire1.setClock(100000);

  // Scan both buses
  scanBus(Wire,  "Wire / I2C0 (GP20/GP21)");
  scanBus(Wire1, "Wire1 / I2C1 (GP2/GP3)");

  // Init devices on their correct buses
  imuOk = initMPU6050();
  tofOk = initVL53L0X();
  testPCA9685Presence();

  Serial.println("\n--- Runtime Output ---");
  Serial.println("IMU: roll | pitch | yaw  | raw accel/gyro non-zero expectation");
  Serial.println("ToF: distance (mm)");
  Serial.println("PCA9685: confirmed via ACK + begin + register writes (no V+ needed)");
  Serial.println("----------------------\n");

  lastMs = millis();
}

void loop() {
  const uint32_t now = millis();
  float dt = (now - lastMs) / 1000.0f;
  if (dt <= 0.0f) dt = 0.005f;
  if (dt > 0.5f)  dt = 0.01f; // clamp on startup/serial delays
  lastMs = now;

  // ---- IMU on Wire / I2C0 ----
  if (imuOk) {
    updateEulerFromMPU(dt);
  }

  // ---- ToF on Wire / I2C0 ----
  uint16_t distance_mm = 0;
  bool tofValid = false;
  if (tofOk) {
    distance_mm = tof.readRangeContinuousMillimeters();
    tofValid = !tof.timeoutOccurred();
  }

  // Print at ~10 Hz
  static uint32_t tPrint = 0;
  if (millis() - tPrint >= 100) {
    tPrint = millis();

    Serial.print("Roll: ");  Serial.print(roll, 2);
    Serial.print(" | Pitch: "); Serial.print(pitch, 2);
    Serial.print(" | Yaw: ");   Serial.print(yaw, 2);

    Serial.print(" | ToF: ");
    if (tofOk && tofValid && distance_mm > 0 && distance_mm < 8000) {
      Serial.print(distance_mm);
      Serial.print(" mm");
    } else if (tofOk) {
      Serial.print("timeout/out-of-range");
    } else {
      Serial.print("not initialised");
    }

    // Quick PCA presence monitor (ACK only; light touch)
    Serial.print(" | PCA(0x40) ACK: ");
    Serial.print(ackAddress(Wire1, PCA_ADDR) ? "YES" : "NO");

    Serial.println();
  }
}
