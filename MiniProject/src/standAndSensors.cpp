#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <VL53L0X.h>

// ============================================================
// Hardware: RP2040 (Earle Philhower core)
// I2C0 (Wire)  : SDA GP20, SCL GP21  -> IMU + ToF
// I2C1 (Wire1) : SDA GP2,  SCL GP3   -> PCA9685
// ============================================================

// ---------------- I2C pins / addrs ----------------
static constexpr uint8_t I2C0_SDA = 20;
static constexpr uint8_t I2C0_SCL = 21;

static constexpr uint8_t I2C1_SDA = 2;
static constexpr uint8_t I2C1_SCL = 3;

static constexpr uint8_t IMU_ADDR = 0x68;
static constexpr uint8_t PCA_ADDR = 0x40;

// ---------------- PCA / Servos ----------------
Adafruit_PWMServoDriver pca(PCA_ADDR, Wire1);

struct ServoCal {
  uint8_t ch;
  uint16_t minPulse;
  uint16_t neutralPulse;
  uint16_t maxPulse;
  const char* name;
};

// SAFE min/max + your confirmed neutrals
static ServoCal servos[] = {
  { 0, 240, 510, 510, "FR horizontal" },
  { 1, 50, 160, 510, "FR vertical"   },
  { 2, 240, 325, 510, "MR horizontal" },
  { 3, 50, 170, 510, "MR vertical"   },
  { 4, 240, 325, 510, "RR horizontal" },
  { 5, 50, 200, 510, "RR vertical"   },
  { 6, 240, 345, 510, "FL horizontal" },
  { 7, 50, 160, 510, "FL vertical"   },
  { 8, 240, 335, 510, "ML horizontal" },
  { 9, 50, 160, 510, "ML vertical"   },
  {10, 240, 375, 510, "RL horizontal" },
  {11, 50, 230, 510, "RL vertical"   },
};
static constexpr size_t N_SERVOS = sizeof(servos) / sizeof(servos[0]);

// Indices (in servos[]) of vertical servos: channels 1,3,5,7,9,11
static constexpr uint8_t VERT_IDX[] = {1, 3, 5, 7, 9, 11};
static constexpr size_t N_VERT = sizeof(VERT_IDX) / sizeof(VERT_IDX[0]);

static uint16_t lastPose[16] = {0}; // last commanded pulse per channel (0 = unknown)

// ---------------- ToF ----------------
VL53L0X tof;
bool tofOk = false;

// ---------------- IMU (register-level) ----------------
bool imuOk = false;
bool referenceSet = false;

float rollDeg  = 0.0f;
float pitchDeg = 0.0f;
float roll0    = 0.0f;
float pitch0   = 0.0f;

int16_t rawAx=0, rawAy=0, rawAz=0, rawGx=0, rawGy=0, rawGz=0;

uint32_t lastIMUMs = 0;

// ============================================================
// Utility helpers
// ============================================================
static bool ack(TwoWire& bus, uint8_t addr) {
  bus.beginTransmission(addr);
  return (bus.endTransmission() == 0);
}

static uint16_t clampPulse(uint16_t p, uint16_t lo, uint16_t hi) {
  if (p < lo) return lo;
  if (p > hi) return hi;
  return p;
}

static void setPulse(uint8_t ch, uint16_t pulse) {
  pca.setPWM(ch, 0, pulse);
}

static void rampTo(uint8_t ch, uint16_t from, uint16_t to, uint16_t step, uint16_t stepDelayMs) {
  if (from == to) return;

  if (to > from) {
    for (uint16_t p = from; p < to; p = (uint16_t)min<uint32_t>(p + step, to)) {
      setPulse(ch, p);
      delay(stepDelayMs);
    }
  } else {
    for (uint16_t p = from; p > to; p = (uint16_t)max<int32_t>((int32_t)p - (int32_t)step, (int32_t)to)) {
      setPulse(ch, p);
      delay(stepDelayMs);
    }
  }
}

static void buildNeutralPose(uint16_t pose[16]) {
  memset(pose, 0, 16 * sizeof(uint16_t));
  for (size_t i = 0; i < N_SERVOS; i++) {
    uint16_t lo = min(servos[i].minPulse, servos[i].maxPulse);
    uint16_t hi = max(servos[i].minPulse, servos[i].maxPulse);
    pose[servos[i].ch] = clampPulse(servos[i].neutralPulse, lo, hi);
  }
}

// Pre-stand = neutral with vertical servos offset a bit to make the transition safe.
static void buildPreStandPose(uint16_t pose[16], int16_t verticalOffset) {
  buildNeutralPose(pose);

  for (size_t k = 0; k < N_VERT; k++) {
    uint8_t idx = VERT_IDX[k];
    uint8_t ch = servos[idx].ch;

    uint16_t lo = min(servos[idx].minPulse, servos[idx].maxPulse);
    uint16_t hi = max(servos[idx].minPulse, servos[idx].maxPulse);

    int32_t p = (int32_t)pose[ch] + (int32_t)verticalOffset;
    p = max<int32_t>(0, p);
    pose[ch] = clampPulse((uint16_t)p, lo, hi);
  }
}

static void applyPose(const uint16_t pose[16], uint16_t step = 3, uint16_t stepDelayMs = 8) {
  for (uint8_t ch = 0; ch < 16; ch++) {
    uint16_t target = pose[ch];
    if (target == 0) continue;

    uint16_t from = lastPose[ch];
    if (from == 0) {
      setPulse(ch, target);
      lastPose[ch] = target;
      delay(20);
    } else {
      rampTo(ch, from, target, step, stepDelayMs);
      lastPose[ch] = target;
    }
  }
}

// ============================================================
// IMU register-level functions (MPU6050/MPU6500 class)
// ============================================================
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

static bool imuInitBasic() {
  if (!ack(Wire, IMU_ADDR)) return false;

  uint8_t who = 0xFF;
  if (!readReg8(Wire, IMU_ADDR, 0x75, who)) return false;

  Serial.print("[IMU] WHO_AM_I = 0x");
  Serial.println(who, HEX);

  // Wake up
  if (!writeReg8(Wire, IMU_ADDR, 0x6B, 0x00)) return false;
  delay(50);

  // Clock source PLL
  writeReg8(Wire, IMU_ADDR, 0x6B, 0x01);

  // Gyro: ±500 dps (FS_SEL=1)
  writeReg8(Wire, IMU_ADDR, 0x1B, 0x08);

  // Accel: ±4g (AFS_SEL=1)
  writeReg8(Wire, IMU_ADDR, 0x1C, 0x08);

  // DLPF ~ 21Hz
  writeReg8(Wire, IMU_ADDR, 0x1A, 0x04);

  lastIMUMs = millis();
  return true;
}

static bool imuReadRaw(int16_t& ax, int16_t& ay, int16_t& az, int16_t& gx, int16_t& gy, int16_t& gz) {
  uint8_t buf[14];
  if (!readBytes(Wire, IMU_ADDR, 0x3B, buf, sizeof(buf))) return false;

  ax = (int16_t)((buf[0] << 8) | buf[1]);
  ay = (int16_t)((buf[2] << 8) | buf[3]);
  az = (int16_t)((buf[4] << 8) | buf[5]);

  gx = (int16_t)((buf[8]  << 8) | buf[9]);
  gy = (int16_t)((buf[10] << 8) | buf[11]);
  gz = (int16_t)((buf[12] << 8) | buf[13]);

  return true;
}

static void imuUpdate() {
  const uint32_t now = millis();
  float dt = (now - lastIMUMs) * 1e-3f;
  if (dt <= 0.0f || dt > 0.5f) dt = 0.01f;
  lastIMUMs = now;

  bool ok = imuReadRaw(rawAx, rawAy, rawAz, rawGx, rawGy, rawGz);
  if (!ok) return;

  // Acc angles (raw counts are fine for ratios)
  const float ax = (float)rawAx;
  const float ay = (float)rawAy;
  const float az = (float)rawAz;

  const float rollAcc  = atan2f(ay, az) * 180.0f / PI;
  const float pitchAcc = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / PI;

  // Gyro scaling: ±500 dps => 65.5 LSB/(deg/s)
  const float gx = (float)rawGx / 65.5f;
  const float gy = (float)rawGy / 65.5f;

  const float alpha = 0.98f;
  rollDeg  = alpha * (rollDeg  + gx * dt) + (1.0f - alpha) * rollAcc;
  pitchDeg = alpha * (pitchDeg + gy * dt) + (1.0f - alpha) * pitchAcc;
}

static void imuZeroReference() {
  roll0 = rollDeg;
  pitch0 = pitchDeg;
  referenceSet = true;
  Serial.println("[IMU] Reference zeroed (post-stand).");
}

// ============================================================
// Main behaviour
// ============================================================
enum class State : uint8_t {
  BOOT,
  STANDUP_PRE,
  STANDUP_NEUTRAL,
  POST_STAND_ZERO,
  RUN
};

static State state = State::BOOT;

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0 < 3000)) delay(10);

  Serial.println("\n=== Integrated: Stand-up + IMU + ToF + PCA ACK ===");
  Serial.println("Flow: stand-up -> auto zero IMU -> run diagnostics");
  Serial.println("Press 'Z' anytime to re-zero IMU reference.\n");

  // ---- I2C0 (Wire) for IMU + ToF ----
  Wire.setSDA(I2C0_SDA);
  Wire.setSCL(I2C0_SCL);
  Wire.begin();
  Wire.setClock(50000);

  // ---- I2C1 (Wire1) for PCA9685 ----
  Wire1.setSDA(I2C1_SDA);
  Wire1.setSCL(I2C1_SCL);
  Wire1.begin();
  Wire1.setClock(100000);

  // ---- PCA init ----
  if (!pca.begin()) {
    Serial.println("ERROR: PCA9685 not found on Wire1 (0x40).");
    while (true) delay(1000);
  }
  pca.setPWMFreq(50);
  delay(20);

  // ---- IMU init ----
  Serial.print("[IMU] ACK 0x68: ");
  Serial.println(ack(Wire, IMU_ADDR) ? "YES" : "NO");
  imuOk = imuInitBasic();
  Serial.print("[IMU] init: ");
  Serial.println(imuOk ? "OK" : "FAIL");

  // ---- ToF init ----
  tof.setTimeout(200);
  tofOk = tof.init();
  Serial.print("[ToF] init: ");
  Serial.println(tofOk ? "OK" : "FAIL");
  if (tofOk) tof.startContinuous();

  // ---- Stand-up immediately ----
  state = State::STANDUP_PRE;

  Serial.println("\nOutput format:");
  Serial.println("roll pitch | rollRel pitchRel | rawAx rawAy rawAz | rawGx rawGy rawGz | ToFmm | PCA_ACK | ref");
  Serial.println("------------------------------------------------------------------------------------------");
}

void loop() {
  // Allow manual re-zero at any time
  if (Serial.available()) {
    char c = (char)Serial.read();
    if (c == 'z' || c == 'Z') {
      imuZeroReference();
    }
  }

  // Always update IMU when possible (needed so zeroing is meaningful)
  if (imuOk) imuUpdate();

  // State machine
  static uint16_t posePre[16];
  static uint16_t poseNeutral[16];
  static bool posesBuilt = false;

  if (!posesBuilt) {
    buildNeutralPose(poseNeutral);
    // verticalOffset: this direction worked for you in stand_up test
    const int16_t verticalOffset = +25;
    buildPreStandPose(posePre, verticalOffset);
    posesBuilt = true;
  }

  switch (state) {
    case State::STANDUP_PRE: {
      Serial.println("[STATE] PRE-STAND...");
      applyPose(posePre, 3, 8);
      delay(800);
      state = State::STANDUP_NEUTRAL;
    } break;

    case State::STANDUP_NEUTRAL: {
      Serial.println("[STATE] NEUTRAL STAND...");
      applyPose(poseNeutral, 3, 8);
      delay(800);
      state = State::POST_STAND_ZERO;
    } break;

    case State::POST_STAND_ZERO: {
      // Give IMU a short moment to settle, then auto-zero
      Serial.println("[STATE] IMU settle + auto zero...");
      delay(300);
      imuZeroReference();
      state = State::RUN;
    } break;

    case State::RUN: {
      // Print diagnostics at ~10 Hz
      static uint32_t lastPrint = 0;
      if (millis() - lastPrint < 100) break;
      lastPrint = millis();

      // ToF
      uint16_t mm = 0;
      bool tofValid = false;
      if (tofOk) {
        mm = tof.readRangeContinuousMillimeters();
        tofValid = !tof.timeoutOccurred();
      }

      // PCA ACK
      const bool pcaAck = ack(Wire1, PCA_ADDR);

      // Relative
      const float rollRel  = rollDeg  - roll0;
      const float pitchRel = pitchDeg - pitch0;

      Serial.print(rollDeg, 2); Serial.print(" ");
      Serial.print(pitchDeg, 2); Serial.print(" | ");
      Serial.print(rollRel, 2);  Serial.print(" ");
      Serial.print(pitchRel, 2); Serial.print(" | ");

      if (imuOk) {
        Serial.print(rawAx); Serial.print(" ");
        Serial.print(rawAy); Serial.print(" ");
        Serial.print(rawAz); Serial.print(" | ");
        Serial.print(rawGx); Serial.print(" ");
        Serial.print(rawGy); Serial.print(" ");
        Serial.print(rawGz);
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
    } break;

    default:
      break;
  }

  delay(5);
}
