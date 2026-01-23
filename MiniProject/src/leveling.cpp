#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <VL53L0X.h>

// ============================================================
// RP2040 (Earle Philhower core)
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
  { 7, 50, 300, 510, "FL vertical"   },
  { 8, 240, 335, 510, "ML horizontal" },
  { 9, 50, 160, 510, "ML vertical"   },
  {10, 240, 375, 510, "RL horizontal" },
  {11, 50, 230, 510, "RL vertical"   },
};
static constexpr size_t N_SERVOS = sizeof(servos) / sizeof(servos[0]);

// Vertical servo entries (in servos[] order): CH1,3,5,7,9,11
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

static void disableChannel(uint8_t ch) {
  // Full OFF (no pulses)
  pca.setPWM(ch, 0, 4096);
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
// IMU register-level
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
  if (bus.endTransmission(false) != 0) return false;
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

  if (!writeReg8(Wire, IMU_ADDR, 0x6B, 0x00)) return false;
  delay(50);

  writeReg8(Wire, IMU_ADDR, 0x6B, 0x01);
  writeReg8(Wire, IMU_ADDR, 0x1B, 0x08);
  writeReg8(Wire, IMU_ADDR, 0x1C, 0x08);
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
  static float roll = 0.0f, pitch = 0.0f;

  const uint32_t now = millis();
  float dt = (now - lastIMUMs) * 1e-3f;
  if (dt <= 0.0f || dt > 0.5f) dt = 0.01f;
  lastIMUMs = now;

  int16_t ax, ay, az, gxRaw, gyRaw, gzRaw;
  if (!imuReadRaw(ax, ay, az, gxRaw, gyRaw, gzRaw)) return;

  const float fax = (float)ax;
  const float fay = (float)ay;
  const float faz = (float)az;

  const float rollAcc  = atan2f(fay, faz) * 180.0f / PI;
  const float pitchAcc = atan2f(-fax, sqrtf(fay*fay + faz*faz)) * 180.0f / PI;

  const float gx = (float)gxRaw / 65.5f;
  const float gy = (float)gyRaw / 65.5f;

  const float alpha = 0.98f;
  roll  = alpha * (roll  + gx * dt) + (1.0f - alpha) * rollAcc;
  pitch = alpha * (pitch + gy * dt) + (1.0f - alpha) * pitchAcc;

  rollDeg = roll;
  pitchDeg = pitch;
}

static void imuZeroReference() {
  roll0 = rollDeg;
  pitch0 = pitchDeg;
  referenceSet = true;
  Serial.println("[IMU] Reference set.");
}

// ============================================================
// Levelling controller
// ============================================================
static float Kp_roll  = 1.20f;
static float Kp_pitch = 1.20f;
static float deadbandDeg = 0.70f;

static int8_t rollSign  = +1;
static int8_t pitchSign = +1;

static int16_t maxOffsetTicks = 35;
static int16_t maxStepPerUpdate = 2;

// Vertical order: FR(1), MR(3), RR(5), FL(7), ML(9), RL(11)
static const int8_t rollWeight[6]     = { +1, +1, +1,  -1, -1, -1 };
static const int8_t pitchWeightInt[6] = { +2, +1, -2,  +2, +1, -2 }; // /2.0

static int16_t vertOffsetApplied[6] = {0,0,0,0,0,0};

static int16_t clampI16(int16_t v, int16_t lo, int16_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static float applyDeadband(float x, float db) {
  if (fabsf(x) < db) return 0.0f;
  return x;
}

static void levellingUpdate(const uint16_t neutralPose[16]) {
  if (!referenceSet) return;

  float rollRel  = rollDeg  - roll0;
  float pitchRel = pitchDeg - pitch0;

  rollRel  = applyDeadband(rollRel, deadbandDeg);
  pitchRel = applyDeadband(pitchRel, deadbandDeg);

  float corrRoll  = (float)rollSign  * (-Kp_roll)  * rollRel;
  float corrPitch = (float)pitchSign * (-Kp_pitch) * pitchRel;

  for (size_t i = 0; i < 6; i++) {
    float wPitch = (float)pitchWeightInt[i] / 2.0f;
    float wRoll  = (float)rollWeight[i];

    int16_t target = (int16_t)lroundf((wRoll * corrRoll) + (wPitch * corrPitch));
    target = clampI16(target, -maxOffsetTicks, +maxOffsetTicks);

    int16_t current = vertOffsetApplied[i];
    int16_t delta = clampI16((int16_t)(target - current), -maxStepPerUpdate, +maxStepPerUpdate);
    int16_t next = current + delta;
    vertOffsetApplied[i] = next;

    uint8_t servoIdx = VERT_IDX[i];
    uint8_t ch = servos[servoIdx].ch;

    uint16_t lo = min(servos[servoIdx].minPulse, servos[servoIdx].maxPulse);
    uint16_t hi = max(servos[servoIdx].minPulse, servos[servoIdx].maxPulse);

    int32_t pulse = (int32_t)neutralPose[ch] + (int32_t)next;
    pulse = max<int32_t>(0, pulse);
    uint16_t finalPulse = clampPulse((uint16_t)pulse, lo, hi);

    setPulse(ch, finalPulse);
    lastPose[ch] = finalPulse;
  }
}

// ============================================================
// State machine for your desired workflow
// ============================================================
enum class Mode : uint8_t {
  IDLE,          // powered, safe, not moving servos
  READY,         // armed, waiting for START
  STANDING,      // stand complete, IMU ref set, levelling OFF
  LEVELLING,     // levelling ON
  EMERGENCY_OFF  // PWM disabled
};

static Mode mode = Mode::IDLE;

static bool armed = false;
static uint16_t poseNeutral[16];
static uint16_t posePre[16];
static bool posesBuilt = false;

static void printMenu() {
  Serial.println("\nCommands:");
  Serial.println("  A = ARM (enable movement commands)");
  Serial.println("  S = START stand-up (go to neutral stand, then IMU ref set)");
  Serial.println("  L = LEVEL ON");
  Serial.println("  K = LEVEL OFF (hold pose)");
  Serial.println("  Z = re-zero IMU reference (standing)");
  Serial.println("  R = flip roll sign | P = flip pitch sign");
  Serial.println("  + / - = adjust gains");
  Serial.println("  X = EMERGENCY OFF (disable PWM outputs)");
  Serial.println("  G = recover from emergency (re-apply stand, levelling OFF)");
  Serial.println();
}

static void emergencyOff() {
  for (size_t i = 0; i < N_SERVOS; i++) {
    disableChannel(servos[i].ch);
    lastPose[servos[i].ch] = 0;
    delay(2);
  }
  mode = Mode::EMERGENCY_OFF;
  Serial.println("[EMERGENCY OFF] PWM outputs disabled.");
}

static void recoverFromEmergency() {
  // Re-apply neutral pose, reset offsets, keep levelling OFF
  applyPose(poseNeutral, 3, 8);
  for (size_t i = 0; i < 6; i++) vertOffsetApplied[i] = 0;
  mode = Mode::STANDING;
  Serial.println("[RECOVER] Neutral stand applied. Levelling OFF.");
}

static void doStandUp() {
  Serial.println("[START] Standing up...");
  applyPose(posePre, 3, 8);
  delay(800);
  applyPose(poseNeutral, 3, 8);
  delay(800);
  Serial.println("[START] Standing achieved.");
  imuZeroReference(); // set reference immediately after standing
  for (size_t i = 0; i < 6; i++) vertOffsetApplied[i] = 0;
  mode = Mode::STANDING;
  Serial.println("[STATE] STANDING (levelling OFF). Press 'L' to enable levelling.");
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0 < 3000)) delay(10);

  Serial.println("\n=== Robot Workflow: START -> Stand -> IMU Ref -> Level ON ===");

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

  // PCA
  if (!pca.begin()) {
    Serial.println("ERROR: PCA9685 not found on Wire1 (0x40).");
    while (true) delay(1000);
  }
  pca.setPWMFreq(50);
  delay(20);

  // IMU
  imuOk = imuInitBasic();
  Serial.print("[IMU] init: "); Serial.println(imuOk ? "OK" : "FAIL");

  // ToF
  tof.setTimeout(200);
  tofOk = tof.init();
  Serial.print("[ToF] init: "); Serial.println(tofOk ? "OK" : "FAIL");
  if (tofOk) tof.startContinuous();

  // Poses
  buildNeutralPose(poseNeutral);
  buildPreStandPose(posePre, +25);
  posesBuilt = true;

  mode = Mode::IDLE;
  armed = false;

  printMenu();

  Serial.println("Status output:");
  Serial.println("mode | rollRel pitchRel | ToFmm | PCA_ACK | REF | lev");
  Serial.println("------------------------------------------------------");
}

void loop() {
  if (imuOk) imuUpdate();

  // Keys
  while (Serial.available()) {
    char c = (char)Serial.read();

    if (c == 'h' || c == 'H') printMenu();

    if (c == 'x' || c == 'X') {
      emergencyOff();
      continue;
    }

    if (c == 'g' || c == 'G') {
      if (mode == Mode::EMERGENCY_OFF && posesBuilt) recoverFromEmergency();
      continue;
    }

    if (c == 'a' || c == 'A') {
      armed = true;
      mode = Mode::READY;
      Serial.println("[ARM] Ready. Press 'S' to stand up.");
      continue;
    }

    if (c == 's' || c == 'S') {
      if (!armed) {
        Serial.println("[WARN] Not armed. Press 'A' first.");
      } else if (mode == Mode::EMERGENCY_OFF) {
        Serial.println("[WARN] Emergency off. Press 'G' to recover.");
      } else {
        doStandUp();
      }
      continue;
    }

    if (c == 'l' || c == 'L') {
      if (mode == Mode::STANDING) {
        mode = Mode::LEVELLING;
        Serial.println("[LEVEL] ON");
      } else {
        Serial.println("[WARN] Not standing yet. Press 'S' after arming.");
      }
      continue;
    }

    if (c == 'k' || c == 'K') {
      if (mode == Mode::LEVELLING) {
        mode = Mode::STANDING;
        Serial.println("[LEVEL] OFF (holding pose).");
      }
      continue;
    }

    if (c == 'z' || c == 'Z') {
      if (mode == Mode::STANDING || mode == Mode::LEVELLING) {
        imuZeroReference();
        for (size_t i = 0; i < 6; i++) vertOffsetApplied[i] = 0;
      } else {
        Serial.println("[WARN] Zero IMU only after standing.");
      }
      continue;
    }

    if (c == 'r' || c == 'R') {
      rollSign = (int8_t)(-rollSign);
      Serial.print("[LEVEL] rollSign -> "); Serial.println((int)rollSign);
      continue;
    }

    if (c == 'p' || c == 'P') {
      pitchSign = (int8_t)(-pitchSign);
      Serial.print("[LEVEL] pitchSign -> "); Serial.println((int)pitchSign);
      continue;
    }

    if (c == '+') {
      Kp_roll += 0.10f; Kp_pitch += 0.10f;
      Serial.println("[LEVEL] gains increased");
      continue;
    }

    if (c == '-') {
      Kp_roll = max(0.0f, Kp_roll - 0.10f);
      Kp_pitch = max(0.0f, Kp_pitch - 0.10f);
      Serial.println("[LEVEL] gains decreased");
      continue;
    }
  }

  // Levelling only when explicitly enabled
  if (mode == Mode::LEVELLING) {
    static uint32_t lastLevel = 0;
    if (millis() - lastLevel >= 20) {
      lastLevel = millis();
      levellingUpdate(poseNeutral);
    }
  }

  // Status print at ~10 Hz
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= 100) {
    lastPrint = millis();

    uint16_t mm = 0;
    bool tofValid = false;
    if (tofOk) {
      mm = tof.readRangeContinuousMillimeters();
      tofValid = !tof.timeoutOccurred();
    }

    const bool pcaAck = ack(Wire1, PCA_ADDR);

    float rollRel  = rollDeg  - roll0;
    float pitchRel = pitchDeg - pitch0;

    const char* modeStr =
      (mode == Mode::IDLE) ? "IDLE" :
      (mode == Mode::READY) ? "READY" :
      (mode == Mode::STANDING) ? "STAND" :
      (mode == Mode::LEVELLING) ? "LEVEL" :
      "EMOFF";

    Serial.print(modeStr);
    Serial.print(" | ");

    if (referenceSet) {
      Serial.print(rollRel, 2); Serial.print(" ");
      Serial.print(pitchRel, 2);
    } else {
      Serial.print("NA NA");
    }

    Serial.print(" | ");
    if (tofOk && tofValid) Serial.print(mm);
    else if (tofOk) Serial.print("timeout");
    else Serial.print("NA");

    Serial.print(" | ");
    Serial.print(pcaAck ? "YES" : "NO");

    Serial.print(" | ");
    Serial.print(referenceSet ? "REF" : "NOREF");

    Serial.print(" | ");
    Serial.println(mode == Mode::LEVELLING ? "ON" : "OFF");
  }

  delay(2);
}
