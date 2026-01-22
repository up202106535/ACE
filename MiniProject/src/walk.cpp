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

// ---------------- PCA ----------------
Adafruit_PWMServoDriver pca(PCA_ADDR, Wire1);

// ---------------- Servo calibration ----------------
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

// Vertical servo indices in servos[] array (CH1,3,5,7,9,11)
static constexpr uint8_t VERT_IDX[] = {1, 3, 5, 7, 9, 11};
static constexpr size_t N_VERT = sizeof(VERT_IDX) / sizeof(VERT_IDX[0]);

// ---------------- Legs (6) ----------------
// Order: FR, MR, RR, FL, ML, RL
struct Leg {
  const char* name;
  uint8_t hCh;
  uint8_t vCh;
  uint16_t hNeutral;
  uint16_t vNeutral;
  uint16_t hMin, hMax;
  uint16_t vMin, vMax;
};

static Leg legs[6];

// ---------------- ToF ----------------
VL53L0X tof;
bool tofOk = false;

// ---------------- IMU (register-level) ----------------
bool imuOk = false;
bool referenceSet = false;
float rollDeg = 0.0f, pitchDeg = 0.0f;
float roll0 = 0.0f, pitch0 = 0.0f;
uint32_t lastIMUMs = 0;

// ---------------- Runtime state ----------------
static uint16_t lastPose[16] = {0};   // last commanded pulses per channel (0=unknown)
static bool emergencyOff = false;
static bool armed = false;

// ============================================================
// Helpers
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

static int16_t clampI16(int16_t v, int16_t lo, int16_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// Slew-limited update: move current -> target by at most step ticks
static uint16_t slew(uint16_t current, uint16_t target, uint16_t step) {
  if (current == 0) return target; // unknown -> jump to target once
  if (current < target) {
    uint32_t next = current + step;
    return (uint16_t)min<uint32_t>(next, target);
  } else {
    int32_t next = (int32_t)current - (int32_t)step;
    return (uint16_t)max<int32_t>(next, (int32_t)target);
  }
}

// ============================================================
// Poses (neutral + pre-stand)
// ============================================================
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

// Apply pose smoothly
static void applyPose(const uint16_t pose[16], uint16_t step = 3, uint16_t stepDelayMs = 6) {
  for (uint8_t ch = 0; ch < 16; ch++) {
    uint16_t target = pose[ch];
    if (target == 0) continue;

    uint16_t current = lastPose[ch];
    uint16_t next = slew(current, target, step);
    setPulse(ch, next);
    lastPose[ch] = next;
    delay(stepDelayMs);
    // Not a full ramp here; we call applyPose multiple times (pre-stand, then neutral)
  }
  // second pass to finish ramp quickly
  for (uint8_t ch = 0; ch < 16; ch++) {
    uint16_t target = pose[ch];
    if (target == 0) continue;

    uint16_t current = lastPose[ch];
    while (current != target) {
      current = slew(current, target, step);
      setPulse(ch, current);
      lastPose[ch] = current;
      delay(stepDelayMs);
    }
  }
}

// ============================================================
// IMU register-level (MPU6050/MPU6500 class)
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

  writeReg8(Wire, IMU_ADDR, 0x6B, 0x01); // PLL
  writeReg8(Wire, IMU_ADDR, 0x1B, 0x08); // gyro ±500 dps
  writeReg8(Wire, IMU_ADDR, 0x1C, 0x08); // accel ±4g
  writeReg8(Wire, IMU_ADDR, 0x1A, 0x04); // DLPF ~ 21Hz

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

  const float gx = (float)gxRaw / 65.5f; // deg/s
  const float gy = (float)gyRaw / 65.5f;

  const float alpha = 0.98f;
  roll  = alpha * (roll  + gx * dt) + (1.0f - alpha) * rollAcc;
  pitch = alpha * (pitch + gy * dt) + (1.0f - alpha) * pitchAcc;

  rollDeg = roll;
  pitchDeg = pitch;
}

static void imuSetReference() {
  roll0 = rollDeg;
  pitch0 = pitchDeg;
  referenceSet = true;
  Serial.println("[IMU] Reference set (standing).");
}

// ============================================================
// Build legs table from servos[]
// ============================================================
static const ServoCal* findServo(uint8_t ch) {
  for (size_t i = 0; i < N_SERVOS; i++) if (servos[i].ch == ch) return &servos[i];
  return nullptr;
}

static void initLegs() {
  // FR: H0 V1, MR: H2 V3, RR: H4 V5, FL: H6 V7, ML: H8 V9, RL: H10 V11
  const struct { const char* name; uint8_t h; uint8_t v; } map[6] = {
    {"FR", 0, 1}, {"MR", 2, 3}, {"RR", 4, 5}, {"FL", 6, 7}, {"ML", 8, 9}, {"RL",10,11}
  };

  for (int i = 0; i < 6; i++) {
    const ServoCal* hs = findServo(map[i].h);
    const ServoCal* vs = findServo(map[i].v);

    legs[i].name = map[i].name;
    legs[i].hCh = map[i].h;
    legs[i].vCh = map[i].v;

    legs[i].hNeutral = hs->neutralPulse;
    legs[i].vNeutral = vs->neutralPulse;

    legs[i].hMin = min(hs->minPulse, hs->maxPulse);
    legs[i].hMax = max(hs->minPulse, hs->maxPulse);

    legs[i].vMin = min(vs->minPulse, vs->maxPulse);
    legs[i].vMax = max(vs->minPulse, vs->maxPulse);
  }
}

// ============================================================
// Gait engine (Tripod)
// ============================================================
enum class WalkMode : uint8_t { NONE, FORWARD, SIDE, ROTATE };
static WalkMode walkMode = WalkMode::NONE;

// Tripod groups (indices in legs[])
// Group A: FR, ML, RR (0,4,2)   Group B: MR, FL, RL (1,3,5)
static const uint8_t tripodA[3] = {0, 4, 2};
static const uint8_t tripodB[3] = {1, 3, 5};

static int8_t horizGlobalSign = +1; // flip with 'H' if forward is reversed
static int8_t liftSign = +1;        // flip with 'V' if lift is reversed

// Speed levels (0 slow, 1 medium, 2 fast)
static int speedLevel = 0;

// Base parameters (conservative)
static uint16_t cycleMs_table[3] = {1200, 800, 500};   // gait cycle length
static int16_t  stepAmp_table[3] = {100,   220,   300};    // horizontal amplitude (ticks)
static int16_t  lift_table[3]    = {180,   220,   260};    // vertical lift (ticks)

// Safety: slew limit for servo pulses during gait
static uint16_t gaitSlewStep = 5; // ticks per update (limits aggressiveness)
static uint16_t gaitUpdateMs = 20; // 50 Hz

static uint16_t neutralPose[16];
static uint16_t preStandPose[16];

static bool standing = false;

// Compute whether leg index is in a tripod group
static bool inGroup(const uint8_t group[3], uint8_t legIdx) {
  return (group[0] == legIdx || group[1] == legIdx || group[2] == legIdx);
}

// Direction mapping for (forward/side/rotate).
// Returns a signed direction for each leg's horizontal servo.
// This is a simplification: it assumes horizontal joint produces useful planar motion.
// You may later refine per-leg signs if one leg moves opposite due to linkage orientation.
static int8_t legDirForMode(uint8_t legIdx, WalkMode mode) {
  // Right side legs: FR(0), MR(1), RR(2)
  bool isFront  = (legIdx == 0 || legIdx == 3);
  bool isRear   = (legIdx == 2 || legIdx == 5);
  bool isMiddle = (legIdx == 1 || legIdx == 4);
  bool isRight  = (legIdx <= 2);

   switch (mode) {

    case WalkMode::FORWARD:
      // Front legs push backward, rear legs push forward
      if (isFront)  return -1;
      if (isRear)   return +1;
      return 0;   // middle legs small / neutral

    case WalkMode::SIDE:
      // Sideways: right legs push opposite to left
      return isRight ? +1 : -1;

    case WalkMode::ROTATE:
      // Rotate in place
      return isRight ? +1 : -1;

    default:
      return 0;
  }
}

// Apply one gait update (standing must be true)
static void gaitUpdate() {
  if (!standing) return;
  if (walkMode == WalkMode::NONE) return;
  if (emergencyOff) return;

  const uint32_t now = millis();
  static uint32_t tStart = 0;
  static bool started = false;

  if (!started) {
    tStart = now;
    started = true;
  }

  const uint16_t cycleMs = cycleMs_table[speedLevel];
  const int16_t stepAmp = stepAmp_table[speedLevel];
  const int16_t lift = lift_table[speedLevel] * liftSign;

  uint32_t t = (now - tStart) % cycleMs;
  float phase = (float)t / (float)cycleMs; // [0,1)

  // Define swing for tripodA in first half, tripodB in second half
  bool aSwing = (phase < 0.5f);
  float local = aSwing ? (phase / 0.5f) : ((phase - 0.5f) / 0.5f); // [0,1)

  // Smooth profile (sinusoidal)
  // swing progression: 0->1
  float s = 0.5f - 0.5f * cosf(PI * local); // 0..1
  // stance progression: 0->1 (same s used, but we'll invert H direction)
  float sStance = s;

  // For each leg, determine if it is in swing group
  for (uint8_t i = 0; i < 6; i++) {
    bool legInA = inGroup(tripodA, i);
    bool swing = aSwing ? legInA : !legInA;

    // Horizontal target: swing moves "forward" +amp, stance moves "back" -amp
    int8_t dir = legDirForMode(i, walkMode);
    dir = (int8_t)(dir * horizGlobalSign);

    int16_t hOff = 0;
    if (swing) {
      // swing: move to +amp
      hOff = (int16_t)lroundf(((2.0f * s) - 1.0f) * (float)stepAmp);  // -amp .. +amp
      // bias to end at +amp during swing
      // (keeps it simple; later we can do more accurate trajectories)
    } else {
      // stance: move from +amp back to -amp (opposite direction)
      hOff = (int16_t)lroundf(((1.0f - 2.0f * sStance)) * (float)stepAmp); // +amp .. -amp
    }
    hOff = (int16_t)(hOff * dir);

    // Vertical target: lift during swing only (use sin for lift height)
    int16_t vOff = 0;
    if (swing) {
      float liftProf = sinf(PI * local); // 0..1..0
      vOff = (int16_t)lroundf(liftProf * (float)lift);
    } else {
      vOff = 0;
    }

    // Compute pulses (clamp to min/max)
    int32_t hPulse = (int32_t)legs[i].hNeutral + (int32_t)hOff;
    int32_t vPulse = (int32_t)legs[i].vNeutral + (int32_t)vOff;

    uint16_t hTarget = clampPulse((uint16_t)max<int32_t>(0, hPulse), legs[i].hMin, legs[i].hMax);
    uint16_t vTarget = clampPulse((uint16_t)max<int32_t>(0, vPulse), legs[i].vMin, legs[i].vMax);

    // Slew to targets (safe)
    uint16_t hCur = lastPose[legs[i].hCh];
    uint16_t vCur = lastPose[legs[i].vCh];

    uint16_t hNext = slew(hCur, hTarget, gaitSlewStep);
    uint16_t vNext = slew(vCur, vTarget, gaitSlewStep);

    setPulse(legs[i].hCh, hNext);
    setPulse(legs[i].vCh, vNext);

    lastPose[legs[i].hCh] = hNext;
    lastPose[legs[i].vCh] = vNext;
  }
}

// Stop walking and return to neutral standing pose (hold)
static void stopWalkingHoldNeutral() {
  walkMode = WalkMode::NONE;
  // Reset gait phase start next time
  // (we keep the static started flag inside gaitUpdate by resetting it via hack: toggle standing false/true)
  // simpler: re-apply neutral pose directly:
  applyPose(neutralPose, 3, 6);
  standing = true;
  Serial.println("[WALK] STOP. Holding neutral stand.");
}

// ============================================================
// Emergency handling
// ============================================================
static void emergencyDisableAll() {
  for (size_t i = 0; i < N_SERVOS; i++) {
    disableChannel(servos[i].ch);
    lastPose[servos[i].ch] = 0;
    delay(2);
  }
  emergencyOff = true;
  walkMode = WalkMode::NONE;
  standing = false;
  Serial.println("[EMERGENCY OFF] PWM disabled. Press 'G' to recover (re-apply stand).");
}

static void recoverFromEmergency() {
  emergencyOff = false;
  // Re-apply neutral pose (safe)
  applyPose(neutralPose, 3, 6);
  standing = true;
  walkMode = WalkMode::NONE;
  Serial.println("[RECOVER] Neutral stand applied. Walking OFF.");
}

// ============================================================
// Stand-up sequence (commanded)
// ============================================================
static void doStandUp() {
  if (!armed) { Serial.println("[WARN] Not armed. Press 'A' first."); return; }
  if (emergencyOff) { Serial.println("[WARN] Emergency off. Press 'G' to recover."); return; }

  Serial.println("[STAND] Pre-stand...");
  applyPose(preStandPose, 3, 6);
  delay(600);

  Serial.println("[STAND] Neutral stand...");
  applyPose(neutralPose, 3, 6);
  delay(600);

  standing = true;
  Serial.println("[STAND] Done. Now press 'Z' to set IMU reference. Then select walk mode 1/2/3.");
}

// ============================================================
// UI / Menu
// ============================================================
static void printMenu() {
  Serial.println("\nCommands:");
  Serial.println("  A = ARM");
  Serial.println("  S = STAND UP (pre-stand -> neutral)");
  Serial.println("  Z = Set IMU reference (standing)");
  Serial.println("  1 = Toggle FORWARD walk");
  Serial.println("  2 = Toggle SIDE walk");
  Serial.println("  3 = Toggle ROTATE walk");
  Serial.println("  0 = STOP walk (hold neutral)");
  Serial.println("  E = Faster | Q = Slower");
  Serial.println("  H = Flip horizontal direction");
  Serial.println("  V = Flip lift direction");
  Serial.println("  X = EMERGENCY OFF | G = Recover");
  Serial.println("  M = Print menu");
  Serial.println();
}

// ============================================================
// Setup / Loop
// ============================================================
void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0 < 3000)) delay(10);

  Serial.println("\n=== HEXAPOD: Stand + IMU + ToF + Basic Tripod Gait ===");

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

  // Sensors
  imuOk = imuInitBasic();
  Serial.print("[IMU] init: "); Serial.println(imuOk ? "OK" : "FAIL");

  tof.setTimeout(200);
  tofOk = tof.init();
  Serial.print("[ToF] init: "); Serial.println(tofOk ? "OK" : "FAIL");
  if (tofOk) tof.startContinuous();

  // Build poses
  buildNeutralPose(neutralPose);
  buildPreStandPose(preStandPose, +25); // your validated pre-stand direction

  // Legs
  initLegs();

  armed = false;
  standing = false;
  emergencyOff = false;
  walkMode = WalkMode::NONE;

  printMenu();

  Serial.println("Status:");
  Serial.println("mode | speed | rollRel pitchRel | ToFmm | PCA_ACK");
  Serial.println("--------------------------------------------------");
}

void loop() {
  if (imuOk) imuUpdate();

  // Handle serial keys
  while (Serial.available()) {
    char c = (char)Serial.read();

    if (c == 'm' || c == 'M') { printMenu(); }

    else if (c == 'a' || c == 'A') {
      armed = true;
      Serial.println("[ARM] OK. Press 'S' to stand.");
    }

    else if (c == 's' || c == 'S') {
      doStandUp();
    }

    else if (c == 'z' || c == 'Z') {
      if (!standing) {
        Serial.println("[WARN] Set reference only after standing.");
      } else {
        imuSetReference();
      }
    }

    else if (c == 'x' || c == 'X') {
      emergencyDisableAll();
    }

    else if (c == 'g' || c == 'G') {
      if (emergencyOff) recoverFromEmergency();
    }

    else if (c == '0') {
      if (standing && !emergencyOff) stopWalkingHoldNeutral();
    }

    else if (c == '1') {
      if (!standing) { Serial.println("[WARN] Stand first."); }
      else if (!referenceSet) { Serial.println("[WARN] Set IMU reference (Z) first."); }
      else {
        walkMode = (walkMode == WalkMode::FORWARD) ? WalkMode::NONE : WalkMode::FORWARD;
        Serial.println(walkMode == WalkMode::FORWARD ? "[WALK] FORWARD ON" : "[WALK] OFF");
        if (walkMode == WalkMode::NONE) stopWalkingHoldNeutral();
      }
    }

    else if (c == '2') {
      if (!standing) { Serial.println("[WARN] Stand first."); }
      else if (!referenceSet) { Serial.println("[WARN] Set IMU reference (Z) first."); }
      else {
        walkMode = (walkMode == WalkMode::SIDE) ? WalkMode::NONE : WalkMode::SIDE;
        Serial.println(walkMode == WalkMode::SIDE ? "[WALK] SIDE ON" : "[WALK] OFF");
        if (walkMode == WalkMode::NONE) stopWalkingHoldNeutral();
      }
    }

    else if (c == '3') {
      if (!standing) { Serial.println("[WARN] Stand first."); }
      else if (!referenceSet) { Serial.println("[WARN] Set IMU reference (Z) first."); }
      else {
        walkMode = (walkMode == WalkMode::ROTATE) ? WalkMode::NONE : WalkMode::ROTATE;
        Serial.println(walkMode == WalkMode::ROTATE ? "[WALK] ROTATE ON" : "[WALK] OFF");
        if (walkMode == WalkMode::NONE) stopWalkingHoldNeutral();
      }
    }

    else if (c == 'e' || c == 'E') {
      speedLevel = min(2, speedLevel + 1);
      Serial.print("[SPEED] "); Serial.println(speedLevel);
    }

    else if (c == 'q' || c == 'Q') {
      speedLevel = max(0, speedLevel - 1);
      Serial.print("[SPEED] "); Serial.println(speedLevel);
    }

    else if (c == 'h' || c == 'H') {
      horizGlobalSign = (int8_t)(-horizGlobalSign);
      Serial.print("[DIR] horizGlobalSign = "); Serial.println((int)horizGlobalSign);
    }

    else if (c == 'v' || c == 'V') {
      liftSign = (int8_t)(-liftSign);
      Serial.print("[DIR] liftSign = "); Serial.println((int)liftSign);
    }
  }

  // Gait update at fixed rate
  static uint32_t lastGait = 0;
  if (millis() - lastGait >= gaitUpdateMs) {
    lastGait = millis();
    if (walkMode != WalkMode::NONE) gaitUpdate();
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

    const char* modeStr =
      emergencyOff ? "EMOFF" :
      (!armed ? "IDLE" :
       (!standing ? "READY" :
        (walkMode == WalkMode::NONE ? "STAND" :
         (walkMode == WalkMode::FORWARD ? "FWD" :
          (walkMode == WalkMode::SIDE ? "SIDE" : "ROT")))));

    float rollRel = referenceSet ? (rollDeg - roll0) : 0.0f;
    float pitchRel = referenceSet ? (pitchDeg - pitch0) : 0.0f;

    Serial.print(modeStr);
    Serial.print(" | ");
    Serial.print(speedLevel);
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
    Serial.println(pcaAck ? "YES" : "NO");
  }

  delay(2);
}
