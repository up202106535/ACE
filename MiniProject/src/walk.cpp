/**
 * ============================================================
 *  HEXAPOD "BEST PRACTICAL" GAIT FOR 2-DOF LEGS (RP2040 + PCA9685)
 *  - Earle Philhower Arduino-Pico core (Wire + Wire1 supported)
 *  - Two buses:
 *      I2C0 (Wire)  on GP20/GP21 -> IMU + ToF
 *      I2C1 (Wire1) on GP2/GP3   -> PCA9685 (servos)
 *
 *  KEY FIXES (vs the previous ripple version you pasted):
 *   1) Correct ripple phase logic:
 *      - Each leg has its own local phase (swing+stance), not shared
 *      - Stance legs no longer "push" in sync (major cause of yaw/slip)
 *   2) More traction / pressure on stance:
 *      - Baseline press + per-leg stancePress (middle legs press more)
 *   3) Same configuration kept:
 *      - Your servo neutrals / limits
 *      - step >= 4, stride/lift >= 100
 *      - same command keys
 *
 *  Controls (Serial):
 *   - A : ARM
 *   - S : STAND (pre-stand -> neutral)
 *   - Z : Set IMU reference (optional; used only for prints)
 *   - 1 : Toggle WALK FORWARD
 *   - 2 : Toggle WALK SIDE
 *   - 3 : Toggle WALK ROTATE
 *   - 0 : STOP walking (hold neutral)
 *   - E / Q : speed up / speed down (3 levels)
 *   - H : flip global horizontal sign
 *   - V : flip lift sign
 *   - T : swap semantics (forward <-> side)  [useful when axes are rotated]
 *   - R : cycle rotate scheme (LR differential / FR differential)
 *   - X : EMERGENCY OFF (disable PWM outputs)
 *   - G : Recover (re-apply stand, walking OFF)
 *   - M : menu
 * ============================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <VL53L0X.h>

// ---------------- I2C pins / addresses ----------------
static constexpr uint8_t I2C0_SDA = 20;
static constexpr uint8_t I2C0_SCL = 21;

static constexpr uint8_t I2C1_SDA = 2;
static constexpr uint8_t I2C1_SCL = 3;

static constexpr uint8_t IMU_ADDR = 0x68;
static constexpr uint8_t PCA_ADDR = 0x40;

// ---------------- PCA ----------------
Adafruit_PWMServoDriver pca(PCA_ADDR, Wire1);

// ---------------- ToF ----------------
VL53L0X tof;
bool tofOk = false;

// ---------------- IMU (minimal register read; used for printing only) ----------------
bool imuOk = false;
bool referenceSet = false;
float rollDeg = 0.0f, pitchDeg = 0.0f;
float roll0 = 0.0f, pitch0 = 0.0f;
uint32_t lastIMUMs = 0;

// ---------------- Servo calibration ----------------
struct ServoCal {
  uint8_t ch;
  uint16_t minPulse;
  uint16_t neutralPulse;
  uint16_t maxPulse;
  const char* name;
};

// Your confirmed neutrals + wide limits (watch mechanical stops!)
static ServoCal servos[] = {
  { 0, 240, 430, 510, "FR horizontal" },
  { 1, 50, 200, 510, "FR vertical"   },
  { 2, 240, 325, 510, "MR horizontal" },
  { 3, 50, 160, 510, "MR vertical"   },
  { 4, 240, 325, 510, "RR horizontal" },
  { 5, 50, 200, 510, "RR vertical"   },
  { 6, 240, 345, 510, "FL horizontal" },
  { 7, 50, 290, 510, "FL vertical"   },
  { 8, 240, 335, 510, "ML horizontal" },
  { 9, 50, 160, 510, "ML vertical"   },
  {10, 240, 325, 510, "RL horizontal" },
  {11, 50, 230, 510, "RL vertical"   },
};
static constexpr size_t N_SERVOS = sizeof(servos) / sizeof(servos[0]);

// Vertical servo indices (into servos[] array): ch 1,3,5,7,9,11
static constexpr uint8_t VERT_IDX[] = {1, 3, 5, 7, 9, 11};
static constexpr size_t N_VERT = sizeof(VERT_IDX) / sizeof(VERT_IDX[0]);

// ---------------- Legs (6) ----------------
// Order: 0 FR, 1 MR, 2 RR, 3 FL, 4 ML, 5 RL
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

// Last commanded pulses
static uint16_t lastPose[16] = {0};

// Global state
static bool emergencyOff = false;
static bool armed = false;
static bool standing = false;

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

static uint16_t slew(uint16_t current, uint16_t target, uint16_t step) {
  if (current == 0) return target;
  if (current < target) {
    uint32_t next = current + step;
    return (uint16_t)min<uint32_t>(next, target);
  } else {
    int32_t next = (int32_t)current - (int32_t)step;
    return (uint16_t)max<int32_t>(next, (int32_t)target);
  }
}

static void setPulse(uint8_t ch, uint16_t pulse) {
  pca.setPWM(ch, 0, pulse);
}

static void disableChannel(uint8_t ch) {
  pca.setPWM(ch, 0, 4096); // full OFF
}

// Smooth profiles
static float smooth01(float x) { return 0.5f - 0.5f * cosf(PI * x); } // 0..1
static float hump01(float x)   { return sinf(PI * x); }              // 0..1..0

// ============================================================
// Poses (neutral + pre-stand)
// ============================================================
static uint16_t neutralPose[16];
static uint16_t preStandPose[16];

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

static void applyPose(const uint16_t pose[16], uint16_t step, uint16_t stepDelayMs) {
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
// Legs table init
// ============================================================
static const ServoCal* findServo(uint8_t ch) {
  for (size_t i = 0; i < N_SERVOS; i++) if (servos[i].ch == ch) return &servos[i];
  return nullptr;
}

static void initLegs() {
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
// IMU minimal (register-level). Used only for prints/reference.
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

static bool imuInitBasic() {
  if (!ack(Wire, IMU_ADDR)) return false;
  if (!writeReg8(Wire, IMU_ADDR, 0x6B, 0x00)) return false; // wake
  delay(50);
  writeReg8(Wire, IMU_ADDR, 0x1B, 0x08); // gyro ±500 dps
  writeReg8(Wire, IMU_ADDR, 0x1C, 0x08); // accel ±4g
  writeReg8(Wire, IMU_ADDR, 0x1A, 0x04); // DLPF ~ 21Hz
  lastIMUMs = millis();
  return true;
}

static void imuUpdate() {
  static float roll = 0.0f, pitch = 0.0f;

  uint32_t now = millis();
  float dt = (now - lastIMUMs) * 1e-3f;
  if (dt <= 0.0f || dt > 0.5f) dt = 0.01f;
  lastIMUMs = now;

  uint8_t buf[14];
  if (!readBytes(Wire, IMU_ADDR, 0x3B, buf, sizeof(buf))) return;

  int16_t ax = (int16_t)((buf[0] << 8) | buf[1]);
  int16_t ay = (int16_t)((buf[2] << 8) | buf[3]);
  int16_t az = (int16_t)((buf[4] << 8) | buf[5]);
  int16_t gxRaw = (int16_t)((buf[8]  << 8) | buf[9]);
  int16_t gyRaw = (int16_t)((buf[10] << 8) | buf[11]);

  float fax = (float)ax, fay = (float)ay, faz = (float)az;
  float rollAcc  = atan2f(fay, faz) * 180.0f / PI;
  float pitchAcc = atan2f(-fax, sqrtf(fay*fay + faz*faz)) * 180.0f / PI;

  float gx = (float)gxRaw / 65.5f; // deg/s
  float gy = (float)gyRaw / 65.5f;

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
  Serial.println("[IMU] Reference set.");
}

// ============================================================
// WALKING (Corrected Ripple gait + per-leg weights + stance press)
// ============================================================
enum class WalkMode : uint8_t { NONE, FORWARD, SIDE, ROTATE };
static WalkMode walkMode = WalkMode::NONE;

// We want: big motion (your requirement)
static int speedLevel = 0;

// Timing (per full cycle)
static uint16_t cycleMs_table[3] = {1200, 900, 700};  // slow/med/fast

// Stride/Lift >= 100 (your requirement)
static int16_t stepAmp_table[3] = {100, 130, 160};
static int16_t lift_table[3]    = {100, 130, 160};

// Update/servo slew
static uint16_t gaitUpdateMs = 20;   // 50 Hz
static uint16_t gaitSlewStep = 5;    // requested >=4

// Global flips
static int8_t horizGlobalSign = +1;
static int8_t liftSign = +1;

// Semantic swap (forward <-> side)
static bool swapForwardSide = false;

// Rotate scheme selection
enum class RotateScheme : uint8_t { LEFT_RIGHT, FRONT_REAR };
static RotateScheme rotScheme = RotateScheme::LEFT_RIGHT;

// Per-leg weighting (FR,MR,RR,FL,ML,RL)
static float hWeight[6]    = {1.00f, 0.45f, 1.00f, 1.00f, 0.45f, 1.00f};
static float liftWeight[6] = {1.00f, 0.70f, 1.00f, 1.00f, 0.70f, 1.00f};

// Extra stance press (ticks) to keep ground contact
static int16_t stancePress[6] = {20, 35, 20, 20, 35, 20};

// Mode direction maps (tunable)
// NOTE: if forward/side swapped mechanically, press 'T' in runtime.
static int8_t dirForward[6] = {-1, 0, +1, -1, 0, +1};   // FR,MR,RR,FL,ML,RL
static int8_t dirSide[6]    = {+1, +1, +1, -1, -1, -1}; // right +, left -
static int8_t dirRotLR[6]   = {+1, +1, +1, -1, -1, -1}; // rotate LR differential
static int8_t dirRotFR[6]   = {+1, 0, -1, +1, 0, -1};   // rotate FR differential

// Ripple order: one leg swings at a time
static uint8_t rippleOrder[6] = {0, 4, 2, 3, 1, 5}; // FR, ML, RR, FL, MR, RL

static int8_t rippleSlotOfLeg(uint8_t legIdx) {
  for (uint8_t s = 0; s < 6; s++) {
    if (rippleOrder[s] == legIdx) return (int8_t)s;
  }
  return -1;
}

static int8_t getDirForLeg(uint8_t legIdx, WalkMode mode) {
  WalkMode effective = mode;
  if (swapForwardSide) {
    if (mode == WalkMode::FORWARD) effective = WalkMode::SIDE;
    else if (mode == WalkMode::SIDE) effective = WalkMode::FORWARD;
  }

  switch (effective) {
    case WalkMode::FORWARD: return dirForward[legIdx];
    case WalkMode::SIDE:    return dirSide[legIdx];
    case WalkMode::ROTATE:
      return (rotScheme == RotateScheme::LEFT_RIGHT) ? dirRotLR[legIdx] : dirRotFR[legIdx];
    default: return 0;
  }
}

static void gaitRippleUpdate() {
  if (!standing || emergencyOff || walkMode == WalkMode::NONE) return;

  static uint32_t tStart = 0;
  static bool started = false;
  const uint32_t now = millis();
  if (!started) { tStart = now; started = true; }

  const uint16_t cycleMs    = cycleMs_table[speedLevel];
  const int16_t  baseStride = stepAmp_table[speedLevel];
  const int16_t  baseLift   = lift_table[speedLevel];

  // Global phase 0..1
  const uint32_t t = (now - tStart) % cycleMs;
  const float phase01 = (float)t / (float)cycleMs; // 0..1
  const float phase6  = phase01 * 6.0f;            // 0..6

  // Baseline stance pressure (traction). Tune these if it slips/binds.
  const int16_t pressBaseCorner = 15;
  const int16_t pressBaseMiddle = 25;

  // Stance horizontal amplitude scale (reduces fighting & yaw)
  const float stanceScale = 0.70f;

  for (uint8_t i = 0; i < 6; i++) {
    const int8_t mySlot = rippleSlotOfLeg(i);
    if (mySlot < 0) continue;

    // Each leg has its own phase relative to when it swings
    float legPhase = phase6 - (float)mySlot;
    while (legPhase < 0.0f) legPhase += 6.0f;
    while (legPhase >= 6.0f) legPhase -= 6.0f;

    const bool isSwing = (legPhase < 1.0f);

    // local: 0..1 within swing, or 0..1 across full stance duration (5 slots)
    const float local = isSwing ? legPhase : (legPhase - 1.0f) / 5.0f;

    // Direction for this mode
    int8_t dir = getDirForLeg(i, walkMode);
    dir = (int8_t)(dir * horizGlobalSign);

    // weights
    const float hw = hWeight[i];
    const float lw = liftWeight[i];

    int16_t stride = (int16_t)lroundf((float)baseStride * hw);
    int16_t lift   = (int16_t)lroundf((float)baseLift   * lw);
    lift = (int16_t)(lift * liftSign);

    // Stance press (baseline + table)
    const bool isMiddle = (i == 1 || i == 4);
    int16_t press = (int16_t)((isMiddle ? pressBaseMiddle : pressBaseCorner) * liftSign);
    press += (int16_t)(stancePress[i] * liftSign);

    // Horizontal trajectory x in [-1..+1]
    float x;
    if (dir == 0) {
      x = 0.0f;
    } else if (isSwing) {
      // swing: back -> front
      x = (2.0f * smooth01(local)) - 1.0f; // -1..+1
    } else {
      // stance: front -> back
      x = 1.0f - 2.0f * smooth01(local);   // +1..-1
    }

    int16_t hOff = 0;
    if (dir != 0) {
      float ampScale = isSwing ? 1.00f : stanceScale;
      hOff = (int16_t)lroundf(x * (float)stride * ampScale);
      hOff = (int16_t)(hOff * dir);
    }

    // Vertical trajectory
    int16_t vOff = 0;
    if (isSwing) {
      vOff = (int16_t)lroundf(hump01(local) * (float)lift);
    } else {
      vOff = press; // keep contact + traction
    }

    // Targets
    int32_t hPulse = (int32_t)legs[i].hNeutral + (int32_t)hOff;
    int32_t vPulse = (int32_t)legs[i].vNeutral + (int32_t)vOff;

    uint16_t hTarget = clampPulse((uint16_t)max<int32_t>(0, hPulse), legs[i].hMin, legs[i].hMax);
    uint16_t vTarget = clampPulse((uint16_t)max<int32_t>(0, vPulse), legs[i].vMin, legs[i].vMax);

    // Slew-limited write
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

// ============================================================
// Safety / Stand / Emergency
// ============================================================
static void stopWalkingHoldNeutral() {
  walkMode = WalkMode::NONE;
  applyPose(neutralPose, 4, 5);
  standing = true;
  Serial.println("[WALK] STOP. Holding neutral stand.");
}

static void emergencyDisableAll() {
  for (size_t i = 0; i < N_SERVOS; i++) {
    disableChannel(servos[i].ch);
    lastPose[servos[i].ch] = 0;
    delay(2);
  }
  emergencyOff = true;
  walkMode = WalkMode::NONE;
  standing = false;
  Serial.println("[EMERGENCY OFF] PWM disabled. Press 'G' to recover.");
}

static void recoverFromEmergency() {
  emergencyOff = false;
  applyPose(neutralPose, 4, 5);
  standing = true;
  walkMode = WalkMode::NONE;
  Serial.println("[RECOVER] Neutral stand applied. Walking OFF.");
}

static void doStandUp() {
  if (!armed) { Serial.println("[WARN] Not armed. Press 'A' first."); return; }
  if (emergencyOff) { Serial.println("[WARN] Emergency off. Press 'G' to recover."); return; }

  Serial.println("[STAND] Pre-stand...");
  applyPose(preStandPose, 4, 5);
  delay(600);

  Serial.println("[STAND] Neutral stand...");
  applyPose(neutralPose, 4, 5);
  delay(600);

  standing = true;
  Serial.println("[STAND] Done. Press 'Z' for IMU ref (optional), then 1/2/3.");
}

// ============================================================
// UI / Menu
// ============================================================
static void printMenu() {
  Serial.println("\nCommands:");
  Serial.println("  A = ARM");
  Serial.println("  S = STAND UP (pre-stand -> neutral)");
  Serial.println("  Z = Set IMU reference (standing)");
  Serial.println("  1 = Toggle WALK FORWARD");
  Serial.println("  2 = Toggle WALK SIDE");
  Serial.println("  3 = Toggle WALK ROTATE");
  Serial.println("  0 = STOP walk (hold neutral)");
  Serial.println("  E = Faster | Q = Slower");
  Serial.println("  H = Flip horizontal sign");
  Serial.println("  V = Flip lift sign");
  Serial.println("  T = Swap forward <-> side semantics");
  Serial.println("  R = Cycle rotate scheme (LR / FR)");
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

  Serial.println("\n=== HEXAPOD: Ripple Gait (Corrected) ===");

  // I2C0 (IMU + ToF)
  Wire.setSDA(I2C0_SDA);
  Wire.setSCL(I2C0_SCL);
  Wire.begin();
  Wire.setClock(50000);

  // I2C1 (PCA)
  Wire1.setSDA(I2C1_SDA);
  Wire1.setSCL(I2C1_SCL);
  Wire1.begin();
  Wire1.setClock(100000);

  // PCA init
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

  // Build poses (pre-stand offset validated as +25 in your work)
  buildNeutralPose(neutralPose);
  buildPreStandPose(preStandPose, +25);

  // Legs
  initLegs();

  armed = false;
  standing = false;
  emergencyOff = false;
  walkMode = WalkMode::NONE;

  printMenu();

  Serial.println("Status:");
  Serial.println("mode | speed | swap | rotSch | rollRel pitchRel | ToFmm | PCA_ACK");
  Serial.println("-------------------------------------------------------------------");
}

void loop() {
  if (imuOk) imuUpdate();

  // Serial commands
  while (Serial.available()) {
    char c = (char)Serial.read();

    if (c == 'm' || c == 'M') { printMenu(); }

    else if (c == 'a' || c == 'A') {
      armed = true;
      Serial.println("[ARM] OK. Press 'S' to stand.");
    }

    else if (c == 's' || c == 'S') { doStandUp(); }

    else if (c == 'z' || c == 'Z') {
      if (!standing) Serial.println("[WARN] Stand first, then set ref.");
      else imuSetReference();
    }

    else if (c == 'x' || c == 'X') { emergencyDisableAll(); }

    else if (c == 'g' || c == 'G') {
      if (emergencyOff) recoverFromEmergency();
    }

    else if (c == '0') {
      if (standing && !emergencyOff) stopWalkingHoldNeutral();
    }

    else if (c == '1') {
      if (!standing) Serial.println("[WARN] Stand first.");
      else {
        walkMode = (walkMode == WalkMode::FORWARD) ? WalkMode::NONE : WalkMode::FORWARD;
        Serial.println(walkMode == WalkMode::FORWARD ? "[WALK] FORWARD ON" : "[WALK] OFF");
        if (walkMode == WalkMode::NONE) stopWalkingHoldNeutral();
      }
    }

    else if (c == '2') {
      if (!standing) Serial.println("[WARN] Stand first.");
      else {
        walkMode = (walkMode == WalkMode::SIDE) ? WalkMode::NONE : WalkMode::SIDE;
        Serial.println(walkMode == WalkMode::SIDE ? "[WALK] SIDE ON" : "[WALK] OFF");
        if (walkMode == WalkMode::NONE) stopWalkingHoldNeutral();
      }
    }

    else if (c == '3') {
      if (!standing) Serial.println("[WARN] Stand first.");
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
      Serial.print("[DIR] horizGlobalSign="); Serial.println((int)horizGlobalSign);
    }

    else if (c == 'v' || c == 'V') {
      liftSign = (int8_t)(-liftSign);
      Serial.print("[DIR] liftSign="); Serial.println((int)liftSign);
    }

    else if (c == 't' || c == 'T') {
      swapForwardSide = !swapForwardSide;
      Serial.print("[MAP] swapForwardSide="); Serial.println(swapForwardSide ? "ON" : "OFF");
    }

    else if (c == 'r' || c == 'R') {
      rotScheme = (rotScheme == RotateScheme::LEFT_RIGHT) ? RotateScheme::FRONT_REAR : RotateScheme::LEFT_RIGHT;
      Serial.print("[ROT] scheme="); Serial.println(rotScheme == RotateScheme::LEFT_RIGHT ? "LEFT_RIGHT" : "FRONT_REAR");
    }
  }

  // Gait update at fixed rate
  static uint32_t lastGait = 0;
  if (millis() - lastGait >= gaitUpdateMs) {
    lastGait = millis();
    if (walkMode != WalkMode::NONE) gaitRippleUpdate();
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
    Serial.print(swapForwardSide ? "Y" : "N");
    Serial.print(" | ");
    Serial.print(rotScheme == RotateScheme::LEFT_RIGHT ? "LR" : "FR");
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
