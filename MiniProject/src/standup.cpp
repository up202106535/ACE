#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ============================
// PCA9685 on I2C1 (Wire1)
// ============================
static constexpr uint8_t I2C1_SDA = 2;
static constexpr uint8_t I2C1_SCL = 3;
static constexpr uint8_t PCA_ADDR = 0x40;

Adafruit_PWMServoDriver pca(PCA_ADDR, Wire1);

// ============================
// Servo calibration (SAFE min/max + your confirmed neutrals)
// ============================
struct ServoCal {
  uint8_t ch;
  uint16_t minPulse;
  uint16_t neutralPulse;
  uint16_t maxPulse;
  const char* name;
};

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

static constexpr size_t N = sizeof(servos) / sizeof(servos[0]);

// Indices of vertical servos in the array above (channels 1,3,5,7,9,11)
static constexpr uint8_t VERT_IDX[] = {1, 3, 5, 7, 9, 11};
static constexpr size_t NV = sizeof(VERT_IDX) / sizeof(VERT_IDX[0]);

// ============================
// Helpers
// ============================
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

static void applyPose(const uint16_t pose[16], const uint16_t step = 3, const uint16_t stepDelayMs = 8) {
  // For channels not used, pose[ch] can be 0 -> skip
  // We ramp from current to target by reading "current" from our own lastPose cache (see below)
  extern uint16_t lastPose[16];

  for (uint8_t ch = 0; ch < 16; ch++) {
    uint16_t target = pose[ch];
    if (target == 0) continue;

    uint16_t from = lastPose[ch];
    if (from == 0) {
      // first time: jump to target then consider it current
      setPulse(ch, target);
      lastPose[ch] = target;
      delay(20);
    } else {
      rampTo(ch, from, target, step, stepDelayMs);
      lastPose[ch] = target;
    }
  }
}

// Last commanded pulse per PCA channel (0 means unknown)
uint16_t lastPose[16] = {0};

// Build pose arrays
static void buildNeutralPose(uint16_t pose[16]) {
  memset(pose, 0, 16 * sizeof(uint16_t));
  for (size_t i = 0; i < N; i++) {
    uint16_t lo = min(servos[i].minPulse, servos[i].maxPulse);
    uint16_t hi = max(servos[i].minPulse, servos[i].maxPulse);
    pose[servos[i].ch] = clampPulse(servos[i].neutralPulse, lo, hi);
  }
}

// “Pre-stand” = same as neutral, but vertical servos offset by a small amount to lift/relieve contact.
// NOTE: direction depends on your mechanics. We start conservatively and let you flip it if needed.
static void buildPreStandPose(uint16_t pose[16], int16_t verticalOffset) {
  buildNeutralPose(pose);

  for (size_t k = 0; k < NV; k++) {
    uint8_t idx = VERT_IDX[k];

    uint8_t ch = servos[idx].ch;
    uint16_t lo = min(servos[idx].minPulse, servos[idx].maxPulse);
    uint16_t hi = max(servos[idx].minPulse, servos[idx].maxPulse);

    int32_t p = (int32_t)pose[ch] + (int32_t)verticalOffset;
    pose[ch] = clampPulse((uint16_t)max<int32_t>(0, p), lo, hi);
  }
}

static void printPose(const char* title, const uint16_t pose[16]) {
  Serial.println(title);
  for (size_t i = 0; i < N; i++) {
    uint8_t ch = servos[i].ch;
    Serial.print("  CH "); Serial.print(ch);
    Serial.print(" = "); Serial.print(pose[ch]);
    Serial.print("  ("); Serial.print(servos[i].name); Serial.println(")");
  }
}

// ============================
// Program flow
// ============================
enum class State : uint8_t {
  BOOT,
  PRESTAND,
  STAND,
  HOLD
};
static State state = State::BOOT;

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0 < 3000)) delay(10);

  Serial.println("\n=== Stand-Up Test ===");
  Serial.println("Sequence: PRE-STAND -> NEUTRAL STAND -> HOLD");
  Serial.println("Type 'Z' after standing to indicate you're ready to zero IMU in the integrated program.\n");

  // Init I2C1
  Wire1.setSDA(I2C1_SDA);
  Wire1.setSCL(I2C1_SCL);
  Wire1.begin();
  Wire1.setClock(100000);

  // Init PCA
  if (!pca.begin()) {
    Serial.println("ERROR: PCA9685 not found on Wire1 (0x40).");
    while (true) delay(1000);
  }
  pca.setPWMFreq(50);
  delay(20);

  // Build poses
  uint16_t neutral[16];
  uint16_t prestand[16];

  buildNeutralPose(neutral);

  // Vertical offset: start small to avoid hitting limits.
  // If this moves the legs the wrong way (pushes into ground), flip sign.
  const int16_t verticalOffset = +25;
  buildPreStandPose(prestand, verticalOffset);

  printPose("[POSE] Pre-stand targets:", prestand);
  printPose("[POSE] Neutral stand targets:", neutral);

  Serial.println("\nApplying PRE-STAND...");
  applyPose(prestand, 3, 8);
  delay(800);

  Serial.println("Transitioning to NEUTRAL STAND...");
  applyPose(neutral, 3, 8);
  delay(800);

  Serial.println("Standing pose applied. HOLDING.");
  state = State::HOLD;
}

void loop() {
  // Hold pose by keeping PWM active; nothing else required.
  // Provide a small serial interaction to support your workflow.
  if (Serial.available()) {
    char c = (char)Serial.read();
    if (c == 'z' || c == 'Z') {
      Serial.println("[WORKFLOW] Standing confirmed. In the integrated program, this is where we will zero the IMU reference.");
    }
  }
  delay(20);
}
