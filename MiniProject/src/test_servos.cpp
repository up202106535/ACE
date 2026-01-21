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
// Servo configuration per channel
// ============================
// IMPORTANT:
// - These are PCA ticks (0..4095).
// - You MUST tune min/max so the joint never hits mechanical stops.
// - neutral should be the "default pose" for that joint.
// Start with conservative min/max and widen carefully.

struct ServoCal {
  uint8_t ch;
  uint16_t minPulse;
  uint16_t neutralPulse;
  uint16_t maxPulse;
  const char* name;
};

// Your mapping (names just for print clarity)
static ServoCal servos[] = {
  { 0, 220, 350, 480, "FR horizontal" },
  { 1, 50, 350, 400, "FR vertical"   },
  { 2, 220, 350, 480, "MR horizontal" },
  { 3, 50, 350, 400, "MR vertical"   },
  { 4, 220, 350, 480, "RR horizontal" },
  { 5, 50, 350, 400, "RR vertical"   },
  { 6, 220, 350, 480, "FL horizontal" },
  { 7, 50, 350, 400, "FL vertical"   },
  { 8, 220, 350, 480, "ML horizontal" },
  { 9, 50, 350, 400, "ML vertical"   },
  {10, 220, 350, 480, "RL horizontal" },
  {11, 50, 350, 400, "RL vertical"   },
};

static constexpr size_t N = sizeof(servos) / sizeof(servos[0]);

// ============================
// Low-level helpers
// ============================
static void setPulse(uint8_t ch, uint16_t pulse) {
  pca.setPWM(ch, 0, pulse);
}

static void disableChannel(uint8_t ch) {
  // Full OFF (no pulses)
  pca.setPWM(ch, 0, 4096);
}

// Move gradually from currentPulse -> targetPulse
static void rampTo(uint8_t ch, uint16_t currentPulse, uint16_t targetPulse,
                   uint16_t step, uint16_t stepDelayMs) {
  if (currentPulse == targetPulse) return;

  if (targetPulse > currentPulse) {
    for (uint16_t p = currentPulse; p < targetPulse; p = (uint16_t)min<uint32_t>(p + step, targetPulse)) {
      setPulse(ch, p);
      delay(stepDelayMs);
    }
  } else {
    for (uint16_t p = currentPulse; p > targetPulse; p = (uint16_t)max<int32_t>((int32_t)p - (int32_t)step, (int32_t)targetPulse)) {
      setPulse(ch, p);
      delay(stepDelayMs);
    }
  }
}

// Clamp a pulse into [min, max]
static uint16_t clampPulse(uint16_t p, uint16_t lo, uint16_t hi) {
  if (p < lo) return lo;
  if (p > hi) return hi;
  return p;
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0 < 3000)) delay(10);

  Serial.println("\n=== Servo Test (Calibrated + Safe) ===");
  Serial.println("Behaviour:");
  Serial.println("  1) All servos -> neutral (per-channel)");
  Serial.println("  2) For each servo: neutral -> min -> max -> neutral (ramped)");
  Serial.println("  3) End: all -> neutral, then outputs disabled\n");

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

  // Current pulse tracking (assume unknown -> start at neutral)
  uint16_t currentPulse[N];
  for (size_t i = 0; i < N; i++) currentPulse[i] = servos[i].neutralPulse;

  // Step 1: bring everything to neutral (slowly)
  Serial.println("Setting ALL servos to neutral...");
  for (size_t i = 0; i < N; i++) {
    setPulse(servos[i].ch, servos[i].neutralPulse);
    delay(40);
  }
  delay(1200);

  // Step 2: test each servo safely
  const uint16_t step = 4;          // smaller = gentler
  const uint16_t stepDelayMs = 10;  // slower = safer

  for (size_t i = 0; i < N; i++) {
    ServoCal &s = servos[i];

    // Sanity clamp (prevents accidental inverted values)
    uint16_t lo = min(s.minPulse, s.maxPulse);
    uint16_t hi = max(s.minPulse, s.maxPulse);
    uint16_t mid = clampPulse(s.neutralPulse, lo, hi);

    Serial.print("\nTesting CH ");
    Serial.print(s.ch);
    Serial.print(" (");
    Serial.print(s.name);
    Serial.println(")");

    Serial.print("  min="); Serial.print(lo);
    Serial.print(" neutral="); Serial.print(mid);
    Serial.print(" max="); Serial.println(hi);

    // Ensure neutral first
    rampTo(s.ch, currentPulse[i], mid, step, stepDelayMs);
    currentPulse[i] = mid;
    delay(400);

    // Go to min
    rampTo(s.ch, currentPulse[i], lo, step, stepDelayMs);
    currentPulse[i] = lo;
    delay(500);

    // Go to max
    rampTo(s.ch, currentPulse[i], hi, step, stepDelayMs);
    currentPulse[i] = hi;
    delay(500);

    // Back to neutral
    rampTo(s.ch, currentPulse[i], mid, step, stepDelayMs);
    currentPulse[i] = mid;
    delay(700);

    Serial.println("  Returned to neutral.");
  }

  // Step 3: end neutral + disable
  Serial.println("\nFinishing: ALL servos to neutral...");
  for (size_t i = 0; i < N; i++) {
    setPulse(servos[i].ch, servos[i].neutralPulse);
    delay(40);
  }
  delay(1200);

  Serial.println("Disabling PWM outputs (servos stop being commanded)...");
  for (size_t i = 0; i < N; i++) {
    disableChannel(servos[i].ch);
    delay(10);
  }

  Serial.println("Done. Program will now idle.");
}

void loop() {
  delay(1000);
}
