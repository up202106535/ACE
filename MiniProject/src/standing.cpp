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
// Servo calibration structure
// ============================
struct ServoCal {
  uint8_t ch;
  uint16_t minPulse;
  uint16_t neutralPulse;
  uint16_t maxPulse;
  const char* name;
};

// Conservative defaults (you will tune neutralPulse first)
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

static constexpr size_t N = sizeof(servos) / sizeof(servos[0]);

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

static void printTable() {
  Serial.println("\n=== Current neutral standing table (copy-paste) ===");
  Serial.println("CH | min  neutral  max  | name");
  for (size_t i = 0; i < N; i++) {
    Serial.print(servos[i].ch); Serial.print("  | ");
    Serial.print(servos[i].minPulse); Serial.print("  ");
    Serial.print(servos[i].neutralPulse); Serial.print("  ");
    Serial.print(servos[i].maxPulse); Serial.print(" | ");
    Serial.println(servos[i].name);
  }
  Serial.println("==================================================\n");
}

static void applyAllNeutralSlow() {
  Serial.println("[POSE] Moving all servos to NEUTRAL (slow)...");
  const uint16_t step = 3;
  const uint16_t stepDelayMs = 8;

  // We assume current unknown; start by commanding neutrals directly once
  for (size_t i = 0; i < N; i++) {
    uint16_t lo = min(servos[i].minPulse, servos[i].maxPulse);
    uint16_t hi = max(servos[i].minPulse, servos[i].maxPulse);
    uint16_t mid = clampPulse(servos[i].neutralPulse, lo, hi);
    setPulse(servos[i].ch, mid);
    delay(30);
  }
  delay(600);

  // Then do a second pass with ramping from mid->mid (no-op in practice)
  // but keeps the function structure for later improvements.
  (void)step; (void)stepDelayMs;

  Serial.println("[POSE] Neutral pose applied.");
}

// ============================
// Serial tuning interface
// ============================
// Commands:
//   list
//   sel <index 0-11>
//   n <delta>      (adjust neutralPulse by delta, e.g. n +5 or n -5)
//   min <delta>    (adjust minPulse)
//   max <delta>    (adjust maxPulse)
//   go             (apply all neutrals)
//   set            (apply selected servo neutral immediately)
//   print          (print table for copy-paste)
//   help
static int selected = 0;

static void help() {
  Serial.println("\nCommands:");
  Serial.println("  help");
  Serial.println("  list                     (show index -> channel -> name)");
  Serial.println("  sel <i>                  (select servo index 0..11)");
  Serial.println("  n <delta>                (neutralPulse += delta)");
  Serial.println("  min <delta>              (minPulse += delta)");
  Serial.println("  max <delta>              (maxPulse += delta)");
  Serial.println("  set                      (apply selected servo neutral now)");
  Serial.println("  go                       (apply ALL neutrals now)");
  Serial.println("  print                    (print copy-paste table)");
  Serial.println();
}

static void listServos() {
  Serial.println("\nIndex | CH | neutral | name");
  for (size_t i = 0; i < N; i++) {
    Serial.print(i); Serial.print("     | ");
    Serial.print(servos[i].ch); Serial.print("  | ");
    Serial.print(servos[i].neutralPulse); Serial.print("   | ");
    Serial.println(servos[i].name);
  }
  Serial.println();
}

static void applySelectedNeutral() {
  ServoCal &s = servos[selected];
  uint16_t lo = min(s.minPulse, s.maxPulse);
  uint16_t hi = max(s.minPulse, s.maxPulse);
  uint16_t mid = clampPulse(s.neutralPulse, lo, hi);

  setPulse(s.ch, mid);

  Serial.print("[SET] ");
  Serial.print(s.name);
  Serial.print(" (CH ");
  Serial.print(s.ch);
  Serial.print(") neutral=");
  Serial.println(mid);
}

static void clampSelected() {
  ServoCal &s = servos[selected];
  uint16_t lo = min(s.minPulse, s.maxPulse);
  uint16_t hi = max(s.minPulse, s.maxPulse);

  s.neutralPulse = clampPulse(s.neutralPulse, lo, hi);

  // If min/max got inverted by edits, keep consistent ordering by swapping
  if (s.minPulse > s.maxPulse) {
    uint16_t tmp = s.minPulse;
    s.minPulse = s.maxPulse;
    s.maxPulse = tmp;
  }
}

static void handleLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  if (line == "help") { help(); return; }
  if (line == "list") { listServos(); return; }
  if (line == "go") { applyAllNeutralSlow(); return; }
  if (line == "set") { applySelectedNeutral(); return; }
  if (line == "print") { printTable(); return; }

  // sel <i>
  if (line.startsWith("sel ")) {
    int i = line.substring(4).toInt();
    if (i < 0 || i >= (int)N) {
      Serial.println("ERR: sel index must be 0..11");
      return;
    }
    selected = i;
    Serial.print("Selected index "); Serial.print(selected);
    Serial.print(" -> "); Serial.println(servos[selected].name);
    applySelectedNeutral();
    return;
  }

  // n <delta>
  if (line.startsWith("n ")) {
    int d = line.substring(2).toInt();
    servos[selected].neutralPulse = (uint16_t)max<int32_t>(0, (int32_t)servos[selected].neutralPulse + d);
    clampSelected();
    applySelectedNeutral();
    return;
  }

  // min <delta>
  if (line.startsWith("min ")) {
    int d = line.substring(4).toInt();
    servos[selected].minPulse = (uint16_t)max<int32_t>(0, (int32_t)servos[selected].minPulse + d);
    clampSelected();
    applySelectedNeutral();
    return;
  }

  // max <delta>
  if (line.startsWith("max ")) {
    int d = line.substring(4).toInt();
    servos[selected].maxPulse = (uint16_t)max<int32_t>(0, (int32_t)servos[selected].maxPulse + d);
    clampSelected();
    applySelectedNeutral();
    return;
  }

  Serial.println("ERR: unknown command. Type 'help'.");
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0 < 3000)) delay(10);

  Serial.println("\n=== Neutral Standing Pose Tuner ===");
  Serial.println("Goal: tune neutralPulse per servo so robot stands stable without hitting limits.");
  Serial.println("Type 'help' for commands.\n");

  // Init I2C1 + PCA
  Wire1.setSDA(I2C1_SDA);
  Wire1.setSCL(I2C1_SCL);
  Wire1.begin();
  Wire1.setClock(100000);

  if (!pca.begin()) {
    Serial.println("ERROR: PCA9685 not found on Wire1 (0x40).");
    while (true) delay(1000);
  }
  pca.setPWMFreq(50);
  delay(20);

  applyAllNeutralSlow();
  listServos();
  Serial.println("Select a servo with: sel <0..11>");
  Serial.println("Adjust neutral with: n +5  or  n -5");
  Serial.println("When happy: print (copy values into your main config)\n");
}

void loop() {
  // Read serial line commands
  static String line;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (line.length() > 0) {
        handleLine(line);
        line = "";
      }
    } else {
      line += c;
    }
  }

  delay(10);
}
