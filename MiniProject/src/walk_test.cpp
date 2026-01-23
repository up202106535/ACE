#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ============================
// PCA9685 (Wire1)
// ============================
static constexpr uint8_t I2C1_SDA = 2;
static constexpr uint8_t I2C1_SCL = 3;
static constexpr uint8_t PCA_ADDR = 0x40;
int cont = 0;

Adafruit_PWMServoDriver pca(PCA_ADDR, Wire1);

// ============================
// Servo structure
// ============================
struct Servo {
  uint8_t ch;
  uint16_t minP;
  uint16_t neutralP;
  uint16_t maxP;
};

typedef struct {
  int state, new_state;
  unsigned long tes, tis;
} fsm_t;

fsm_t fsm1;

enum {
  sm1_initial,
  sm1_walking,
};

void set_state(fsm_t& fsm, int new_state)
{
  if (fsm.state != new_state) {
    fsm.state = new_state;
    fsm.tes = millis();
    fsm.tis = 0;
  }
}

// ============================
// Servo map (YOUR VALUES)
// ============================
Servo servos[] = {
  { 0, 240, 430, 480 },
  { 1, 50, 200, 510  },
  { 2, 240, 325, 400 },
  { 3, 50, 160, 510  },
  { 4, 240, 325, 450 },
  { 5, 50, 200, 510  },
  { 6, 240, 345, 510 },
  { 7, 50, 290, 510  },
  { 8, 240, 335, 510 },
  { 9, 50, 160, 510  },
  {10, 240, 325, 510 },
  {11, 50, 230, 510  },
};

// ============================
// Helpers
// ============================
inline void setMin(uint8_t i)     { pca.setPWM(servos[i].ch, 0, servos[i].maxP); }
inline void setNeutral(uint8_t i) { pca.setPWM(servos[i].ch, 0, servos[i].neutralP); }
inline void setMax(uint8_t i)     { pca.setPWM(servos[i].ch, 0, servos[i].minP); }

// ============================
// Vertical semantics (EXPLICIT)
// ============================
inline void vertUP(uint8_t leg)   { setNeutral(leg + 1); }
inline void vertDOWN(uint8_t leg) { setMin(leg + 1); }
inline void vertNEUTRAL(uint8_t leg) { setNeutral(leg + 1); }

// ============================
// Leg helpers (index = horizontal)
// ============================
void legStand(uint8_t leg) {
  setNeutral(leg);
  vertDOWN(leg);
}

void legLift(uint8_t leg) {
  vertNEUTRAL(leg);
}

void legSwingForward(uint8_t leg) {
  setMax(leg);
}

void legSwingBackward(uint8_t leg) {
  setMin(leg);
}

// ============================
// Tripod definitions
// ============================
const uint8_t tripodA[] = { 0, 8, 4 };
const uint8_t tripodB[] = { 6, 2, 10 };

// ============================
// Timing
// ============================
const uint16_t STEP_DELAY = 300;

// ============================
// Poses
// ============================
void armPose() {
  Serial.println("ARM");
  for (uint8_t i = 0; i < 12; i += 2) {
    setNeutral(i);
  }
}

void standPose() {
  Serial.println("STAND");
  for (uint8_t i = 0; i < 12; i += 2) {
    vertNEUTRAL(i);
  }
}

// ============================
// Commands
// ============================
static bool cmdArmed = false;
static bool cmdStart = false;
static bool holdApplied = false;

void applyHoldStandOnce() {
  if (holdApplied) return;
  standPose();
  holdApplied = true;
}

void handleSerialCommands() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  cmd.toLowerCase();

  if (cmd == "arm") {
    cmdArmed = true;
    cmdStart = false;
    holdApplied = false;
    Serial.println("ARMED");
    armPose();
    return;
  }

  if (cmd == "stand up" || cmd == "standup") {
    if (!cmdArmed) { Serial.println("IGNORED: not armed"); return; }
    Serial.println("STAND UP REQUESTED");
    cont = 0;
    set_state(fsm1, sm1_initial);
    holdApplied = false;
    return;
  }

  if (cmd == "start") {
    if (!cmdArmed) { Serial.println("IGNORED: not armed"); return; }
    cmdStart = true;
    holdApplied = false;
    Serial.println("STARTED");
    return;
  }

  if (cmd == "stop") {
    if (!cmdArmed) { Serial.println("IGNORED: not armed"); return; }
    cmdStart = false;
    holdApplied = false;
    Serial.println("STOPPED");
    return;
  }

  Serial.println("UNKNOWN CMD");
}

// ============================
// Stand up
// ============================
void standUp() {
  Serial.println("Standing up...");
  standPose();
  delay(600);
  cont++;
}

// ============================
// Walking phase (MUST be global / persistent)
// ============================
enum { WALK_A, WALK_B };
uint8_t walkPhase = WALK_A;

// ============================
// Arduino setup / loop
// ============================
void setup() {
  Serial.begin(115200);

  set_state(fsm1, sm1_initial);

  Wire1.setSDA(I2C1_SDA);
  Wire1.setSCL(I2C1_SCL);
  Wire1.begin();

  if (!pca.begin()) {
    Serial.println("PCA9685 not found!");
    while (1);
  }

  pca.setPWMFreq(50);
  delay(20);
}

void loop() {

  handleSerialCommands();

  if (!cmdStart) {
    applyHoldStandOnce();
    return;
  }

  // IMPORTANT: always initialise new_state
  fsm1.new_state = fsm1.state;

  if (fsm1.state == sm1_initial && cont >= 1) {
    fsm1.new_state = sm1_walking;
  }

  set_state(fsm1, fsm1.new_state);

  if (fsm1.state == sm1_initial) {
    standUp();
  }

  if (fsm1.state == sm1_walking && cmdStart) {

    if (walkPhase == WALK_A) {
      legSwingForward(tripodA[0]);
      legSwingBackward(tripodA[1]);
      legSwingForward(tripodA[2]);
      walkPhase = WALK_B;
    } else {
      legSwingForward(tripodB[0]);
      legSwingBackward(tripodB[1]);
      legSwingForward(tripodB[2]);
      walkPhase = WALK_A;
    }

    delay(STEP_DELAY);
  }
}
