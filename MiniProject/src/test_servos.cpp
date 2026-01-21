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
// Servo pulse tuning (adjust if needed)
// ============================
// These are PCA "tick" values (0..4095). Safe starting points.
static constexpr uint16_t SERVO_MIN = 200;  // conservative
static constexpr uint16_t SERVO_MID = 375;  // ~1.5ms
static constexpr uint16_t SERVO_MAX = 550;  // conservative

// ============================
// Your channel mapping (test order)
// ============================
static constexpr uint8_t servoChannels[] = {
  0, 1,   // Front right: horiz, vert
  2, 3,   // Middle right: horiz, vert
  4, 5,   // Rear right: horiz, vert
  6, 7,   // Front left: horiz, vert
  8, 9,   // Middle left: horiz, vert
  10, 11  // Rear left: horiz, vert
};

static constexpr size_t NUM_SERVOS =
  sizeof(servoChannels) / sizeof(servoChannels[0]);

static void setPulse(uint8_t ch, uint16_t pulse) {
  pca.setPWM(ch, 0, pulse);
}

// Disables output for a channel (no pulses).
// This stops commanding the servo; the servo may go "limp" depending on mechanics.
static void disableChannel(uint8_t ch) {
  // Full OFF: set OFF bit (bit 12) in LEDn_OFF_H by using value 4096.
  // Adafruit library supports it via setPWM with off = 4096.
  pca.setPWM(ch, 0, 4096);
}

static void testOneServo(uint8_t ch) {
  Serial.print("Testing PCA channel ");
  Serial.println(ch);

  // Always start at neutral
  setPulse(ch, SERVO_MID);
  delay(600);

  // Sweep
  setPulse(ch, SERVO_MIN);
  delay(600);

  setPulse(ch, SERVO_MAX);
  delay(600);

  // Back to neutral
  setPulse(ch, SERVO_MID);
  delay(900);

  Serial.print("Channel ");
  Serial.print(ch);
  Serial.println(" returned to neutral.");
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0 < 3000)) delay(10);

  Serial.println("\n=== PCA9685 Servo Test (Single Run) ===");
  Serial.println("Per channel: MID -> MIN -> MAX -> MID");
  Serial.println("End: set all MID then disable outputs (no more movement)\n");

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

  pca.setPWMFreq(50); // servo frequency
  delay(10);

  Serial.println("PCA9685 initialised at 50 Hz.");

  // Pre-neutral all channels once
  Serial.println("Setting all tested channels to neutral...");
  for (size_t i = 0; i < NUM_SERVOS; i++) {
    setPulse(servoChannels[i], SERVO_MID);
    delay(30);
  }
  delay(800);

  // Run the test once
  for (size_t i = 0; i < NUM_SERVOS; i++) {
    testOneServo(servoChannels[i]);
    delay(300);
  }

  // Finish: neutral + disable outputs
  Serial.println("\nFinishing: neutralising and disabling outputs...");
  for (size_t i = 0; i < NUM_SERVOS; i++) {
    setPulse(servoChannels[i], SERVO_MID);
    delay(30);
  }
  delay(800);

  for (size_t i = 0; i < NUM_SERVOS; i++) {
    disableChannel(servoChannels[i]);
    delay(10);
  }

  Serial.println("Done. Outputs disabled. Program will now idle.");
}

void loop() {
  // Do nothing forever. Outputs already disabled.
  delay(1000);
}
