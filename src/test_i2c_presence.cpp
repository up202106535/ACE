#include <Arduino.h>
#include <Wire.h>

// ============================
// Your wiring (Philhower core)
// ============================
// I2C0 -> Wire : GP20 SDA, GP21 SCL
static constexpr uint8_t I2C0_SDA = 20;
static constexpr uint8_t I2C0_SCL = 21;

// I2C1 -> Wire1: GP2 SDA, GP3 SCL
static constexpr uint8_t I2C1_SDA = 2;
static constexpr uint8_t I2C1_SCL = 3;

// Expected device addresses
static constexpr uint8_t ADDR_TOF = 0x29;
static constexpr uint8_t ADDR_MPU_68 = 0x68;
static constexpr uint8_t ADDR_MPU_69 = 0x69;
static constexpr uint8_t ADDR_PCA = 0x40;

static bool ack(TwoWire& bus, uint8_t addr) {
  bus.beginTransmission(addr);
  return (bus.endTransmission() == 0);
}

static void scan(TwoWire& bus, const char* name) {
  Serial.print("\n=== I2C scan: ");
  Serial.print(name);
  Serial.println(" ===");

  int n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    bus.beginTransmission(a);
    if (bus.endTransmission() == 0) {
      Serial.print("  Found 0x");
      if (a < 16) Serial.print("0");
      Serial.println(a, HEX);
      n++;
    }
    delay(2);
  }

  if (n == 0) Serial.println("  (none)");
  else {
    Serial.print("  Total devices: ");
    Serial.println(n);
  }
}

void setup() {
  Serial.begin(115200);

  // Give the monitor time to attach
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0 < 3000)) delay(10);

  Serial.println("\n=== Dual I2C Presence Test ===");

  // ---------- I2C0: Wire on GP20/GP21 ----------
  Wire.setSDA(I2C0_SDA);
  Wire.setSCL(I2C0_SCL);
  Wire.begin();
  Wire.setClock(100000); // robust on breadboards

  // ---------- I2C1: Wire1 on GP2/GP3 ----------
  Wire1.setSDA(I2C1_SDA);
  Wire1.setSCL(I2C1_SCL);
  Wire1.begin();
  Wire1.setClock(100000);

  scan(Wire,  "Wire  (I2C0 GP20/21)");
  scan(Wire1, "Wire1 (I2C1 GP2/3)");

  Serial.println("\nACK summary (expected YES):");
  Serial.print("  ToF 0x29 on Wire  : "); Serial.println(ack(Wire,  ADDR_TOF) ? "YES" : "NO");
  Serial.print("  MPU 0x68 on Wire  : "); Serial.println(ack(Wire,  ADDR_MPU_68) ? "YES" : "NO");
  Serial.print("  MPU 0x69 on Wire  : "); Serial.println(ack(Wire,  ADDR_MPU_69) ? "YES" : "NO");
  Serial.print("  PCA 0x40 on Wire1 : "); Serial.println(ack(Wire1, ADDR_PCA) ? "YES" : "NO");

  Serial.println("\nIf something is NO:");
  Serial.println("  - Check SDA/SCL on the correct pins for that bus");
  Serial.println("  - Check VCC=3.3V and GND common");
  Serial.println("  - PCA: ensure VCC (logic) is powered; V+ not required for I2C");
}

void loop() {
  // Re-scan every 3 seconds so you can hot-fix wiring and see it change
  static uint32_t last = 0;
  if (millis() - last >= 3000) {
    last = millis();
    scan(Wire,  "Wire  (I2C0 GP20/21)");
    scan(Wire1, "Wire1 (I2C1 GP2/3)");
  }
}
