#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <VL53L0X.h>

// --- I2C Device Addresses ---
#define PWM_ADDR 0x40  // PWM controller (typical)
#define TOF_ADDR 0x29  // VL53L0X Time of Flight
#define IMU_ADDR 0x68  // MPU6050 IMU

// --- Sensors ---
Adafruit_MPU6050 mpu;
VL53L0X tof = VL53L0X();

// --- I2C Bus Configuration ---
// MBED Arduino core ONLY supports: Wire on GPIO 4/5
// LIMITATION: Cannot use GPIO 0/1 or GPIO 14/15 for I2C with MBED
// WORKAROUND: Put PWM controller and sensors on same bus (GPIO 4/5) using different addresses

// --- Euler Angles ---
float roll = 0.0;
float pitch = 0.0;
float yaw = 0.0;

// --- Time ---
unsigned long lastTime = 0;

void setup()
{
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n=== Starting Sensor System ===");
  Serial.println("Using MBED Arduino Core - Single I2C Bus on GPIO 4/5");

  // --- Initialize I2C Bus (Wire - GPIO 4/5 only) ---
  Serial.println("\nInitializing I2C Bus (GPIO 4/5)...");
  Wire.begin();  // MBED only supports this
  delay(500);

  // --- Scan I2C Bus ---
  Serial.println("\nScanning I2C Bus (GPIO 4/5):");
  byte error, address;
  int nDevices = 0;

  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    delay(10);

    if (error == 0) {
      Serial.print("  Device found at 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      Serial.flush();
      nDevices++;
    }
  }

  if (nDevices == 0) {
    Serial.println("  No devices found!");
    Serial.println("  ERROR: Rewire sensors to GPIO 4 (SDA) and GPIO 5 (SCL)");
  } else {
    Serial.print("  Found ");
    Serial.print(nDevices);
    Serial.println(" device(s)");
  }
  Serial.flush();

  // --- Initialize MPU6050 on I2C Bus (GPIO 4/5) ---
  Serial.println("\nInitializing MPU6050 (IMU)...");
  delay(100);
  if (!mpu.begin(IMU_ADDR, &Wire)) {
    Serial.println("  ERROR: MPU6050 not found at 0x68!");
    Serial.println("  IMPORTANT: Rewire to GPIO 4 (SDA) and GPIO 5 (SCL)");
    Serial.println("  MBED Arduino does NOT support GPIO 14/15 for I2C");
  } else {
    Serial.println("  OK - MPU6050 initialized!");
    
    // Configure MPU6050
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }
  Serial.flush();

  // --- Initialize VL53L0X (ToF) on I2C Bus (GPIO 4/5) ---
  Serial.println("\nInitializing VL53L0X (ToF)...");
  delay(100);
  if (!tof.init()) {
    Serial.println("  ERROR: VL53L0X initialization failed!");
  } else {
    Serial.println("  OK - VL53L0X initialized!");
    tof.setTimeout(500);
    tof.startContinuous();
  }
  Serial.flush();

  Serial.println("\n--- System Ready ---");
  Serial.println("Output: Roll | Pitch | Yaw");
  Serial.println("---------------------------\n");
  Serial.flush();
  
  lastTime = millis();
}

void loop()
{
  unsigned long currentMillis = millis();
  
  // --- Read IMU ---
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);
  
  // Calculate delta time (limit to reasonable values)
  float dt = (currentMillis - lastTime) / 1000.0;
  if (dt > 0.5) dt = 0.01;  // Prevent huge jumps on startup
  lastTime = currentMillis;
  
  // Accelerometer (in m/s^2)
  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;
  
  // Gyroscope (in rad/s, convert to deg/s, subtract bias for drift compensation)
  float gx = (g.gyro.x - gyroBiasX) * 180.0 / PI;
  float gy = (g.gyro.y - gyroBiasY) * 180.0 / PI;
  float gz = (g.gyro.z - gyroBiasZ) * 180.0 / PI;
  
  // --- Calculate Euler Angles ---
  
  // Angles from accelerometer only
  float accel_roll = atan2(ay, az) * 180.0 / PI;
  float accel_pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  
  // Integrate gyroscope (with bias compensation)
  roll += gx * dt;
  pitch += gy * dt;
  yaw += gz * dt;
  
  // Complementary filter (98% gyro, 2% accelerometer)
  float alpha = 0.98;
  roll = alpha * roll + (1.0 - alpha) * accel_roll;
  pitch = alpha * pitch + (1.0 - alpha) * accel_pitch;
  // Yaw has no correction (no magnetometer)
  
  // --- Read ToF Sensor ---
  uint16_t distance_mm = tof.readRangeContinuousMillimeters();
  
  // Print all sensor data
  Serial.print("Roll: "); Serial.print(roll, 2);
  Serial.print(" | Pitch: "); Serial.print(pitch, 2);
  Serial.print(" | Yaw: "); Serial.print(yaw, 2);
  Serial.print(" | ToF: ");
  
  if (distance_mm > 0 && distance_mm < 8000) {  // Valid range (8 meters max)
    Serial.print(distance_mm);
    Serial.print(" mm");
  } else {
    Serial.print("Out of range");
  }
  Serial.println();
  Serial.flush();
}

