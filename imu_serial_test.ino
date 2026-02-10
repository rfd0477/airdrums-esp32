/*
 * ============================================================================
 * MPU6050 DUAL-STICK SERIAL CALIBRATION TEST (ESP32-WROOM)
 * ============================================================================
 *
 * Purpose:
 *   - Stream raw accelerometer and gyroscope data from TWO MPU6050 sensors
 *     over Serial Monitor for calibration and wiring validation.
 *   - Confirms I2C addresses (0x68 and 0x69) and sensor orientation.
 *
 * Wiring (I2C @ 400kHz):
 *   - MPU6050 #1: AD0=LOW  -> 0x68
 *   - MPU6050 #2: AD0=HIGH -> 0x69
 *   - SDA -> GPIO21, SCL -> GPIO22, VCC -> 3.3V, GND -> common GND
 *
 * Usage:
 *   1) Upload this sketch.
 *   2) Open Serial Monitor @ 115200 baud.
 *   3) Keep both sticks still to observe offsets.
 *   4) Move sticks to verify axes and noise levels.
 *
 * Output format (CSV):
 *   time_ms, a1x, a1y, a1z, g1x, g1y, g1z, a2x, a2y, a2z, g2x, g2y, g2z
 *   Units: accel = m/s^2, gyro = rad/s
 *
 * ============================================================================
 */

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

constexpr uint8_t I2C_SDA = 21;
constexpr uint8_t I2C_SCL = 22;
constexpr uint8_t MPU_ADDRESS_1 = 0x68;
constexpr uint8_t MPU_ADDRESS_2 = 0x69;

constexpr uint32_t SAMPLE_PERIOD_MS = 5; // 200 Hz output

Adafruit_MPU6050 mpuLeft;
Adafruit_MPU6050 mpuRight;

void initSensor(Adafruit_MPU6050 &mpu, uint8_t address, const char *label) {
  Serial.print("Initializing ");
  Serial.print(label);
  Serial.print(" at 0x");
  Serial.println(address, HEX);

  int retries = 0;
  while (!mpu.begin(address) && retries < 5) {
    Serial.print(label);
    Serial.println(" not found. Retrying...");
    delay(500);
    retries++;
  }

  if (retries >= 5) {
    Serial.print("ERROR: ");
    Serial.print(label);
    Serial.println(" failed to initialize.");
  } else {
    Serial.print(label);
    Serial.println(" OK");
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  initSensor(mpuLeft, MPU_ADDRESS_1, "MPU6050 #1");
  initSensor(mpuRight, MPU_ADDRESS_2, "MPU6050 #2");

  Serial.println("time_ms,a1x,a1y,a1z,g1x,g1y,g1z,a2x,a2y,a2z,g2x,g2y,g2z");
}

void loop() {
  static uint32_t lastSample = 0;
  uint32_t now = millis();
  if (now - lastSample < SAMPLE_PERIOD_MS) {
    return;
  }
  lastSample = now;

  sensors_event_t a1, g1, t1;
  sensors_event_t a2, g2, t2;
  mpuLeft.getEvent(&a1, &g1, &t1);
  mpuRight.getEvent(&a2, &g2, &t2);

  Serial.print(now);
  Serial.print(',');
  Serial.print(a1.acceleration.x, 4);
  Serial.print(',');
  Serial.print(a1.acceleration.y, 4);
  Serial.print(',');
  Serial.print(a1.acceleration.z, 4);
  Serial.print(',');
  Serial.print(g1.gyro.x, 4);
  Serial.print(',');
  Serial.print(g1.gyro.y, 4);
  Serial.print(',');
  Serial.print(g1.gyro.z, 4);
  Serial.print(',');
  Serial.print(a2.acceleration.x, 4);
  Serial.print(',');
  Serial.print(a2.acceleration.y, 4);
  Serial.print(',');
  Serial.print(a2.acceleration.z, 4);
  Serial.print(',');
  Serial.print(g2.gyro.x, 4);
  Serial.print(',');
  Serial.print(g2.gyro.y, 4);
  Serial.print(',');
  Serial.println(g2.gyro.z, 4);
}
