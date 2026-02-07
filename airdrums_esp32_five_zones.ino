/*
 * ============================================================================
 * ADVANCED DUAL-STICK AIR DRUM SYSTEM (5-ZONE VARIANT)
 * ============================================================================
 *
 * This version keeps CENTER detection the same but simplifies zone switching:
 *   - Move left  -> LEFT zone
 *   - Move right -> RIGHT zone
 *   - Move up while in LEFT/RIGHT -> LEFT_UP / RIGHT_UP
 *   - Move up from CENTER uses the most recent side (lastSide)
 *
 * Zones (shared across both sticks):
 *   0: CENTER
 *   1: LEFT
 *   2: RIGHT
 *   3: LEFT_UP
 *   4: RIGHT_UP
 *
 * Audio files (shared set, LittleFS):
 *   zone_center.wav, zone_left.wav, zone_right.wav, zone_left_up.wav, zone_right_up.wav
 *
 * ============================================================================
 */

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <MadgwickAHRS.h>
#include <driver/dac.h>
#include <XT_DAC_Audio.h>
#include <LittleFS.h>

// ========== HARDWARE CONFIGURATION ==========
constexpr uint8_t I2C_SDA = 21;
constexpr uint8_t I2C_SCL = 22;
constexpr uint8_t DAC_PIN = 25;
constexpr uint8_t MPU_ADDRESS_1 = 0x68;
constexpr uint8_t MPU_ADDRESS_2 = 0x69;

// ========== TIMING PARAMETERS ==========
constexpr uint16_t SAMPLE_RATE = 200;
constexpr uint32_t LOOP_PERIOD_US = 5000;

// ========== HIT DETECTION ==========
float HIT_THRESHOLD = 3.0f;
float HIT_MAX = 16.0f;
uint16_t DEBOUNCE_MS = 30;

// ========== GESTURE DETECTION ==========
float VELOCITY_THRESHOLD = 2.0f;        // m/s
float DISTANCE_THRESHOLD = 0.05f;       // meters
float GESTURE_TIME_WINDOW = 150.0f;     // ms

// ========== SMART ZONE SWITCHING ==========
float SOFT_SWITCH_THRESHOLD = 1.0f;
float HARD_SWITCH_THRESHOLD = 2.5f;
uint32_t ZONE_LOCK_DURATION = 300;      // faster switching for simple zones

// Gesture strength weights
float SPEED_WEIGHT = 0.4f;
float DURATION_WEIGHT = 0.3f;
float DISTANCE_WEIGHT = 0.3f;

// ========== CALIBRATION ==========
int CALIBRATION_SAMPLES = 600;

// ========== DEBUGGING ==========
bool DEBUG_ENABLED = true;
bool DEBUG_ZONES = true;
bool DEBUG_HITS = true;

// ========== AUDIO ==========
bool AUDIO_ENABLED = true;
uint8_t VOLUME_MIN = 10;
uint8_t VOLUME_MAX = 100;
float VELOCITY_CURVE_EXPONENT = 1.2f;

struct Vector3f {
  float x;
  float y;
  float z;
};

struct SensorData {
  float accelX;
  float accelY;
  float accelZ;
  float gyroX;
  float gyroY;
  float gyroZ;
  float linearAccelX;
  float linearAccelY;
  float linearAccelZ;
  float roll;
  float pitch;
  float yaw;
};

enum Zone : uint8_t {
  ZONE_CENTER = 0,
  ZONE_LEFT = 1,
  ZONE_RIGHT = 2,
  ZONE_LEFT_UP = 3,
  ZONE_RIGHT_UP = 4
};

enum Gesture : uint8_t {
  GESTURE_NONE = 0,
  GESTURE_SWIPE_LEFT = 1,
  GESTURE_SWIPE_RIGHT = 2,
  GESTURE_SWIPE_UP = 3
};

struct GestureDetector {
  Vector3f velocity{0.0f, 0.0f, 0.0f};
  Vector3f displacement{0.0f, 0.0f, 0.0f};
  uint32_t gestureStart = 0;

  void reset() {
    velocity = {0.0f, 0.0f, 0.0f};
    displacement = {0.0f, 0.0f, 0.0f};
    gestureStart = millis();
  }

  Gesture detect(const Vector3f &linearAccel, float dt) {
    if (gestureStart == 0) {
      gestureStart = millis();
    }

    velocity.x += linearAccel.x * dt;
    velocity.y += linearAccel.y * dt;
    velocity.z += linearAccel.z * dt;

    displacement.x += velocity.x * dt;
    displacement.y += velocity.y * dt;
    displacement.z += velocity.z * dt;

    if (displacement.x < -DISTANCE_THRESHOLD && velocity.x < -VELOCITY_THRESHOLD) {
      return GESTURE_SWIPE_LEFT;
    }
    if (displacement.x > DISTANCE_THRESHOLD && velocity.x > VELOCITY_THRESHOLD) {
      return GESTURE_SWIPE_RIGHT;
    }
    if (displacement.y > DISTANCE_THRESHOLD && velocity.y > VELOCITY_THRESHOLD) {
      return GESTURE_SWIPE_UP;
    }

    if (millis() - gestureStart > static_cast<uint32_t>(GESTURE_TIME_WINDOW)) {
      reset();
    }

    return GESTURE_NONE;
  }
};

struct HitDetector {
  uint32_t lastHitTime = 0;
  bool hitDetected = false;
  float hitVelocity = 0.0f;
  uint8_t hitVolume = 0;

  void detect(const Vector3f &linearAccel) {
    float magnitude = sqrtf(linearAccel.x * linearAccel.x +
                            linearAccel.y * linearAccel.y +
                            linearAccel.z * linearAccel.z);

    uint32_t now = millis();
    hitDetected = false;

    if (magnitude > HIT_THRESHOLD && (now - lastHitTime) > DEBOUNCE_MS) {
      hitDetected = true;
      hitVelocity = magnitude;
      lastHitTime = now;
      hitVolume = mapVelocityToVolume(magnitude);
    }
  }

  uint8_t mapVelocityToVolume(float magnitude) const {
    float normalized = (magnitude - HIT_THRESHOLD) / (HIT_MAX - HIT_THRESHOLD);
    normalized = constrain(normalized, 0.0f, 1.0f);
    normalized = powf(normalized, VELOCITY_CURVE_EXPONENT);
    return static_cast<uint8_t>(normalized * 255.0f);
  }
};

struct SimpleZoneSwitcher {
  Zone currentZone = ZONE_CENTER;
  Zone lastSide = ZONE_CENTER;
  uint32_t lastSwitchTime = 0;

  Zone evaluate(Gesture gesture, float strength) {
    uint32_t now = millis();
    if (now - lastSwitchTime < ZONE_LOCK_DURATION) {
      return currentZone;
    }

    if (strength < SOFT_SWITCH_THRESHOLD) {
      return currentZone;
    }

    switch (gesture) {
      case GESTURE_SWIPE_LEFT:
        currentZone = ZONE_LEFT;
        lastSide = ZONE_LEFT;
        lastSwitchTime = now;
        break;
      case GESTURE_SWIPE_RIGHT:
        currentZone = ZONE_RIGHT;
        lastSide = ZONE_RIGHT;
        lastSwitchTime = now;
        break;
      case GESTURE_SWIPE_UP:
        if (lastSide == ZONE_RIGHT || currentZone == ZONE_RIGHT) {
          currentZone = ZONE_RIGHT_UP;
          lastSide = ZONE_RIGHT;
          lastSwitchTime = now;
        } else if (lastSide == ZONE_LEFT || currentZone == ZONE_LEFT) {
          currentZone = ZONE_LEFT_UP;
          lastSide = ZONE_LEFT;
          lastSwitchTime = now;
        }
        break;
      default:
        break;
    }

    return currentZone;
  }
};

struct StickState {
  SensorData sensor{};
  Vector3f accelOffset{0.0f, 0.0f, 0.0f};
  Vector3f gyroOffset{0.0f, 0.0f, 0.0f};
  GestureDetector gestureDetector{};
  HitDetector hitDetector{};
  SimpleZoneSwitcher zoneSwitcher{};
  float gestureStrength = 0.0f;
};

Adafruit_MPU6050 mpuLeft;
Adafruit_MPU6050 mpuRight;
Madgwick filterLeft;
Madgwick filterRight;
StickState leftStick;
StickState rightStick;

XT_DAC_Audio_Class DacAudio(DAC_PIN, 0);
XT_Wav_Class zoneCenter("/spiffs/zone_center.wav");
XT_Wav_Class zoneLeft("/spiffs/zone_left.wav");
XT_Wav_Class zoneRight("/spiffs/zone_right.wav");
XT_Wav_Class zoneLeftUp("/spiffs/zone_left_up.wav");
XT_Wav_Class zoneRightUp("/spiffs/zone_right_up.wav");

void initSensors();
void initAudio();
void calibrateSensors();
void readSensor(Adafruit_MPU6050 &mpu, SensorData &data, const Vector3f &accelOffset,
                const Vector3f &gyroOffset);
void updateFusion(SensorData &data, Madgwick &filter);
float calculateGestureStrength(const Vector3f &velocity, const Vector3f &displacement, float durationMs);
void processStick(StickState &stick, Madgwick &filter, float dt);
void triggerAudio(const StickState &stick);
void debugPrint();

void setup() {
  Serial.begin(115200);
  delay(100);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  initSensors();
  filterLeft.begin(SAMPLE_RATE);
  filterRight.begin(SAMPLE_RATE);

  initAudio();
  calibrateSensors();
}

void loop() {
  static uint32_t lastUpdate = 0;
  uint32_t now = micros();
  if (now - lastUpdate < LOOP_PERIOD_US) {
    if (AUDIO_ENABLED) {
      DacAudio.FillBuffer();
    }
    return;
  }

  float dt = (now - lastUpdate) / 1000000.0f;
  lastUpdate = now;

  readSensor(mpuLeft, leftStick.sensor, leftStick.accelOffset, leftStick.gyroOffset);
  readSensor(mpuRight, rightStick.sensor, rightStick.accelOffset, rightStick.gyroOffset);

  updateFusion(leftStick.sensor, filterLeft);
  updateFusion(rightStick.sensor, filterRight);

  processStick(leftStick, filterLeft, dt);
  processStick(rightStick, filterRight, dt);

  if (leftStick.hitDetector.hitDetected) {
    triggerAudio(leftStick);
  }
  if (rightStick.hitDetector.hitDetected) {
    triggerAudio(rightStick);
  }

  debugPrint();

  if (AUDIO_ENABLED) {
    DacAudio.FillBuffer();
  }
}

void initSensors() {
  int retries = 0;
  while (!mpuLeft.begin(MPU_ADDRESS_1) && retries < 5) {
    delay(500);
    retries++;
  }
  retries = 0;
  while (!mpuRight.begin(MPU_ADDRESS_2) && retries < 5) {
    delay(500);
    retries++;
  }

  mpuLeft.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpuLeft.setGyroRange(MPU6050_RANGE_500_DEG);
  mpuLeft.setFilterBandwidth(MPU6050_BAND_21_HZ);

  mpuRight.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpuRight.setGyroRange(MPU6050_RANGE_500_DEG);
  mpuRight.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void readSensor(Adafruit_MPU6050 &mpu, SensorData &data, const Vector3f &accelOffset,
                const Vector3f &gyroOffset) {
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  data.accelX = accel.acceleration.x - accelOffset.x;
  data.accelY = accel.acceleration.y - accelOffset.y;
  data.accelZ = accel.acceleration.z - accelOffset.z;

  data.gyroX = gyro.gyro.x - gyroOffset.x;
  data.gyroY = gyro.gyro.y - gyroOffset.y;
  data.gyroZ = gyro.gyro.z - gyroOffset.z;
}

void updateFusion(SensorData &data, Madgwick &filter) {
  filter.updateIMU(data.gyroX, data.gyroY, data.gyroZ,
                   data.accelX, data.accelY, data.accelZ);

  data.roll = filter.getRoll();
  data.pitch = filter.getPitch();
  data.yaw = filter.getYaw();

  Quaternion q = filter.getQuaternion();
  Vector3f gravity{
    2.0f * (q.x * q.z - q.w * q.y),
    2.0f * (q.w * q.x + q.y * q.z),
    q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z
  };
  gravity.x *= 9.81f;
  gravity.y *= 9.81f;
  gravity.z *= 9.81f;

  data.linearAccelX = data.accelX - gravity.x;
  data.linearAccelY = data.accelY - gravity.y;
  data.linearAccelZ = data.accelZ - gravity.z;
}

float calculateGestureStrength(const Vector3f &velocity, const Vector3f &displacement, float durationMs) {
  float speed = sqrtf(velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z);
  float distance = sqrtf(displacement.x * displacement.x + displacement.y * displacement.y +
                         displacement.z * displacement.z);

  float normalizedSpeed = constrain(speed / 4.0f, 0.0f, 1.0f);
  float normalizedDuration = constrain(durationMs / 300.0f, 0.0f, 1.0f);
  float normalizedDistance = constrain(distance / 0.15f, 0.0f, 1.0f);

  return (normalizedSpeed * SPEED_WEIGHT +
          normalizedDuration * DURATION_WEIGHT +
          normalizedDistance * DISTANCE_WEIGHT) * 3.0f;
}

void processStick(StickState &stick, Madgwick &filter, float dt) {
  Vector3f linearAccel{
    stick.sensor.linearAccelX,
    stick.sensor.linearAccelY,
    stick.sensor.linearAccelZ
  };

  Gesture gesture = stick.gestureDetector.detect(linearAccel, dt);
  float durationMs = millis() - stick.gestureDetector.gestureStart;
  stick.gestureStrength = calculateGestureStrength(stick.gestureDetector.velocity,
                                                   stick.gestureDetector.displacement,
                                                   durationMs);
  stick.zoneSwitcher.evaluate(gesture, stick.gestureStrength);
  stick.hitDetector.detect(linearAccel);

  if (gesture != GESTURE_NONE) {
    stick.gestureDetector.reset();
  }
}

void initAudio() {
  if (!LittleFS.begin(true)) {
    Serial.println("ERROR: LittleFS mount failed!");
    AUDIO_ENABLED = false;
    return;
  }
  if (!LittleFS.exists("/zone_center.wav")) {
    AUDIO_ENABLED = false;
  }
}

XT_Wav_Class *getZoneSound(Zone zone) {
  switch (zone) {
    case ZONE_CENTER:
      return &zoneCenter;
    case ZONE_LEFT:
      return &zoneLeft;
    case ZONE_RIGHT:
      return &zoneRight;
    case ZONE_LEFT_UP:
      return &zoneLeftUp;
    case ZONE_RIGHT_UP:
      return &zoneRightUp;
    default:
      return &zoneCenter;
  }
}

void triggerAudio(const StickState &stick) {
  if (!AUDIO_ENABLED) {
    return;
  }
  uint8_t dacVolume = map(stick.hitDetector.hitVolume, 0, 255, VOLUME_MIN, VOLUME_MAX);
  XT_Wav_Class *sound = getZoneSound(stick.zoneSwitcher.currentZone);
  sound->Volume = dacVolume;
  DacAudio.Play(sound);
}

void calibrateSensors() {
  Vector3f accelOffset[2] = {{0, 0, 0}, {0, 0, 0}};
  Vector3f gyroOffset[2] = {{0, 0, 0}, {0, 0, 0}};

  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    sensors_event_t a1, g1, t1, a2, g2, t2;
    mpuLeft.getEvent(&a1, &g1, &t1);
    mpuRight.getEvent(&a2, &g2, &t2);

    accelOffset[0].x += a1.acceleration.x;
    accelOffset[0].y += a1.acceleration.y;
    accelOffset[0].z += a1.acceleration.z - 9.81f;

    gyroOffset[0].x += g1.gyro.x;
    gyroOffset[0].y += g1.gyro.y;
    gyroOffset[0].z += g1.gyro.z;

    accelOffset[1].x += a2.acceleration.x;
    accelOffset[1].y += a2.acceleration.y;
    accelOffset[1].z += a2.acceleration.z - 9.81f;

    gyroOffset[1].x += g2.gyro.x;
    gyroOffset[1].y += g2.gyro.y;
    gyroOffset[1].z += g2.gyro.z;

    delay(5);
  }

  for (int i = 0; i < 2; i++) {
    accelOffset[i].x /= CALIBRATION_SAMPLES;
    accelOffset[i].y /= CALIBRATION_SAMPLES;
    accelOffset[i].z /= CALIBRATION_SAMPLES;

    gyroOffset[i].x /= CALIBRATION_SAMPLES;
    gyroOffset[i].y /= CALIBRATION_SAMPLES;
    gyroOffset[i].z /= CALIBRATION_SAMPLES;
  }

  leftStick.accelOffset = accelOffset[0];
  leftStick.gyroOffset = gyroOffset[0];
  rightStick.accelOffset = accelOffset[1];
  rightStick.gyroOffset = gyroOffset[1];
}

void debugPrint() {
  if (!DEBUG_ENABLED) {
    return;
  }

  if (DEBUG_ZONES) {
    Serial.print("L_Zone: ");
    Serial.print(leftStick.zoneSwitcher.currentZone);
    Serial.print(" | R_Zone: ");
    Serial.println(rightStick.zoneSwitcher.currentZone);
  }

  if (DEBUG_HITS) {
    if (leftStick.hitDetector.hitDetected) {
      Serial.print("LEFT HIT! Zone=");
      Serial.println(leftStick.zoneSwitcher.currentZone);
    }
    if (rightStick.hitDetector.hitDetected) {
      Serial.print("RIGHT HIT! Zone=");
      Serial.println(rightStick.zoneSwitcher.currentZone);
    }
  }
}
