/*
 * ============================================================================
 * ADVANCED DUAL-STICK AIR DRUM SYSTEM
 * ============================================================================
 *
 * Hardware: ESP32-WROOM + 2× MPU6050 + PAM8403 + RC Filter
 * Features: 8-zone gesture detection, ultra-low latency, velocity mapping
 *
 * WIRING SUMMARY:
 * ---------------
 * Power System:
 *   - Battery: 1S2P Li-ion → TP4056 → Boost Converter (5V)
 *   - ESP32 VIN ← 5V (regulates to 3.3V internally)
 *   - PAM8403 VDD ← 5V
 *
 * Sensors (I²C @ 400kHz):
 *   - MPU6050 #1: AD0=LOW (0x68), SDA=GPIO21, SCL=GPIO22, VCC=3.3V
 *   - MPU6050 #2: AD0=HIGH (0x69), SDA=GPIO21, SCL=GPIO22, VCC=3.3V
 *
 * Audio Path:
 *   - ESP32 GPIO25 (DAC) → 1kΩ resistor → 10nF cap to GND → PAM8403 INL/INR
 *   - PAM8403 OUTL± → 2× 8Ω speakers (parallel)
 *
 * CALIBRATION:
 * ------------
 * 1. Power on system
 * 2. When prompted, hold both sticks VERTICAL and STILL for 3 seconds
 * 3. After calibration completes, hold sticks in neutral drumming position
 * 4. Send 'z' command via Serial Monitor to set zone baseline
 *
 * TUNING GUIDE:
 * -------------
 * - Hit too sensitive? Increase HIT_THRESHOLD (default 3.0g)
 * - Zones switching too easily? Increase SOFT_SWITCH_THRESHOLD
 * - Opposite zones hard to reach? Decrease HARD_SWITCH_THRESHOLD
 * - Zone flickering? Increase ZONE_LOCK_DURATION
 *
 * SERIAL COMMANDS:
 * ----------------
 * c - Recalibrate sensors
 * z - Set zone baseline
 * t - Test mode (10s data logging)
 * +/- - Adjust hit threshold
 * h - Show help
 *
 * AUDIO FILES (upload to SPIFFS):
 * --------------------------------
 * Format: 8-bit mono WAV @ 16kHz
 * Files (default shared set for both sticks):
 *   zone_center.wav, zone_left.wav, zone_right.wav, zone_up.wav, zone_front.wav
 *
 * If you want separate left/right sound sets, set USE_SHARED_ZONES = false and
 * provide left_*.wav and right_*.wav files as noted in loadAudioFiles().
 *
 * ============================================================================
 */

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <MadgwickAHRS.h>
#include <driver/dac.h>
#include <XT_DAC_Audio.h>
#include <SPIFFS.h>

// ============================================================================
// 1. Configuration constants
// ============================================================================

// ========== HARDWARE CONFIGURATION ==========
constexpr uint8_t I2C_SDA = 21;
constexpr uint8_t I2C_SCL = 22;
constexpr uint8_t DAC_PIN = 25;
constexpr uint8_t MPU_ADDRESS_1 = 0x68;
constexpr uint8_t MPU_ADDRESS_2 = 0x69;

// ========== TIMING PARAMETERS ==========
constexpr uint16_t SAMPLE_RATE = 200;          // Hz (5ms loop period)
constexpr uint32_t LOOP_PERIOD_US = 5000;      // Microseconds

// ========== HIT DETECTION ==========
float HIT_THRESHOLD = 3.0f;       // g (minimum acceleration for hit)
float HIT_MAX = 16.0f;            // g (maximum for volume scaling)
uint16_t DEBOUNCE_MS = 30;        // Milliseconds between hits

// ========== ZONE DETECTION (ORIENTATION) ==========
float YAW_ROTATION_THRESHOLD = 45.0f;   // degrees (Z-rotation for FRONT zone)
float ROLL_LEFT_THRESHOLD = -30.0f;     // degrees (X-tilt for LEFT zone)
float ROLL_RIGHT_THRESHOLD = 30.0f;     // degrees (X-tilt for RIGHT zone)
float PITCH_UP_THRESHOLD = 45.0f;       // degrees (Y-tilt for UP zone)

// ========== GESTURE DETECTION ==========
float VELOCITY_THRESHOLD = 2.0f;        // m/s (fast movement detection)
float DISTANCE_THRESHOLD = 0.05f;       // meters (5cm minimum travel)
float ROTATION_THRESHOLD = 200.0f;      // deg/s (Z-axis rotation)
float GESTURE_TIME_WINDOW = 150.0f;     // ms (gesture must complete within)

// ========== SMART ZONE SWITCHING ==========
float SOFT_SWITCH_THRESHOLD = 1.0f;      // Easy switch to adjacent zone
float HARD_SWITCH_THRESHOLD = 2.5f;      // Strong gesture for opposite zone
uint32_t ZONE_LOCK_DURATION = 500;       // ms (prevent rapid zone flickering)

// Gesture strength weights
float SPEED_WEIGHT = 0.4f;
float DURATION_WEIGHT = 0.3f;
float DISTANCE_WEIGHT = 0.3f;

// ========== CALIBRATION ==========
int CALIBRATION_SAMPLES = 600;          // Samples for startup calibration (3 sec @ 200Hz)

// ========== DEBUGGING ==========
bool DEBUG_ENABLED = true;
bool DEBUG_SENSORS = false;
bool DEBUG_FUSION = false;
bool DEBUG_GESTURES = true;
bool DEBUG_ZONES = true;
bool DEBUG_HITS = true;
bool DEBUG_TIMING = false;

// ========== AUDIO ==========
bool AUDIO_ENABLED = true;
bool USE_SHARED_ZONES = true;           // True = both sticks use same sound set
uint8_t VOLUME_MIN = 10;                // Minimum DAC audio volume (0-255)
uint8_t VOLUME_MAX = 100;               // Maximum DAC audio volume
float VELOCITY_CURVE_EXPONENT = 1.2f;   // Volume response curve (1.0 = linear)

// ========== MPU6050 SCALE FACTORS ==========
constexpr float ACCEL_SCALE = 8.0f / 32768.0f * 9.81f;  // g to m/s²
constexpr float GYRO_SCALE = 500.0f / 32768.0f;         // LSB to deg/s

// ============================================================================
// 2. Data structures
// ============================================================================

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
  ZONE_UP = 3,
  ZONE_FRONT = 4,
  ZONE_NONE = 255
};

enum Gesture : uint8_t {
  GESTURE_NONE = 0,
  GESTURE_SWIPE_LEFT = 1,
  GESTURE_SWIPE_RIGHT = 2,
  GESTURE_SWIPE_UP = 3,
  GESTURE_ROTATE_Z = 4
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

struct SmartZoneSwitcher {
  Zone currentZone = ZONE_CENTER;
  uint32_t lastSwitchTime = 0;

  Zone evaluate(Zone baseZone, Gesture gesture, float strength) {
    uint32_t now = millis();
    if (now - lastSwitchTime < ZONE_LOCK_DURATION) {
      return currentZone;
    }

    Zone targetZone = mapGestureToZone(gesture);
    if (targetZone == ZONE_NONE) {
      targetZone = baseZone;
    }

    bool opposite = isOpposite(currentZone, targetZone);
    float threshold = opposite ? HARD_SWITCH_THRESHOLD : SOFT_SWITCH_THRESHOLD;

    if (strength > threshold) {
      currentZone = targetZone;
      lastSwitchTime = now;
    }

    return currentZone;
  }

  static Zone mapGestureToZone(Gesture gesture) {
    switch (gesture) {
      case GESTURE_SWIPE_LEFT:
        return ZONE_LEFT;
      case GESTURE_SWIPE_RIGHT:
        return ZONE_RIGHT;
      case GESTURE_SWIPE_UP:
        return ZONE_UP;
      case GESTURE_ROTATE_Z:
        return ZONE_FRONT;
      default:
        return ZONE_NONE;
    }
  }

  static bool isOpposite(Zone current, Zone target) {
    return (current == ZONE_LEFT && target == ZONE_RIGHT) ||
           (current == ZONE_RIGHT && target == ZONE_LEFT);
  }
};

struct StickState {
  SensorData sensor{};
  Vector3f accelOffset{0.0f, 0.0f, 0.0f};
  Vector3f gyroOffset{0.0f, 0.0f, 0.0f};
  float neutralRoll = 0.0f;
  float neutralPitch = 0.0f;
  float neutralYaw = 0.0f;
  GestureDetector gestureDetector{};
  HitDetector hitDetector{};
  SmartZoneSwitcher zoneSwitcher{};
  float gestureStrength = 0.0f;
};

// ============================================================================
// 3. Global objects
// ============================================================================

Adafruit_MPU6050 mpuLeft;
Adafruit_MPU6050 mpuRight;
Madgwick filterLeft;
Madgwick filterRight;
StickState leftStick;
StickState rightStick;

XT_DAC_Audio_Class DacAudio(DAC_PIN, 0);

// Use a shared set of 5 zones by default. If USE_SHARED_ZONES is false, load
// left/right files (10 total) so each stick can still hit all zones with its
// own sound bank.
XT_Wav_Class zoneCenter("/spiffs/zone_center.wav");
XT_Wav_Class zoneLeft("/spiffs/zone_left.wav");
XT_Wav_Class zoneRight("/spiffs/zone_right.wav");
XT_Wav_Class zoneUp("/spiffs/zone_up.wav");
XT_Wav_Class zoneFront("/spiffs/zone_front.wav");

XT_Wav_Class leftCenter("/spiffs/left_center.wav");
XT_Wav_Class leftLeft("/spiffs/left_left.wav");
XT_Wav_Class leftRight("/spiffs/left_right.wav");
XT_Wav_Class leftUp("/spiffs/left_up.wav");
XT_Wav_Class leftFront("/spiffs/left_front.wav");
XT_Wav_Class rightCenter("/spiffs/right_center.wav");
XT_Wav_Class rightLeft("/spiffs/right_left.wav");
XT_Wav_Class rightRight("/spiffs/right_right.wav");
XT_Wav_Class rightUp("/spiffs/right_up.wav");
XT_Wav_Class rightFront("/spiffs/right_front.wav");

// ============================================================================
// 4. Forward declarations
// ============================================================================

void initSensors();
void initAudio();
void calibrateSensors();
void calibrateZoneBaseline();
void readSensor(Adafruit_MPU6050 &mpu, SensorData &data, const Vector3f &accelOffset,
                const Vector3f &gyroOffset);
void updateFusion(SensorData &data, Madgwick &filter, float dt);
Zone getBaseZone(const StickState &stick);
Gesture detectRotation(float gyroZ);
float calculateGestureStrength(const Vector3f &velocity, const Vector3f &displacement, float durationMs);
void processStick(StickState &stick, Madgwick &filter, float dt);
void triggerAudio(const StickState &stick, bool isLeftStick);
void debugPrint(uint32_t loopTimeUs);
void processSerialCommands();
void printHelp();
void checkI2CBusHealth();
void testMode();

// ============================================================================
// 5. Setup
// ============================================================================

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
  calibrateZoneBaseline();
  printHelp();
}

// ============================================================================
// 6. Main loop
// ============================================================================

void loop() {
  static uint32_t lastUpdate = 0;
  uint32_t now = micros();
  uint32_t loopTimeUs = now - lastUpdate;

  if (loopTimeUs >= LOOP_PERIOD_US) {
    float dt = loopTimeUs / 1000000.0f;
    lastUpdate = now;

    readSensor(mpuLeft, leftStick.sensor, leftStick.accelOffset, leftStick.gyroOffset);
    readSensor(mpuRight, rightStick.sensor, rightStick.accelOffset, rightStick.gyroOffset);

    updateFusion(leftStick.sensor, filterLeft, dt);
    updateFusion(rightStick.sensor, filterRight, dt);

    processStick(leftStick, filterLeft, dt);
    processStick(rightStick, filterRight, dt);

    if (leftStick.hitDetector.hitDetected) {
      triggerAudio(leftStick, true);
    }
    if (rightStick.hitDetector.hitDetected) {
      triggerAudio(rightStick, false);
    }

    debugPrint(loopTimeUs);
    processSerialCommands();
    checkI2CBusHealth();
  }

  // Keep audio buffer flowing (non-blocking)
  if (AUDIO_ENABLED) {
    DacAudio.FillBuffer();
  }
}

// ============================================================================
// 7. Sensor & fusion functions
// ============================================================================

void initSensors() {
  Serial.println("Initializing MPU6050 sensors...");
  int retries = 0;
  while (!mpuLeft.begin(MPU_ADDRESS_1) && retries < 5) {
    Serial.println("MPU6050 #1 (0x68) not found. Retrying...");
    delay(500);
    retries++;
  }
  if (retries >= 5) {
    Serial.println("ERROR: MPU6050 #1 failed to initialize!");
  } else {
    Serial.println("MPU6050 #1 OK");
  }

  retries = 0;
  while (!mpuRight.begin(MPU_ADDRESS_2) && retries < 5) {
    Serial.println("MPU6050 #2 (0x69) not found. Retrying...");
    delay(500);
    retries++;
  }
  if (retries >= 5) {
    Serial.println("ERROR: MPU6050 #2 failed to initialize!");
  } else {
    Serial.println("MPU6050 #2 OK");
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

void updateFusion(SensorData &data, Madgwick &filter, float dt) {
  float ax = data.accelX;
  float ay = data.accelY;
  float az = data.accelZ;
  float gx = data.gyroX;
  float gy = data.gyroY;
  float gz = data.gyroZ;

  filter.updateIMU(gx, gy, gz, ax, ay, az);

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

  data.linearAccelX = ax - gravity.x;
  data.linearAccelY = ay - gravity.y;
  data.linearAccelZ = az - gravity.z;
}

// ============================================================================
// 8. Zone and gesture processing
// ============================================================================

Zone getBaseZone(const StickState &stick) {
  float roll = stick.sensor.roll - stick.neutralRoll;
  float pitch = stick.sensor.pitch - stick.neutralPitch;
  float yaw = stick.sensor.yaw - stick.neutralYaw;

  if (fabsf(yaw) > YAW_ROTATION_THRESHOLD) {
    return ZONE_FRONT;
  }
  if (roll < ROLL_LEFT_THRESHOLD) {
    return ZONE_LEFT;
  }
  if (roll > ROLL_RIGHT_THRESHOLD) {
    return ZONE_RIGHT;
  }
  if (pitch > PITCH_UP_THRESHOLD) {
    return ZONE_UP;
  }
  return ZONE_CENTER;
}

Gesture detectRotation(float gyroZ) {
  if (fabsf(gyroZ) > ROTATION_THRESHOLD * DEG_TO_RAD) {
    return GESTURE_ROTATE_Z;
  }
  return GESTURE_NONE;
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

  Gesture rotationGesture = detectRotation(stick.sensor.gyroZ);
  Gesture swipeGesture = stick.gestureDetector.detect(linearAccel, dt);
  Gesture gesture = (rotationGesture != GESTURE_NONE) ? rotationGesture : swipeGesture;

  float durationMs = millis() - stick.gestureDetector.gestureStart;
  stick.gestureStrength = calculateGestureStrength(stick.gestureDetector.velocity,
                                                   stick.gestureDetector.displacement,
                                                   durationMs);

  Zone baseZone = getBaseZone(stick);
  stick.zoneSwitcher.evaluate(baseZone, gesture, stick.gestureStrength);

  stick.hitDetector.detect(linearAccel);

  if (gesture != GESTURE_NONE) {
    stick.gestureDetector.reset();
  }
}

// ============================================================================
// 9. Audio functions
// ============================================================================

void initAudio() {
  if (!SPIFFS.begin(true)) {
    Serial.println("ERROR: SPIFFS mount failed!");
    Serial.println("Upload WAV files via Arduino IDE: Tools > ESP32 Sketch Data Upload");
    AUDIO_ENABLED = false;
    return;
  }

  if (USE_SHARED_ZONES) {
    if (!SPIFFS.exists("/zone_center.wav")) {
      Serial.println("WARNING: Missing shared zone audio files. Audio disabled.");
      AUDIO_ENABLED = false;
      return;
    }
  } else {
    if (!SPIFFS.exists("/left_center.wav") || !SPIFFS.exists("/right_center.wav")) {
      Serial.println("WARNING: Missing per-stick audio files. Audio disabled.");
      AUDIO_ENABLED = false;
      return;
    }
  }

  Serial.println("Audio system initialized successfully");
}

XT_Wav_Class *getZoneSound(Zone zone, bool isLeftStick) {
  if (USE_SHARED_ZONES) {
    switch (zone) {
      case ZONE_CENTER:
        return &zoneCenter;
      case ZONE_LEFT:
        return &zoneLeft;
      case ZONE_RIGHT:
        return &zoneRight;
      case ZONE_UP:
        return &zoneUp;
      case ZONE_FRONT:
        return &zoneFront;
      default:
        return &zoneCenter;
    }
  }

  if (isLeftStick) {
    switch (zone) {
      case ZONE_CENTER:
        return &leftCenter;
      case ZONE_LEFT:
        return &leftLeft;
      case ZONE_RIGHT:
        return &leftRight;
      case ZONE_UP:
        return &leftUp;
      case ZONE_FRONT:
        return &leftFront;
      default:
        return &leftCenter;
    }
  }

  switch (zone) {
    case ZONE_CENTER:
      return &rightCenter;
    case ZONE_LEFT:
      return &rightLeft;
    case ZONE_RIGHT:
      return &rightRight;
    case ZONE_UP:
      return &rightUp;
    case ZONE_FRONT:
      return &rightFront;
    default:
      return &rightCenter;
  }
}

void triggerAudio(const StickState &stick, bool isLeftStick) {
  if (!AUDIO_ENABLED) {
    return;
  }

  uint8_t dacVolume = map(stick.hitDetector.hitVolume, 0, 255, VOLUME_MIN, VOLUME_MAX);
  XT_Wav_Class *sound = getZoneSound(stick.zoneSwitcher.currentZone, isLeftStick);
  sound->Volume = dacVolume;
  DacAudio.Play(sound);
}

// ============================================================================
// 10. Calibration functions
// ============================================================================

void calibrateSensors() {
  Serial.println("=== CALIBRATION MODE ===");
  Serial.println("Hold both sticks VERTICAL and STILL for 3 seconds...");
  delay(2000);

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

  Serial.println("Calibration complete! Offsets stored.");
}

void calibrateZoneBaseline() {
  Serial.println("Setting zone baseline to current stick positions...");
  leftStick.neutralRoll = leftStick.sensor.roll;
  leftStick.neutralPitch = leftStick.sensor.pitch;
  leftStick.neutralYaw = leftStick.sensor.yaw;

  rightStick.neutralRoll = rightStick.sensor.roll;
  rightStick.neutralPitch = rightStick.sensor.pitch;
  rightStick.neutralYaw = rightStick.sensor.yaw;

  Serial.println("Zone baseline set!");
}

// ============================================================================
// 11. Debug & utility
// ============================================================================

void debugPrint(uint32_t loopTimeUs) {
  if (!DEBUG_ENABLED) {
    return;
  }

  if (DEBUG_ZONES) {
    Serial.print("L_Zone: ");
    Serial.print(leftStick.zoneSwitcher.currentZone);
    Serial.print(" | R_Zone: ");
    Serial.print(rightStick.zoneSwitcher.currentZone);
    Serial.print(" | L_Strength: ");
    Serial.print(leftStick.gestureStrength, 2);
    Serial.print(" | R_Strength: ");
    Serial.println(rightStick.gestureStrength, 2);
  }

  if (DEBUG_HITS) {
    if (leftStick.hitDetector.hitDetected) {
      Serial.print("LEFT HIT! Zone=");
      Serial.print(leftStick.zoneSwitcher.currentZone);
      Serial.print(" Velocity=");
      Serial.print(leftStick.hitDetector.hitVelocity, 2);
      Serial.print(" Volume=");
      Serial.println(leftStick.hitDetector.hitVolume);
    }
    if (rightStick.hitDetector.hitDetected) {
      Serial.print("RIGHT HIT! Zone=");
      Serial.print(rightStick.zoneSwitcher.currentZone);
      Serial.print(" Velocity=");
      Serial.print(rightStick.hitDetector.hitVelocity, 2);
      Serial.print(" Volume=");
      Serial.println(rightStick.hitDetector.hitVolume);
    }
  }

  if (DEBUG_TIMING) {
    Serial.print("Loop time: ");
    Serial.print(loopTimeUs);
    Serial.println(" us");
  }
}

void processSerialCommands() {
  if (!Serial.available()) {
    return;
  }

  char cmd = Serial.read();
  switch (cmd) {
    case 'c':
      calibrateSensors();
      break;
    case 'z':
      calibrateZoneBaseline();
      break;
    case 't':
      testMode();
      break;
    case '+':
      HIT_THRESHOLD += 0.5f;
      Serial.print("HIT_THRESHOLD: ");
      Serial.println(HIT_THRESHOLD);
      break;
    case '-':
      HIT_THRESHOLD -= 0.5f;
      Serial.print("HIT_THRESHOLD: ");
      Serial.println(HIT_THRESHOLD);
      break;
    case 'h':
      printHelp();
      break;
    case 'r':
      ESP.restart();
      break;
    default:
      break;
  }
}

void printHelp() {
  Serial.println("\n=== AIR DRUM CONTROL COMMANDS ===");
  Serial.println("c - Calibrate sensors (hold sticks still)");
  Serial.println("z - Set zone baseline (hold neutral position)");
  Serial.println("t - Enter test mode (10 seconds data logging)");
  Serial.println("+ - Increase hit threshold");
  Serial.println("- - Decrease hit threshold");
  Serial.println("h - Show this help");
  Serial.println("r - Reset system");
}

void checkI2CBusHealth() {
  static uint32_t lastCheck = 0;
  if (millis() - lastCheck > 5000) {
    Wire.beginTransmission(MPU_ADDRESS_1);
    uint8_t error1 = Wire.endTransmission();

    Wire.beginTransmission(MPU_ADDRESS_2);
    uint8_t error2 = Wire.endTransmission();

    if (error1 != 0 || error2 != 0) {
      Serial.println("I2C bus error detected! Resetting bus...");
      Wire.end();
      delay(100);
      Wire.begin(I2C_SDA, I2C_SCL);
      Wire.setClock(400000);
    }
    lastCheck = millis();
  }
}

void testMode() {
  Serial.println("Entering test mode for 10 seconds...");
  uint32_t start = millis();
  while (millis() - start < 10000) {
    Serial.print(leftStick.sensor.linearAccelX, 3);
    Serial.print(',');
    Serial.print(leftStick.sensor.linearAccelY, 3);
    Serial.print(',');
    Serial.print(leftStick.sensor.linearAccelZ, 3);
    Serial.print(',');
    Serial.print(rightStick.sensor.linearAccelX, 3);
    Serial.print(',');
    Serial.print(rightStick.sensor.linearAccelY, 3);
    Serial.print(',');
    Serial.println(rightStick.sensor.linearAccelZ, 3);
    delay(10);
  }
  Serial.println("Test mode complete.");
}

