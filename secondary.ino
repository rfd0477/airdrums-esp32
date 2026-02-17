/*
 * ADVANCED DUAL-STICK AIR DRUM SYSTEM (ESP32-WROOM)
 *
 * FIXED + EXTENDED from prototype:
 * - Dual MPU6050 sticks (0x68, 0x69)
 * - LittleFS + XT_DAC_Audio DAC playback
 * - Proper gyro integration with dt
 * - Band-threshold zone selection with no gaps
 * - Linear-accel hit detection with debounce
 * - NVS calibration persistence
 */

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <MadgwickAHRS.h>
#include <XT_DAC_Audio.h>
#include <LittleFS.h>
#include <Preferences.h>

#define WAV_PATH(path) reinterpret_cast<const unsigned char *>(path)

// ===== HARDWARE =====
#define DAC_PIN   25
#define I2C_SDA   21
#define I2C_SCL   22
#define MPU_LEFT  0x68
#define MPU_RIGHT 0x69

// ===== TIMING =====
#define SAMPLE_HZ        200
#define LOOP_PERIOD_US   5000

// ===== HIT DETECTION =====
float HIT_THRESHOLD_G  = 3.0f;    // FIX #8/#9: accel-based hit threshold
float HIT_MAX_G        = 16.0f;
#define DEBOUNCE_MS      30

// ===== ZONE THRESHOLDS =====
#define ZONE_THRESH_FAR   60.0f
#define ZONE_THRESH_MID   30.0f
#define ZONE_DEADBAND      2.0f

// ===== DRIFT CORRECTION =====
#define ANGLE_DECAY_RATE  0.995f
#define STILL_GYRO_THRESH 0.5f

// ===== CALIBRATION =====
#define CALIBRATION_SAMPLES 500

// ===== AUDIO =====
#define VOLUME_MIN  10
#define VOLUME_MAX  100

// ===== DEBUG =====
#define DEBUG_ENABLED  true
#define DEBUG_ZONES    true
#define DEBUG_HITS     true
#define DEBUG_TIMING   false

enum Zone5 : uint8_t {
  Z_CENTER = 0,
  Z_LEFT = 1,
  Z_RIGHT = 2,
  Z_TOPLEFT = 3,
  Z_TOPRIGHT = 4
};

struct SensorData {
  int16_t rawGyroX = 0;
  int16_t rawGyroY = 0;
  int16_t rawGyroZ = 0;

  float gyroX_dps = 0.0f;
  float gyroY_dps = 0.0f;
  float gyroZ_dps = 0.0f;

  float accelX = 0.0f; // m/s^2
  float accelY = 0.0f;
  float accelZ = 0.0f;

  float linX = 0.0f;
  float linY = 0.0f;
  float linZ = 0.0f;

  float roll = 0.0f;
  float pitch = 0.0f;
  float yaw = 0.0f;
};

struct CalibrationStick {
  float gyroOffsetX = 0.0f;
  float gyroOffsetY = 0.0f;
  float gyroOffsetZ = 0.0f;
  float accelOffsetX = 0.0f;
  float accelOffsetY = 0.0f;
  float accelOffsetZ = 0.0f;
  float neutralAngleZ = 0.0f;
};

struct PersistCal {
  CalibrationStick left;
  CalibrationStick right;
  uint32_t magic;
};

struct StickState {
  uint8_t address;
  SensorData data;
  CalibrationStick cal;
  Madgwick fusion;

  float angleZ = 0.0f;      // integrated zone angle (degrees)
  Zone5 zone = Z_CENTER;
  uint32_t lastHitMs = 0;
  bool hit = false;
  uint8_t hitVolume = 0;
};

Adafruit_MPU6050 mpuLeft;
Adafruit_MPU6050 mpuRight;
StickState leftStick{MPU_LEFT};
StickState rightStick{MPU_RIGHT};

Preferences prefs;
bool audioEnabled = true;

XT_DAC_Audio_Class DacAudio(DAC_PIN, 0);

// 10 zones total (5 per stick)
XT_Wav_Class wavLeftCenter(WAV_PATH("/left_center.wav"));
XT_Wav_Class wavLeftLeft(WAV_PATH("/left_left.wav"));
XT_Wav_Class wavLeftRight(WAV_PATH("/left_right.wav"));
XT_Wav_Class wavLeftTopLeft(WAV_PATH("/left_topleft.wav"));
XT_Wav_Class wavLeftTopRight(WAV_PATH("/left_topright.wav"));

XT_Wav_Class wavRightCenter(WAV_PATH("/right_center.wav"));
XT_Wav_Class wavRightLeft(WAV_PATH("/right_left.wav"));
XT_Wav_Class wavRightRight(WAV_PATH("/right_right.wav"));
XT_Wav_Class wavRightTopLeft(WAV_PATH("/right_topleft.wav"));
XT_Wav_Class wavRightTopRight(WAV_PATH("/right_topright.wav"));

XT_Wav_Class* leftZones[5]  = {&wavLeftCenter, &wavLeftLeft, &wavLeftRight, &wavLeftTopLeft, &wavLeftTopRight};
XT_Wav_Class* rightZones[5] = {&wavRightCenter, &wavRightLeft, &wavRightRight, &wavRightTopLeft, &wavRightTopRight};

float hitThresholdMS2() { return HIT_THRESHOLD_G * 9.81f; }
float hitMaxMS2() { return HIT_MAX_G * 9.81f; }

// -------- Function declarations (keep design intent separation) --------
void setupMPU(uint8_t address);
bool recordGyroRegisters(uint8_t address, SensorData &data);
void processGyroData(StickState &s, float dt);
bool readSensorRegisters(Adafruit_MPU6050 &mpu, StickState &s);
void processSensorData(StickState &s, float dt);
void updateAngleIntegration(StickState &s, float dt);
void updateZone(StickState &s);
void detectHit(StickState &s);
void triggerAudio(const StickState &s, bool isLeft);
void calibrateOffsets();
void calibrateZoneBaseline();
bool loadCalibrationFromNVS();
void saveCalibrationToNVS();
void handleSerialCommands();
void debugPrint(uint32_t loopUs);

void setup() {
  Serial.begin(115200); // FIX #13
  delay(50);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  Wire.setTimeOut(5); // FIX #11: I2C timeout guard

  // init Adafruit wrappers
  if (!mpuLeft.begin(MPU_LEFT)) Serial.println("ERROR: MPU_LEFT missing");
  if (!mpuRight.begin(MPU_RIGHT)) Serial.println("ERROR: MPU_RIGHT missing");

  // keep original-style register setup function
  setupMPU(MPU_LEFT);
  setupMPU(MPU_RIGHT);

  leftStick.fusion.begin(SAMPLE_HZ);
  rightStick.fusion.begin(SAMPLE_HZ);

  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS fail - silent mode");
    audioEnabled = false;
  }

  // FIX #2: audio settings flow is not unreachable; set usable defaults after init attempt
  if (audioEnabled) {
    const char *files[] = {
      "/left_center.wav", "/left_left.wav", "/left_right.wav", "/left_topleft.wav", "/left_topright.wav",
      "/right_center.wav", "/right_left.wav", "/right_right.wav", "/right_topleft.wav", "/right_topright.wav"
    };
    for (const char *f : files) {
      if (!LittleFS.exists(f)) {
        Serial.print("Missing WAV: "); Serial.println(f);
        audioEnabled = false;
      }
    }
  }

  if (!loadCalibrationFromNVS()) {
    calibrateOffsets();
    calibrateZoneBaseline();
    saveCalibrationToNVS();
  }

  Serial.println("Ready. Commands: c z t + - h r");
}

void loop() {
  static uint32_t lastLoopUs = 0;
  uint32_t nowUs = micros();
  uint32_t loopUs = nowUs - lastLoopUs;

  if (loopUs >= LOOP_PERIOD_US) {
    float dt = loopUs / 1000000.0f;
    lastLoopUs = nowUs;

    // FIX #10: accel is actually read and used
    bool lOk = readSensorRegisters(mpuLeft, leftStick);
    bool rOk = readSensorRegisters(mpuRight, rightStick);

    if (lOk) {
      processSensorData(leftStick, dt);
      updateAngleIntegration(leftStick, dt); // FIX #3/#5 proper integration with dt
      updateZone(leftStick);                  // FIX #6/#7 else-if, no gaps
      detectHit(leftStick);                   // FIX #8/#9 accel-based debounce
      if (leftStick.hit) triggerAudio(leftStick, true);
    }

    if (rOk) {
      processSensorData(rightStick, dt);
      updateAngleIntegration(rightStick, dt);
      updateZone(rightStick);
      detectHit(rightStick);
      if (rightStick.hit) triggerAudio(rightStick, false);
    }

    debugPrint(loopUs);
    handleSerialCommands();
  }

  if (audioEnabled) {
    DacAudio.FillBuffer(); // non-blocking, no delay in main loop
  }
}

void setupMPU(uint8_t address) {
  // Keep raw register style from original design
  Wire.beginTransmission(address);
  Wire.write(0x6B);
  Wire.write(0x00); // wake
  Wire.endTransmission();

  Wire.beginTransmission(address);
  Wire.write(0x1B);
  Wire.write(0x18); // ±2000 dps
  Wire.endTransmission();

  Wire.beginTransmission(address);
  Wire.write(0x1C);
  Wire.write(0x18); // ±16g
  Wire.endTransmission();
}

bool recordGyroRegisters(uint8_t address, SensorData &data) {
  Wire.beginTransmission(address);
  Wire.write(0x43);
  if (Wire.endTransmission(false) != 0) return false;

  Wire.requestFrom((int)address, 6);

  // FIX #11: timeout, no infinite spin
  uint32_t t0 = millis();
  while (Wire.available() < 6) {
    if (millis() - t0 > 5) {
      Serial.print("I2C timeout gyro @0x");
      Serial.println(address, HEX);
      return false;
    }
  }

  data.rawGyroX = (Wire.read() << 8) | Wire.read();
  data.rawGyroY = (Wire.read() << 8) | Wire.read();
  data.rawGyroZ = (Wire.read() << 8) | Wire.read();
  return true;
}

bool readSensorRegisters(Adafruit_MPU6050 &mpu, StickState &s) {
  if (!recordGyroRegisters(s.address, s.data)) {
    return false;
  }

  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);
  s.data.accelX = a.acceleration.x;
  s.data.accelY = a.acceleration.y;
  s.data.accelZ = a.acceleration.z;
  return true;
}

void processGyroData(StickState &s, float dt) {
  (void)dt;
  // FIX #4: correct ±2000 dps scale = 16.4 LSB/(dps)
  const float GYRO_SCALE = 16.4f;

  // FIX #12: calibrated offsets, no hardcoded magic constants
  s.data.gyroX_dps = ((float)s.data.rawGyroX / GYRO_SCALE) - s.cal.gyroOffsetX;
  s.data.gyroY_dps = ((float)s.data.rawGyroY / GYRO_SCALE) - s.cal.gyroOffsetY;
  s.data.gyroZ_dps = ((float)s.data.rawGyroZ / GYRO_SCALE) - s.cal.gyroOffsetZ;

  // small deadzone for stillness
  if (fabsf(s.data.gyroZ_dps) < 0.05f) s.data.gyroZ_dps = 0.0f;
}

void processSensorData(StickState &s, float dt) {
  processGyroData(s, dt);

  float ax = s.data.accelX - s.cal.accelOffsetX;
  float ay = s.data.accelY - s.cal.accelOffsetY;
  float az = s.data.accelZ - s.cal.accelOffsetZ;

  // Adafruit gyro in rad/s; Madgwick lib expects rad/s in this variant
  float gx = s.data.gyroX_dps * DEG_TO_RAD;
  float gy = s.data.gyroY_dps * DEG_TO_RAD;
  float gz = s.data.gyroZ_dps * DEG_TO_RAD;

  s.fusion.updateIMU(gx, gy, gz, ax, ay, az);
  s.data.roll = s.fusion.getRoll();
  s.data.pitch = s.fusion.getPitch();
  s.data.yaw = s.fusion.getYaw();

  // gravity estimate from roll/pitch
  float rollRad = s.data.roll * DEG_TO_RAD;
  float pitchRad = s.data.pitch * DEG_TO_RAD;
  float gxv = sinf(pitchRad) * 9.81f;
  float gyv = -sinf(rollRad) * cosf(pitchRad) * 9.81f;
  float gzv = cosf(rollRad) * cosf(pitchRad) * 9.81f;

  s.data.linX = ax - gxv;
  s.data.linY = ay - gyv;
  s.data.linZ = az - gzv;
}

void updateAngleIntegration(StickState &s, float dt) {
  // FIX #5: proper dt-based integration
  s.angleZ += s.data.gyroZ_dps * dt;

  // FIX #3: prevent runaway drift with stationary decay
  if (fabsf(s.data.gyroZ_dps) < STILL_GYRO_THRESH) {
    s.angleZ *= ANGLE_DECAY_RATE;
  }

  // recenter baseline
  s.data.yaw = s.angleZ - s.cal.neutralAngleZ;
}

void updateZone(StickState &s) {
  float angle = s.data.yaw;

  // FIX #6/#7: else-if chain, gapless, full coverage
  if      (angle >  ZONE_THRESH_FAR)                            s.zone = Z_TOPRIGHT;
  else if (angle >  ZONE_THRESH_MID && angle <= ZONE_THRESH_FAR) s.zone = Z_RIGHT;
  else if (angle > -ZONE_DEADBAND && angle <= ZONE_DEADBAND)     s.zone = Z_CENTER;
  else if (angle >= -ZONE_THRESH_MID && angle < -ZONE_DEADBAND)  s.zone = Z_LEFT;
  else if (angle >= -ZONE_THRESH_FAR && angle < -ZONE_THRESH_MID)s.zone = Z_TOPLEFT;
  else                                                            s.zone = Z_CENTER;
}

void detectHit(StickState &s) {
  s.hit = false;
  float m = sqrtf(s.data.linX*s.data.linX + s.data.linY*s.data.linY + s.data.linZ*s.data.linZ);

  // FIX #8: debounce by millis instead of delay
  uint32_t now = millis();
  if (m > hitThresholdMS2() && (now - s.lastHitMs) > DEBOUNCE_MS) {
    s.lastHitMs = now;
    s.hit = true;

    // velocity to volume mapping
    float n = (m - hitThresholdMS2()) / (hitMaxMS2() - hitThresholdMS2());
    n = constrain(n, 0.0f, 1.0f);
    s.hitVolume = (uint8_t)(VOLUME_MIN + n * (VOLUME_MAX - VOLUME_MIN));
  }
}

void triggerAudio(const StickState &s, bool isLeft) {
  if (!audioEnabled) return;

  XT_Wav_Class *sample = isLeft ? leftZones[s.zone] : rightZones[s.zone];
  sample->Volume = s.hitVolume;
  DacAudio.Play(sample);
}

void calibrateOffsets() {
  Serial.println("Calibrating offsets... keep sticks still");

  double lGx=0, lGy=0, lGz=0, lAx=0, lAy=0, lAz=0;
  double rGx=0, rGy=0, rGz=0, rAx=0, rAy=0, rAz=0;

  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    readSensorRegisters(mpuLeft, leftStick);
    readSensorRegisters(mpuRight, rightStick);

    lGx += leftStick.data.rawGyroX; lGy += leftStick.data.rawGyroY; lGz += leftStick.data.rawGyroZ;
    rGx += rightStick.data.rawGyroX; rGy += rightStick.data.rawGyroY; rGz += rightStick.data.rawGyroZ;

    lAx += leftStick.data.accelX; lAy += leftStick.data.accelY; lAz += (leftStick.data.accelZ - 9.81f);
    rAx += rightStick.data.accelX; rAy += rightStick.data.accelY; rAz += (rightStick.data.accelZ - 9.81f);

    delay(2);
  }

  const float GYRO_SCALE = 16.4f;
  leftStick.cal.gyroOffsetX = (lGx / CALIBRATION_SAMPLES) / GYRO_SCALE;
  leftStick.cal.gyroOffsetY = (lGy / CALIBRATION_SAMPLES) / GYRO_SCALE;
  leftStick.cal.gyroOffsetZ = (lGz / CALIBRATION_SAMPLES) / GYRO_SCALE;
  rightStick.cal.gyroOffsetX = (rGx / CALIBRATION_SAMPLES) / GYRO_SCALE;
  rightStick.cal.gyroOffsetY = (rGy / CALIBRATION_SAMPLES) / GYRO_SCALE;
  rightStick.cal.gyroOffsetZ = (rGz / CALIBRATION_SAMPLES) / GYRO_SCALE;

  leftStick.cal.accelOffsetX = lAx / CALIBRATION_SAMPLES;
  leftStick.cal.accelOffsetY = lAy / CALIBRATION_SAMPLES;
  leftStick.cal.accelOffsetZ = lAz / CALIBRATION_SAMPLES;
  rightStick.cal.accelOffsetX = rAx / CALIBRATION_SAMPLES;
  rightStick.cal.accelOffsetY = rAy / CALIBRATION_SAMPLES;
  rightStick.cal.accelOffsetZ = rAz / CALIBRATION_SAMPLES;

  Serial.println("Calibration complete.");
}

void calibrateZoneBaseline() {
  leftStick.cal.neutralAngleZ = leftStick.angleZ;
  rightStick.cal.neutralAngleZ = rightStick.angleZ;
  Serial.println("Zone baseline set.");
}

bool loadCalibrationFromNVS() {
  prefs.begin("drumcal", true);
  PersistCal pc{};
  size_t n = prefs.getBytes("cal", &pc, sizeof(pc));
  prefs.end();
  if (n != sizeof(pc) || pc.magic != 0xDA7A6001UL) {
    Serial.println("No valid NVS calibration.");
    return false;
  }
  leftStick.cal = pc.left;
  rightStick.cal = pc.right;
  Serial.println("Loaded calibration from NVS.");
  return true;
}

void saveCalibrationToNVS() {
  PersistCal pc{};
  pc.left = leftStick.cal;
  pc.right = rightStick.cal;
  pc.magic = 0xDA7A6001UL;

  prefs.begin("drumcal", false);
  prefs.putBytes("cal", &pc, sizeof(pc));
  prefs.end();
  Serial.println("Saved calibration to NVS.");
}

void handleSerialCommands() {
  if (!Serial.available()) return;
  char c = Serial.read();
  switch (c) {
    case 'c': calibrateOffsets(); saveCalibrationToNVS(); break;
    case 'z': calibrateZoneBaseline(); saveCalibrationToNVS(); break;
    case 't': {
      uint32_t start = millis();
      while (millis() - start < 10000) {
        Serial.print(leftStick.data.gyroZ_dps, 3); Serial.print(',');
        Serial.print(leftStick.data.linX, 3); Serial.print(',');
        Serial.print(leftStick.data.linY, 3); Serial.print(',');
        Serial.println(leftStick.data.linZ, 3);
        delay(10);
      }
    } break;
    case '+': HIT_THRESHOLD_G += 0.5f; Serial.println(HIT_THRESHOLD_G); break;
    case '-': HIT_THRESHOLD_G = max(0.5f, HIT_THRESHOLD_G - 0.5f); Serial.println(HIT_THRESHOLD_G); break;
    case 'h': Serial.println("c=calibrate z=zone baseline t=test +=thresh -=thresh r=restart"); break;
    case 'r': ESP.restart(); break;
    default: break;
  }
}

void debugPrint(uint32_t loopUs) {
  if (!DEBUG_ENABLED) return;

  if (DEBUG_ZONES) {
    Serial.print("Lz="); Serial.print((int)leftStick.zone);
    Serial.print(" Rz="); Serial.print((int)rightStick.zone);
    Serial.print(" La="); Serial.print(leftStick.data.yaw, 1);
    Serial.print(" Ra="); Serial.println(rightStick.data.yaw, 1);
  }

  if (DEBUG_HITS) {
    if (leftStick.hit) {
      Serial.print("L HIT vol="); Serial.println(leftStick.hitVolume);
    }
    if (rightStick.hit) {
      Serial.print("R HIT vol="); Serial.println(rightStick.hitVolume);
    }
  }

  if (DEBUG_TIMING) {
    Serial.print("loop_us=");
    Serial.println(loopUs);
  }
}
