// Platform levelling - accelerometer tilt -> three servo pulse widths (us), runs on the MCU

#include "platformLevel.h"

#include <Arduino.h>
#include <Arduino_RouterBridge.h>
#include <math.h>

#include "accelerometerReadings.h"
#include "constants.h"
#include "platform.h"

namespace PlatformLevel {
namespace {

  using Constants::PLATFORM_SERVOS;
  using Constants::PLATFORM_SERVO_COUNT;

  const float PI_F = 3.14159265f;

  const int MEDIAN_SIZE = 5;
  float histX[MEDIAN_SIZE], histY[MEDIAN_SIZE], histZ[MEDIAN_SIZE];
  int   histCount = 0;
  int   histNext = 0;

  float filtAx = 0, filtAy = 0, filtAz = 1;
  bool  filterSeeded = false;
  float offsetUs[PLATFORM_SERVO_COUNT];  // integrated servo offset from centre, us
  int   lastUs[PLATFORM_SERVO_COUNT];
  unsigned long lastUpdate = 0;
  unsigned long lastDebug = 0;
  float lastRoll = 0;
  float lastPitch = 0;

  // Measured pseudo-inverse of the servo -> (roll, pitch) response: us of servo change per degree of error
  const float INV_ROLL[PLATFORM_SERVO_COUNT]  = { Constants::LEVEL_INV_ROLL_0,  Constants::LEVEL_INV_ROLL_1,  Constants::LEVEL_INV_ROLL_2 };
  const float INV_PITCH[PLATFORM_SERVO_COUNT] = { Constants::LEVEL_INV_PITCH_0, Constants::LEVEL_INV_PITCH_1, Constants::LEVEL_INV_PITCH_2 };

  float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
  }

  // Median of the filled part of a history buffer (rejects single-sample spikes)
  float median(const float *h, int n) {
    float t[MEDIAN_SIZE];
    for (int i = 0; i < n; i++) t[i] = h[i];
    for (int i = 1; i < n; i++) {
      float v = t[i];
      int j = i - 1;
      while (j >= 0 && t[j] > v) { t[j + 1] = t[j]; j--; }
      t[j + 1] = v;
    }
    return t[n / 2];
  }

  // Soft deadband: 0 inside +/- db, then error minus db (continuous at the edge)
  float softDeadband(float e, float db) {
    float m = fabsf(e) - db;
    return m <= 0.0f ? 0.0f : (e < 0.0f ? -m : m);
  }

  // ax, ay, az in g (X = right, Y = forward, Z = up when flat), dt in seconds.
  // Incremental controller: offset_i -= K * (a_i * rollError + b_i * pitchError), clamped to the servo's min/max.
  // Writes pulses to outUs[]; returns |a| - 1 so the caller can skip frames while accelerating hard.
  float platformUpdate(float ax, float ay, float az, float dt, int outUs[PLATFORM_SERVO_COUNT],
                       float &rollOut, float &pitchOut) {
    // 1. Median of the last 5 samples, then low-pass
    histX[histNext] = ax; histY[histNext] = ay; histZ[histNext] = az;
    histNext = (histNext + 1) % MEDIAN_SIZE;
    if (histCount < MEDIAN_SIZE) histCount++;
    float mx = median(histX, histCount), my = median(histY, histCount), mz = median(histZ, histCount);
    if (!filterSeeded) { filtAx = mx; filtAy = my; filtAz = mz; filterSeeded = true; }
    filtAx += Constants::LEVEL_FILTER_ALPHA * (mx - filtAx);
    filtAy += Constants::LEVEL_FILTER_ALPHA * (my - filtAy);
    filtAz += Constants::LEVEL_FILTER_ALPHA * (mz - filtAz);

    // 2. Tilt error from gravity (degrees), measured minus the level reference
    float roll  = atan2f(filtAx, filtAz) * 180.0f / PI_F - Constants::LEVEL_ROLL_ZERO_DEG;
    float pitch = atan2f(filtAy, filtAz) * 180.0f / PI_F - Constants::LEVEL_PITCH_ZERO_DEG;
    rollOut = roll;
    pitchOut = pitch;
    const float rollErr = roll, pitchErr = pitch;
    roll = softDeadband(roll, Constants::LEVEL_ANGLE_DEADBAND_DEG);
    pitch = softDeadband(pitch, Constants::LEVEL_ANGLE_DEADBAND_DEG);

    // 3. Gain scheduled on the combined error: gentle when small (noise), fast on real tilts
    float errMag = sqrtf(rollErr * rollErr + pitchErr * pitchErr);
    float t = clampf((errMag - Constants::LEVEL_GAIN_ERR_LOW_DEG) /
                     (Constants::LEVEL_GAIN_ERR_HIGH_DEG - Constants::LEVEL_GAIN_ERR_LOW_DEG), 0.0f, 1.0f);
    const float gain = Constants::LEVEL_GAIN_LOW + (Constants::LEVEL_GAIN_HIGH - Constants::LEVEL_GAIN_LOW) * t;

    // 4. Integrate each servo's offset, slew-limited per update, clamped to its travel
    const float maxStep = Constants::LEVEL_MAX_SLEW_US_S * dt;
    for (int i = 0; i < PLATFORM_SERVO_COUNT; i++) {
      float step = clampf(-gain * (INV_ROLL[i] * roll + INV_PITCH[i] * pitch), -maxStep, maxStep);
      const Constants::PlatformServoConfig &s = PLATFORM_SERVOS[i];
      offsetUs[i] = clampf(offsetUs[i] + step, (float)(s.min - s.centre), (float)(s.max - s.centre));
      int target = s.centre + (int)lroundf(offsetUs[i]);
      outUs[i] = (abs(target - lastUs[i]) < Constants::LEVEL_DEADBAND_US) ? lastUs[i] : target;  // ignore tiny pulse changes
    }

    return sqrtf(filtAx * filtAx + filtAy * filtAy + filtAz * filtAz) - 1.0f;
  }

}  // namespace

void begin() {
  for (int i = 0; i < PLATFORM_SERVO_COUNT; i++) {
    offsetUs[i] = 0.0f;
    lastUs[i] = PLATFORM_SERVOS[i].centre;
  }
  lastUpdate = millis();
}

void update() {
  if (Constants::LEVEL_CALIBRATION_MODE != 0) return;  // servoCalibration owns the servos
  unsigned long now = millis();
  if (now - lastUpdate < Constants::SENSOR_READ_INTERVAL_MS) return;  // one step per new sample
  float dt = clampf((now - lastUpdate) / 1000.0f, 0.001f, 0.1f);
  lastUpdate = now;

  float ax = AccelerometerReadings::getAccelX();
  float ay = AccelerometerReadings::getAccelY();
  float az = AccelerometerReadings::getAccelZ();
  if (ax == 0.0f && ay == 0.0f && az == 0.0f) return;  // no valid sample yet: hold centre

  int out[PLATFORM_SERVO_COUNT];
  float roll = 0, pitch = 0;
  float magError = platformUpdate(ax, ay, az, dt, out, roll, pitch);
  lastRoll = roll;
  lastPitch = pitch;

  if (fabsf(magError) <= Constants::LEVEL_MAX_ACCEL_ERROR_G) {  // else accelerating hard: hold last pulses
    for (int i = 0; i < PLATFORM_SERVO_COUNT; i++) {
      lastUs[i] = out[i];
      Platform::setPulseUs(i, out[i]);
    }
  } else {
    for (int i = 0; i < PLATFORM_SERVO_COUNT; i++) offsetUs[i] = (float)(lastUs[i] - PLATFORM_SERVOS[i].centre);  // undo this frame's integration
  }

  if (now - lastDebug >= Constants::LEVEL_DEBUG_INTERVAL_MS) {
    lastDebug = now;
    Monitor.print("[level] g=(");
    Monitor.print(ax); Monitor.print(","); Monitor.print(ay); Monitor.print(","); Monitor.print(az);
    Monitor.print(") roll="); Monitor.print(roll);
    Monitor.print(" pitch="); Monitor.print(pitch);
    Monitor.print(" us=(");
    Monitor.print(lastUs[0]); Monitor.print(","); Monitor.print(lastUs[1]); Monitor.print(","); Monitor.print(lastUs[2]);
    Monitor.println(")");
  }
}

float getRoll() { return lastRoll; }
float getPitch() { return lastPitch; }
int getUs1() { return lastUs[0]; }
int getUs2() { return lastUs[1]; }
int getUs3() { return lastUs[2]; }

}  // namespace PlatformLevel
