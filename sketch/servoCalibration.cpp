// Open-loop servo response test: measures each servo's effect on the sensor's roll/pitch with the levelling loop off.
// Sequence: all centre (level reference), then per servo: +step, -step, back to centre. Results are averaged degrees.

#include "servoCalibration.h"

#include <Arduino.h>
#include <Arduino_RouterBridge.h>
#include <math.h>

#include "accelerometerReadings.h"
#include "constants.h"
#include "mode.h"
#include "platform.h"

namespace ServoCalibration {
namespace {

  using Constants::PLATFORM_SERVOS;
  using Constants::PLATFORM_SERVO_COUNT;

  const float PI_F = 3.14159265f;
  const int STEP_COUNT = 1 + 3 * PLATFORM_SERVO_COUNT;  // reference + (+, -, recentre) per servo
  const int VALUE_COUNT = 2 + 4 * PLATFORM_SERVO_COUNT;

  int step = 0;
  unsigned long stepStart = 0;
  unsigned long lastSample = 0;
  float sumRoll = 0, sumPitch = 0;
  int samples = 0;
  float values[VALUE_COUNT];
  bool started = false;

  // Servo being moved and direction for the current step; servo -1 = all centred
  void stepTarget(int s, int &servo, int &dir, bool &record) {
    if (s == 0) { servo = -1; dir = 0; record = true; return; }
    servo = (s - 1) / 3;
    int phase = (s - 1) % 3;
    dir = phase == 0 ? +1 : (phase == 1 ? -1 : 0);
    record = phase != 2;
  }

  int valueIndex(int servo, int dir) { return 2 + 4 * servo + (dir > 0 ? 0 : 2); }

  void enterStep(int s) {
    step = s;
    stepStart = millis();
    sumRoll = sumPitch = 0;
    samples = 0;
  }

}  // namespace

void begin() {
  enterStep(0);
  started = true;
}

void update() {
  if (Mode::get() != Constants::MODE_SERVO_CAL || !started) return;

  unsigned long now = millis();
  if (step >= STEP_COUNT) {  // finished: hold centre
    for (int i = 0; i < PLATFORM_SERVO_COUNT; i++) Platform::setPulseUs(i, PLATFORM_SERVOS[i].centre);
    return;
  }

  int servo, dir;
  bool record;
  stepTarget(step, servo, dir, record);

  // Command pulses every loop (also keeps Platform's command timeout from centring them)
  for (int i = 0; i < PLATFORM_SERVO_COUNT; i++) {
    int us = PLATFORM_SERVOS[i].centre;
    if (i == servo) us += dir * Constants::CAL_STEP_US;
    Platform::setPulseUs(i, us);
  }

  unsigned long elapsed = now - stepStart;
  if (elapsed >= Constants::CAL_SETTLE_MS && record && now - lastSample >= Constants::SENSOR_READ_INTERVAL_MS) {
    lastSample = now;
    float ax = AccelerometerReadings::getAccelX();
    float ay = AccelerometerReadings::getAccelY();
    float az = AccelerometerReadings::getAccelZ();
    if (!(ax == 0.0f && ay == 0.0f && az == 0.0f)) {
      sumRoll  += atan2f(ax, az) * 180.0f / PI_F;
      sumPitch += atan2f(ay, az) * 180.0f / PI_F;
      samples++;
    }
  }

  unsigned long needed = Constants::CAL_SETTLE_MS + (record ? Constants::CAL_AVERAGE_MS : 0);
  if (elapsed < needed) return;

  if (record && samples > 0) {
    int idx = servo < 0 ? 0 : valueIndex(servo, dir);
    values[idx] = sumRoll / samples;
    values[idx + 1] = sumPitch / samples;
    Monitor.print("[cal] step "); Monitor.print(step);
    Monitor.print(" roll="); Monitor.print(values[idx]);
    Monitor.print(" pitch="); Monitor.println(values[idx + 1]);
    enterStep(step + 1);
  } else if (record) {
    enterStep(step);  // sensor gave no valid samples: repeat this step
  } else {
    enterStep(step + 1);
  }
  if (step >= STEP_COUNT) Monitor.println("[cal] done");
}

int getStep() { return step; }

float getValue(int idx) { return (idx >= 0 && idx < VALUE_COUNT) ? values[idx] : 0.0f; }

}  // namespace ServoCalibration
