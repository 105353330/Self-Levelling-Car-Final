// Runtime mode - which module owns the motor/steering/platform pins, picked from the python menu

#include "mode.h"

#include "constants.h"
#include "platformLevel.h"
#include "servoCalibration.h"

namespace Mode {
namespace {

  volatile int pending = Constants::MODE_IDLE;  // written by the Bridge handler, applied in loop()
  int current = Constants::MODE_IDLE;

}  // namespace

void begin() {
  pending = Constants::MODE_IDLE;
  current = Constants::MODE_IDLE;
}

void update() {
  int next = pending;
  if (next == current) return;
  current = next;
  if (current == Constants::MODE_LEVEL || current == Constants::MODE_DRIVE) PlatformLevel::begin();
  if (current == Constants::MODE_SERVO_CAL) ServoCalibration::begin();
}

int get() { return current; }

int set(int mode) {
  if (mode < Constants::MODE_IDLE || mode > Constants::MODE_SERVO_CAL) return current;
  pending = mode;
  return mode;
}

}  // namespace Mode
