// PWM calibration shell - lets the Linux side pick a pin then push raw microsecond pulse widths to it

#include "pwmCalibration.h"

#include <Arduino.h>
#include <Servo.h>

#include "constants.h"
#include "mode.h"

namespace PwmCalibration {
namespace {

Servo output;
int pendingPin = -1;
int pendingPWM = -1;
int attachedPin = -1;

}  // namespace

int setPin(int pin) {
  pendingPin = pin;
  pendingPWM = -1;  // a new pin must never inherit the previous pin's pulse width
  return pendingPin;
}

int setPWM(int us) {
  pendingPWM = us;
  return pendingPWM;
}

void begin() {}

void update() {
  if (Mode::get() != Constants::MODE_PWM_CAL) {  // only owns the pin in PWM calibration mode
    if (attachedPin >= 0) {
      output.detach();
      digitalWrite(attachedPin, LOW);  // detach can leave the pin stuck high
    }
    attachedPin = -1;
    pendingPin = -1;
    pendingPWM = -1;
    return;
  }

  if (pendingPin >= 0 && pendingPin != attachedPin) {
    if (attachedPin >= 0) output.detach();
    output.attach(pendingPin);
    attachedPin = pendingPin;
  }

  if (attachedPin >= 0 && pendingPWM >= 0) {
    output.writeMicroseconds(pendingPWM);
  }
}

}  // namespace PwmCalibration
