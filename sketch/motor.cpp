// QuicRun 1060 speed control - see about.txt for arming/failsafe/units design notes

#include "motor.h"

#include <Arduino.h>
#include <Arduino_RouterBridge.h>
#include <Servo.h>

#include "constants.h"
#include "lidarReadings.h"
#include "mode.h"

namespace Motor {
namespace {

int incomingSpeed = Constants::ESC_NEUTRAL_US;  // value received from the Linux side over the Bridge
bool ARMED = false;
bool attached = false;
unsigned long armStart;
unsigned long lastCommandTime;

Servo ESC;

void attachEsc() {
  ESC.attach(Constants::ESC_PIN, 1000, 2000);
  attached = true;
  ARMED = false;  // re-arm after every (re)attach
  armStart = millis();
  lastCommandTime = armStart;
}

}  // namespace

int setSpeed(int us) {
  incomingSpeed = us;
  lastCommandTime = millis();
  return incomingSpeed;
}

void begin() {}  // ESC is only attached in MODE_DRIVE, see update()

void update() {
  unsigned long now = millis();
  int mode = Mode::get();

  // Uno Q Servo is software PWM: every attached servo costs time in a 4 us timer interrupt, and 5 at
  // once (ESC + steering + 3 platform) broke the platform pulses. So the ESC is only attached in Drive.
  if (mode != Constants::MODE_DRIVE) {
    if (attached) {
      ESC.detach();
      digitalWrite(Constants::ESC_PIN, LOW);  // detach can leave the pin stuck high
      attached = false;
    }
    return;
  }
  if (!attached) attachEsc();

  if (!ARMED) {
    ESC.writeMicroseconds(Constants::ESC_NEUTRAL_US);

    if (now - armStart >= Constants::ESC_ARM_TIME_MS) {
      ARMED = true;
    }

    while (Monitor.available()) Monitor.read();
    return;
  }

  if (now - lastCommandTime >= Constants::ESC_COMMAND_TIMEOUT_MS) {  // comms assumed lost
    ESC.writeMicroseconds(Constants::ESC_NEUTRAL_US);
    return;
  }

  if (LidarReadings::getOverride()) {
    ESC.writeMicroseconds(Constants::ESC_NEUTRAL_US);
    return;
  }

  // incomingSpeed may be a 0-100 percent or an already-converted pulse width, see about.txt
  int pulse = incomingSpeed;
  if (pulse >= Constants::CONTROL_SPEED_MIN && pulse <= Constants::CONTROL_SPEED_MAX) {
    pulse = Constants::ESC_NEUTRAL_US +
            pulse * (Constants::ESC_MAXIMUM_US - Constants::ESC_NEUTRAL_US) / Constants::CONTROL_SPEED_MAX;
  }

  ESC.writeMicroseconds(pulse);
}

}  // namespace Motor
