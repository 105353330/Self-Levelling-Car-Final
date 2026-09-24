// QuicRun 1060 speed control - see about.txt for arming/failsafe/units design notes

#include "motor.h"

#include <Arduino.h>
#include <Arduino_RouterBridge.h>
#include <Servo.h>

#include "constants.h"
#include "lidarReadings.h"

namespace Motor {
namespace {

int incomingSpeed = Constants::ESC_NEUTRAL_US;  // value received from the Linux side over the Bridge
bool ARMED = false;
unsigned long armStart;
unsigned long lastCommandTime;

Servo ESC;

}  // namespace

int setSpeed(int us) {
  incomingSpeed = us;
  lastCommandTime = millis();
  return incomingSpeed;
}

void begin() {
  ESC.attach(Constants::ESC_PIN, 1000, 2000);
  armStart = millis();
  lastCommandTime = armStart;
}

void update() {
  unsigned long now = millis();

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
