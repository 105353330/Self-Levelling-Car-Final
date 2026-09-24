// Steering servo control - see about.txt for pre-run TODOs

#include "steering.h"

#include <Arduino.h>
#include <Arduino_RouterBridge.h>
#include <Servo.h>

#include "constants.h"

namespace Steering {
namespace {

int incomingTurn = 0;
unsigned long lastCommandTime;

Servo servo;

}


int setTurn(int degrees) {
  incomingTurn = constrain(degrees, Constants::CONTROL_TURN_MIN, Constants::CONTROL_TURN_MAX);
  lastCommandTime = millis();
  return incomingTurn;
}

void begin() {
  // servo write TEMP-disabled, see about.txt - uncomment to restore
  // servo.attach(Constants::STEERING_PIN, Constants::STEERING_MIN_US, Constants::STEERING_MAX_US);
  lastCommandTime = millis();
  // servo.writeMicroseconds(Constants::STEERING_CENTRE_US);
}

void update() {
  if (millis() - lastCommandTime >= Constants::STEERING_COMMAND_TIMEOUT_MS) {  // comms assumed lost
    // servo.writeMicroseconds(Constants::STEERING_CENTRE_US);
    return;
  }

  int pulse = map(incomingTurn, Constants::CONTROL_TURN_MIN, Constants::CONTROL_TURN_MAX,
                   Constants::STEERING_MIN_US, Constants::STEERING_MAX_US);
  // servo.writeMicroseconds(pulse);  // TEMP-disabled, see begin()
}

}  // namespace Steering
