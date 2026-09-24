// Steering servo control - see about.txt for pre-run TODOs

#include "steering.h"

#include <Arduino.h>
#include <Arduino_RouterBridge.h>
#include <Servo.h>

#include "constants.h"
#include "mode.h"

namespace Steering {
namespace {

int incomingTurn = 0;
unsigned long lastCommandTime;
bool attached = false;

Servo servo;

void attachServo() {
  servo.attach(Constants::STEERING_PIN, Constants::STEERING_MIN_US, Constants::STEERING_MAX_US);
  servo.writeMicroseconds(Constants::STEERING_CENTRE_US);
  attached = true;
}

}


int setTurn(int degrees) {
  incomingTurn = constrain(degrees, Constants::CONTROL_TURN_MIN, Constants::CONTROL_TURN_MAX);
  lastCommandTime = millis();
  return incomingTurn;
}

void begin() {  // servo is only attached in MODE_DRIVE, see update()
  lastCommandTime = millis();
}

void update() {
  int mode = Mode::get();
  if (mode != Constants::MODE_DRIVE) {  // software-PWM budget, see motor.cpp: only attached while driving
    if (attached) {
      servo.detach();
      digitalWrite(Constants::STEERING_PIN, LOW);  // detach can leave the pin stuck high
      attached = false;
    }
    return;
  }
  if (!attached) attachServo();

  if (millis() - lastCommandTime >= Constants::STEERING_COMMAND_TIMEOUT_MS) {  // comms assumed lost
    servo.writeMicroseconds(Constants::STEERING_CENTRE_US);
    return;
  }

  int pulse = map(incomingTurn, Constants::CONTROL_TURN_MIN, Constants::CONTROL_TURN_MAX,
                   Constants::STEERING_MIN_US, Constants::STEERING_MAX_US);
  servo.writeMicroseconds(pulse);
}

}  // namespace Steering
