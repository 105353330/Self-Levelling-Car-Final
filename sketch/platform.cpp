// Platform servo output - drives the three levelling servos from angles sent by the Linux side

#include "platform.h"

#include <Arduino.h>
#include <Servo.h>

#include "constants.h"

namespace Platform {
namespace {

  using Constants::PLATFORM_SERVOS;
  using Constants::PLATFORM_SERVO_COUNT;

  Servo servos[PLATFORM_SERVO_COUNT];
  int pulses[PLATFORM_SERVO_COUNT];
  unsigned long lastCommandTime;

  int setAngle(int i, int deg) {
    int pulse = map(deg, 0, 180, PLATFORM_SERVOS[i].min, PLATFORM_SERVOS[i].max);
    pulses[i] = constrain(pulse, PLATFORM_SERVOS[i].min, PLATFORM_SERVOS[i].max);
    lastCommandTime = millis();
    return pulses[i];
  }

}

void begin() {
  for (int i = 0; i < PLATFORM_SERVO_COUNT; i++) {
    pulses[i] = PLATFORM_SERVOS[i].centre;
    servos[i].attach(PLATFORM_SERVOS[i].pin);
    servos[i].writeMicroseconds(pulses[i]);
  }
  lastCommandTime = millis();
}

void update() {
  bool timedOut = millis() - lastCommandTime >= Constants::PLATFORM_COMMAND_TIMEOUT_MS;  // comms assumed lost
  for (int i = 0; i < PLATFORM_SERVO_COUNT; i++) {
    if (timedOut) pulses[i] = PLATFORM_SERVOS[i].centre;
    servos[i].writeMicroseconds(pulses[i]);
  }
}

void setPulseUs(int i, int us) {
  pulses[i] = constrain(us, PLATFORM_SERVOS[i].min, PLATFORM_SERVOS[i].max);
  lastCommandTime = millis();
}

int setServo1(int deg) { return setAngle(0, deg); }
int setServo2(int deg) { return setAngle(1, deg); }
int setServo3(int deg) { return setAngle(2, deg); }

}
