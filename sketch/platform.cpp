// Platform servo output - drives the three levelling servos from angles sent by the Linux side

#include "platform.h"

#include <Arduino.h>
#include <Servo.h>

#include "constants.h"
#include "mode.h"

namespace Platform {
namespace {

  using Constants::PLATFORM_SERVOS;
  using Constants::PLATFORM_SERVO_COUNT;

  Servo servos[PLATFORM_SERVO_COUNT];
  int pulses[PLATFORM_SERVO_COUNT];
  unsigned long lastCommandTime;
  bool attached = false;
  int lastMode = -1;
  unsigned long idleStart = 0;

  int setAngle(int i, int deg) {
    int pulse = map(deg, 0, 180, PLATFORM_SERVOS[i].min, PLATFORM_SERVOS[i].max);
    pulses[i] = constrain(pulse, PLATFORM_SERVOS[i].min, PLATFORM_SERVOS[i].max);
    lastCommandTime = millis();
    return pulses[i];
  }

  void attachAll() {
    for (int i = 0; i < PLATFORM_SERVO_COUNT; i++) {
      pulses[i] = PLATFORM_SERVOS[i].centre;
      servos[i].attach(PLATFORM_SERVOS[i].pin);
      servos[i].writeMicroseconds(pulses[i]);
    }
    attached = true;
  }

  void detachAll() {
    for (int i = 0; i < PLATFORM_SERVO_COUNT; i++) {
      servos[i].detach();
      digitalWrite(PLATFORM_SERVOS[i].pin, LOW);  // detach can leave the pin stuck high
    }
    attached = false;
  }

}

void begin() {
  lastCommandTime = millis();  // servos stay unattached until update() sees the (idle) mode
}

void update() {
  int mode = Mode::get();
  bool entered = mode != lastMode;
  lastMode = mode;

  if (mode == Constants::MODE_PWM_CAL) {  // pins 9/10/11 shared with pwmCalibration.cpp
    if (attached) detachAll();
    return;
  }

  if (mode == Constants::MODE_IDLE) {
    // Menu: send centre once for PLATFORM_IDLE_SETTLE_MS so the servos get there, then no signal at all
    if (entered) {
      if (attached) {
        for (int i = 0; i < PLATFORM_SERVO_COUNT; i++) {
          pulses[i] = PLATFORM_SERVOS[i].centre;
          servos[i].writeMicroseconds(pulses[i]);
        }
      } else {
        attachAll();
      }
      idleStart = millis();
    } else if (attached && millis() - idleStart >= Constants::PLATFORM_IDLE_SETTLE_MS) {
      detachAll();
    }
    return;
  }

  if (!attached) attachAll();

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
