// Self-levelling car - MCU side entry point: board bring-up, Bridge RPC registration, and scheduling of module updates

#include <Arduino_RouterBridge.h>

#include "accelerometerReadings.h"
#include "lidarReadings.h"
#include "motor.h"
#include "platform.h"
#include "platformLevel.h"
#include "pwmCalibration.h"
#include "servoCalibration.h"
#include "steering.h"

void setup() {
  Monitor.begin();
  Bridge.begin();
  Bridge.provide("setSpeed", Motor::setSpeed);
  Bridge.provide("setTurn", Steering::setTurn);
  Bridge.provide("setServo1", Platform::setServo1);
  Bridge.provide("setServo2", Platform::setServo2);
  Bridge.provide("setServo3", Platform::setServo3);
  Bridge.provide("setCalPin", PwmCalibration::setPin);
  Bridge.provide("setCalPWM", PwmCalibration::setPWM);
  Bridge.provide("getLidarOverride", LidarReadings::getOverride);
  Bridge.provide("getAccelX", AccelerometerReadings::getAccelX);
  Bridge.provide("getAccelY", AccelerometerReadings::getAccelY);
  Bridge.provide("getAccelZ", AccelerometerReadings::getAccelZ);
  Bridge.provide("getCalStep", ServoCalibration::getStep);
  Bridge.provide("getCalValue", ServoCalibration::getValue);
  Bridge.provide("getLevelRoll", PlatformLevel::getRoll);
  Bridge.provide("getLevelPitch", PlatformLevel::getPitch);
  Bridge.provide("getLevelUs1", PlatformLevel::getUs1);
  Bridge.provide("getLevelUs2", PlatformLevel::getUs2);
  Bridge.provide("getLevelUs3", PlatformLevel::getUs3);

  Motor::begin();
  Platform::begin();
  Steering::begin();
  PwmCalibration::begin();
  LidarReadings::begin();
  AccelerometerReadings::begin();
  PlatformLevel::begin();
  ServoCalibration::begin();
}

void loop() {
  LidarReadings::update();
  AccelerometerReadings::update();
  PlatformLevel::update();
  ServoCalibration::update();
  Motor::update();
  Platform::update();
  Steering::update();
  PwmCalibration::update();
}
