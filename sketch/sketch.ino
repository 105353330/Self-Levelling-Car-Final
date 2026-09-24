// Self-levelling car - MCU side entry point: board bring-up, Bridge RPC registration, and scheduling of module updates

#include <Arduino_RouterBridge.h>

#include "accelerometerReadings.h"
#include "lidarReadings.h"
#include "mode.h"
#include "motor.h"
#include "platform.h"
#include "platformLevel.h"
#include "pwmCalibration.h"
#include "servoCalibration.h"
#include "steering.h"

unsigned long loopCount = 0;  // diagnostics: stops increasing if something blocks loop()

int getLoopCount() { return (int)(loopCount & 0x7FFFFFFF); }

void setup() {
  Monitor.begin();
  Bridge.begin();
  Bridge.provide("setMode", Mode::set);
  Bridge.provide("setSpeed", Motor::setSpeed);
  Bridge.provide("setTurn", Steering::setTurn);
  Bridge.provide("setServo1", Platform::setServo1);
  Bridge.provide("setServo2", Platform::setServo2);
  Bridge.provide("setServo3", Platform::setServo3);
  Bridge.provide("setCalPin", PwmCalibration::setPin);
  Bridge.provide("setCalPWM", PwmCalibration::setPWM);
  Bridge.provide("getLidarOverride", LidarReadings::getOverride);
  Bridge.provide("getLidarDistance", LidarReadings::getDistance);
  Bridge.provide("getLidarStatus", LidarReadings::getStatus);
  Bridge.provide("getAccelX", AccelerometerReadings::getAccelX);
  Bridge.provide("getAccelY", AccelerometerReadings::getAccelY);
  Bridge.provide("getAccelZ", AccelerometerReadings::getAccelZ);
  Bridge.provide("getAccelConfigured", AccelerometerReadings::isConfigured);
  Bridge.provide("getLoopCount", getLoopCount);
  Bridge.provide("getCalStep", ServoCalibration::getStep);
  Bridge.provide("getCalValue", ServoCalibration::getValue);
  Bridge.provide("getLevelRoll", PlatformLevel::getRoll);
  Bridge.provide("getLevelPitch", PlatformLevel::getPitch);
  Bridge.provide("getLevelUs1", PlatformLevel::getUs1);
  Bridge.provide("getLevelUs2", PlatformLevel::getUs2);
  Bridge.provide("getLevelUs3", PlatformLevel::getUs3);
  Bridge.provide("setLevelParam", PlatformLevel::setParam);

  Mode::begin();
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
  loopCount++;
  Mode::update();
  LidarReadings::update();
  AccelerometerReadings::update();
  PlatformLevel::update();
  ServoCalibration::update();
  Motor::update();
  Platform::update();
  Steering::update();
  PwmCalibration::update();
}
