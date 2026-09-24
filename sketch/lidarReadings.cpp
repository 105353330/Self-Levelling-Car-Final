// LiDAR (VL53L0X) reading shell - reads distance and derives the override flag

#include "lidarReadings.h"

#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

#include "constants.h"

namespace LidarReadings {
namespace {

VL53L0X sensor;
bool initAttempted = false;
bool libOK = false;
int initStatus = -1;
bool overrideActive = false;

uint8_t rd8(uint8_t reg) {
  Wire1.beginTransmission(0x29);
  Wire1.write(reg);
  Wire1.endTransmission();
  Wire1.requestFrom((uint8_t)0x29, (uint8_t)1);
  return Wire1.available() ? Wire1.read() : 0xFF;
}

uint16_t rd16(uint8_t reg) {
  Wire1.beginTransmission(0x29);
  Wire1.write(reg);
  Wire1.endTransmission();
  Wire1.requestFrom((uint8_t)0x29, (uint8_t)2);
  uint16_t hi = Wire1.available() ? Wire1.read() : 0xFF;
  uint16_t lo = Wire1.available() ? Wire1.read() : 0xFF;
  return (hi << 8) | lo;
}

void wr8(uint8_t reg, uint8_t val) {
  Wire1.beginTransmission(0x29);
  Wire1.write(reg);
  Wire1.write(val);
  Wire1.endTransmission();
}

}  // namespace

void begin() {
  Wire1.begin();
  Wire1.setClock(Constants::I2C_CLOCK_HZ);
}

void update() {
  if (!initAttempted) {
    delay(Constants::LIDAR_INIT_DELAY_MS);
    sensor.setBus(&Wire1);
    libOK = sensor.init();
    initStatus = sensor.last_status;
    if (libOK) sensor.startContinuous();
    initAttempted = true;
  }

  if (libOK) {
    uint16_t mm = sensor.readRangeContinuousMillimeters();
    overrideActive = mm < Constants::LIDAR_OVERRIDE_THRESHOLD_MM;
  } else {
    wr8(0x00, 0x01);  // raw single-shot start
    uint32_t pollStart = millis();
    while ((rd8(0x13) & 0x07) == 0) {  // bounded poll, see about.txt
      if (millis() - pollStart > Constants::LIDAR_POLL_TIMEOUT_MS) {
        overrideActive = false;
        delay(Constants::SENSOR_READ_INTERVAL_MS);
        return;
      }
      delay(5);
    }
    uint16_t mm = rd16(0x1E);  // result register (0x14 + 10)
    wr8(0x0B, 0x01);           // clear interrupt
    overrideActive = mm < Constants::LIDAR_OVERRIDE_THRESHOLD_MM;
  }
  delay(Constants::SENSOR_READ_INTERVAL_MS);
}

bool getOverride() {
  return overrideActive;
}

}  // namespace LidarReadings
