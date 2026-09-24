// LiDAR (VL53L0X) reading shell - reads distance and derives the override flag.
// Never blocks: every other module shares this loop(), and a waiting lidar read slowed levelling to ~8 Hz.

#include "lidarReadings.h"

#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

#include "constants.h"

namespace LidarReadings {
namespace {

const uint8_t ADDR = 0x29;
const uint8_t REG_SYSRANGE_START = 0x00;
const uint8_t REG_INTERRUPT_CLEAR = 0x0B;
const uint8_t REG_RESULT_INTERRUPT_STATUS = 0x13;
const uint8_t REG_RESULT_RANGE_MM = 0x1E;  // result register (0x14 + 10)

VL53L0X sensor;
int status = 0;  // 0 = not initialised yet, 1 = library (continuous), 2 = raw single-shot fallback
bool overrideActive = false;
int lastMm = -1;
unsigned long lastPoll = 0;
unsigned long lastReading = 0;
bool rawPending = false;
unsigned long rawStart = 0;

uint8_t rd8(uint8_t reg) {
  Wire.beginTransmission(ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}

uint16_t rd16(uint8_t reg) {
  Wire.beginTransmission(ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(ADDR, (uint8_t)2);
  uint16_t hi = Wire.available() ? Wire.read() : 0xFF;
  uint16_t lo = Wire.available() ? Wire.read() : 0xFF;
  return (hi << 8) | lo;
}

void wr8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool ready() {
  uint8_t s = rd8(REG_RESULT_INTERRUPT_STATUS);
  return s != 0xFF && (s & 0x07) != 0;  // 0xFF = no answer on the bus
}

void setReading(uint16_t mm, unsigned long now) {
  lastMm = mm;
  overrideActive = mm < Constants::LIDAR_OVERRIDE_THRESHOLD_MM;
  lastReading = now;
}

}  // namespace

void begin() {
  Wire.begin();
  Wire.setClock(Constants::I2C_CLOCK_HZ);
}

void update() {
  unsigned long now = millis();

  if (status == 0) {
    if (now < Constants::LIDAR_INIT_DELAY_MS) return;  // sensor boot time, without blocking loop()
    sensor.setBus(&Wire);
    sensor.setTimeout(Constants::LIDAR_POLL_TIMEOUT_MS);  // library default 0 = wait forever
    if (sensor.init()) {
      sensor.startContinuous();
      status = 1;
    } else {
      status = 2;
    }
    lastReading = now;
    return;
  }

  if (now - lastPoll < Constants::SENSOR_READ_INTERVAL_MS) return;
  lastPoll = now;

  if (status == 1) {
    if (ready()) {
      uint16_t mm = sensor.readRangeContinuousMillimeters();  // data is ready, so this returns straight away
      if (!sensor.timeoutOccurred()) setReading(mm, now);
    }
  } else if (!rawPending) {
    wr8(REG_SYSRANGE_START, 0x01);  // raw single-shot start
    rawPending = true;
    rawStart = now;
  } else if (ready()) {
    uint16_t mm = rd16(REG_RESULT_RANGE_MM);
    wr8(REG_INTERRUPT_CLEAR, 0x01);
    rawPending = false;
    setReading(mm, now);
  } else if (now - rawStart > Constants::LIDAR_POLL_TIMEOUT_MS) {
    rawPending = false;  // no result, start a fresh shot next poll
  }

  if (now - lastReading > Constants::LIDAR_POLL_TIMEOUT_MS * 5) {  // no fresh reading: don't act on a stale one
    lastMm = -1;
    overrideActive = false;
  }
}

bool getOverride() {
  return overrideActive;
}

int getDistance() {
  return lastMm;
}

int getStatus() {
  return status;
}

}  // namespace LidarReadings
