// MMA8452 (12-bit, +/-2g) accelerometer for Arduino Uno Q - see the repeated-start note below

// The Uno Q Zephyr core ignores the register pointer on reads: every read starts at reg 0x00
// (STATUS, XOUT_MSB/LSB, YOUT_MSB/LSB, ZOUT_MSB/LSB). So config uses plain writes and the
// 7-byte block is read with a bare requestFrom.

#include "accelerometerReadings.h"

#include <Arduino.h>
#include <Wire.h>

#include "constants.h"

namespace AccelerometerReadings {
namespace {

const uint8_t ADDR          = 0x1C;
const uint8_t REG_XYZ_CFG   = 0x0E;
const uint8_t REG_CTRL_REG1 = 0x2A;

const int   BITS         = 12;
const float COUNTS_PER_G = 1024.0f;  // 12-bit, +/-2g

const unsigned long RETRY_INTERVAL_MS = 2000;
const uint8_t       MAX_FAIL_STREAK   = 5;

bool     configured  = false;
uint32_t lastAttempt = 0;
uint32_t lastRead    = 0;
uint16_t failStreak  = 0;
float    lastGx      = 0;
float    lastGy      = 0;
float    lastGz      = 0;

bool wr8(uint8_t reg, uint8_t val) {
  Wire1.beginTransmission(ADDR);
  Wire1.write(reg);
  Wire1.write(val);
  return Wire1.endTransmission() == 0;
}

void drain() {
  while (Wire1.available()) Wire1.read();
}

bool readBlock(uint8_t *raw) {
  drain();  // discard leftovers so frames stay aligned
  if (Wire1.requestFrom(ADDR, (uint8_t)7) != 7) {
    drain();
    return false;
  }
  for (uint8_t i = 0; i < 7; i++) raw[i] = Wire1.read();
  return true;
}

int16_t decode(const uint8_t *p) {
  return (int16_t)((p[0] << 8) | p[1]) >> (16 - BITS);
}

bool configure() {
  // Standby -> +/-2g -> active @ 100 Hz (config regs only writable in standby)
  if (!wr8(REG_CTRL_REG1, 0x00)) return false;
  delay(Constants::ACCELEROMETER_STARTUP_DELAY_MS);
  if (!wr8(REG_XYZ_CFG, 0x00)) return false;
  if (!wr8(REG_CTRL_REG1, 0x19)) return false;
  delay(Constants::ACCELEROMETER_READING_DELAY_MS);

  uint8_t raw[7];
  return readBlock(raw);
}

}  // namespace

void begin() {
  Wire1.begin();
  Wire1.setClock(Constants::I2C_CLOCK_HZ);
  Wire1.setTimeout(25);  // ms; a wedged sensor must not freeze the whole MCU loop
}

void update() {
  uint32_t now = millis();

  if (!configured) {
    if (now - lastAttempt < RETRY_INTERVAL_MS) return;
    lastAttempt = now;
    configured = configure();
    return;
  }

  if (now - lastRead < Constants::SENSOR_READ_INTERVAL_MS) return;
  lastRead = now;

  uint8_t raw[7];
  if (!readBlock(raw)) {
    if (++failStreak >= MAX_FAIL_STREAK) {  // sensor lost - reconfigure
      configured = false;
      failStreak = 0;
    }
    return;
  }
  failStreak = 0;

  lastGx = decode(&raw[1]) / COUNTS_PER_G;
  lastGy = decode(&raw[3]) / COUNTS_PER_G;
  lastGz = decode(&raw[5]) / COUNTS_PER_G;
}

bool isConfigured() { return configured; }

float getAccelX() { return lastGx; }
float getAccelY() { return lastGy; }
float getAccelZ() { return lastGz; }

}  // namespace AccelerometerReadings
