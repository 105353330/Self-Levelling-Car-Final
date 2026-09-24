#pragma once

// Shared values - python/constants.py parses this file's plain scalar constexpr lines

namespace Constants {

// ---- I2C bus / accelerometer ----
constexpr unsigned long I2C_CLOCK_HZ = 100000;
constexpr unsigned long SENSOR_READ_INTERVAL_MS = 20;
constexpr unsigned long ACCELEROMETER_STARTUP_DELAY_MS = 10;
constexpr unsigned long ACCELEROMETER_READING_DELAY_MS = 100;

}  // namespace Constants
