#pragma once

// Shared values - python/constants.py parses this file's plain scalar constexpr lines

namespace Constants {

// ---- Lidar ----
constexpr int LIDAR_OVERRIDE_THRESHOLD_MM = 2000;
constexpr unsigned long LIDAR_INIT_DELAY_MS = 2000;
constexpr unsigned long LIDAR_POLL_TIMEOUT_MS = 100;

// ---- I2C bus / accelerometer ----
constexpr unsigned long I2C_CLOCK_HZ = 100000;
constexpr unsigned long SENSOR_READ_INTERVAL_MS = 20;
constexpr unsigned long ACCELEROMETER_STARTUP_DELAY_MS = 10;
constexpr unsigned long ACCELEROMETER_READING_DELAY_MS = 100;

// ---- Ports (python/ side sockets) ----
constexpr int PWM_CALIBRATION_PORT = 7008;

}  // namespace Constants
