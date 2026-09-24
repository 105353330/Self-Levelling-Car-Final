#pragma once

// Shared values - python/constants.py parses this file's plain scalar constexpr lines

namespace Constants {

// ---- Motor / ESC (motor.cpp) ----
constexpr int ESC_PIN = 5;
constexpr int ESC_NEUTRAL_US = 1500;                   // µs
constexpr int ESC_MAXIMUM_US = 1600;                   // µs
constexpr unsigned long ESC_ARM_TIME_MS = 3000;
constexpr unsigned long ESC_COMMAND_TIMEOUT_MS = 50;

// ---- Steering (steering.cpp) ----
constexpr int STEERING_PIN = 6;
constexpr int STEERING_MIN_US = 600;
constexpr int STEERING_CENTRE_US = 900;
constexpr int STEERING_MAX_US = 1200;
constexpr unsigned long STEERING_COMMAND_TIMEOUT_MS = 50;

// ---- Control ranges (also read by the python/ side) ----
constexpr int CONTROL_SPEED_MIN = 0;
constexpr int CONTROL_SPEED_MAX = 100;
constexpr int CONTROL_TURN_MIN = -180;
constexpr int CONTROL_TURN_MAX = 180;

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
constexpr int ESC_PORT = 7001;
constexpr int STEERING_SERVO_PORT = 7002;
constexpr int PWM_CALIBRATION_PORT = 7008;
constexpr int LIDAR_OVERRIDE_PORT = 7009;

}  // namespace Constants
