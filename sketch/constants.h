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

// ---- Platform servos (platform.cpp) ----
constexpr int PLATFORM_SERVO_COUNT = 3;
constexpr unsigned long PLATFORM_COMMAND_TIMEOUT_MS = 500;  // servos return to centre if no command arrives

struct PlatformServoConfig {
  int pin;
  int min;     // µs
  int centre;  // µs
  int max;     // µs
};

constexpr PlatformServoConfig PLATFORM_SERVOS[PLATFORM_SERVO_COUNT] = {
  {9, 400, 650, 900},
  {10, 600, 850, 1100},
  {11, 500, 750, 1000},
};

// ---- Levelling (platformLevel.cpp) ----
// Level reference measured with all servos at centre. Redo if the sensor or setup moves.
constexpr float LEVEL_ROLL_ZERO_DEG = -1.31f;
constexpr float LEVEL_PITCH_ZERO_DEG = -3.89f;

// Pseudo-inverse of the measured servo response: us of servo change per degree of tilt error (servo 0, 1, 2)
constexpr float LEVEL_INV_ROLL_0 = 10.24f;
constexpr float LEVEL_INV_ROLL_1 = -10.91f;
constexpr float LEVEL_INV_ROLL_2 = 3.13f;
constexpr float LEVEL_INV_PITCH_0 = 10.66f;
constexpr float LEVEL_INV_PITCH_1 = 10.60f;
constexpr float LEVEL_INV_PITCH_2 = -14.47f;

constexpr float LEVEL_GAIN_LOW = 0.03f;        // K per update for small errors (<= LEVEL_GAIN_ERR_LOW_DEG): gentle, mostly noise
constexpr float LEVEL_GAIN_HIGH = 0.1f;        // K per update for real tilts (>= LEVEL_GAIN_ERR_HIGH_DEG): fast
constexpr float LEVEL_GAIN_ERR_LOW_DEG = 2.0f; // combined roll/pitch error where K starts rising
constexpr float LEVEL_GAIN_ERR_HIGH_DEG = 8.0f; // combined error where K reaches LEVEL_GAIN_HIGH
constexpr float LEVEL_FILTER_ALPHA = 0.15f;     // accel low-pass, applied after the 5-sample median
constexpr float LEVEL_ANGLE_DEADBAND_DEG = 1.0f; // soft deadband per axis: e' = sign(e) * max(|e| - this, 0)
constexpr int LEVEL_DEADBAND_US = 8;          // ignore pulse changes smaller than this
constexpr float LEVEL_MAX_SLEW_US_S = 300.0f; // max pulse change per second
constexpr float LEVEL_MAX_ACCEL_ERROR_G = 0.1f; // hold pulses when |a| differs from 1 g by more than this
constexpr unsigned long LEVEL_DEBUG_INTERVAL_MS = 500;
constexpr int LEVEL_CALIBRATION_MODE = 0;      // 1 = run open-loop servo test instead of levelling, 0 = level

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
