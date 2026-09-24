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
constexpr unsigned long PLATFORM_IDLE_SETTLE_MS = 700;  // menu/idle: centre pulses this long, then the servo signal is switched off

struct PlatformServoConfig {
  int pin;
  int min;     // µs
  int centre;  // µs
  int max;     // µs
};

constexpr PlatformServoConfig PLATFORM_SERVOS[PLATFORM_SERVO_COUNT] = {
  {9, 400, 650, 900},
  {10, 500, 750, 1000},
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

// ---- Runtime modes (mode.cpp), picked from python/main.py's menu via Bridge setMode ----
constexpr int MODE_IDLE = 0;       // motor neutral, steering + platform centred
constexpr int MODE_DRIVE = 1;      // keyboard driving, platform levelling on
constexpr int MODE_PWM_CAL = 2;    // pwmCalibration.cpp owns the pins, everything else detached
constexpr int MODE_LEVEL = 3;      // platform levelling only
constexpr int MODE_SERVO_CAL = 4;  // open-loop servo response test (servoCalibration.cpp)

// ---- Servo response calibration (servoCalibration.cpp) ----
constexpr int CAL_STEP_US = 100;               // pulse offset applied to one servo at a time
constexpr unsigned long CAL_SETTLE_MS = 1500;  // wait after each move before averaging
constexpr unsigned long CAL_AVERAGE_MS = 2000; // averaging window

// ---- Control ranges (also read by the python/ side) ----
constexpr int CONTROL_SPEED_MIN = 0;
constexpr int CONTROL_SPEED_MAX = 100;
constexpr int CONTROL_TURN_MIN = -180;
constexpr int CONTROL_TURN_MAX = 180;

// ---- Lidar ----
constexpr int LIDAR_OVERRIDE_THRESHOLD_MM = 200;
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
constexpr int CONTROL_PORT = 7012;  // main.py menu <-> service (control.py)
constexpr int CAMERA_STREAM_PORT = 8080;  // menu option 6: annotated camera view as an MJPEG web page
constexpr int SERVICE_LOOP_HZ = 50;  // python service loop rate; unthrottled Bridge traffic jitters the software servo PWM. Must beat ESC_COMMAND_TIMEOUT_MS

// ---- Object avoidance (Linux/python side only) ----
constexpr int CAMERA_AVOIDANCE_LOOP_HZ = 10;
constexpr int CAMERA_DEVICE_INDEX = 1;
constexpr int CAMERA_WIDTH_PX = 640;
constexpr int CAMERA_HEIGHT_PX = 480;
constexpr int CAMERA_FPS = 10;
constexpr float DETECTION_CONFIDENCE_THRESHOLD = 0.5;
constexpr float CAMERA_FOCAL_PX = 700;  // uncalibrated placeholder, see about.txt
constexpr int DEFAULT_OBJECT_HEIGHT_MM = 300;

// Real-world height (mm) per COCO class, for distance.py's pinhole estimate - placeholders, see about.txt
constexpr int OBJECT_HEIGHT_PERSON_MM = 1700;
constexpr int OBJECT_HEIGHT_BICYCLE_MM = 1100;
constexpr int OBJECT_HEIGHT_CAR_MM = 1500;
constexpr int OBJECT_HEIGHT_MOTORCYCLE_MM = 1300;
constexpr int OBJECT_HEIGHT_AIRPLANE_MM = 4000;
constexpr int OBJECT_HEIGHT_BUS_MM = 3200;
constexpr int OBJECT_HEIGHT_TRAIN_MM = 3500;
constexpr int OBJECT_HEIGHT_TRUCK_MM = 2500;
constexpr int OBJECT_HEIGHT_BOAT_MM = 1500;
constexpr int OBJECT_HEIGHT_TRAFFIC_LIGHT_MM = 3000;
constexpr int OBJECT_HEIGHT_FIRE_HYDRANT_MM = 600;
constexpr int OBJECT_HEIGHT_STOP_SIGN_MM = 2100;
constexpr int OBJECT_HEIGHT_PARKING_METER_MM = 1200;
constexpr int OBJECT_HEIGHT_BENCH_MM = 800;
constexpr int OBJECT_HEIGHT_BIRD_MM = 250;
constexpr int OBJECT_HEIGHT_CAT_MM = 250;
constexpr int OBJECT_HEIGHT_DOG_MM = 500;
constexpr int OBJECT_HEIGHT_HORSE_MM = 1600;
constexpr int OBJECT_HEIGHT_SHEEP_MM = 800;
constexpr int OBJECT_HEIGHT_COW_MM = 1400;
constexpr int OBJECT_HEIGHT_ELEPHANT_MM = 3000;
constexpr int OBJECT_HEIGHT_BEAR_MM = 1200;
constexpr int OBJECT_HEIGHT_ZEBRA_MM = 1400;
constexpr int OBJECT_HEIGHT_GIRAFFE_MM = 5000;
constexpr int OBJECT_HEIGHT_BACKPACK_MM = 450;
constexpr int OBJECT_HEIGHT_UMBRELLA_MM = 1000;
constexpr int OBJECT_HEIGHT_HANDBAG_MM = 300;
constexpr int OBJECT_HEIGHT_TIE_MM = 400;
constexpr int OBJECT_HEIGHT_SUITCASE_MM = 600;
constexpr int OBJECT_HEIGHT_FRISBEE_MM = 250;
constexpr int OBJECT_HEIGHT_SKIS_MM = 1700;
constexpr int OBJECT_HEIGHT_SNOWBOARD_MM = 1500;
constexpr int OBJECT_HEIGHT_SPORTS_BALL_MM = 220;
constexpr int OBJECT_HEIGHT_KITE_MM = 1000;
constexpr int OBJECT_HEIGHT_BASEBALL_BAT_MM = 850;
constexpr int OBJECT_HEIGHT_BASEBALL_GLOVE_MM = 300;
constexpr int OBJECT_HEIGHT_SKATEBOARD_MM = 800;
constexpr int OBJECT_HEIGHT_SURFBOARD_MM = 2000;
constexpr int OBJECT_HEIGHT_TENNIS_RACKET_MM = 680;
constexpr int OBJECT_HEIGHT_BOTTLE_MM = 250;
constexpr int OBJECT_HEIGHT_WINE_GLASS_MM = 200;
constexpr int OBJECT_HEIGHT_CUP_MM = 100;
constexpr int OBJECT_HEIGHT_FORK_MM = 200;
constexpr int OBJECT_HEIGHT_KNIFE_MM = 220;
constexpr int OBJECT_HEIGHT_SPOON_MM = 180;
constexpr int OBJECT_HEIGHT_BOWL_MM = 120;
constexpr int OBJECT_HEIGHT_BANANA_MM = 180;
constexpr int OBJECT_HEIGHT_APPLE_MM = 80;
constexpr int OBJECT_HEIGHT_SANDWICH_MM = 80;
constexpr int OBJECT_HEIGHT_ORANGE_MM = 80;
constexpr int OBJECT_HEIGHT_BROCCOLI_MM = 150;
constexpr int OBJECT_HEIGHT_CARROT_MM = 180;
constexpr int OBJECT_HEIGHT_HOT_DOG_MM = 150;
constexpr int OBJECT_HEIGHT_PIZZA_MM = 300;
constexpr int OBJECT_HEIGHT_DONUT_MM = 100;
constexpr int OBJECT_HEIGHT_CAKE_MM = 150;
constexpr int OBJECT_HEIGHT_CHAIR_MM = 900;
constexpr int OBJECT_HEIGHT_COUCH_MM = 850;
constexpr int OBJECT_HEIGHT_POTTED_PLANT_MM = 500;
constexpr int OBJECT_HEIGHT_BED_MM = 600;
constexpr int OBJECT_HEIGHT_DINING_TABLE_MM = 750;
constexpr int OBJECT_HEIGHT_TOILET_MM = 400;
constexpr int OBJECT_HEIGHT_TV_MM = 600;
constexpr int OBJECT_HEIGHT_LAPTOP_MM = 250;
constexpr int OBJECT_HEIGHT_MOUSE_MM = 40;
constexpr int OBJECT_HEIGHT_REMOTE_MM = 180;
constexpr int OBJECT_HEIGHT_KEYBOARD_MM = 30;
constexpr int OBJECT_HEIGHT_CELL_PHONE_MM = 150;
constexpr int OBJECT_HEIGHT_MICROWAVE_MM = 300;
constexpr int OBJECT_HEIGHT_OVEN_MM = 850;
constexpr int OBJECT_HEIGHT_TOASTER_MM = 200;
constexpr int OBJECT_HEIGHT_SINK_MM = 200;
constexpr int OBJECT_HEIGHT_REFRIGERATOR_MM = 1700;
constexpr int OBJECT_HEIGHT_BOOK_MM = 240;
constexpr int OBJECT_HEIGHT_CLOCK_MM = 300;
constexpr int OBJECT_HEIGHT_VASE_MM = 300;
constexpr int OBJECT_HEIGHT_SCISSORS_MM = 180;
constexpr int OBJECT_HEIGHT_TEDDY_BEAR_MM = 400;
constexpr int OBJECT_HEIGHT_HAIR_DRIER_MM = 250;
constexpr int OBJECT_HEIGHT_TOOTHBRUSH_MM = 180;

constexpr float CAMERA_AVOIDANCE_STEERING_GAIN = 1.0;
constexpr int CAMERA_AVOIDANCE_OVERRIDE_DISTANCE_MM = 4000;  // also the trusted-threat cutoff, see about.txt
constexpr int CAMERA_AVOIDANCE_HARD_STOP_DISTANCE_MM = 700;
constexpr float CAMERA_AVOIDANCE_MIN_SPEED_SCALE = 0.1;
constexpr int CAMERA_AVOIDANCE_OVERRIDE_PORT = 7011;
constexpr unsigned long CAMERA_AVOIDANCE_OVERRIDE_TIMEOUT_MS = 300;

}  // namespace Constants
