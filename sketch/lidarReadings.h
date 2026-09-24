#pragma once

namespace LidarReadings {

void begin();
void update();
bool getOverride();  // Bridge RPC handler: true when last reading < LIDAR_OVERRIDE_THRESHOLD_MM

}  // namespace LidarReadings
