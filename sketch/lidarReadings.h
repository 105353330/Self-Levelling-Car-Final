#pragma once

namespace LidarReadings {

void begin();
void update();
bool getOverride();  // Bridge RPC handler: true when last reading < LIDAR_OVERRIDE_THRESHOLD_MM
int  getStatus();    // Bridge RPC handler: 0 = not initialised yet, 1 = library init OK, 2 = library init failed (raw fallback)
int  getDistance();  // Bridge RPC handler: last reading, mm (-1 if the last read timed out)

}  // namespace LidarReadings
