#pragma once

namespace AccelerometerReadings {

void begin();
void update();
bool  isConfigured();  // Bridge RPC handler: true once the sensor answered the configure sequence
float getAccelX();  // Bridge RPC handler: latest gx reading, g
float getAccelY();  // Bridge RPC handler: latest gy reading, g
float getAccelZ();  // Bridge RPC handler: latest gz reading, g

}  // namespace AccelerometerReadings
