#pragma once

namespace PlatformLevel {

void begin();   // Seed the filter/slew state at servo centre
void update();  // Call every loop(): accelerometer -> servo pulses via Platform::setPulseUs
float getRoll();   // Bridge RPC handler: latest roll error, degrees
float getPitch();  // Bridge RPC handler: latest pitch error, degrees
int getUs1();      // Bridge RPC handler: latest commanded pulse, servo on pin 9
int getUs2();      // Bridge RPC handler: latest commanded pulse, servo on pin 10
int getUs3();      // Bridge RPC handler: latest commanded pulse, servo on pin 11

}  // namespace PlatformLevel
