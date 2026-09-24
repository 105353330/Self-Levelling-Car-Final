#pragma once

namespace PlatformLevel {

void begin();   // Reset the filter and seed the slew state at servo centre (called again on entering LEVEL/DRIVE)
void update();  // Call every loop(): accelerometer -> servo pulses via Platform::setPulseUs
float getRoll();   // Bridge RPC handler: latest roll error, degrees
float getPitch();  // Bridge RPC handler: latest pitch error, degrees
int getUs1();      // Bridge RPC handler: latest commanded pulse, servo on pin 9
int getUs2();      // Bridge RPC handler: latest commanded pulse, servo on pin 10
int getUs3();      // Bridge RPC handler: latest commanded pulse, servo on pin 11
int setParam(int idx, float value);  // Bridge RPC handler: 0,1 = roll,pitch zero; 2..4 = INV_ROLL_0..2; 5..7 = INV_PITCH_0..2

}  // namespace PlatformLevel
