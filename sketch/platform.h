#pragma once

namespace Platform {

void begin();             // Attach the three servos, centre them
void update();            // Call every loop(): writes current targets
int  setServo1(int deg);  // Bridge RPC handler, servo on pin 9
int  setServo2(int deg);  // Bridge RPC handler, servo on pin 10
int  setServo3(int deg);  // Bridge RPC handler, servo on pin 11
void setPulseUs(int i, int us);  // Direct pulse command (µs, clamped to the servo's min..max)

}  // namespace Platform
