#pragma once

namespace Motor {

void begin();            // Attach ESC, start the arming timer
void update();           // Call every loop(): arming, failsafe, ESC write
int  setSpeed(int us);   // Bridge RPC handler

}  // namespace Motor