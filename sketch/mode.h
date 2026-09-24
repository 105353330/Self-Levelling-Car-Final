#pragma once

namespace Mode {

void begin();       // Start in MODE_IDLE
void update();      // Call first in loop(): applies a pending mode change
int  get();         // Current mode, one of Constants::MODE_*
int  set(int mode); // Bridge RPC handler: queues a mode change, applied on the next update()

}  // namespace Mode
