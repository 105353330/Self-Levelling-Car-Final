#pragma once

namespace Steering {

void begin();            // Attach steering servo, centre it
void update();           // Call every loop(): failsafe, servo write
int  setTurn(int deg);   // Bridge RPC handler, -180 (full left) .. 180 (full right)

}  // namespace Steering
