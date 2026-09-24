#pragma once

namespace PwmCalibration {

void begin();           // No-op, ready before first command
void update();          // Call every loop(): (re)attaches pin, writes pending PWM
int  setPin(int pin);   // Bridge RPC handler: selects the output pin, clears any pending PWM
int  setPWM(int us);    // Bridge RPC handler: pulse width to write, µs

}  // namespace PwmCalibration
