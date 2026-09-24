#pragma once

namespace ServoCalibration {

void begin();            // Restart the test from step 0 (called again on entering MODE_SERVO_CAL)
void update();           // Call every loop(): runs the open-loop servo response test while in MODE_SERVO_CAL
int  getStep();          // Bridge RPC handler: 0..CAL_STEP_COUNT-1 while running, CAL_STEP_COUNT when finished
float getValue(int idx); // Bridge RPC handler: 0,1 = level roll,pitch; 2+4*s+{0,1,2,3} = servo s at +step roll,pitch, -step roll,pitch

}  // namespace ServoCalibration
