#pragma once

namespace ServoCalibration {

void begin();
void update();           // Call every loop(): runs the open-loop servo response test when LEVEL_CALIBRATION_MODE = 1
int  getStep();          // Bridge RPC handler: 0..CAL_STEP_COUNT-1 while running, CAL_STEP_COUNT when finished
float getValue(int idx); // Bridge RPC handler: 0,1 = level roll,pitch; 2+4*s+{0,1,2,3} = servo s at +step roll,pitch, -step roll,pitch

}  // namespace ServoCalibration
