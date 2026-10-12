#include "navigation/BrokenLineBridge.h"

#include "control/MotorDriver.h"
#include "RobotConfig.h"
#include "RobotTypes.h"

void BrokenLineBridge::drive(MotorDriver& motors,
                             const ControlOutput& lastControl) const {
    const float correction = lastControl.correction * cfg::kGapCorrectionScale;
    const int16_t left = static_cast<int16_t>(constrain(
        cfg::kGapBridgePwm + correction,
        -static_cast<float>(cfg::kMotorMaxPwm),
        static_cast<float>(cfg::kMotorMaxPwm)));
    const int16_t right = static_cast<int16_t>(constrain(
        cfg::kGapBridgePwm - correction,
        -static_cast<float>(cfg::kMotorMaxPwm),
        static_cast<float>(cfg::kMotorMaxPwm)));
    motors.command(left, right);
}
