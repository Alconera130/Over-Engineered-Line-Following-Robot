#include "navigation/BrokenLineBridge.h"

#include "control/MotorDriver.h"
#include "RobotConfig.h"
#include "RobotTypes.h"

void BrokenLineBridge::drive(MotorDriver& motors,
                             const ControlOutput& lastControl) const {
    const int16_t correction = static_cast<int16_t>(constrain(
        lastControl.correction * cfg::kGapCorrectionScale,
        -static_cast<float>(cfg::kMaxCorrection),
        static_cast<float>(cfg::kMaxCorrection)));
    motors.command(constrain(cfg::kGapBridgePwm + correction,
                             -cfg::kMotorMaxPwm, cfg::kMotorMaxPwm),
                   constrain(cfg::kGapBridgePwm - correction,
                             -cfg::kMotorMaxPwm, cfg::kMotorMaxPwm));
}
