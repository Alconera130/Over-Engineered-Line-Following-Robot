#include "system/CalibrationSweep.h"

#include "control/MotorDriver.h"
#include "RobotConfig.h"
#include "sensors/ViperX16.h"

void CalibrationSweep::begin(ViperX16& sensors, MotorDriver& motors) {
    motors.emergencyStop();
    sensors.startCalibration();
    startedMs_ = millis();
    active_ = true;
    succeeded_ = false;
}

bool CalibrationSweep::update(ViperX16& sensors, MotorDriver& motors) {
    if (!active_) {
        return false;
    }

    const uint32_t elapsed = millis() - startedMs_;
    const int8_t direction = ((elapsed / cfg::kCalibrationPhaseMs) & 1U) == 0U ? 1 : -1;
    motors.command(-direction * cfg::kCalibrationTurnPwm,
                    direction * cfg::kCalibrationTurnPwm);
    sensors.sampleCalibration();

    if (elapsed < cfg::kCalibrationDurationMs) {
        return false;
    }

    motors.emergencyStop();
    active_ = false;
    succeeded_ = sensors.finishCalibration();
    return true;
}
