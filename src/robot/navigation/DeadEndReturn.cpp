#include "navigation/DeadEndReturn.h"

#include <stdlib.h>

#include "navigation/JunctionHandler.h"
#include "control/MotorDriver.h"
#include "RobotConfig.h"

void DeadEndReturn::reset() {
    startedMs_ = 0;
    turnStartedMs_ = 0;
    returns_ = 0;
    direction_ = 1;
    turning_ = false;
}

bool DeadEndReturn::begin(int16_t lastSeenPosition) {
    if (returns_ >= cfg::kMaximumDeadEndReturns) {
        return false;
    }
    ++returns_;
    direction_ = lastSeenPosition >= 0 ? 1 : -1;
    startedMs_ = millis();
    turnStartedMs_ = 0;
    turning_ = false;
    return true;
}

DeadEndResult DeadEndReturn::update(const SensorFrame& frame, MotorDriver& motors) {
    const uint32_t now = millis();
    if (!turning_) {
        if (now - startedMs_ < cfg::kDeadEndBackupMs) {
            motors.command(-cfg::kDeadEndBackupPwm, -cfg::kDeadEndBackupPwm);
            return DeadEndResult::BACKING_UP;
        }
        turning_ = true;
        turnStartedMs_ = now;
    }

    const uint32_t elapsed = now - turnStartedMs_;
    motors.command(direction_ * cfg::kDeadEndTurnPwm,
                   -direction_ * cfg::kDeadEndTurnPwm);

    JunctionHandler junctions;
    if (elapsed >= cfg::kDeadEndMinimumTurnMs && junctions.centreReacquired(frame)) {
        return DeadEndResult::REACQUIRED;
    }
    if (elapsed >= cfg::kDeadEndMaximumTurnMs) {
        return DeadEndResult::FAILED;
    }
    return DeadEndResult::TURNING;
}
