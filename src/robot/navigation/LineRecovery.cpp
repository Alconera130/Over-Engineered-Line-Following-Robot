#include "navigation/LineRecovery.h"

#include "control/MotorDriver.h"
#include "RobotConfig.h"

void LineRecovery::begin() {
    startedMs_ = millis();
}

bool LineRecovery::deadEndDue(uint32_t nowMs) const {
    return nowMs - startedMs_ >= cfg::kRecoveryBeforeDeadEndMs;
}

bool LineRecovery::timedOut(uint32_t nowMs) const {
    return nowMs - startedMs_ >= cfg::kRecoveryTimeoutMs;
}

void LineRecovery::drive(MotorDriver& motors, int16_t lastSeenPosition) const {
    const int16_t direction = lastSeenPosition >= 0 ? 1 : -1;
    motors.command(direction * cfg::kRecoveryTurnPwm,
                   -direction * cfg::kRecoveryTurnPwm);
}
