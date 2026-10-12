#pragma once

#include <Arduino.h>

class MotorDriver;

class LineRecovery {
public:
    void begin();
    bool deadEndDue(uint32_t nowMs) const;
    bool timedOut(uint32_t nowMs) const;
    void drive(MotorDriver& motors, int16_t lastSeenPosition) const;

private:
    uint32_t startedMs_ = 0;
};
