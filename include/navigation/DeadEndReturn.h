#pragma once

#include <Arduino.h>

#include "RobotTypes.h"

class MotorDriver;

enum class DeadEndResult : uint8_t { BACKING_UP, TURNING, REACQUIRED, FAILED };

class DeadEndReturn {
public:
    void reset();
    bool begin(int16_t lastSeenPosition);
    DeadEndResult update(const SensorFrame& frame, MotorDriver& motors);

private:
    uint32_t startedMs_ = 0;
    uint32_t turnStartedMs_ = 0;
    uint8_t returns_ = 0;
    int8_t direction_ = 1;
    bool turning_ = false;
};
