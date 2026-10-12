#pragma once

#include <Arduino.h>

#include "RobotTypes.h"

class TrackTimer {
public:
    void reset();
    void start(uint32_t nowMs);
    bool poll(uint32_t nowMs, TrackTimeEvent& event);
    uint32_t elapsedMs(uint32_t nowMs) const;
    const TrackTimeEvent* peekNextEvent() const;
    uint8_t speedPercent() const { return speedPercent_; }

private:
    uint32_t startedMs_ = 0;
    uint8_t nextEvent_ = 0;
    uint8_t speedPercent_ = 100;
    bool active_ = false;
};
