#pragma once

#include <Arduino.h>

#include "RobotConfig.h"
#include "RobotTypes.h"

class RoutePlanner {
public:
    void reset();
    void retryLastChoice();
    JunctionAction choose(const SensorFrame& frame, bool returningFromDeadEnd,
                         bool sharpTurnOnly = false);
    JunctionAction peekNextAction() const;
    uint8_t peekNextRouteSlot() const;
    uint8_t previousRouteSlot() const;

private:
    bool available(JunctionAction action, const SensorFrame& frame,
                   bool sharpTurnOnly) const;
    JunctionAction firstAvailable(const SensorFrame& frame,
                                 bool sharpTurnOnly,
                                 bool avoidLastChoice = false,
                                 JunctionAction avoided = JunctionAction::STRAIGHT) const;
    void remember(JunctionAction action);

    uint32_t routeIndex_ = 0;
    uint8_t pathDepth_ = 0;
    JunctionAction path_[cfg::kPathMemoryDepth]{};
};
