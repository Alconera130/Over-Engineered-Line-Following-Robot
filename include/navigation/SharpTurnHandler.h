#pragma once

#include "RobotTypes.h"

class SharpTurnHandler {
public:
    void reset();
    bool detected(const SensorFrame& frame, const LineSegment& trackedSegment);

private:
    uint8_t confirmingFrames_ = 0;
    bool latched_ = false;
};
