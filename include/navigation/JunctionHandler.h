#pragma once

#include "RobotTypes.h"

class JunctionHandler {
public:
    bool detected(const SensorFrame& frame, const LineSegment& trackedSegment) const;
    bool centreReacquired(const SensorFrame& frame) const;
};
