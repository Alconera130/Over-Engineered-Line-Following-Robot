#pragma once

#include "RobotTypes.h"

class JunctionHandler {
public:
    static bool detected(const SensorFrame& frame,
                         const LineSegment& trackedSegment);
    static bool centreReacquired(const SensorFrame& frame);
};
