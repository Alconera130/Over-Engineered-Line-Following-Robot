#pragma once

#include "RobotTypes.h"

class SegmentTracker {
public:
    const LineSegment* select(const SensorFrame& frame, bool allowDistantReacquire = false);
    void reset();
    int16_t position() const { return trackedPosition_; }

private:
    int16_t trackedPosition_ = 0;
    bool initialised_ = false;
};
