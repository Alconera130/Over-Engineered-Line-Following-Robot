#include "navigation/SegmentTracker.h"

#include <limits.h>
#include <stdlib.h>

#include "RobotConfig.h"

const LineSegment* SegmentTracker::select(const SensorFrame& frame,
                                          bool allowDistantReacquire) {
    if (!frame.lineVisible || frame.segmentCount == 0) {
        return nullptr;
    }

    const LineSegment* candidate = nullptr;
    int16_t smallestDistance = INT16_MAX;
    const int16_t reference = initialised_ ? trackedPosition_ : 0;
    for (uint8_t i = 0; i < frame.segmentCount; ++i) {
        const LineSegment& segment = frame.segments[i];
        const int16_t distance = abs(segment.position - reference);
        if (candidate == nullptr || distance < smallestDistance ||
            (distance == smallestDistance && segment.strength > candidate->strength)) {
            candidate = &segment;
            smallestDistance = distance;
        }
    }

    if (candidate == nullptr ||
        (cfg::kEnableLineSegmentLock && initialised_ && !allowDistantReacquire &&
         frame.segmentCount > 1 &&
         smallestDistance > cfg::kLineLockMaximumPositionJump)) {
        return nullptr;
    }

    trackedPosition_ = candidate->position;
    initialised_ = true;
    return candidate;
}

void SegmentTracker::reset() {
    trackedPosition_ = 0;
    initialised_ = false;
}
