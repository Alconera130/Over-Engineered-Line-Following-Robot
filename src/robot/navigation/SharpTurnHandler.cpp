#include "navigation/SharpTurnHandler.h"

#include <stdlib.h>

#include "RobotConfig.h"

void SharpTurnHandler::reset() {
    confirmingFrames_ = 0;
    latched_ = false;
}

bool SharpTurnHandler::detected(const SensorFrame& frame,
                                const LineSegment& trackedSegment) {
    const uint16_t centreStrength = max(frame.line[7], frame.line[8]);
    const bool leftCorner = frame.leftEdge && !frame.rightEdge &&
                            trackedSegment.firstSensor == 0 &&
                            trackedSegment.position <= -cfg::kSharpTurnPositionLimit &&
                            centreStrength <= cfg::kSharpTurnCenterMaximum &&
                            trackedSegment.activeCount <= cfg::kSharpTurnMaximumActiveSensors;
    const bool rightCorner = frame.rightEdge && !frame.leftEdge &&
                             trackedSegment.lastSensor == kViperSensorCount - 1 &&
                             trackedSegment.position >= cfg::kSharpTurnPositionLimit &&
                             centreStrength <= cfg::kSharpTurnCenterMaximum &&
                             trackedSegment.activeCount <= cfg::kSharpTurnMaximumActiveSensors;

    if (!frame.leftEdge && !frame.rightEdge &&
        abs(trackedSegment.position) < cfg::kSharpTurnReleasePosition) {
        latched_ = false;
        confirmingFrames_ = 0;
    }
    if (latched_ || (!leftCorner && !rightCorner)) {
        confirmingFrames_ = 0;
        return false;
    }

    if (++confirmingFrames_ < cfg::kSharpTurnConfirmFrames) {
        return false;
    }

    latched_ = true;
    return true;
}
