#include "navigation/JunctionHandler.h"

#include <stdlib.h>

#include "RobotConfig.h"

bool JunctionHandler::detected(const SensorFrame& frame,
                               const LineSegment& trackedSegment) const {
    const bool pathIsWide = trackedSegment.activeCount >= cfg::kJunctionActiveSensors;
    const bool pathReachesEdge = trackedSegment.firstSensor == 0 ||
                                 trackedSegment.lastSensor == kViperSensorCount - 1;
    return frame.wide && pathIsWide && pathReachesEdge;
}

bool JunctionHandler::centreReacquired(const SensorFrame& frame) const {
    return frame.lineVisible && abs(frame.position) <= cfg::kTurnReacquirePosition &&
           (frame.line[7] >= cfg::kActiveThreshold ||
            frame.line[8] >= cfg::kActiveThreshold);
}
