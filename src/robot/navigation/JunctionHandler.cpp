#include "navigation/JunctionHandler.h"

#include <stdlib.h>

#include "RobotConfig.h"

namespace {
uint8_t continuousLineLength(const SensorFrame& frame, uint8_t centre,
                             int8_t direction) {
    uint8_t length = 0;
    int16_t sensor = centre;
    while (sensor >= 0 && sensor < kViperSensorCount &&
           frame.line[sensor] >= cfg::kActiveThreshold) {
        ++length;
        sensor += direction;
    }
    return length;
}

bool hasLongConnectedRunFromCentre(const SensorFrame& frame) {
    const uint8_t longest = max(
        max(continuousLineLength(frame, 7, -1),
            continuousLineLength(frame, 7, 1)),
        max(continuousLineLength(frame, 8, -1),
            continuousLineLength(frame, 8, 1)));
    return longest >= cfg::kJunctionMinimumConnectedSensors;
}
}  // namespace

bool JunctionHandler::detected(const SensorFrame& frame,
                               const LineSegment& trackedSegment) {
    const bool pathIsWide = trackedSegment.activeCount >= cfg::kJunctionActiveSensors;

    // Segment construction tolerates a weak one-sensor gap for steering, but
    // a junction needs a connected line run from the centre. This rejects
    // acute corners that only light the centre and outer sensors as separate
    // patches. Its required length is adjustable in RobotConfig.h.
    return frame.wide && pathIsWide && hasLongConnectedRunFromCentre(frame);
}

bool JunctionHandler::centreReacquired(const SensorFrame& frame) {
    // A wide crossbar has a near-zero whole-array centroid even while the
    // robot is still facing straight through the junction. Do not mistake it
    // for the narrow line reacquired after a left/right turn.
    return frame.lineVisible && !frame.wide &&
           abs(frame.position) <= cfg::kTurnReacquirePosition &&
           (frame.line[7] >= cfg::kActiveThreshold ||
            frame.line[8] >= cfg::kActiveThreshold);
}
