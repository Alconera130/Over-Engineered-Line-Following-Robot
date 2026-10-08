#include "navigation/RoutePlanner.h"

#include <string.h>

#include "RobotConfig.h"

void RoutePlanner::reset() {
    routeIndex_ = 0;
    pathDepth_ = 0;
}

void RoutePlanner::retryLastChoice() {
    if (routeIndex_ > 0) {
        --routeIndex_;
    }
    if (pathDepth_ > 0) {
        --pathDepth_;
    }
}

JunctionAction RoutePlanner::choose(const SensorFrame& frame, bool returningFromDeadEnd,
                                    bool sharpTurnOnly) {
    JunctionAction action;
    if (returningFromDeadEnd) {
        bool avoidLastChoice = false;
        JunctionAction avoided = JunctionAction::STRAIGHT;
        if (pathDepth_ > 0) {
            avoided = path_[--pathDepth_];
            avoidLastChoice = true;
        }
        action = firstAvailable(frame, sharpTurnOnly, avoidLastChoice, avoided);
    } else {
        const JunctionAction requested = cfg::kRoute[routeIndex_ % cfg::kRouteLength];
        ++routeIndex_;
        action = available(requested, frame, sharpTurnOnly)
                     ? requested
                     : firstAvailable(frame, sharpTurnOnly);
    }
    remember(action);
    return action;
}

bool RoutePlanner::available(JunctionAction action, const SensorFrame& frame,
                             bool sharpTurnOnly) const {
    switch (action) {
        case JunctionAction::LEFT:
            return frame.leftEdge;
        case JunctionAction::RIGHT:
            return frame.rightEdge;
        case JunctionAction::STRAIGHT:
            return !sharpTurnOnly &&
                   (frame.line[6] >= cfg::kBranchThreshold ||
                    frame.line[7] >= cfg::kBranchThreshold ||
                    frame.line[8] >= cfg::kBranchThreshold ||
                    frame.line[9] >= cfg::kBranchThreshold);
    }
    return false;
}

JunctionAction RoutePlanner::firstAvailable(const SensorFrame& frame,
                                            bool sharpTurnOnly,
                                            bool avoidLastChoice,
                                            JunctionAction avoided) const {
    const JunctionAction choices[] = {
        JunctionAction::LEFT, JunctionAction::STRAIGHT, JunctionAction::RIGHT
    };
    for (JunctionAction choice : choices) {
        if ((!avoidLastChoice || choice != avoided) &&
            available(choice, frame, sharpTurnOnly)) {
            return choice;
        }
    }
    if (avoidLastChoice) {
        return firstAvailable(frame, sharpTurnOnly);
    }
    return JunctionAction::STRAIGHT;
}

void RoutePlanner::remember(JunctionAction action) {
    if (pathDepth_ < cfg::kPathMemoryDepth) {
        path_[pathDepth_++] = action;
    } else {
        memmove(path_, path_ + 1, sizeof(JunctionAction) * (cfg::kPathMemoryDepth - 1));
        path_[cfg::kPathMemoryDepth - 1] = action;
    }
}
