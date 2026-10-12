#include "Robot.h"

void Robot::updateDisplay(uint32_t nowMs) {
    if (junctionMessageActive_ &&
        nowMs - junctionMessageStartedMs_ >= cfg::kOledJunctionMessageMs) {
        junctionMessageActive_ = false;
    }

    OledDisplayState view{};
    view.sensors = &lastFrame_;
    view.sensorDataValid = sensorFrameValid_ && sensors_.calibrationValid();
    view.calibrationRequired = state_ == RobotState::CALIBRATION_REQUIRED;
    view.calibrating = state_ == RobotState::CALIBRATING;
    view.runActive = isRunningState();
    view.trackClockActive = view.runActive && !startJunctionPending_;
    view.elapsedMs = trackTimer_.elapsedMs(nowMs);
    view.nextTrackEvent = trackTimer_.peekNextEvent();
    view.trackTimeEnabled = cfg::kEnableTrackTimeActions;
    view.junctionRoutingEnabled = junctionRoutingEnabled_;
    view.junctionDetected = junctionMessageActive_;
    switch (state_) {
        case RobotState::CALIBRATION_REQUIRED:
            view.statusMessage = "HOLD START TO CAL";
            break;
        case RobotState::WAIT_START:
            view.statusMessage = "READY - PRESS START";
            break;
        case RobotState::CALIBRATING:
            view.statusMessage = "CALIBRATION SWEEP";
            break;
        case RobotState::START_EXIT:
            view.statusMessage = "CLEARING START";
            break;
        case RobotState::LINE_RECOVERY:
            view.statusMessage = "LINE RECOVERY";
            break;
        case RobotState::DEAD_END_TURN:
            view.statusMessage = "U TURN RECOVERY";
            break;
        case RobotState::FINISHED:
            view.statusMessage = "FINISHED";
            break;
        case RobotState::FAULT:
            view.statusMessage = "FAULT - TAP START";
            break;
        case RobotState::RUNNING:
        case RobotState::JUNCTION_ENTRY:
        case RobotState::JUNCTION_TURN:
            if (startJunctionPending_ && !junctionMessageActive_) {
                view.statusMessage = "WAIT FIRST JUNCTION";
            }
            break;
    }
    view.routeActionCurrent = routeChoicePending_;
    view.routeSlot = routeChoicePending_ ? route_.previousRouteSlot()
                                         : route_.peekNextRouteSlot();
    view.routeAction = routeChoicePending_ ? pendingTurn_ : route_.peekNextAction();
    if (view.nextTrackEvent != nullptr) {
        view.timeToTrackEventMs = view.nextTrackEvent->atMs > view.elapsedMs
                                      ? view.nextTrackEvent->atMs - view.elapsedMs
                                      : 0U;
    }
    display_.update(nowMs, view, button_.isDown());
}
