#include "Robot.h"

#include "Diagnostics.h"

void Robot::startCalibration() {
    faultRestartPending_ = false;
    controller_.reset();
    junctionLocked_ = false;
    sharpTurns_.reset();
    calibration_.begin(sensors_, motors_);
    setState(RobotState::CALIBRATING);
    diagnostics::port().println(F("Calibrating for 4.8 s: robot will sweep left/right. Keep it over line and floor."));
}

void Robot::updateCalibration() {
    if (!calibration_.update(sensors_, motors_)) {
        return;
    }

    if (calibration_.succeeded()) {
        setState(RobotState::WAIT_START);
        diagnostics::port().println(F("Calibration saved. Release START to run."));
    } else {
        setState(RobotState::CALIBRATION_REQUIRED);
        diagnostics::port().println(F("Calibration failed: every X16 channel must see both line and floor. Try again."));
    }
}

void Robot::beginRun(bool resumeRoute) {
    if (!sensors_.calibrationValid()) {
        return;
    }

    faultRestartPending_ = false;
    controller_.resetGains();
    controller_.reset();
    trackTimer_.reset();
    startJunctionPending_ = cfg::kEnableStartZoneExit;
    if (!startJunctionPending_) {
        trackTimer_.start(millis());
    }
    junctionRoutingEnabled_ = cfg::kEnableJunctionRouting;
    motors_.setSpeedScalePercent(100);
    if (!resumeRoute) {
        route_.reset();
    }
    routeChoicePending_ = false;
    routeSelectionPending_ = false;
    junctionWasReturningFromDeadEnd_ = false;
    deadEnd_.reset();
    tracker_.reset();
    sharpTurns_.reset();
    lostSinceMs_ = 0;
    stopSinceMs_ = 0;
    lastSeenPosition_ = 0;
    junctionLocked_ = false;
    returningFromDeadEnd_ = false;
    startExitNarrowFrames_ = 0;
    lastControlUs_ = micros();

    if (cfg::kEnableStartZoneExit) {
        setState(RobotState::START_EXIT);
        diagnostics::port().println(F("Leaving start zone; the first junction will start the run."));
    } else {
        setState(RobotState::RUNNING);
        diagnostics::port().println(F("RUN"));
    }
}

void Robot::processTrackTime() {
    if (!isRunningState()) {
        return;
    }

    TrackTimeEvent event{};
    if (!trackTimer_.poll(millis(), event)) {
        return;
    }

    if (event.command == TrackTimeCommand::STOP) {
        motors_.emergencyStop();
        controller_.reset();
        setState(RobotState::WAIT_START);
        diagnostics::port().print(F("Track-time stop at "));
        diagnostics::port().print(trackTimer_.elapsedMs(millis()));
        diagnostics::port().println(F(" ms. Tap START to run again."));
        return;
    }

    if (event.command == TrackTimeCommand::ENABLE_JUNCTION_ROUTING ||
        event.command == TrackTimeCommand::DISABLE_JUNCTION_ROUTING) {
        junctionRoutingEnabled_ =
            event.command == TrackTimeCommand::ENABLE_JUNCTION_ROUTING;
        if (!junctionRoutingEnabled_ && cfg::kStraightThroughWhenJunctionRoutingDisabled) {
            if (state_ == RobotState::JUNCTION_ENTRY) {
                if (routeChoicePending_) {
                    route_.retryLastChoice();
                }
                routeSelectionPending_ = false;
                routeChoicePending_ = false;
                pendingTurn_ = JunctionAction::STRAIGHT;
                junctionWasReturningFromDeadEnd_ = false;
            } else if (state_ == RobotState::JUNCTION_TURN &&
                       (pendingTurn_ != JunctionAction::STRAIGHT || routeChoicePending_)) {
                if (routeChoicePending_) {
                    route_.retryLastChoice();
                }
                routeSelectionPending_ = false;
                routeChoicePending_ = false;
                pendingTurn_ = JunctionAction::STRAIGHT;
                junctionWasReturningFromDeadEnd_ = false;
                setState(RobotState::JUNCTION_TURN);
            }
        }
        diagnostics::port().print(F("Track-time junction routing "));
        diagnostics::port().print(junctionRoutingEnabled_ ? F("enabled at ") : F("disabled at "));
        diagnostics::port().print(trackTimer_.elapsedMs(millis()));
        if (!junctionRoutingEnabled_ && cfg::kStraightThroughWhenJunctionRoutingDisabled) {
            diagnostics::port().println(F(" ms; active and future junction routes go straight."));
        } else {
            diagnostics::port().println(F(" ms."));
        }
        return;
    }

    if (event.command == TrackTimeCommand::SET_SPEED_PERCENT) {
        diagnostics::port().print(F("Track-time speed set to "));
        diagnostics::port().print(event.speedPercent);
        diagnostics::port().print(F("% at "));
    } else if (event.command == TrackTimeCommand::SET_PID_GAINS) {
        controller_.setGains(event.pidP, event.pidI, event.pidD);
        diagnostics::port().print(F("Track-time PID set to P="));
        diagnostics::port().print(controller_.pidP(), 2);
        diagnostics::port().print(F(" I="));
        diagnostics::port().print(controller_.pidI(), 2);
        diagnostics::port().print(F(" D="));
        diagnostics::port().print(controller_.pidD(), 2);
        diagnostics::port().print(F(" at "));
    }
    diagnostics::port().print(trackTimer_.elapsedMs(millis()));
    diagnostics::port().println(F(" ms."));
}
