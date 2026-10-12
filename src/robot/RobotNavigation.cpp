#include "Robot.h"

#include "Diagnostics.h"

namespace {
const __FlashStringHelper* junctionActionName(JunctionAction action) {
    switch (action) {
        case JunctionAction::LEFT: return F("left");
        case JunctionAction::RIGHT: return F("right");
        case JunctionAction::STRAIGHT: return F("straight");
        case JunctionAction::ACUTE_LEFT: return F("acute left");
        case JunctionAction::ACUTE_RIGHT: return F("acute right");
    }
    return F("unknown");
}
}  // namespace

void Robot::updateStartExit() {
    sensors_.read(lastFrame_);
    const bool narrowLine = lastFrame_.lineVisible &&
                            lastFrame_.activeCount <= cfg::kStartExitMaximumActiveSensors;
    startExitNarrowFrames_ = narrowLine ? startExitNarrowFrames_ + 1U : 0U;

    if (startExitNarrowFrames_ >= cfg::kStartExitConfirmFrames) {
        controller_.reset();
        tracker_.reset();
        lastControlUs_ = micros();
        setState(RobotState::RUNNING);
        diagnostics::port().println(startJunctionPending_
                                        ? F("Start zone clear. Waiting for first junction.")
                                        : F("Start zone clear. RUN"));
        return;
    }

    if (millis() - stateStartedMs_ >= cfg::kStartExitTimeoutMs) {
        fault(F("start zone exit timed out"));
        return;
    }

    motors_.command(cfg::kStartExitPwm, cfg::kStartExitPwm);
}

void Robot::updateRunning() {
    sensors_.read(lastFrame_);
    const uint32_t now = millis();

    if (lastFrame_.polarityChanged) {
        diagnostics::port().print(F("Line polarity switched to "));
        diagnostics::port().println(lastFrame_.polarity == LinePolarity::DARK ? F("dark") : F("light"));
    }

    const LineSegment* trackedSegment = tracker_.select(lastFrame_);
    if (trackedSegment == nullptr) {
        if (lostSinceMs_ == 0) {
            lostSinceMs_ = now;
        }
        if (now - lostSinceMs_ <= cfg::kBrokenLineBridgeMs) {
            bridge_.drive(motors_, lastControl_);
            lastControlUs_ = micros();
            emitTelemetry();
            return;
        }
        startRecovery();
        return;
    }

    const int16_t trackedPosition = cfg::kEnableLineSegmentLock
                                       ? trackedSegment->position : lastFrame_.position;
    lastSeenPosition_ = trackedPosition;
    lostSinceMs_ = 0;

    if (cfg::kEnableStopLine && lastFrame_.activeCount >= cfg::kStopActiveSensors &&
        lastFrame_.strength >= cfg::kStopStrength) {
        if (stopSinceMs_ == 0) {
            stopSinceMs_ = now;
        }
        if (now - stopSinceMs_ >= cfg::kStopHoldMs) {
            finish();
            return;
        }
    } else {
        stopSinceMs_ = 0;
    }

    const bool junctionPresent =
        JunctionHandler::detected(lastFrame_, *trackedSegment);
    if (!junctionPresent) {
        junctionLocked_ = false;
    } else if (!junctionLocked_) {
        junctionLocked_ = true;
        signalJunctionDetected();
        if (startJunctionPending_) {
            beginStraightThrough(true);
            return;
        }
        if (junctionRoutingEnabled_) {
            beginTurn(lastFrame_, false);
            return;
        } else if (cfg::kStraightThroughWhenJunctionRoutingDisabled) {
            beginStraightThrough();
            return;
        }
    }

    if (!junctionPresent && !returningFromDeadEnd_ &&
        sharpTurns_.detected(lastFrame_, *trackedSegment)) {
        signalJunctionDetected();
        if (junctionRoutingEnabled_ && cfg::kEnableSharpTurnRouting) {
            beginTurn(lastFrame_, true);
            return;
        } else if (!junctionRoutingEnabled_ &&
                   cfg::kStraightThroughWhenJunctionRoutingDisabled) {
            beginStraightThrough();
            return;
        }
    }

    const uint32_t nowUs = micros();
    const float dt = (nowUs - lastControlUs_) / 1000000.0F;
    lastControlUs_ = nowUs;
    lastControl_ = controller_.update(trackedPosition, dt);
    motors_.command(lastControl_.left, lastControl_.right);
    emitTelemetry();
}

void Robot::beginTurn(const SensorFrame& frame, bool sharpTurn) {
    const bool wasReturningFromDeadEnd = returningFromDeadEnd_;
    returningFromDeadEnd_ = false;
    routeChoicePending_ = false;
    routeSelectionPending_ = !sharpTurn;
    junctionWasReturningFromDeadEnd_ = wasReturningFromDeadEnd;
    if (sharpTurn) {
        pendingTurn_ = route_.choose(frame, wasReturningFromDeadEnd, true);
        routeChoicePending_ = !wasReturningFromDeadEnd;
    }
    controller_.reset();

    if (sharpTurn) {
        setState(RobotState::JUNCTION_TURN);
        diagnostics::port().print(F("Sharp turn: "));
        diagnostics::port().println(junctionActionName(pendingTurn_));
    } else {
        pendingTurn_ = JunctionAction::STRAIGHT;
        setState(RobotState::JUNCTION_ENTRY);
        diagnostics::port().println(F("Junction detected; checking route branch after entry."));
    }
}

void Robot::beginStraightThrough(bool startGate) {
    returningFromDeadEnd_ = false;
    routeChoicePending_ = false;
    routeSelectionPending_ = false;
    junctionWasReturningFromDeadEnd_ = false;
    pendingTurn_ = JunctionAction::STRAIGHT;
    controller_.reset();
    setState(RobotState::JUNCTION_ENTRY);
    diagnostics::port().println(startGate
                                    ? F("Start junction: crossing straight before starting run timer and route.")
                                    : F("Junction routing disabled; crossing straight."));
}

void Robot::updateJunction() {
    sensors_.read(lastFrame_);
    const uint32_t elapsed = millis() - stateStartedMs_;

    if (state_ == RobotState::JUNCTION_ENTRY) {
        motors_.command(cfg::kJunctionEntryPwm, cfg::kJunctionEntryPwm);
        if (elapsed >= cfg::kJunctionEntryMs) {
            if (routeSelectionPending_) {
                pendingTurn_ = route_.choose(lastFrame_,
                                             junctionWasReturningFromDeadEnd_, false);
                routeChoicePending_ = !junctionWasReturningFromDeadEnd_;
                routeSelectionPending_ = false;
                diagnostics::port().print(F("Junction route: "));
                diagnostics::port().println(junctionActionName(pendingTurn_));
            }
            setState(RobotState::JUNCTION_TURN);
        }
        return;
    }

    if (pendingTurn_ == JunctionAction::STRAIGHT) {
        motors_.command(cfg::kJunctionEntryPwm, cfg::kJunctionEntryPwm);
        if (elapsed >= cfg::kStraightTraverseMs) {
            routeChoicePending_ = false;
            if (startJunctionPending_) {
                trackTimer_.start(millis());
                startJunctionPending_ = false;
                diagnostics::port().println(F("Start junction passed. Run timer and TrackTime started; route selection continues at the next junction."));
            }
            setState(RobotState::RUNNING);
            tracker_.reset();
            lastControlUs_ = micros();
        }
        return;
    }

    const bool acute = pendingTurn_ == JunctionAction::ACUTE_LEFT ||
                       pendingTurn_ == JunctionAction::ACUTE_RIGHT;
    const bool left = pendingTurn_ == JunctionAction::LEFT ||
                      pendingTurn_ == JunctionAction::ACUTE_LEFT;
    const int16_t outsidePwm = acute ? cfg::kAcuteTurnOutsidePwm
                                     : cfg::kTurnOutsidePwm;
    const int16_t insidePwm = acute ? cfg::kAcuteTurnInsidePwm
                                    : cfg::kTurnInsidePwm;
    const uint16_t minimumTurnMs = acute ? cfg::kAcuteMinimumTurnMs
                                         : cfg::kMinimumTurnMs;
    const uint16_t maximumTurnMs = acute ? cfg::kAcuteMaximumTurnMs
                                         : cfg::kMaximumTurnMs;
    motors_.command(left ? insidePwm : outsidePwm,
                    left ? outsidePwm : insidePwm);
    if (elapsed >= minimumTurnMs &&
        JunctionHandler::centreReacquired(lastFrame_)) {
        routeChoicePending_ = false;
        controller_.reset();
        setState(RobotState::RUNNING);
        tracker_.reset();
        lastControlUs_ = micros();
    } else if (elapsed >= maximumTurnMs) {
        diagnostics::port().println(F("Turn did not reacquire a line."));
        startRecovery();
    }
}

void Robot::signalJunctionDetected() {
    junctionMessageStartedMs_ = millis();
    junctionMessageActive_ = true;
}

void Robot::startRecovery() {
    if (!cfg::kEnablePathRecovery) {
        fault(F("line lost; path recovery is disabled"));
        return;
    }
    controller_.reset();
    recovery_.begin();
    setState(RobotState::LINE_RECOVERY);
    diagnostics::port().println(F("Line lost: searching last-known side."));
}

void Robot::updateRecovery() {
    sensors_.read(lastFrame_);
    const LineSegment* recoveredSegment = tracker_.select(lastFrame_, true);
    if (recoveredSegment != nullptr) {
        routeChoicePending_ = false;
        lastSeenPosition_ = cfg::kEnableLineSegmentLock
                                ? recoveredSegment->position : lastFrame_.position;
        controller_.reset();
        lastControlUs_ = micros();
        setState(RobotState::RUNNING);
        diagnostics::port().println(F("Line reacquired."));
        return;
    }

    const uint32_t now = millis();
    if (cfg::kEnableDeadEndReturn && recovery_.deadEndDue(now)) {
        if (!deadEnd_.begin(lastSeenPosition_)) {
            fault(F("dead-end return limit reached"));
            return;
        }
        setState(RobotState::DEAD_END_TURN);
        diagnostics::port().println(F("Likely dead end: executing timed U-turn."));
        return;
    }
    if (recovery_.timedOut(now)) {
        fault(F("line not found before recovery timeout"));
        return;
    }

    recovery_.drive(motors_, lastSeenPosition_);
}

void Robot::updateDeadEndTurn() {
    sensors_.read(lastFrame_);
    const DeadEndResult result = deadEnd_.update(lastFrame_, motors_);
    if (result == DeadEndResult::REACQUIRED) {
        routeChoicePending_ = false;
        controller_.reset();
        returningFromDeadEnd_ = true;
        junctionLocked_ = false;
        tracker_.reset();
        lastControlUs_ = micros();
        setState(RobotState::RUNNING);
        diagnostics::port().println(F("Return line acquired; seeking another branch."));
    } else if (result == DeadEndResult::FAILED) {
        fault(F("dead-end U-turn did not reacquire a return line"));
    }
}

void Robot::finish() {
    motors_.emergencyStop();
    setState(RobotState::FINISHED);
    diagnostics::port().println(F("Stop marker confirmed. Robot finished."));
}

void Robot::fault(const __FlashStringHelper* reason) {
    faultRestartPending_ = false;
    if (routeChoicePending_) {
        route_.retryLastChoice();
        routeChoicePending_ = false;
    }
    motors_.emergencyStop();
    setState(RobotState::FAULT);
    diagnostics::port().print(F("FAULT: "));
    diagnostics::port().println(reason);
    diagnostics::port().println(F("Tap START once to resume route; double tap to restart it; hold one second to recalibrate."));
}

void Robot::setState(RobotState state) {
    state_ = state;
    stateStartedMs_ = millis();
}

bool Robot::isRunningState() const {
    return state_ == RobotState::START_EXIT || state_ == RobotState::RUNNING ||
           state_ == RobotState::JUNCTION_ENTRY || state_ == RobotState::JUNCTION_TURN ||
           state_ == RobotState::LINE_RECOVERY || state_ == RobotState::DEAD_END_TURN;
}
