#include "Robot.h"

#include "Diagnostics.h"

void Robot::begin() {
    diagnostics::port().begin(115200);
    controller_.resetGains();
    button_.begin();
    digitalWrite(cfg::kStatusLedPin, cfg::kStatusLedActiveLow ? HIGH : LOW);
    pinMode(cfg::kStatusLedPin, OUTPUT);
    setStatusLed(false);

    motors_.begin();
    sensors_.begin();
    state_ = sensors_.calibrationValid() ? RobotState::WAIT_START
                                          : RobotState::CALIBRATION_REQUIRED;
    stateStartedMs_ = millis();
    lastControlUs_ = micros();

    diagnostics::port().println();
    diagnostics::port().println(F("Linea Viper X16 competition controller"));
    if (sensors_.calibrationValid()) {
        diagnostics::port().println(F("Calibration loaded. Release START to run; hold, then release, to calibrate."));
    } else {
        diagnostics::port().println(F("No valid calibration. Hold START for one second, then release, to calibrate."));
    }
    diagnostics::port().println(F("Console: r=run, c=calibrate, x=stop, s=status, t=telemetry"));
}

void Robot::update() {
    pollConsole();
    pollButton();
    processTrackTime();
    motors_.setSpeedScalePercent(state_ == RobotState::RUNNING
                                     ? trackTimer_.speedPercent() : 100U);
    motors_.update(micros());

    switch (state_) {
        case RobotState::CALIBRATING: updateCalibration(); break;
        case RobotState::START_EXIT: updateStartExit(); break;
        case RobotState::RUNNING: updateRunning(); break;
        case RobotState::JUNCTION_ENTRY:
        case RobotState::JUNCTION_TURN: updateJunction(); break;
        case RobotState::LINE_RECOVERY: updateRecovery(); break;
        case RobotState::DEAD_END_TURN: updateDeadEndTurn(); break;
        case RobotState::CALIBRATION_REQUIRED:
        case RobotState::WAIT_START:
        case RobotState::FINISHED:
        case RobotState::FAULT: break;
    }

    updateStatusLed();
}

void Robot::pollButton() {
    uint32_t heldMs = 0;
    const bool released = button_.poll(heldMs);
    const uint32_t now = millis();

    if (released) {
        if (isRunningState()) {
            faultRestartPending_ = false;
            motors_.emergencyStop();
            controller_.reset();
            setState(RobotState::WAIT_START);
            diagnostics::port().println(F("Stopped by START button. Release START to run."));
            return;
        }

        const bool canCalibrate = state_ == RobotState::WAIT_START ||
                                  state_ == RobotState::CALIBRATION_REQUIRED ||
                                  state_ == RobotState::FINISHED || state_ == RobotState::FAULT;
        if (canCalibrate && heldMs >= cfg::kLongPressMs) {
            faultRestartPending_ = false;
            startCalibration();
            return;
        }

        if (state_ == RobotState::FAULT && sensors_.calibrationValid()) {
            if (faultRestartPending_) {
                const uint32_t sinceFirstTap = now - faultRestartTapMs_;
                faultRestartPending_ = false;
                if (sinceFirstTap <= cfg::kStartDoubleTapWindowMs) {
                    diagnostics::port().println(F("Double tap: restarting route from the beginning."));
                    beginRun(false);
                } else {
                    diagnostics::port().println(F("Resuming from the saved route position."));
                    beginRun(true);
                }
                return;
            }

            faultRestartPending_ = true;
            faultRestartTapMs_ = now;
            diagnostics::port().println(F("Tap once to resume; tap again within 400 ms to restart route."));
        } else if (state_ == RobotState::WAIT_START && sensors_.calibrationValid()) {
            faultRestartPending_ = false;
            beginRun(false);
        } else if (state_ == RobotState::CALIBRATION_REQUIRED) {
            diagnostics::port().println(F("Calibration is required; hold START for one second, then release."));
        }
    }

    if (faultRestartPending_ && !button_.isDown() &&
        now - faultRestartTapMs_ >= cfg::kStartDoubleTapWindowMs) {
        faultRestartPending_ = false;
        diagnostics::port().println(F("Resuming from the saved route position."));
        beginRun(true);
    }
}

void Robot::pollConsole() {
    while (diagnostics::port().available() > 0) {
        const char command = static_cast<char>(diagnostics::port().read());
        switch (command) {
            case 'r':
            case 'R':
                if (sensors_.calibrationValid()) {
                    beginRun();
                } else {
                    diagnostics::port().println(F("Cannot run: Viper calibration is invalid."));
                }
                break;
            case 'c':
            case 'C': startCalibration(); break;
            case 'x':
            case 'X':
                faultRestartPending_ = false;
                motors_.emergencyStop();
                controller_.reset();
                setState(sensors_.calibrationValid() ? RobotState::WAIT_START
                                                     : RobotState::CALIBRATION_REQUIRED);
                diagnostics::port().println(F("Stopped. Release START or send r to run."));
                break;
            case 's':
            case 'S': showStatus(); break;
            case 't':
            case 'T':
                telemetryEnabled_ = !telemetryEnabled_;
                diagnostics::port().println(telemetryEnabled_ ? F("Telemetry enabled")
                                                               : F("Telemetry disabled"));
                break;
            default: break;
        }
    }
}

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
    motors_.setSpeedScalePercent(100);
    if (!resumeRoute) {
        route_.reset();
    }
    routeChoicePending_ = false;
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
        diagnostics::port().println(F("Leaving start zone."));
    } else {
        setState(RobotState::RUNNING);
        trackTimer_.start(millis());
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
        trackTimer_.start(millis());
        diagnostics::port().println(F("Start zone clear. RUN"));
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

    const bool junctionPresent = junctions_.detected(lastFrame_, *trackedSegment);
    if (!junctionPresent) {
        junctionLocked_ = false;
    } else if (!junctionLocked_) {
        junctionLocked_ = true;
        beginTurn(lastFrame_, false);
        return;
    }

    if (!junctionPresent && !returningFromDeadEnd_ &&
        sharpTurns_.detected(lastFrame_, *trackedSegment)) {
        beginTurn(lastFrame_, true);
        return;
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
    pendingTurn_ = route_.choose(frame, wasReturningFromDeadEnd, sharpTurn);
    routeChoicePending_ = !wasReturningFromDeadEnd;
    returningFromDeadEnd_ = false;
    controller_.reset();
    turnLedStartedMs_ = millis();
    turnLedPulseActive_ = true;

    if (sharpTurn) {
        setState(RobotState::JUNCTION_TURN);
    } else {
        setState(RobotState::JUNCTION_ENTRY);
    }

    diagnostics::port().print(sharpTurn ? F("Sharp turn: ") : F("Junction: "));
    if (pendingTurn_ == JunctionAction::LEFT) diagnostics::port().println(F("left"));
    if (pendingTurn_ == JunctionAction::RIGHT) diagnostics::port().println(F("right"));
    if (pendingTurn_ == JunctionAction::STRAIGHT) diagnostics::port().println(F("straight"));
}

void Robot::updateJunction() {
    sensors_.read(lastFrame_);
    const uint32_t elapsed = millis() - stateStartedMs_;

    if (state_ == RobotState::JUNCTION_ENTRY) {
        motors_.command(cfg::kJunctionEntryPwm, cfg::kJunctionEntryPwm);
        if (elapsed >= cfg::kJunctionEntryMs) {
            setState(RobotState::JUNCTION_TURN);
        }
        return;
    }

    if (pendingTurn_ == JunctionAction::STRAIGHT) {
        motors_.command(cfg::kJunctionEntryPwm, cfg::kJunctionEntryPwm);
        if (elapsed >= cfg::kStraightTraverseMs) {
            routeChoicePending_ = false;
            setState(RobotState::RUNNING);
            tracker_.reset();
            lastControlUs_ = micros();
        }
        return;
    }

    const bool left = pendingTurn_ == JunctionAction::LEFT;
    motors_.command(left ? cfg::kTurnInsidePwm : cfg::kTurnOutsidePwm,
                    left ? cfg::kTurnOutsidePwm : cfg::kTurnInsidePwm);
    if (elapsed >= cfg::kMinimumTurnMs && junctions_.centreReacquired(lastFrame_)) {
        routeChoicePending_ = false;
        controller_.reset();
        setState(RobotState::RUNNING);
        tracker_.reset();
        lastControlUs_ = micros();
    } else if (elapsed >= cfg::kMaximumTurnMs) {
        diagnostics::port().println(F("Turn did not reacquire a line."));
        startRecovery();
    }
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

void Robot::updateStatusLed() {
    const uint32_t now = millis();
    const uint32_t phase = now % 1200U;
    bool on = false;

    if (turnLedPulseActive_ && now - turnLedStartedMs_ < cfg::kTurnLedPulseMs) {
        on = true;
    } else {
        turnLedPulseActive_ = false;
    }

    if (!on && !button_.armed()) {
        on = ((now / 125U) & 1U) == 0U;
    } else if (!on) {
        switch (state_) {
            case RobotState::CALIBRATION_REQUIRED:
                on = phase < 120U || (phase >= 240U && phase < 360U);
                break;
            case RobotState::WAIT_START:
                on = phase < 90U;
                break;
            case RobotState::CALIBRATING:
                on = ((now / 300U) & 1U) == 0U;
                break;
            case RobotState::START_EXIT:
            case RobotState::RUNNING:
            case RobotState::JUNCTION_ENTRY:
            case RobotState::JUNCTION_TURN:
            case RobotState::DEAD_END_TURN:
                break;
            case RobotState::LINE_RECOVERY:
                on = ((now / 160U) & 1U) == 0U;
                break;
            case RobotState::FINISHED:
                on = true;
                break;
            case RobotState::FAULT:
                on = ((now / 120U) & 1U) == 0U;
                break;
        }
    }

    setStatusLed(on);
}

void Robot::setStatusLed(bool on) {
    if (statusLedOn_ == on) {
        return;
    }
    statusLedOn_ = on;
    const bool driveLow = on == cfg::kStatusLedActiveLow;
    digitalWrite(cfg::kStatusLedPin, driveLow ? LOW : HIGH);
}

void Robot::showStatus() const {
    diagnostics::port().print(F("state="));
    diagnostics::port().print(stateName());
    diagnostics::port().print(F(" track_ms="));
    diagnostics::port().print(trackTimer_.elapsedMs(millis()));
    diagnostics::port().print(F(" speed_percent="));
    diagnostics::port().print(trackTimer_.speedPercent());
    diagnostics::port().print(F(" kp="));
    diagnostics::port().print(controller_.pidP(), 2);
    diagnostics::port().print(F(" ki="));
    diagnostics::port().print(controller_.pidI(), 2);
    diagnostics::port().print(F(" kd="));
    diagnostics::port().print(controller_.pidD(), 2);
    diagnostics::port().print(F(" calibrated="));
    diagnostics::port().print(sensors_.calibrationValid() ? F("yes") : F("no"));
    diagnostics::port().print(F(" position="));
    diagnostics::port().print(tracker_.position());
    diagnostics::port().print(F(" full_position="));
    diagnostics::port().print(lastFrame_.position);
    diagnostics::port().print(F(" strength="));
    diagnostics::port().print(lastFrame_.strength);
    diagnostics::port().print(F(" active="));
    diagnostics::port().print(lastFrame_.activeCount);
    diagnostics::port().print(F(" segments="));
    diagnostics::port().print(lastFrame_.segmentCount);
    diagnostics::port().print(F(" polarity="));
    diagnostics::port().println(lastFrame_.polarity == LinePolarity::DARK ? F("dark") : F("light"));
}

void Robot::emitTelemetry() {
    if (!telemetryEnabled_ || millis() - lastTelemetryMs_ < cfg::kTelemetryIntervalMs) {
        return;
    }
    lastTelemetryMs_ = millis();
    diagnostics::port().print(F("p="));
    diagnostics::port().print(tracker_.position());
    diagnostics::port().print(F(" g="));
    diagnostics::port().print(lastFrame_.position);
    diagnostics::port().print(F(" a="));
    diagnostics::port().print(lastFrame_.activeCount);
    diagnostics::port().print(F(" n="));
    diagnostics::port().print(lastFrame_.segmentCount);
    diagnostics::port().print(F(" q="));
    diagnostics::port().print(lastFrame_.strength);
    diagnostics::port().print(F(" m="));
    diagnostics::port().print(lastFrame_.polarity == LinePolarity::DARK ? 'D' : 'L');
    diagnostics::port().print(F(" e="));
    diagnostics::port().print(lastControl_.error, 3);
    diagnostics::port().print(F(" l="));
    diagnostics::port().print(lastControl_.left);
    diagnostics::port().print(F(" r="));
    diagnostics::port().print(lastControl_.right);
    diagnostics::port().print(F(" tm="));
    diagnostics::port().print(trackTimer_.elapsedMs(millis()));
    diagnostics::port().print(F(" v="));
    diagnostics::port().print(trackTimer_.speedPercent());
    diagnostics::port().print(F(" kp="));
    diagnostics::port().print(controller_.pidP(), 2);
    diagnostics::port().print(F(" ki="));
    diagnostics::port().print(controller_.pidI(), 2);
    diagnostics::port().print(F(" kd="));
    diagnostics::port().println(controller_.pidD(), 2);
}

const __FlashStringHelper* Robot::stateName() const {
    switch (state_) {
        case RobotState::CALIBRATION_REQUIRED: return F("CALIBRATION_REQUIRED");
        case RobotState::WAIT_START: return F("WAIT_START");
        case RobotState::CALIBRATING: return F("CALIBRATING");
        case RobotState::START_EXIT: return F("START_EXIT");
        case RobotState::RUNNING: return F("RUNNING");
        case RobotState::JUNCTION_ENTRY: return F("JUNCTION_ENTRY");
        case RobotState::JUNCTION_TURN: return F("JUNCTION_TURN");
        case RobotState::LINE_RECOVERY: return F("LINE_RECOVERY");
        case RobotState::DEAD_END_TURN: return F("DEAD_END_TURN");
        case RobotState::FINISHED: return F("FINISHED");
        case RobotState::FAULT: return F("FAULT");
    }
    return F("UNKNOWN");
}
