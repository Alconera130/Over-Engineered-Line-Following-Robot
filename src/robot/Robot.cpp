#include "Robot.h"

#include <ctype.h>
#include <math.h>
#include <stdlib.h>

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

void Robot::begin() {
    diagnostics::port().begin(115200);
    controller_.resetGains();
    button_.begin();
    motors_.begin();
    sensors_.begin();
    display_.begin();
    state_ = sensors_.calibrationValid() ? RobotState::WAIT_START
                                          : RobotState::CALIBRATION_REQUIRED;
    if (sensors_.calibrationValid()) {
        sensors_.read(lastFrame_);
        sensorFrameValid_ = true;
    }
    lastIdleSensorReadMs_ = millis();
    stateStartedMs_ = millis();
    lastControlUs_ = micros();

    diagnostics::port().println();
    diagnostics::port().println(F("Linea Viper X16 competition controller"));
    if (sensors_.calibrationValid()) {
        diagnostics::port().println(F("Calibration loaded. Release START to run; hold, then release, to calibrate."));
    } else {
        diagnostics::port().println(F("No valid calibration. Hold START for one second, then release, to calibrate."));
    }
    diagnostics::port().println(F("Console: r=run, c=calibrate, m=manual calibration, x=stop, s=status, t=telemetry, g=PID gains"));
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

    const uint32_t nowMs = millis();
    if (!isRunningState() && state_ != RobotState::CALIBRATING &&
        sensors_.calibrationValid() &&
        nowMs - lastIdleSensorReadMs_ >= cfg::kOledRenderIntervalMs) {
        sensors_.read(lastFrame_);
        sensorFrameValid_ = true;
        lastIdleSensorReadMs_ = nowMs;
    }
    updateDisplay(nowMs);
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
        if (consoleInputMode_ != ConsoleInputMode::NONE) {
            if (consoleInputSkipCommandTerminator_) {
                if (command == '\r') {
                    continue;
                }
                if (command == '\n') {
                    consoleInputSkipCommandTerminator_ = false;
                    continue;
                }
                consoleInputSkipCommandTerminator_ = false;
            }
            if (command == 'q' || command == 'Q') {
                consoleInputMode_ = ConsoleInputMode::NONE;
                consoleInputLength_ = 0;
                consoleInputOverflow_ = false;
                consoleInputSkipCommandTerminator_ = false;
                diagnostics::port().println(F("Console input cancelled."));
                continue;
            }
            if (command == '\n') {
                consoleInputBuffer_[consoleInputLength_] = '\0';
                if (consoleInputOverflow_) {
                    diagnostics::port().println(F("Input too long; try again or send q to cancel."));
                    consoleInputLength_ = 0;
                    consoleInputOverflow_ = false;
                } else {
                    processConsoleInput();
                    consoleInputLength_ = 0;
                }
            } else if (command != '\r') {
                if (consoleInputLength_ < sizeof(consoleInputBuffer_) - 1U) {
                    consoleInputBuffer_[consoleInputLength_++] = command;
                } else {
                    consoleInputOverflow_ = true;
                }
            }
            continue;
        }

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
            case 'g':
            case 'G':
                consoleInputLength_ = 0;
                consoleInputOverflow_ = false;
                consoleInputSkipCommandTerminator_ = true;
                consoleInputMode_ = ConsoleInputMode::PID_GAINS;
                diagnostics::port().println(F("Enter gains as: g P I D, then press Enter (example: g 75 0 2.5)"));
                break;
            case 'm':
            case 'M':
                if (isRunningState() || state_ == RobotState::CALIBRATING) {
                    diagnostics::port().println(F("Stop the robot before entering manual calibration."));
                    break;
                }
                consoleInputLength_ = 0;
                consoleInputOverflow_ = false;
                consoleInputSkipCommandTerminator_ = true;
                consoleInputMode_ = ConsoleInputMode::WHITE_VALUES;
                diagnostics::port().println(F("Paste 16 comma-separated white-floor raw ADC values, channels 0..15, then press Enter. Each value is 0..4095."));
                break;
            default: break;
        }
    }
}

void Robot::processConsoleInput() {
    if (consoleInputMode_ == ConsoleInputMode::PID_GAINS) {
        consoleInputMode_ = ConsoleInputMode::NONE;
        applyPidCommand();
        return;
    }

    if (consoleInputMode_ == ConsoleInputMode::WHITE_VALUES) {
        if (!parseCalibrationValues(consoleInputBuffer_, manualWhite_)) {
            diagnostics::port().println(F("Expected exactly 16 comma-separated integers from 0 to 4095; try again or send q to cancel."));
            return;
        }
        consoleInputMode_ = ConsoleInputMode::BLACK_VALUES;
        if (cfg::kSensorOutputIncreasesWithReflectance) {
            diagnostics::port().println(F("Now paste 16 black-line raw ADC values; each must be lower than its white value and differ by more than the calibration span."));
        } else {
            diagnostics::port().println(F("Now paste 16 black-line raw ADC values; each must be higher than its white value and differ by more than the calibration span."));
        }
        return;
    }

    if (consoleInputMode_ == ConsoleInputMode::BLACK_VALUES) {
        if (!parseCalibrationValues(consoleInputBuffer_, manualBlack_)) {
            diagnostics::port().println(F("Expected exactly 16 comma-separated integers from 0 to 4095; try again or send q to cancel."));
            return;
        }
        if (!sensors_.setManualCalibration(manualWhite_, manualBlack_)) {
            diagnostics::port().println(F("Calibration rejected or could not be saved. Check that every white/black pair spans enough ADC counts and has the configured raw order; retry black values or send q."));
            return;
        }

        consoleInputMode_ = ConsoleInputMode::NONE;
        faultRestartPending_ = false;
        setState(RobotState::WAIT_START);
        diagnostics::port().println(F("Manual calibration saved to NVS. It will be loaded on the next battery-powered boot."));
    }
}

void Robot::applyPidCommand() {
    char* cursor = consoleInputBuffer_;
    char* end = nullptr;
    const float p = strtof(cursor, &end);
    if (end == cursor) {
        diagnostics::port().println(F("PID unchanged: enter three numbers, for example g 75 0 2.5"));
        return;
    }
    cursor = end;
    const float i = strtof(cursor, &end);
    if (end == cursor) {
        diagnostics::port().println(F("PID unchanged: enter three numbers, for example g 75 0 2.5"));
        return;
    }
    cursor = end;
    const float d = strtof(cursor, &end);
    if (end == cursor) {
        diagnostics::port().println(F("PID unchanged: enter three numbers, for example g 75 0 2.5"));
        return;
    }
    while (*end != '\0' && isspace(static_cast<unsigned char>(*end))) {
        ++end;
    }
    if (*end != '\0' || !isfinite(p) || !isfinite(i) || !isfinite(d) ||
        p < 0.0F || p > 500.0F || i < 0.0F || i > 500.0F ||
        d < 0.0F || d > 500.0F) {
        diagnostics::port().println(F("PID unchanged: each gain must be a number from 0 to 500"));
        return;
    }
    if (!isRunningState()) {
        diagnostics::port().println(F("PID unchanged: start the robot before tuning gains"));
        return;
    }

    controller_.setGains(p, i, d);
    controller_.reset();
    diagnostics::port().print(F("PID set: P="));
    diagnostics::port().print(controller_.pidP(), 2);
    diagnostics::port().print(F(" I="));
    diagnostics::port().print(controller_.pidI(), 2);
    diagnostics::port().print(F(" D="));
    diagnostics::port().println(controller_.pidD(), 2);
}

bool Robot::parseCalibrationValues(const char* text, uint16_t values[]) const {
    const char* cursor = text;
    for (uint8_t i = 0; i < kViperSensorCount; ++i) {
        while (*cursor != '\0' && isspace(static_cast<unsigned char>(*cursor))) {
            ++cursor;
        }
        if (*cursor == '\0') {
            return false;
        }

        char* end = nullptr;
        const unsigned long value = strtoul(cursor, &end, 10);
        if (end == cursor || value > 4095UL) {
            return false;
        }
        values[i] = static_cast<uint16_t>(value);
        cursor = end;
        while (*cursor != '\0' && isspace(static_cast<unsigned char>(*cursor))) {
            ++cursor;
        }
        if (i + 1U < kViperSensorCount) {
            if (*cursor != ',') {
                return false;
            }
            ++cursor;
        }
    }

    while (*cursor != '\0' && isspace(static_cast<unsigned char>(*cursor))) {
        ++cursor;
    }
    return *cursor == '\0';
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
    trackTimer_.start(millis());
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
        diagnostics::port().println(F("Leaving start zone."));
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

    const bool junctionPresent = junctions_.detected(lastFrame_, *trackedSegment);
    if (!junctionPresent) {
        junctionLocked_ = false;
    } else if (!junctionLocked_) {
        junctionLocked_ = true;
        signalJunctionDetected();
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

void Robot::beginStraightThrough() {
    returningFromDeadEnd_ = false;
    routeChoicePending_ = false;
    routeSelectionPending_ = false;
    junctionWasReturningFromDeadEnd_ = false;
    pendingTurn_ = JunctionAction::STRAIGHT;
    controller_.reset();
    setState(RobotState::JUNCTION_ENTRY);
    diagnostics::port().println(F("Junction routing disabled; crossing straight."));
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
    if (elapsed >= minimumTurnMs && junctions_.centreReacquired(lastFrame_)) {
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
            break;
    }
    view.routeActionCurrent = routeChoicePending_;
    view.routeSlot = routeChoicePending_ ? route_.previousRouteSlot()
                                         : route_.peekNextRouteSlot();
    view.routeAction = routeChoicePending_ ? pendingTurn_ : route_.peekNextAction();
    if (view.nextTrackEvent != nullptr) {
        const uint32_t elapsed = trackTimer_.elapsedMs(nowMs);
        view.timeToTrackEventMs = view.nextTrackEvent->atMs > elapsed
                                      ? view.nextTrackEvent->atMs - elapsed
                                      : 0U;
    }
    display_.update(nowMs, view, button_.isDown());
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
    diagnostics::port().print(F(" visible="));
    diagnostics::port().print(lastFrame_.lineVisible ? F("yes") : F("no"));
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
    diagnostics::port().print(F(" vis="));
    diagnostics::port().print(lastFrame_.lineVisible ? 1 : 0);
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
