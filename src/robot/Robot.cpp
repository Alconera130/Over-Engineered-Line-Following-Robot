#include "Robot.h"

#include "Diagnostics.h"

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
    const uint8_t speedPercent = state_ == RobotState::RUNNING
                                     ? trackTimer_.speedPercent()
                                     : 100U;
    motors_.setSpeedScalePercent(speedPercent);
    motors_.update(micros());

    switch (state_) {
        case RobotState::CALIBRATING:
            updateCalibration();
            break;
        case RobotState::START_EXIT:
            updateStartExit();
            break;
        case RobotState::RUNNING:
            updateRunning();
            break;
        case RobotState::JUNCTION_ENTRY:
        case RobotState::JUNCTION_TURN:
            updateJunction();
            break;
        case RobotState::LINE_RECOVERY:
            updateRecovery();
            break;
        case RobotState::DEAD_END_TURN:
            updateDeadEndTurn();
            break;
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
