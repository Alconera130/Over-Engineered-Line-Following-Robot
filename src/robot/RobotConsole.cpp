#include "Robot.h"

#include <ctype.h>
#include <math.h>
#include <stdlib.h>

#include "Diagnostics.h"

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
