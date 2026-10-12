#pragma once

#include <Arduino.h>

#include "navigation/BrokenLineBridge.h"
#include "system/CalibrationSweep.h"
#include "navigation/DeadEndReturn.h"
#include "navigation/JunctionHandler.h"
#include "control/LineController.h"
#include "navigation/LineRecovery.h"
#include "control/MotorDriver.h"
#include "RobotConfig.h"
#include "RobotTypes.h"
#include "navigation/RoutePlanner.h"
#include "navigation/SegmentTracker.h"
#include "navigation/SharpTurnHandler.h"
#include "system/StartButton.h"
#include "system/TrackTimer.h"
#include "sensors/ViperX16.h"

class Robot {
public:
    void begin();
    void update();

private:
    enum class ConsoleInputMode : uint8_t { NONE, PID_GAINS, WHITE_VALUES, BLACK_VALUES };

    void pollButton();
    void pollConsole();
    void processConsoleInput();
    void applyPidCommand();
    bool parseCalibrationValues(const char* text, uint16_t values[]) const;
    void startCalibration();
    void updateCalibration();
    void beginRun(bool resumeRoute = false);
    void processTrackTime();
    void updateStartExit();
    void updateRunning();
    void beginTurn(const SensorFrame& frame, bool sharpTurn);
    void beginStraightThrough();
    void updateJunction();
    void signalJunctionDetected();
    void startRecovery();
    void updateRecovery();
    void updateDeadEndTurn();
    void finish();
    void fault(const __FlashStringHelper* reason);
    void setState(RobotState state);
    bool isRunningState() const;
    void updateStatusLed();
    void setStatusLed(bool on);
    void showStatus() const;
    void emitTelemetry();
    const __FlashStringHelper* stateName() const;

    ViperX16 sensors_;
    MotorDriver motors_;
    LineController controller_;
    StartButton button_;
    TrackTimer trackTimer_;
    CalibrationSweep calibration_;
    RoutePlanner route_;
    JunctionHandler junctions_;
    SharpTurnHandler sharpTurns_;
    SegmentTracker tracker_;
    BrokenLineBridge bridge_;
    LineRecovery recovery_;
    DeadEndReturn deadEnd_;

    RobotState state_ = RobotState::CALIBRATION_REQUIRED;
    JunctionAction pendingTurn_ = JunctionAction::STRAIGHT;
    SensorFrame lastFrame_{};
    ControlOutput lastControl_{};

    uint32_t stateStartedMs_ = 0;
    uint32_t lostSinceMs_ = 0;
    uint32_t stopSinceMs_ = 0;
    uint32_t lastTelemetryMs_ = 0;
    uint32_t lastControlUs_ = 0;
    uint32_t turnLedStartedMs_ = 0;
    int16_t lastSeenPosition_ = 0;
    uint8_t startExitNarrowFrames_ = 0;

    bool telemetryEnabled_ = false;
    bool junctionRoutingEnabled_ = cfg::kEnableJunctionRouting;
    bool junctionLocked_ = false;
    bool returningFromDeadEnd_ = false;
    bool statusLedOn_ = false;
    bool turnLedPulseActive_ = false;
    bool routeChoicePending_ = false;
    bool routeSelectionPending_ = false;
    bool junctionWasReturningFromDeadEnd_ = false;
    bool faultRestartPending_ = false;
    ConsoleInputMode consoleInputMode_ = ConsoleInputMode::NONE;
    char consoleInputBuffer_[192]{};
    uint16_t consoleInputLength_ = 0;
    bool consoleInputOverflow_ = false;
    bool consoleInputSkipCommandTerminator_ = false;
    uint16_t manualWhite_[kViperSensorCount]{};
    uint16_t manualBlack_[kViperSensorCount]{};
    uint32_t faultRestartTapMs_ = 0;
};
