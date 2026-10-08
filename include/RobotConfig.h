#pragma once

#include <Arduino.h>

#include "RobotTypes.h"

/*
    ESP32-C3 SuperMini pin map. The Viper X16 uses its onboard 16:1 analogue
    mux; these are its select inputs and common SIG output.
*/

namespace cfg {
    // GPIO2, GPIO8, and GPIO9 are strapping pins on the ESP32-C3. GPIO8 also
    // drives the common active-low built-in LED on C3 Mini boards.
    // Keep GPIO2 high at reset and avoid holding START low while resetting.
    constexpr uint8_t kViperAdcPin = 4;  // ADC1_CH4, Viper mux SIG/OUT
    constexpr uint8_t kMuxSelectPins[4] = {3, 2, 1, 5};  // Viper S0..S3
    constexpr uint8_t kStartButtonPin = 9;  // START button to GND; boot strap
    constexpr int8_t kStatusLedPin = 8;
    constexpr bool kStatusLedActiveLow = true;

    constexpr uint8_t kLeftIn1Pin = 7;   // TB6612 AIN1
    constexpr uint8_t kLeftIn2Pin = 6;   // TB6612 AIN2
    constexpr uint8_t kLeftPwmPin = 10;  // TB6612 PWMA
    constexpr uint8_t kRightIn1Pin = 20; // TB6612 BIN1
    constexpr uint8_t kRightIn2Pin = 21; // TB6612 BIN2
    constexpr uint8_t kRightPwmPin = 0;  // TB6612 PWMB
    constexpr int8_t kMotorStandbyPin = -1;  // wire TB6612 STBY to 3.3 V
    constexpr uint8_t kLeftPwmChannel = 0;
    constexpr uint8_t kRightPwmChannel = 1;
    constexpr uint32_t kMotorPwmHz = 20000;
    constexpr uint8_t kMotorPwmResolution = 8;

    // The sensor output's electrical direction is fixed by the X16 hardware. Set
    // this false if telemetry shows a white surface reads lower than a black one.
    constexpr bool kSensorOutputIncreasesWithReflectance = false;
    // Selected line colour when automatic polarity switching is disabled.
    constexpr bool kDefaultLineIsDark = true;
    // Enable only on courses that change between dark-on-light and light-on-dark.
    // Otherwise choose the starting line colour with kDefaultLineIsDark.
    constexpr bool kEnableAutomaticPolarity = false;
    constexpr uint8_t kPolaritySwitchConfirmFrames = 5;
    constexpr int16_t kPolaritySwitchScoreMargin = 90;
    constexpr uint8_t kNominalLineMaximumSensors = 6;
    constexpr uint8_t kUniformBackgroundSensors = 13;
    constexpr uint8_t kSamplesPerSensor = 2;
    constexpr uint16_t kMuxSettleUs = 4;
    constexpr uint16_t kMinimumCalibrationSpan = 120;
    constexpr float kSensorFilterAlpha = 0.48F;
    constexpr uint16_t kActiveThreshold = 280;
    constexpr uint16_t kLineVisibleStrength = 75;
    constexpr uint16_t kBranchThreshold = 620;
    constexpr uint8_t kJunctionActiveSensors = 7;

    // Keep steering on the same physical line when the X16 can see a nearby,
    // separate track. A one-channel inactive gap is merged so a noisy line is
    // not incorrectly split into two paths. The maximum position jump is in
    // X16 centroid units (one sensor pitch is about 1000 units).
    constexpr bool kEnableLineSegmentLock = true;
    constexpr uint8_t kLineSegmentMergeGapSensors = 1;
    constexpr int16_t kLineLockMaximumPositionJump = 3200;

    // Sustained one-edge corners use the same route list as wide junctions.
    constexpr bool kEnableSharpTurnRouting = true;
    constexpr int16_t kSharpTurnPositionLimit = 6000;
    constexpr int16_t kSharpTurnReleasePosition = 4200;
    constexpr uint16_t kSharpTurnCenterMaximum = 380;
    constexpr uint8_t kSharpTurnMaximumActiveSensors = 5;
    constexpr uint8_t kSharpTurnConfirmFrames = 7;

    // PID uses normalised lateral error (-1.0 to +1.0), then produces PWM units.
    // Tune kPidP first, then kPidD; use a little kPidI only for a persistent bias.
    constexpr float kPidP = 75.0F;
    constexpr float kPidI = 0.0F;
    constexpr float kPidD = 2.5F;
    constexpr float kDerivativeFilter = 0.72F;
    constexpr float kIntegralLimit = 0.55F;
    constexpr int16_t kMaxCorrection = 140;
    constexpr int16_t kCruisePwm = 50;
    constexpr int16_t kCornerPwm = 20;
    constexpr float kDerivativeSlowdown = 0.15F;

    constexpr int16_t kMotorMaxPwm = 255;
    constexpr int16_t kMotorMinimumPwm = 48;  // measure the real motor deadband
    constexpr float kMotorSlewPwmPerSecond = 1200.0F;

    // After START, drive clear of a solid black/white start marker before
    // enabling junction logic. This prevents a fully covered X16 from spending
    // the first route action. The robot resumes PID only after a narrow line is
    // seen for several frames. Disable only if the event has no start marker.
    constexpr bool kEnableStartZoneExit = true;
    constexpr int16_t kStartExitPwm = 105;
    constexpr uint16_t kStartExitTimeoutMs = 1800;
    constexpr uint8_t kStartExitMaximumActiveSensors = 6;
    constexpr uint8_t kStartExitConfirmFrames = 3;

    // Route is consumed at each detected junction or confirmed sharp turn.
    constexpr JunctionAction kRoute[] = {
        JunctionAction::LEFT, JunctionAction::RIGHT, JunctionAction::STRAIGHT
    };

    constexpr uint8_t kRouteLength = sizeof(kRoute) / sizeof(kRoute[0]);
    static_assert(kRouteLength > 0, "kRoute must contain at least one junction action.");
    constexpr uint16_t kJunctionEntryMs = 38;
    constexpr uint16_t kStraightTraverseMs = 105;
    constexpr uint16_t kMinimumTurnMs = 75;
    constexpr uint16_t kMaximumTurnMs = 700;
    constexpr uint16_t kTurnLedPulseMs = 350;
    constexpr int16_t kJunctionEntryPwm = 112;
    constexpr int16_t kTurnOutsidePwm = 150;
    constexpr int16_t kTurnInsidePwm = -64;
    constexpr int16_t kTurnReacquirePosition = 2500;

    // Events are timed from the first normal line-following frame after START.
    // Speed percentage affects PID following and gap bridging; timed maneuvers
    // retain their tuned PWM so scaling does not invalidate their durations.
    constexpr bool kEnableTrackTimeActions = false;
    constexpr TrackTimeEvent kTrackTimeEvents[] = {
        {10000U, TrackTimeCommand::SET_SPEED_PERCENT, 70, 0.0F, 0.0F, 0.0F},
        {20000U, TrackTimeCommand::SET_PID_GAINS, 100, 85.0F, 0.0F, 2.5F},
        {30000U, TrackTimeCommand::SET_SPEED_PERCENT, 100, 0.0F, 0.0F, 0.0F},
        {60000U, TrackTimeCommand::STOP, 0, 0.0F, 0.0F, 0.0F}
    };
    constexpr uint8_t kTrackTimeEventCount =
        sizeof(kTrackTimeEvents) / sizeof(kTrackTimeEvents[0]);
    static_assert(kTrackTimeEventCount > 0, "Track-time schedule must contain an event.");

    // Cross small breaks/dots by holding the final steering vector briefly. This
    // prevents a normal dotted-line gap from becoming a search manoeuvre.
    constexpr uint16_t kBrokenLineBridgeMs = 110;
    constexpr int16_t kGapBridgePwm = 118;
    constexpr float kGapCorrectionScale = 0.65F;
    constexpr uint16_t kRecoveryBeforeDeadEndMs = 360;
    constexpr uint16_t kRecoveryTimeoutMs = 900;
    constexpr int16_t kRecoveryTurnPwm = 115;

    // Recovery searches toward the last line position. Optional dead-end return
    // performs a timed U-turn; it is approximate without wheel encoders.
    constexpr bool kEnablePathRecovery = true;  // false stops the motors on line loss
    constexpr bool kEnableDeadEndReturn = false;
    constexpr uint16_t kDeadEndBackupMs = 320;
    constexpr int16_t kDeadEndBackupPwm = 90;
    constexpr uint16_t kDeadEndMinimumTurnMs = 330;
    constexpr uint16_t kDeadEndMaximumTurnMs = 1150;
    constexpr int16_t kDeadEndTurnPwm = 128;
    constexpr uint8_t kMaximumDeadEndReturns = 8;
    constexpr uint8_t kPathMemoryDepth = 16;

    // Disabled by default: course marking conventions differ between competitions.
    constexpr bool kEnableStopLine = false;
    constexpr uint8_t kStopActiveSensors = 13;
    constexpr uint16_t kStopStrength = 760;
    constexpr uint16_t kStopHoldMs = 180;

    constexpr uint16_t kCalibrationDurationMs = 4800;
    constexpr uint16_t kCalibrationPhaseMs = 600;
    constexpr int16_t kCalibrationTurnPwm = 66;
    constexpr uint16_t kLongPressMs = 1000;
    // The START button is normally released to commit an action. Debouncing its
    // contacts prevents that release from being misread as a second tap.
    constexpr uint16_t kStartButtonDebounceMs = 30;
    constexpr uint16_t kStartDoubleTapWindowMs = 400;
    // START must be released briefly after reset before it is accepted. This
    // prevents a BOOT/USB-reset transient from becoming a calibration hold.
    constexpr uint16_t kStartButtonReleaseArmMs = 60;
    constexpr uint16_t kTelemetryIntervalMs = 100;
}
