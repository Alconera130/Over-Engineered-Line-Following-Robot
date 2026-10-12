#pragma once

#include <Arduino.h>

#include "RobotTypes.h"

/*
    ESP32-C3 SuperMini pin map. The Viper X16 uses its onboard 16:1 analogue
    mux; these are its select inputs and common SIG output.
*/

namespace cfg {
    // GPIO2, GPIO8, and GPIO9 are ESP32-C3 strapping pins. OLED SCL uses GPIO8;
    // OLED SDA and the active-low BOOT/START button share GPIO9. Power the OLED
    // at 3.3 V so its I2C pull-ups hold the lines high during reset. Release
    // BOOT after startup before the button arms. The onboard LED on GPIO8 is not
    // firmware-driven, but can respond to SCL clock pulses.
    constexpr uint8_t kViperAdcPin = 4;                 // ADC1_CH4, Viper mux SIG/OUT
    constexpr uint8_t kMuxSelectPins[4] = {3, 2, 1, 5}; // Viper S0..S3
    constexpr uint8_t kStartButtonPin = 9;              // START button to GND; boot strap
    constexpr uint8_t kOledSdaPin = 9;                   // Shared open-drain line with BOOT/START
    constexpr uint8_t kOledSclPin = 8;                   // OLED clock; onboard LED may load/flicker on this pin
    constexpr uint8_t kOledI2cAddress = 0x3C;
    constexpr uint16_t kOledSurfaceThreshold = 500;      // 0=black, 1000=white after calibration
    constexpr uint16_t kOledRenderIntervalMs = 100;
    constexpr uint16_t kOledPageWriteIntervalMs = 60;
    constexpr uint16_t kOledJunctionMessageMs = 1000;

    static_assert(kOledSdaPin == kStartButtonPin,
                  "OLED SDA intentionally shares the active-low BOOT/START button.");

    constexpr uint8_t kLeftIn1Pin = 7;      // TB6612 AIN1
    constexpr uint8_t kLeftIn2Pin = 6;      // TB6612 AIN2
    constexpr uint8_t kLeftPwmPin = 10;     // TB6612 PWMA
    constexpr uint8_t kRightIn1Pin = 20;    // TB6612 BIN1
    constexpr uint8_t kRightIn2Pin = 21;    // TB6612 BIN2
    constexpr uint8_t kRightPwmPin = 0;     // TB6612 PWMB
    constexpr int8_t kMotorStandbyPin = -1; // wire TB6612 STBY to 3.3 V
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
    constexpr uint16_t kActiveThreshold = 400;
    constexpr uint16_t kLineVisibleStrength = 75;
    constexpr uint16_t kBranchThreshold = 620;
    constexpr uint8_t kJunctionActiveSensors = 7;
    // Minimum unbroken run of active sensors from the array centre, in either
    // direction, to count as a junction branch. The centre sensor is included.
    // Lower this to accept shorter crossbars; raise it to reject them.
    constexpr uint8_t kJunctionMinimumConnectedSensors = 8;
    static_assert(kJunctionMinimumConnectedSensors > 0 &&
                      kJunctionMinimumConnectedSensors <= kViperSensorCount / 2 + 1,
                  "Junction connected-run length must fit from the centre to an edge.");

    // When enabled, steering follows only the tracked segment. When disabled,
    // the full-array centroid lets separate detections bias steering toward
    // them; currently disabled because the X16 sensor pitch is small.
    constexpr bool kEnableLineSegmentLock = false;
    // A one-channel inactive gap is merged so a noisy line is not incorrectly
    // split into two paths. The maximum position jump is in X16 centroid units
    // (one sensor pitch is about 1000 units).
    constexpr uint8_t kLineSegmentMergeGapSensors = 1;
    constexpr int16_t kLineLockMaximumPositionJump = 3200;

    // Master switch for consuming route entries at junctions and sharp turns.
    // Track-time events can temporarily toggle this during a run.
    constexpr bool kEnableJunctionRouting = true;
    // When routing is disabled, cross detected junctions/edge turns straight
    // for the configured entry and traversal durations instead of following a branch.
    constexpr bool kStraightThroughWhenJunctionRoutingDisabled = true;
    // If false while routing is enabled, one-edge turns remain under PID. The
    // OLED still reports when the detector sees one.
    constexpr bool kEnableSharpTurnRouting = false;
    constexpr int16_t kSharpTurnPositionLimit = 5200;
    constexpr int16_t kSharpTurnReleasePosition = 4200;
    constexpr uint16_t kSharpTurnCenterMaximum = 380;
    constexpr uint8_t kSharpTurnMaximumActiveSensors = 5;
    constexpr uint8_t kSharpTurnConfirmFrames = 3;

    // PID uses normalised lateral error (-1.0 to +1.0), then produces PWM units.
    // Tune kPidP first, then kPidD; use a little kPidI only for a persistent bias.
    constexpr float kPidP = 200.0F;
    constexpr float kPidI = 0.0F;
    constexpr float kPidD = 19.0F;
    constexpr float kDerivativeFilter = 0.72F;
    constexpr float kIntegralLimit = 0.55F;
    constexpr int16_t kCruisePwm = 120;
    constexpr int16_t kCornerPwm = 50;
    constexpr float kDerivativeSlowdown = 0.15F;

    constexpr int16_t kMotorMaxPwm = 255;
    constexpr int16_t kMotorMinimumPwm = 20; // measure the real motor deadband
    constexpr float kMotorSlewPwmPerSecond = 700000.0F;
    // When enabled, clear the start marker, then cross the first detected wide
    // junction straight without consuming a route entry. Start the run timer
    // and TrackTime schedule after that crossing; route selection starts at the
    // next junction. Disable when the course has no start marker/gate junction.
    constexpr bool kEnableStartZoneExit = false;
    constexpr int16_t kStartExitPwm = 80;
    constexpr uint16_t kStartExitTimeoutMs = 700;
    constexpr uint8_t kStartExitMaximumActiveSensors = 6;
    constexpr uint8_t kStartExitConfirmFrames = 3;

    // When junction routing is enabled, consume one route entry at each
    // detected junction or confirmed sharp turn.
    constexpr JunctionAction kRoute[] = {
        JunctionAction::STRAIGHT,
        JunctionAction::RIGHT,
        JunctionAction::STRAIGHT,
        JunctionAction::STRAIGHT,
        JunctionAction::STRAIGHT,
        JunctionAction::STRAIGHT,
        JunctionAction::STRAIGHT,
        JunctionAction::STRAIGHT,
        JunctionAction::LEFT,
        JunctionAction::ACUTE_LEFT,
        JunctionAction::RIGHT,
        JunctionAction::RIGHT,
        JunctionAction::RIGHT,
        JunctionAction::RIGHT,
        JunctionAction::LEFT,
        JunctionAction::RIGHT,
        JunctionAction::RIGHT,
        JunctionAction::LEFT,
        JunctionAction::ACUTE_LEFT,
        JunctionAction::STRAIGHT,
        JunctionAction::STRAIGHT,
        JunctionAction::STRAIGHT,
        JunctionAction::ACUTE_LEFT,
        JunctionAction::STRAIGHT,
        JunctionAction::STRAIGHT,
        JunctionAction::STRAIGHT,
        JunctionAction::STRAIGHT,
        JunctionAction::STRAIGHT,
        JunctionAction::RIGHT,
        JunctionAction::RIGHT,
        JunctionAction::STRAIGHT,
        JunctionAction::RIGHT,
        JunctionAction::LEFT,
        JunctionAction::LEFT,
        JunctionAction::RIGHT,
        JunctionAction::LEFT,
        JunctionAction::RIGHT,
        JunctionAction::LEFT,
        JunctionAction::RIGHT,
        JunctionAction::LEFT,
        JunctionAction::LEFT,
        JunctionAction::LEFT,
        JunctionAction::RIGHT,
        JunctionAction::LEFT,
        JunctionAction::RIGHT,
        JunctionAction::RIGHT,
        JunctionAction::RIGHT,
        JunctionAction::LEFT,
        JunctionAction::RIGHT,
        JunctionAction::STRAIGHT,
        JunctionAction::RIGHT,
        JunctionAction::RIGHT,
        JunctionAction::STRAIGHT,
        JunctionAction::LEFT,
        JunctionAction::RIGHT,
        JunctionAction::RIGHT,
        JunctionAction::STRAIGHT,
        JunctionAction::RIGHT,
        JunctionAction::LEFT,
        JunctionAction::STRAIGHT,
        JunctionAction::LEFT,
        JunctionAction::RIGHT,
        JunctionAction::STRAIGHT,
        JunctionAction::RIGHT,
        JunctionAction::LEFT
    };

    constexpr uint8_t kRouteLength = sizeof(kRoute) / sizeof(kRoute[0]);
    static_assert(kRouteLength > 0, "kRoute must contain at least one junction action.");
    constexpr uint16_t kJunctionEntryMs = 0;
    constexpr uint16_t kStraightTraverseMs = 80;
    constexpr uint16_t kMinimumTurnMs = 20;
    constexpr uint16_t kMaximumTurnMs = 100;
    // Acute route actions wait at least this long before checking for the line,
    // then recover if they have not reacquired it by the maximum.
    constexpr uint16_t kAcuteMinimumTurnMs = 100;
    constexpr uint16_t kAcuteMaximumTurnMs = 150;
    static_assert(kAcuteMinimumTurnMs <= kAcuteMaximumTurnMs,
                  "Acute minimum turn time must not exceed its timeout.");
    constexpr int16_t kJunctionEntryPwm = 50;
    constexpr int16_t kTurnOutsidePwm = 120;
    constexpr int16_t kTurnInsidePwm = -80;
    // Higher differential drive for route entries marked ACUTE_LEFT/ACUTE_RIGHT.
    // Keep both magnitudes within kMotorMaxPwm.
    constexpr int16_t kAcuteTurnOutsidePwm = 200;
    constexpr int16_t kAcuteTurnInsidePwm = -160;
    static_assert(kAcuteTurnOutsidePwm >= 0 &&
                      kAcuteTurnOutsidePwm <= kMotorMaxPwm &&
                      kAcuteTurnInsidePwm <= 0 &&
                      kAcuteTurnInsidePwm >= -kMotorMaxPwm,
                  "Acute turn PWM must stay within the motor range.");
    constexpr int16_t kTurnReacquirePosition = 2500;

    // Events are timed from the run timer start. With start-zone exit enabled,
    // that begins after the first wide junction is crossed straight. Keep events
    // in ascending atMs order. Speed scaling affects PID and gap bridging,
    // while timed junction maneuvers retain their configured PWM.
    constexpr bool kEnableTrackTimeActions = true;
    constexpr TrackTimeEvent kTrackTimeEvents[] = {
        {0000U, TrackTimeCommand::SET_SPEED_PERCENT, 50, 0.0F, 0.0F, 0.0F},
        {5700U, TrackTimeCommand::DISABLE_JUNCTION_ROUTING, 0, 0.0F, 0.0F, 0.0F},
        {6200U, TrackTimeCommand::ENABLE_JUNCTION_ROUTING, 0, 0.0F, 0.0F, 0.0F}
    };

    constexpr uint8_t kTrackTimeEventCount = sizeof(kTrackTimeEvents) / sizeof(kTrackTimeEvents[0]);
    static_assert(kTrackTimeEventCount > 0, "Track-time schedule must contain an event.");

    // Cross small breaks/dots by holding the final steering vector briefly. This
    // prevents a normal dotted-line gap from becoming a search manoeuvre.
    constexpr uint16_t kBrokenLineBridgeMs = 250;
    constexpr int16_t kGapBridgePwm = 200;
    constexpr float kGapCorrectionScale = 0.65F;
    constexpr uint16_t kRecoveryBeforeDeadEndMs = 360;
    constexpr uint16_t kRecoveryTimeoutMs = 900;
    constexpr int16_t kRecoveryTurnPwm = 115;

    // Recovery searches toward the last line position. Optional dead-end return
    // performs a timed U-turn; it is approximate without wheel encoders.
    constexpr bool kEnablePathRecovery = true; // false stops the motors on line loss
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
