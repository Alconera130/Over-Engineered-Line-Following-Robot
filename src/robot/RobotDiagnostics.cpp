#include "Robot.h"

#include "Diagnostics.h"

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
