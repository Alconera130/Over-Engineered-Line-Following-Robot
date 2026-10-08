#pragma once

#include <Arduino.h>

class MotorDriver;
class ViperX16;

class CalibrationSweep {
public:
    void begin(ViperX16& sensors, MotorDriver& motors);
    bool update(ViperX16& sensors, MotorDriver& motors);
    bool succeeded() const { return succeeded_; }

private:
    uint32_t startedMs_ = 0;
    bool active_ = false;
    bool succeeded_ = false;
};
