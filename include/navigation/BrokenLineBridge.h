#pragma once

class MotorDriver;
struct ControlOutput;

class BrokenLineBridge {
public:
    void drive(MotorDriver& motors, const ControlOutput& lastControl) const;
};
