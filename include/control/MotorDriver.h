#pragma once

#include <Arduino.h>

class MotorDriver {
public:
    void begin();
    void command(int16_t left, int16_t right);
    void setSpeedScalePercent(uint8_t percent);
    void update(uint32_t nowUs);
    void emergencyStop();

private:
    void writeMotor(uint8_t in1, uint8_t in2, uint8_t channel, float speed);

    float targetLeft_ = 0.0F;
    float targetRight_ = 0.0F;
    float actualLeft_ = 0.0F;
    float actualRight_ = 0.0F;
    uint8_t speedScalePercent_ = 100;
    uint32_t lastUpdateUs_ = 0;
};
