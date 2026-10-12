#pragma once

#include <Arduino.h>

#include "RobotTypes.h"

class LineController {
    public:
        void reset();
        void resetGains();
        void setGains(float p, float i, float d);
        ControlOutput update(int16_t position, float dtSeconds);
        float pidP() const { return proportionalGain_; }
        float pidI() const { return integralGain_; }
        float pidD() const { return derivativeGain_; }

    private:
        float previousError_ = 0.0F;
        float integral_ = 0.0F;
        float filteredDerivative_ = 0.0F;
        float proportionalGain_ = 0.0F;
        float integralGain_ = 0.0F;
        float derivativeGain_ = 0.0F;
};
