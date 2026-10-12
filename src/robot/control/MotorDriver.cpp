#include "control/MotorDriver.h"

#include "RobotConfig.h"

namespace {
    float moveTowards(float current, float target, float maximumStep) {
        if (target > current) {
            return min(target, current + maximumStep);
        }

        return max(target, current - maximumStep);
    }
}  // namespace

void MotorDriver::begin() {
    pinMode(cfg::kLeftIn1Pin, OUTPUT);
    pinMode(cfg::kLeftIn2Pin, OUTPUT);
    pinMode(cfg::kRightIn1Pin, OUTPUT);
    pinMode(cfg::kRightIn2Pin, OUTPUT);
    if (cfg::kMotorStandbyPin >= 0) {
        pinMode(cfg::kMotorStandbyPin, OUTPUT);
        digitalWrite(cfg::kMotorStandbyPin, HIGH);
    }
    ledcSetup(cfg::kLeftPwmChannel, cfg::kMotorPwmHz, cfg::kMotorPwmResolution);
    ledcSetup(cfg::kRightPwmChannel, cfg::kMotorPwmHz, cfg::kMotorPwmResolution);
    ledcAttachPin(cfg::kLeftPwmPin, cfg::kLeftPwmChannel);
    ledcAttachPin(cfg::kRightPwmPin, cfg::kRightPwmChannel);
    emergencyStop();
}

void MotorDriver::command(int16_t left, int16_t right) {
    if (cfg::kMotorStandbyPin >= 0) {
        digitalWrite(cfg::kMotorStandbyPin, HIGH);
    }
    const int32_t scaledLeft = static_cast<int32_t>(left) * speedScalePercent_ / 100;
    const int32_t scaledRight = static_cast<int32_t>(right) * speedScalePercent_ / 100;
    targetLeft_ = constrain(scaledLeft, -cfg::kMotorMaxPwm, cfg::kMotorMaxPwm);
    targetRight_ = constrain(scaledRight, -cfg::kMotorMaxPwm, cfg::kMotorMaxPwm);
}

void MotorDriver::setSpeedScalePercent(uint8_t percent) {
    if (percent < 1U) {
        percent = 1U;
    } else if (percent > 100U) {
        percent = 100U;
    }
    speedScalePercent_ = percent;
}

void MotorDriver::writeMotor(uint8_t in1, uint8_t in2, uint8_t channel,
                             float speed) {
    int16_t pwm = static_cast<int16_t>(fabsf(speed) + 0.5F);
    if (pwm == 0) {
        // TB6612FNG: IN1=IN2=HIGH with STBY=HIGH selects short brake.
        digitalWrite(in1, HIGH);
        digitalWrite(in2, HIGH);
        ledcWrite(channel, cfg::kMotorMaxPwm);
        return;
    }

    if (pwm < cfg::kMotorMinimumPwm) {
        pwm = cfg::kMotorMinimumPwm;
    }
    pwm = constrain(pwm, 0, cfg::kMotorMaxPwm);
    digitalWrite(in1, speed > 0.0F ? HIGH : LOW);
    digitalWrite(in2, speed > 0.0F ? LOW : HIGH);
    ledcWrite(channel, pwm);
}

void MotorDriver::update(uint32_t nowUs) {
    if (lastUpdateUs_ == 0) {
        lastUpdateUs_ = nowUs;
        return;
    }

    const float dt = min((nowUs - lastUpdateUs_) / 1000000.0F, 0.05F);
    lastUpdateUs_ = nowUs;

    const float step = cfg::kMotorSlewPwmPerSecond * dt;

    actualLeft_ = moveTowards(actualLeft_, targetLeft_, step);
    actualRight_ = moveTowards(actualRight_, targetRight_, step);
    
    writeMotor(cfg::kLeftIn1Pin, cfg::kLeftIn2Pin, cfg::kLeftPwmChannel, actualLeft_);
    writeMotor(cfg::kRightIn1Pin, cfg::kRightIn2Pin, cfg::kRightPwmChannel, actualRight_);
}

void MotorDriver::emergencyStop() {
    targetLeft_ = targetRight_ = actualLeft_ = actualRight_ = 0.0F;

    if (cfg::kMotorStandbyPin >= 0) {
        digitalWrite(cfg::kMotorStandbyPin, HIGH);
    }

    writeMotor(cfg::kLeftIn1Pin, cfg::kLeftIn2Pin, cfg::kLeftPwmChannel, 0.0F);
    writeMotor(cfg::kRightIn1Pin, cfg::kRightIn2Pin, cfg::kRightPwmChannel, 0.0F);
}
