#include "control/LineController.h"

#include "RobotConfig.h"

void LineController::reset() {
    previousError_ = 0.0F;
    integral_ = 0.0F;
    filteredDerivative_ = 0.0F;
}

void LineController::resetGains() {
    setGains(cfg::kPidP, cfg::kPidI, cfg::kPidD);
}

void LineController::setGains(float p, float i, float d) {
    proportionalGain_ = p;
    integralGain_ = i;
    derivativeGain_ = d;
    integral_ = 0.0F;
}

ControlOutput LineController::update(int16_t position, float dtSeconds) {
    const float dt = constrain(dtSeconds, 0.001F, 0.050F);
    const float error = constrain(position / static_cast<float>(kPositionLimit), -1.0F, 1.0F);
    const float rawDerivative = (error - previousError_) / dt;
    filteredDerivative_ = cfg::kDerivativeFilter * filteredDerivative_ +
                            (1.0F - cfg::kDerivativeFilter) * rawDerivative;

    const float proposedIntegral = constrain(integral_ + error * dt,
                                               -cfg::kIntegralLimit,
                                               cfg::kIntegralLimit);
    const float curvature = constrain(fabsf(error) +
                                        cfg::kDerivativeSlowdown * fabsf(filteredDerivative_),
                                    0.0F, 1.0F);
    const int16_t cruise = static_cast<int16_t>(cfg::kCruisePwm -
        curvature * (cfg::kCruisePwm - cfg::kCornerPwm));

    const float proposedCorrection = proportionalGain_ * error +
                                     integralGain_ * proposedIntegral +
                                     derivativeGain_ * filteredDerivative_;
    const float proposedLeft = cruise + proposedCorrection;
    const float proposedRight = cruise - proposedCorrection;
    const bool actuatorSaturated =
        proposedLeft > cfg::kMotorMaxPwm || proposedLeft < -cfg::kMotorMaxPwm ||
        proposedRight > cfg::kMotorMaxPwm || proposedRight < -cfg::kMotorMaxPwm;
    const bool saturationWorsenedByIntegral = actuatorSaturated &&
                                               error * proposedCorrection > 0.0F;
    if (!saturationWorsenedByIntegral) {
        integral_ = proposedIntegral;
    }

    const float correction = proportionalGain_ * error + integralGain_ * integral_ +
                             derivativeGain_ * filteredDerivative_;
    previousError_ = error;

    ControlOutput output{};
    output.error = error;
    output.correction = correction;
    output.cruise = cruise;
    output.left = static_cast<int16_t>(constrain(
        cruise + correction, -static_cast<float>(cfg::kMotorMaxPwm),
        static_cast<float>(cfg::kMotorMaxPwm)));
    output.right = static_cast<int16_t>(constrain(
        cruise - correction, -static_cast<float>(cfg::kMotorMaxPwm),
        static_cast<float>(cfg::kMotorMaxPwm)));
    return output;
}
