#pragma once

#include <Arduino.h>

#include "RobotTypes.h"

class ViperX16 {
    public:
        void begin();
        void read(SensorFrame& frame);

        void startCalibration();
        void sampleCalibration();
        bool finishCalibration();
        bool loadCalibration();
        bool saveCalibration() const;
        bool calibrationValid() const { return calibrationValid_; }
        LinePolarity polarity() const { return activePolarity_; }

    private:
        uint16_t readChannel(uint8_t channel) const;
        uint16_t normaliseReflectance(uint8_t channel, uint16_t raw) const;
        void buildFrame(const uint16_t response[], LinePolarity polarity,
                        SensorFrame& frame) const;
        int16_t candidateScore(const SensorFrame& frame,
                               bool isCurrentPolarity) const;
        bool rangesAreValid() const;

        uint16_t minimum_[kViperSensorCount]{};
        uint16_t maximum_[kViperSensorCount]{};
        float darkFiltered_[kViperSensorCount]{};
        float lightFiltered_[kViperSensorCount]{};
        bool filterInitialised_ = false;
        bool calibrationValid_ = false;
        LinePolarity activePolarity_ = LinePolarity::DARK;
        int16_t previousPosition_ = 0;
        bool previousLineVisible_ = false;
        uint8_t switchCandidateFrames_ = 0;
};
