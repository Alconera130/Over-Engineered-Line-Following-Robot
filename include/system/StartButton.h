#pragma once

#include <Arduino.h>

class StartButton {
public:
    void begin();
    bool poll(uint32_t& heldMs);
    bool armed() const { return armed_; }
    bool isDown() const { return rawDown_; }

private:
    uint32_t pressedMs_ = 0;
    uint32_t releasedMs_ = 0;
    uint32_t lastChangeMs_ = 0;
    bool wasDown_ = false;
    bool rawDown_ = false;
    bool armed_ = false;
    bool releaseSeen_ = false;
};
