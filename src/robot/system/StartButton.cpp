#include "system/StartButton.h"

#include "RobotConfig.h"

void StartButton::begin() {
    pinMode(cfg::kStartButtonPin, INPUT_PULLUP);
    wasDown_ = false;
    rawDown_ = false;
    armed_ = false;
    releaseSeen_ = false;
}

bool StartButton::poll(uint32_t& heldMs) {
    const bool rawDown = digitalRead(cfg::kStartButtonPin) == LOW;
    const uint32_t now = millis();

    if (!armed_) {
        rawDown_ = rawDown;
        if (rawDown) {
            releaseSeen_ = false;
        } else if (!releaseSeen_) {
            releaseSeen_ = true;
            releasedMs_ = now;
        } else if (now - releasedMs_ >= cfg::kStartButtonReleaseArmMs) {
            armed_ = true;
            wasDown_ = false;
            rawDown_ = false;
            lastChangeMs_ = now;
        }
        return false;
    }

    if (rawDown != rawDown_) {
        rawDown_ = rawDown;
        lastChangeMs_ = now;
    }

    if (now - lastChangeMs_ < cfg::kStartButtonDebounceMs || rawDown_ == wasDown_) {
        return false;
    }

    if (rawDown_) {
        pressedMs_ = now;
        wasDown_ = true;
        return false;
    }

    heldMs = now - pressedMs_;
    wasDown_ = false;
    return true;
}
