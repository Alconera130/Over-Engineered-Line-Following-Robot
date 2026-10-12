#include "system/OledDisplay.h"

#include <Wire.h>
#include <stdio.h>
#include <string.h>

#include "RobotConfig.h"

namespace {
constexpr uint8_t kDigits[10][5] = {
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, {0x00, 0x42, 0x7F, 0x40, 0x00},
    {0x42, 0x61, 0x51, 0x49, 0x46}, {0x21, 0x41, 0x45, 0x4B, 0x31},
    {0x18, 0x14, 0x12, 0x7F, 0x10}, {0x27, 0x45, 0x45, 0x45, 0x39},
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, {0x01, 0x71, 0x09, 0x05, 0x03},
    {0x36, 0x49, 0x49, 0x49, 0x36}, {0x06, 0x49, 0x49, 0x29, 0x1E}
};

constexpr uint8_t kLetters[26][5] = {
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, {0x7F, 0x49, 0x49, 0x49, 0x36},
    {0x3E, 0x41, 0x41, 0x41, 0x22}, {0x7F, 0x41, 0x41, 0x22, 0x1C},
    {0x7F, 0x49, 0x49, 0x49, 0x41}, {0x7F, 0x09, 0x09, 0x09, 0x01},
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, {0x7F, 0x08, 0x08, 0x08, 0x7F},
    {0x00, 0x41, 0x7F, 0x41, 0x00}, {0x20, 0x40, 0x41, 0x3F, 0x01},
    {0x7F, 0x08, 0x14, 0x22, 0x41}, {0x7F, 0x40, 0x40, 0x40, 0x40},
    {0x7F, 0x02, 0x0C, 0x02, 0x7F}, {0x7F, 0x04, 0x08, 0x10, 0x7F},
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, {0x7F, 0x09, 0x09, 0x09, 0x06},
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, {0x7F, 0x09, 0x19, 0x29, 0x46},
    {0x46, 0x49, 0x49, 0x49, 0x31}, {0x01, 0x01, 0x7F, 0x01, 0x01},
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, {0x1F, 0x20, 0x40, 0x20, 0x1F},
    {0x3F, 0x40, 0x38, 0x40, 0x3F}, {0x63, 0x14, 0x08, 0x14, 0x63},
    {0x07, 0x08, 0x70, 0x08, 0x07}, {0x61, 0x51, 0x49, 0x45, 0x43}
};

const char* trackEventName(const TrackTimeEvent& event, char* buffer, size_t size) {
    switch (event.command) {
        case TrackTimeCommand::SET_SPEED_PERCENT:
            snprintf(buffer, size, "SPD%u", static_cast<unsigned>(event.speedPercent));
            return buffer;
        case TrackTimeCommand::SET_PID_GAINS:
            return "PID";
        case TrackTimeCommand::ENABLE_JUNCTION_ROUTING:
            return "JUNC ON";
        case TrackTimeCommand::DISABLE_JUNCTION_ROUTING:
            return "JUNC OFF";
        case TrackTimeCommand::STOP:
            return "STOP";
    }
    return "EVENT";
}

const char* routeActionName(JunctionAction action) {
    switch (action) {
        case JunctionAction::LEFT: return "LEFT";
        case JunctionAction::RIGHT: return "RIGHT";
        case JunctionAction::STRAIGHT: return "STRAIGHT";
        case JunctionAction::ACUTE_LEFT: return "ACUTE-L";
        case JunctionAction::ACUTE_RIGHT: return "ACUTE-R";
    }
    return "?";
}
}  // namespace

bool OledDisplay::begin() {
    wireStarted_ = Wire.begin(cfg::kOledSdaPin, cfg::kOledSclPin);
    if (wireStarted_) {
        Wire.setClock(400000);
    }
    memset(displayed_, 0xFF, sizeof(displayed_));
    for (uint8_t page = 0; page < kPageCount; ++page) {
        dirtyFirst_[page] = 0;
        dirtyLast_[page] = kWidth - 1U;
    }
    dirtyPages_ = (1U << kPageCount) - 1U;
    return wireStarted_;
}

void OledDisplay::update(uint32_t nowMs, const OledDisplayState& state,
                         bool startButtonDown) {
    if (!wireStarted_) {
        return;
    }

    if (!controllerReady_) {
        if (startButtonDown || nowMs - lastInitAttemptMs_ < 500U) {
            return;
        }
        lastInitAttemptMs_ = nowMs;
        if (!initialiseController()) {
            return;
        }
        controllerReady_ = true;
        lastRenderMs_ = nowMs - cfg::kOledRenderIntervalMs;
        lastPageWriteMs_ = nowMs - cfg::kOledPageWriteIntervalMs;
    }

    if (!firstRenderDone_ || nowMs - lastRenderMs_ >= cfg::kOledRenderIntervalMs) {
        if (state.runActive) {
            frozenElapsedMs_ = state.elapsedMs;
            wasRunning_ = true;
        } else if (wasRunning_) {
            frozenElapsedMs_ = state.elapsedMs;
            wasRunning_ = false;
        }
        render(state, state.runActive ? state.elapsedMs : frozenElapsedMs_);
        lastRenderMs_ = nowMs;
        firstRenderDone_ = true;
    }

    // GPIO9 is both SDA and the active-low BOOT/START button. Avoid bus writes
    // while it is held so the button cannot corrupt an OLED data transaction.
    if (startButtonDown || digitalRead(cfg::kOledSdaPin) == LOW) {
        return;
    }

    if (dirtyPages_ == 0) {
        if (!displayOn_ &&
            nowMs - lastPageWriteMs_ >= cfg::kOledPageWriteIntervalMs) {
            static constexpr uint8_t kDisplayOn = 0xAF;
            displayOn_ = sendCommands(&kDisplayOn, 1);
            lastPageWriteMs_ = nowMs;
        }
        return;
    }
    if (nowMs - lastPageWriteMs_ < cfg::kOledPageWriteIntervalMs) {
        return;
    }

    for (uint8_t offset = 0; offset < kPageCount; ++offset) {
        const uint8_t page = static_cast<uint8_t>((nextPage_ + offset) % kPageCount);
        if ((dirtyPages_ & (1U << page)) == 0) {
            continue;
        }

        const bool sent = sendPage(page, dirtyFirst_[page], dirtyLast_[page],
                                   startButtonDown);
        lastPageWriteMs_ = nowMs;
        nextPage_ = static_cast<uint8_t>((page + 1U) % kPageCount);
        if (sent) {
            const uint16_t base = static_cast<uint16_t>(page) * kWidth;
            for (uint16_t column = dirtyFirst_[page]; column <= dirtyLast_[page]; ++column) {
                displayed_[base + column] = framebuffer_[base + column];
            }
            dirtyPages_ &= static_cast<uint8_t>(~(1U << page));
            dirtyFirst_[page] = 0xFF;
            dirtyLast_[page] = 0;
        }
        break;
    }
}

bool OledDisplay::initialiseController() {
    Wire.beginTransmission(cfg::kOledI2cAddress);
    if (Wire.endTransmission() != 0) {
        return false;
    }

    static constexpr uint8_t kInitialCommands[] = {
        0xAE,       // Display off
        0xD5, 0x80, // Clock divide / oscillator
        0xA8, 0x1F, // 32 multiplex rows
        0xD3, 0x00, // Display offset
        0x40,       // Start line
        0x8D, 0x14, // Charge pump
        0x20, 0x02, // Page addressing mode
        0xA1,       // Segment remap
        0xC8,       // COM scan direction
        0xDA, 0x02, // 128x32 COM pins
        0x81, 0x7F, // Contrast
        0xD9, 0xF1, // Pre-charge period
        0xDB, 0x40, // VCOMH deselect level
        0xA4,       // Resume RAM display
        0xA6        // Normal (not inverted) display
    };

    if (!sendCommands(kInitialCommands, sizeof(kInitialCommands))) {
        return false;
    }
    memset(framebuffer_, 0, sizeof(framebuffer_));
    for (uint8_t page = 0; page < kPageCount; ++page) {
        dirtyFirst_[page] = 0;
        dirtyLast_[page] = kWidth - 1U;
    }
    dirtyPages_ = (1U << kPageCount) - 1U;
    return true;
}

bool OledDisplay::sendCommands(const uint8_t* commands, uint8_t length) {
    if (length > 32U || digitalRead(cfg::kOledSdaPin) == LOW) {
        return false;
    }
    Wire.beginTransmission(cfg::kOledI2cAddress);
    Wire.write(static_cast<uint8_t>(0x00));
    Wire.write(commands, length);
    return Wire.endTransmission() == 0;
}

bool OledDisplay::sendPage(uint8_t page, uint8_t firstColumn, uint8_t lastColumn,
                           bool startButtonDown) {
    if (startButtonDown || digitalRead(cfg::kOledSdaPin) == LOW) {
        return false;
    }
    const uint8_t addressCommands[] = {
        static_cast<uint8_t>(0xB0U | page),
        static_cast<uint8_t>(firstColumn & 0x0FU),
        static_cast<uint8_t>(0x10U | (firstColumn >> 4U))
    };
    if (!sendCommands(addressCommands, sizeof(addressCommands))) {
        return false;
    }

    const uint16_t base = static_cast<uint16_t>(page) * kWidth;
    uint16_t column = firstColumn;
    while (column <= lastColumn) {
        if (digitalRead(cfg::kOledSdaPin) == LOW) {
            return false;
        }
        const uint8_t count = static_cast<uint8_t>(min<uint16_t>(
            kDataChunkSize, static_cast<uint16_t>(lastColumn - column + 1U)));
        Wire.beginTransmission(cfg::kOledI2cAddress);
        Wire.write(static_cast<uint8_t>(0x40));
        Wire.write(framebuffer_ + base + column, count);
        if (Wire.endTransmission() != 0) {
            return false;
        }
        column = static_cast<uint16_t>(column + count);
    }
    return true;
}

void OledDisplay::render(const OledDisplayState& state, uint32_t elapsedToShowMs) {
    memset(framebuffer_, 0, sizeof(framebuffer_));
    drawSensorRow(state);

    char text[24]{};
    if (state.calibrating) {
        drawText(0, 8, "CALIBRATING");
    } else if (state.calibrationRequired) {
        drawText(0, 8, "CAL REQUIRED");
    } else {
        const uint32_t totalSeconds = elapsedToShowMs / 1000U;
        snprintf(text, sizeof(text), "T+%02lu:%02lu.%lu",
                 static_cast<unsigned long>(totalSeconds / 60U),
                 static_cast<unsigned long>(totalSeconds % 60U),
                 static_cast<unsigned long>((elapsedToShowMs / 100U) % 10U));
        drawText(0, 8, text);
    }

    drawTrackTimeLine(state);
    drawRouteLine(state);
    compareWithDisplayed();
}

void OledDisplay::compareWithDisplayed() {
    for (uint8_t page = 0; page < kPageCount; ++page) {
        uint8_t first = 0xFF;
        uint8_t last = 0;
        const uint16_t base = static_cast<uint16_t>(page) * kWidth;
        for (uint8_t column = 0; column < kWidth; ++column) {
            if (framebuffer_[base + column] != displayed_[base + column]) {
                first = min(first, column);
                last = column;
            }
        }
        if (first == 0xFF) {
            continue;
        }
        if ((dirtyPages_ & (1U << page)) != 0) {
            dirtyFirst_[page] = min(dirtyFirst_[page], first);
            dirtyLast_[page] = max(dirtyLast_[page], last);
        } else {
            dirtyFirst_[page] = first;
            dirtyLast_[page] = last;
        }
        dirtyPages_ |= static_cast<uint8_t>(1U << page);
    }
}

void OledDisplay::drawSensorRow(const OledDisplayState& state) {
    constexpr uint8_t kBoxWidth = 5;
    constexpr uint8_t kBoxHeight = 7;
    constexpr uint8_t kPitch = 8;
    for (uint8_t sensor = 0; sensor < kViperSensorCount; ++sensor) {
        const uint8_t x = static_cast<uint8_t>(1U + sensor * kPitch);
        if (!state.sensorDataValid || state.sensors == nullptr) {
            drawRect(x, 0, kBoxWidth, kBoxHeight);
            drawLine(static_cast<uint8_t>(x + 1U), 1,
                     static_cast<uint8_t>(x + kBoxWidth - 2U),
                     static_cast<uint8_t>(kBoxHeight - 2U));
            drawLine(static_cast<uint8_t>(x + kBoxWidth - 2U), 1,
                     static_cast<uint8_t>(x + 1U),
                     static_cast<uint8_t>(kBoxHeight - 2U));
        } else if (state.sensors->reflectance[sensor] < cfg::kOledSurfaceThreshold) {
            // On the white OLED, a filled marker means the floor under that
            // sensor is dark/black. A hollow marker means bright/white.
            fillRect(x, 0, kBoxWidth, kBoxHeight);
        } else {
            drawRect(x, 0, kBoxWidth, kBoxHeight);
        }
    }
}

void OledDisplay::drawPixel(uint8_t x, uint8_t y) {
    if (x >= kWidth || y >= kHeight) {
        return;
    }
    const uint16_t index = static_cast<uint16_t>(y / 8U) * kWidth + x;
    framebuffer_[index] |= static_cast<uint8_t>(1U << (y & 7U));
}

void OledDisplay::drawRect(uint8_t x, uint8_t y, uint8_t width, uint8_t height) {
    if (width == 0 || height == 0) {
        return;
    }
    for (uint8_t column = 0; column < width; ++column) {
        drawPixel(static_cast<uint8_t>(x + column), y);
        drawPixel(static_cast<uint8_t>(x + column), static_cast<uint8_t>(y + height - 1U));
    }
    for (uint8_t row = 0; row < height; ++row) {
        drawPixel(x, static_cast<uint8_t>(y + row));
        drawPixel(static_cast<uint8_t>(x + width - 1U), static_cast<uint8_t>(y + row));
    }
}

void OledDisplay::fillRect(uint8_t x, uint8_t y, uint8_t width, uint8_t height) {
    for (uint8_t row = 0; row < height; ++row) {
        for (uint8_t column = 0; column < width; ++column) {
            drawPixel(static_cast<uint8_t>(x + column), static_cast<uint8_t>(y + row));
        }
    }
}

void OledDisplay::drawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) {
    int16_t x = x0;
    int16_t y = y0;
    const int16_t dx = abs(static_cast<int16_t>(x1) - x0);
    const int16_t sx = x0 < x1 ? 1 : -1;
    const int16_t dy = -abs(static_cast<int16_t>(y1) - y0);
    const int16_t sy = y0 < y1 ? 1 : -1;
    int16_t error = dx + dy;
    while (true) {
        drawPixel(static_cast<uint8_t>(x), static_cast<uint8_t>(y));
        if (x == x1 && y == y1) {
            break;
        }
        const int16_t twiceError = 2 * error;
        if (twiceError >= dy) {
            error += dy;
            x += sx;
        }
        if (twiceError <= dx) {
            error += dx;
            y += sy;
        }
    }
}

void OledDisplay::drawText(uint8_t x, uint8_t y, const char* text) {
    while (*text != '\0' && x + 5U < kWidth) {
        for (uint8_t column = 0; column < 5; ++column) {
            const uint8_t bits = glyphColumn(*text, column);
            for (uint8_t row = 0; row < 7; ++row) {
                if ((bits & (1U << row)) != 0) {
                    drawPixel(static_cast<uint8_t>(x + column), static_cast<uint8_t>(y + row));
                }
            }
        }
        x = static_cast<uint8_t>(x + 6U);
        ++text;
    }
}

uint8_t OledDisplay::glyphColumn(char character, uint8_t column) const {
    if (column >= 5) {
        return 0;
    }
    if (character >= 'a' && character <= 'z') {
        character = static_cast<char>(character - 'a' + 'A');
    }
    if (character >= '0' && character <= '9') {
        return kDigits[character - '0'][column];
    }
    if (character >= 'A' && character <= 'Z') {
        return kLetters[character - 'A'][column];
    }
    switch (character) {
        case '+': return column == 2 ? 0x3E : 0x08;
        case '-': return 0x08;
        case ':': return (column == 1 || column == 2) ? 0x24 : 0x00;
        case '.': return (column == 1 || column == 2) ? 0x60 : 0x00;
        case '#': return (column == 1 || column == 3) ? 0x7F : 0x14;
        default: return 0;
    }
}

void OledDisplay::drawTrackTimeLine(const OledDisplayState& state) {
    char text[24]{};
    char action[12]{};
    if (!state.trackTimeEnabled) {
        if (state.nextTrackEvent != nullptr) {
            snprintf(text, sizeof(text), "TT OFF %s",
                     trackEventName(*state.nextTrackEvent, action, sizeof(action)));
        } else {
            snprintf(text, sizeof(text), "TT OFF");
        }
    } else if (state.nextTrackEvent == nullptr) {
        snprintf(text, sizeof(text), "TT DONE");
    } else if (!state.trackClockActive) {
        snprintf(text, sizeof(text), "TT WAIT %s",
                 trackEventName(*state.nextTrackEvent, action, sizeof(action)));
    } else {
        const uint32_t tenths = state.timeToTrackEventMs / 100U;
        snprintf(text, sizeof(text), "TT+%02lu.%lu %s",
                 static_cast<unsigned long>(tenths / 10U),
                 static_cast<unsigned long>(tenths % 10U),
                 trackEventName(*state.nextTrackEvent, action, sizeof(action)));
    }
    drawText(0, 16, text);
}

void OledDisplay::drawRouteLine(const OledDisplayState& state) {
    char text[24]{};
    if (state.statusMessage != nullptr) {
        drawText(0, 24, state.statusMessage);
        return;
    }
    if (state.junctionDetected) {
        drawText(0, 24, "JUNCTION DETECTED");
        return;
    }
    if (!state.junctionRoutingEnabled) {
        snprintf(text, sizeof(text), "KR OFF %02u %s",
                 static_cast<unsigned>(state.routeSlot), routeActionName(state.routeAction));
    } else if (state.routeActionCurrent) {
        snprintf(text, sizeof(text), "KR%02u NOW %s",
                 static_cast<unsigned>(state.routeSlot), routeActionName(state.routeAction));
    } else {
        snprintf(text, sizeof(text), "KR%02u NEXT %s",
                 static_cast<unsigned>(state.routeSlot), routeActionName(state.routeAction));
    }
    drawText(0, 24, text);
}
