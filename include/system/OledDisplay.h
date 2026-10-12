#pragma once

#include <Arduino.h>

#include "RobotTypes.h"

struct OledDisplayState {
    const SensorFrame* sensors = nullptr;
    bool sensorDataValid = false;
    bool calibrationRequired = false;
    bool calibrating = false;
    bool runActive = false;
    uint32_t elapsedMs = 0;
    const TrackTimeEvent* nextTrackEvent = nullptr;
    uint32_t timeToTrackEventMs = 0;
    bool trackTimeEnabled = false;
    bool junctionRoutingEnabled = false;
    bool junctionDetected = false;
    const char* statusMessage = nullptr;
    uint8_t routeSlot = 1;
    JunctionAction routeAction = JunctionAction::STRAIGHT;
    bool routeActionCurrent = false;
};

class OledDisplay {
public:
    bool begin();
    void update(uint32_t nowMs, const OledDisplayState& state, bool startButtonDown);

private:
    static constexpr uint8_t kWidth = 128;
    static constexpr uint8_t kHeight = 32;
    static constexpr uint8_t kPageCount = kHeight / 8;
    static constexpr uint16_t kBufferSize = kWidth * kPageCount;
    static constexpr uint8_t kDataChunkSize = 32;

    bool initialiseController();
    bool sendCommands(const uint8_t* commands, uint8_t length);
    bool sendPage(uint8_t page, uint8_t firstColumn, uint8_t lastColumn,
                  bool startButtonDown);
    void render(const OledDisplayState& state, uint32_t elapsedToShowMs);
    void compareWithDisplayed();
    void drawSensorRow(const OledDisplayState& state);
    void drawPixel(uint8_t x, uint8_t y);
    void drawRect(uint8_t x, uint8_t y, uint8_t width, uint8_t height);
    void fillRect(uint8_t x, uint8_t y, uint8_t width, uint8_t height);
    void drawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
    void drawText(uint8_t x, uint8_t y, const char* text);
    uint8_t glyphColumn(char character, uint8_t column) const;
    void drawTrackTimeLine(const OledDisplayState& state);
    void drawRouteLine(const OledDisplayState& state);

    uint8_t framebuffer_[kBufferSize]{};
    uint8_t displayed_[kBufferSize]{};
    uint8_t dirtyPages_ = 0;
    uint8_t dirtyFirst_[kPageCount]{};
    uint8_t dirtyLast_[kPageCount]{};
    uint8_t nextPage_ = 0;
    uint32_t lastRenderMs_ = 0;
    uint32_t lastPageWriteMs_ = 0;
    uint32_t lastInitAttemptMs_ = 0;
    uint32_t frozenElapsedMs_ = 0;
    bool wireStarted_ = false;
    bool controllerReady_ = false;
    bool displayOn_ = false;
    bool firstRenderDone_ = false;
    bool wasRunning_ = false;
};
