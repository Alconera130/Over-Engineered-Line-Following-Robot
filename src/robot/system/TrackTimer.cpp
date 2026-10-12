#include "system/TrackTimer.h"

#include "RobotConfig.h"

void TrackTimer::reset() {
    startedMs_ = 0;
    nextEvent_ = 0;
    speedPercent_ = 100;
    active_ = false;
}

void TrackTimer::start(uint32_t nowMs) {
    reset();
    startedMs_ = nowMs;
    active_ = true;
}

bool TrackTimer::poll(uint32_t nowMs, TrackTimeEvent& event) {
    if (!cfg::kEnableTrackTimeActions || !active_ ||
        nextEvent_ >= cfg::kTrackTimeEventCount) {
        return false;
    }

    const TrackTimeEvent& scheduled = cfg::kTrackTimeEvents[nextEvent_];
    if (nowMs - startedMs_ < scheduled.atMs) {
        return false;
    }

    event = scheduled;
    ++nextEvent_;
    if (event.command == TrackTimeCommand::SET_SPEED_PERCENT) {
        speedPercent_ = event.speedPercent < 1U ? 1U
                      : event.speedPercent > 100U ? 100U
                                                  : event.speedPercent;
        event.speedPercent = speedPercent_;
    }
    return true;
}

uint32_t TrackTimer::elapsedMs(uint32_t nowMs) const {
    return active_ ? nowMs - startedMs_ : 0U;
}

const TrackTimeEvent* TrackTimer::peekNextEvent() const {
    return nextEvent_ < cfg::kTrackTimeEventCount
               ? &cfg::kTrackTimeEvents[nextEvent_]
               : nullptr;
}
