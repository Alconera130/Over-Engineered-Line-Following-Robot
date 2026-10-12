#pragma once

#include <Arduino.h>

constexpr uint8_t kViperSensorCount = 16;
constexpr int16_t kPositionLimit = 7500;
// Alternating active/inactive channels are the densest possible arrangement,
// so an X16 can contain no more than eight independent line segments.
constexpr uint8_t kMaximumLineSegments = (kViperSensorCount + 1U) / 2U;

// The line's optical polarity after the sensor output has been normalised.
// DARK means a dark line on a lighter floor; LIGHT means a bright line on a
// darker floor. The sensor driver can change this automatically at a boundary.
enum class LinePolarity : uint8_t { DARK, LIGHT };

enum class RobotState : uint8_t {
  CALIBRATION_REQUIRED,
  WAIT_START,
  CALIBRATING,
  START_EXIT,
  RUNNING,
  JUNCTION_ENTRY,
  JUNCTION_TURN,
  LINE_RECOVERY,
  DEAD_END_TURN,
  FINISHED,
  FAULT
};

enum class JunctionAction : uint8_t {
  LEFT,
  RIGHT,
  STRAIGHT,
  ACUTE_LEFT,
  ACUTE_RIGHT
};

enum class TrackTimeCommand : uint8_t {
    SET_SPEED_PERCENT,
    SET_PID_GAINS,
    ENABLE_JUNCTION_ROUTING,
    DISABLE_JUNCTION_ROUTING,
    STOP
};

struct TrackTimeEvent {
    uint32_t atMs;
    TrackTimeCommand command;
    uint8_t speedPercent;
    float pidP;
    float pidI;
    float pidD;
};

// One contiguous (or nearly contiguous) group of sensors seeing the line.
// Its position is a centroid calculated from this group only, not from every
// dark/light feature currently visible across the entire X16.
struct LineSegment {
    int16_t position;
    uint16_t strength;
    uint8_t firstSensor;
    uint8_t lastSensor;
    uint8_t activeCount;
};

struct SensorFrame {
    uint16_t raw[kViperSensorCount];
    uint16_t line[kViperSensorCount];  // 0 = floor, 1000 = line
    // Full-array centroid. It remains useful for polarity and junction
    // recognition; normal steering instead uses the locked segment centroid.
    int16_t position;                  // -7500 = left, +7500 = right
    uint16_t strength;                 // mean line response, 0..1000
    uint8_t activeCount;
    LineSegment segments[kMaximumLineSegments];
    uint8_t segmentCount;
    bool lineVisible;
    bool leftEdge;
    bool rightEdge;
    bool wide;
    LinePolarity polarity;
    bool polarityChanged;
};

struct ControlOutput {
    int16_t left;
    int16_t right;
    float correction;
    int16_t cruise;
    float error;
};
