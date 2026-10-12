#include "sensors/ViperX16.h"

#include <Preferences.h>
#include <string.h>

#include "RobotConfig.h"

namespace {
	constexpr uint32_t kCalibrationMagic = 0x58313643UL;  // "X16C"
	constexpr uint16_t kCalibrationVersion = 2;

	struct CalibrationBlob {
		uint32_t magic;
		uint16_t version;
		uint16_t crc;
		uint16_t minimum[kViperSensorCount];
		uint16_t maximum[kViperSensorCount];
	};

	uint16_t crc16(const uint8_t* data, size_t length) {
		uint16_t crc = 0xFFFF;

		for (size_t i = 0; i < length; ++i) {
			crc ^= static_cast<uint16_t>(data[i]) << 8;
			for (uint8_t bit = 0; bit < 8; ++bit) {
			crc = (crc & 0x8000U) ? static_cast<uint16_t>((crc << 1) ^ 0x1021U)
									: static_cast<uint16_t>(crc << 1);
			}
		}
		
		return crc;
	}
}  // namespace

void ViperX16::begin() {
	for (uint8_t pin : cfg::kMuxSelectPins) {
		pinMode(pin, OUTPUT);
		digitalWrite(pin, LOW);
	}
	pinMode(cfg::kViperAdcPin, INPUT);
	analogReadResolution(12);
	analogSetPinAttenuation(cfg::kViperAdcPin, ADC_11db);
	activePolarity_ = cfg::kDefaultLineIsDark ? LinePolarity::DARK
											 : LinePolarity::LIGHT;
	loadCalibration();
}

uint16_t ViperX16::readChannel(uint8_t channel) const {
	for (uint8_t bit = 0; bit < 4; ++bit) {
		digitalWrite(cfg::kMuxSelectPins[bit], (channel >> bit) & 0x01U);
	}

	// A mux holds charge from the prior input; discard the first conversion so it
	// cannot bias the next sensor, especially when black and white are adjacent.
	delayMicroseconds(cfg::kMuxSettleUs);
	analogRead(cfg::kViperAdcPin);

	uint32_t sum = 0;
	for (uint8_t sample = 0; sample < cfg::kSamplesPerSensor; ++sample) {
		sum += analogRead(cfg::kViperAdcPin);
	}
	return static_cast<uint16_t>(sum / cfg::kSamplesPerSensor);
}

uint16_t ViperX16::normaliseReflectance(uint8_t channel, uint16_t raw) const {
	const uint16_t low = minimum_[channel];
	const uint16_t high = maximum_[channel];

	if (high <= low + cfg::kMinimumCalibrationSpan) {
		return 0;
	}

	const int32_t scaled = constrain(
		(static_cast<int32_t>(raw) - low) * 1000L / static_cast<int32_t>(high - low),
		0L, 1000L);

	return cfg::kSensorOutputIncreasesWithReflectance
			? static_cast<uint16_t>(scaled)
			: static_cast<uint16_t>(1000 - scaled);
}

void ViperX16::buildFrame(const uint16_t response[], LinePolarity polarity,
							 SensorFrame& frame) const {
	int32_t weightedSum = 0;
	uint32_t lineSum = 0;
	uint32_t denominator = 0;
	frame.activeCount = 0;
	frame.segmentCount = 0;

	for (uint8_t i = 0; i < kViperSensorCount; ++i) {
		frame.line[i] = response[i];

		lineSum += frame.line[i];
		denominator += frame.line[i];

		const int16_t weight = static_cast<int16_t>((static_cast<int16_t>(i) * 1000) -
													kPositionLimit);
		
		weightedSum += static_cast<int32_t>(frame.line[i]) * weight;

		if (frame.line[i] >= cfg::kActiveThreshold) {
			++frame.activeCount;
		}
	}

	frame.strength = static_cast<uint16_t>(lineSum / kViperSensorCount);
	frame.position = denominator == 0
						? 0
						: static_cast<int16_t>(constrain(
								weightedSum / static_cast<int32_t>(denominator),
								-static_cast<int32_t>(kPositionLimit),
								static_cast<int32_t>(kPositionLimit)));
	
	frame.leftEdge = frame.line[0] >= cfg::kBranchThreshold;
	frame.rightEdge = frame.line[kViperSensorCount - 1] >= cfg::kBranchThreshold;
	frame.wide = frame.activeCount >= cfg::kJunctionActiveSensors;

	// Build separate line candidates from thresholded sensor groups. A short
	// inactive gap is allowed inside a segment because one weakly calibrated or
	// noisy channel should not make a single physical line appear as two paths.
	uint16_t strongestSegmentStrength = 0;
	uint8_t index = 0;
	while (index < kViperSensorCount) {
		while (index < kViperSensorCount &&
			   frame.line[index] < cfg::kActiveThreshold) {
			++index;
		}
		if (index >= kViperSensorCount || frame.segmentCount >= kMaximumLineSegments) {
			break;
		}

		LineSegment& segment = frame.segments[frame.segmentCount];
		segment.firstSensor = index;
		segment.lastSensor = index;
		segment.activeCount = 0;
		uint32_t segmentSum = 0;
		int32_t segmentWeightedSum = 0;
		uint8_t inactiveGap = 0;

		for (; index < kViperSensorCount; ++index) {
			if (frame.line[index] < cfg::kActiveThreshold) {
				++inactiveGap;
				if (inactiveGap > cfg::kLineSegmentMergeGapSensors) {
					break;
				}
				continue;
			}

			inactiveGap = 0;
			segment.lastSensor = index;
			++segment.activeCount;
			segmentSum += frame.line[index];
			const int16_t weight = static_cast<int16_t>(
				(static_cast<int16_t>(index) * 1000) - kPositionLimit);
			segmentWeightedSum += static_cast<int32_t>(frame.line[index]) * weight;
		}

		segment.strength = segment.activeCount == 0 ? 0 : static_cast<uint16_t>(
			segmentSum / segment.activeCount);
		segment.position = segmentSum == 0 ? 0 : static_cast<int16_t>(constrain(
			segmentWeightedSum / static_cast<int32_t>(segmentSum),
			-static_cast<int32_t>(kPositionLimit),
			static_cast<int32_t>(kPositionLimit)));
		strongestSegmentStrength = max(strongestSegmentStrength, segment.strength);
		++frame.segmentCount;
	}
	// Use the strength within a candidate segment. Averaging over all 16 sensors
	// diluted a narrow line enough that even one clearly active sensor was marked
	// invisible, so the tracker kept driving its stale command instead.
	frame.lineVisible = frame.segmentCount > 0 &&
		strongestSegmentStrength >= cfg::kLineVisibleStrength;

	frame.polarity = polarity;
	frame.polarityChanged = false;
}

int16_t ViperX16::candidateScore(const SensorFrame& frame,
							  bool isCurrentPolarity) const {
	if (!frame.lineVisible) {
		return -500;
	}

	int16_t score = 240;
	if (frame.activeCount <= cfg::kNominalLineMaximumSensors) {
		score += static_cast<int16_t>((cfg::kNominalLineMaximumSensors -
			frame.activeCount) * 18);
	} else {
		score -= static_cast<int16_t>((frame.activeCount -
			cfg::kNominalLineMaximumSensors) * 42);
	}

	// A line normally has a limited footprint; an almost-full array is usually
	// the background seen through the wrong polarity. Keep the current choice
	// sticky so a legitimate wide junction does not flip mode for one frame.
	if (frame.activeCount >= cfg::kUniformBackgroundSensors) {
		score -= 180;
	}
	if (isCurrentPolarity) {
		score += 65;
	}
	if (previousLineVisible_) {
		const int16_t displacement = abs(frame.position - previousPosition_);
		score -= static_cast<int16_t>(min(90, static_cast<int>(displacement / 95)));
	}
	return score;
}

void ViperX16::read(SensorFrame& frame) {
	uint16_t raw[kViperSensorCount]{};
	uint16_t darkResponse[kViperSensorCount]{};
	uint16_t lightResponse[kViperSensorCount]{};

	for (uint8_t i = 0; i < kViperSensorCount; ++i) {
		raw[i] = readChannel(i);
		const uint16_t reflectance = normaliseReflectance(i, raw[i]);
		const uint16_t dark = 1000 - reflectance;
		const uint16_t light = reflectance;

		if (!filterInitialised_) {
			darkFiltered_[i] = dark;
			lightFiltered_[i] = light;
		} else {
			darkFiltered_[i] += cfg::kSensorFilterAlpha * (dark - darkFiltered_[i]);
			lightFiltered_[i] += cfg::kSensorFilterAlpha * (light - lightFiltered_[i]);
		}
		darkResponse[i] = static_cast<uint16_t>(constrain(
			static_cast<int32_t>(darkFiltered_[i] + 0.5F), 0L, 1000L));
		lightResponse[i] = static_cast<uint16_t>(constrain(
			static_cast<int32_t>(lightFiltered_[i] + 0.5F), 0L, 1000L));
	}
	filterInitialised_ = true;

	SensorFrame darkFrame{};
	SensorFrame lightFrame{};
	buildFrame(darkResponse, LinePolarity::DARK, darkFrame);
	buildFrame(lightResponse, LinePolarity::LIGHT, lightFrame);

	SensorFrame* current = activePolarity_ == LinePolarity::DARK ? &darkFrame : &lightFrame;
	SensorFrame* alternative = activePolarity_ == LinePolarity::DARK ? &lightFrame : &darkFrame;
	const int16_t currentScore = candidateScore(*current, true);
	const int16_t alternativeScore = candidateScore(*alternative, false);
	const bool protectWideJunction = current->wide &&
		current->activeCount < cfg::kUniformBackgroundSensors;
	const bool shouldSwitch = cfg::kEnableAutomaticPolarity && !protectWideJunction &&
		alternative->lineVisible &&
		alternativeScore >= currentScore + cfg::kPolaritySwitchScoreMargin;

	if (shouldSwitch) {
		++switchCandidateFrames_;
	} else {
		switchCandidateFrames_ = 0;
	}

	bool changed = false;
	if (switchCandidateFrames_ >= cfg::kPolaritySwitchConfirmFrames) {
		activePolarity_ = activePolarity_ == LinePolarity::DARK
			? LinePolarity::LIGHT : LinePolarity::DARK;
		switchCandidateFrames_ = 0;
		current = activePolarity_ == LinePolarity::DARK ? &darkFrame : &lightFrame;
		changed = true;
	}

	frame = *current;
	for (uint8_t i = 0; i < kViperSensorCount; ++i) {
		frame.raw[i] = raw[i];
	}
	frame.polarityChanged = changed;
	if (frame.lineVisible) {
		previousPosition_ = frame.position;
	}
	previousLineVisible_ = frame.lineVisible;
}

void ViperX16::startCalibration() {
	calibrationValid_ = false;
	filterInitialised_ = false;
	switchCandidateFrames_ = 0;
	previousLineVisible_ = false;
	for (uint8_t i = 0; i < kViperSensorCount; ++i) {
		minimum_[i] = 4095;
		maximum_[i] = 0;
	}
}

void ViperX16::sampleCalibration() {
	for (uint8_t i = 0; i < kViperSensorCount; ++i) {
		const uint16_t raw = readChannel(i);
		minimum_[i] = min(minimum_[i], raw);
		maximum_[i] = max(maximum_[i], raw);
	}
}

bool ViperX16::rangesAreValid() const {
	for (uint8_t i = 0; i < kViperSensorCount; ++i) {
		if (maximum_[i] <= minimum_[i] + cfg::kMinimumCalibrationSpan) {
			return false;
		}
	}
	
	return true;
}

bool ViperX16::finishCalibration() {
    calibrationValid_ = rangesAreValid();
    filterInitialised_ = false;
    return calibrationValid_ && saveCalibration();
}

bool ViperX16::setManualCalibration(const uint16_t white[], const uint16_t black[]) {
	uint16_t proposedMinimum[kViperSensorCount];
	uint16_t proposedMaximum[kViperSensorCount];
	for (uint8_t i = 0; i < kViperSensorCount; ++i) {
		if (white[i] > 4095U || black[i] > 4095U) {
			return false;
		}
		const bool expectedOrder = cfg::kSensorOutputIncreasesWithReflectance
			? black[i] < white[i]
			: black[i] > white[i];
		if (!expectedOrder) {
			return false;
		}
		proposedMinimum[i] = min(white[i], black[i]);
		proposedMaximum[i] = max(white[i], black[i]);
		if (proposedMaximum[i] <= proposedMinimum[i] + cfg::kMinimumCalibrationSpan) {
			return false;
		}
	}

	uint16_t previousMinimum[kViperSensorCount];
	uint16_t previousMaximum[kViperSensorCount];
	memcpy(previousMinimum, minimum_, sizeof(minimum_));
	memcpy(previousMaximum, maximum_, sizeof(maximum_));
	const bool previousValidity = calibrationValid_;
	memcpy(minimum_, proposedMinimum, sizeof(minimum_));
	memcpy(maximum_, proposedMaximum, sizeof(maximum_));
	calibrationValid_ = true;
	if (!saveCalibration()) {
		memcpy(minimum_, previousMinimum, sizeof(minimum_));
		memcpy(maximum_, previousMaximum, sizeof(maximum_));
		calibrationValid_ = previousValidity;
		return false;
	}

	filterInitialised_ = false;
	previousLineVisible_ = false;
	switchCandidateFrames_ = 0;
	activePolarity_ = cfg::kDefaultLineIsDark ? LinePolarity::DARK : LinePolarity::LIGHT;
	return true;
}

bool ViperX16::saveCalibration() const {
	if (!calibrationValid_) {
		return false;
	}

	CalibrationBlob blob{};
	blob.magic = kCalibrationMagic;
	blob.version = kCalibrationVersion;

	memcpy(blob.minimum, minimum_, sizeof(minimum_));
	memcpy(blob.maximum, maximum_, sizeof(maximum_));

	blob.crc = 0;
	blob.crc = crc16(reinterpret_cast<const uint8_t*>(&blob), sizeof(blob));

	Preferences preferences;
	if (!preferences.begin("linebot", false)) {
		return false;
	}

	const size_t written = preferences.putBytes("viper-cal", &blob, sizeof(blob));
	preferences.end();

	return written == sizeof(blob);
}

bool ViperX16::loadCalibration() {
	Preferences preferences;
	if (!preferences.begin("linebot", true)) {
		return false;
	}

	if (preferences.getBytesLength("viper-cal") != sizeof(CalibrationBlob)) {
		preferences.end();
		return false;
	}

	CalibrationBlob blob{};
	const size_t read = preferences.getBytes("viper-cal", &blob, sizeof(blob));
	preferences.end();
	const uint16_t expectedCrc = blob.crc;
	blob.crc = 0;

	if (read != sizeof(blob) || blob.magic != kCalibrationMagic ||
		blob.version != kCalibrationVersion ||
		crc16(reinterpret_cast<const uint8_t*>(&blob), sizeof(blob)) != expectedCrc) {
		return false;
	}

	memcpy(minimum_, blob.minimum, sizeof(minimum_));
	memcpy(maximum_, blob.maximum, sizeof(maximum_));

	calibrationValid_ = rangesAreValid();
	filterInitialised_ = false;
	
	return calibrationValid_;
}
