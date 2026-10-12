#include <Arduino.h>

#include "RobotConfig.h"

/*
    Isolated Linea Viper X16 diagnostic for the temporary ESP32-C3 wiring.

    This program deliberately never configures a motor-driver pin and never
    commands a motor. It only addresses the X16's apparent on-board 16:1 mux
    and samples its SIG/OUT signal with the ESP32 ADC.
*/

namespace {
constexpr uint16_t kAdcMaximum = 4095;
constexpr uint32_t kFrameIntervalMs = 250;

uint16_t minimum[kViperSensorCount];
uint16_t maximum[kViperSensorCount];
uint16_t reading[kViperSensorCount];
bool streaming = true;
uint32_t lastFrameMs = 0;

void resetCapture() {
    for (uint8_t channel = 0; channel < kViperSensorCount; ++channel) {
        minimum[channel] = kAdcMaximum;
        maximum[channel] = 0;
    }
}

uint16_t readChannel(uint8_t channel) {
    for (uint8_t bit = 0; bit < 4; ++bit) {
        digitalWrite(cfg::kMuxSelectPins[bit], (channel >> bit) & 0x01U);
    }

    // The mux output capacitance retains a little of the preceding channel.
    // Let it settle, discard one conversion, then average the configured
    // number of samples so adjacent black/white sensors do not contaminate
    // one another's diagnostic values.
    delayMicroseconds(cfg::kMuxSettleUs);
    analogRead(cfg::kViperAdcPin);

    uint32_t total = 0;
    for (uint8_t sample = 0; sample < cfg::kSamplesPerSensor; ++sample) {
        total += analogRead(cfg::kViperAdcPin);
    }
    return static_cast<uint16_t>(total / cfg::kSamplesPerSensor);
}

void sampleFrame() {
    for (uint8_t channel = 0; channel < kViperSensorCount; ++channel) {
        const uint16_t raw = readChannel(channel);
        reading[channel] = raw;
        minimum[channel] = min(minimum[channel], raw);
        maximum[channel] = max(maximum[channel], raw);
    }
}

void printFrame() {
    Serial.print(F("raw"));
    for (uint8_t channel = 0; channel < kViperSensorCount; ++channel) {
        Serial.print(channel == 0 ? F(" ") : F(","));
        Serial.print(reading[channel]);
    }
    Serial.println();
}

void printReport() {
    Serial.println(F("channel  minimum  maximum  span  result"));
    for (uint8_t channel = 0; channel < kSensorCount; ++channel) {
        const uint16_t span = maximum[channel] - minimum[channel];
        Serial.print(channel);
        Serial.print(F("        "));
        Serial.print(minimum[channel]);
        Serial.print(F("      "));
        Serial.print(maximum[channel]);
        Serial.print(F("     "));
        Serial.print(span);
        Serial.print(F("    "));
        Serial.println(span >= cfg::kMinimumCalibrationSpan ? F("OK") : F("LOW"));
    }
    Serial.print(F("A channel is OK when its span is at least "));
    Serial.println(cfg::kMinimumCalibrationSpan);
}

void printHelp() {
    Serial.println(F("Commands: c=clear min/max capture, p=print report, t=toggle raw stream, h=help"));
}

void pollConsole() {
    while (Serial.available() > 0) {
        switch (static_cast<char>(Serial.read())) {
            case 'c':
            case 'C':
                resetCapture();
                Serial.println(F("Capture cleared. Move black and white material under every sensor, then send p."));
                break;
            case 'p':
            case 'P':
                printReport();
                break;
            case 't':
            case 'T':
                streaming = !streaming;
                Serial.println(streaming ? F("Raw stream enabled.") : F("Raw stream paused."));
                break;
            case 'h':
            case 'H':
            case '?':
                printHelp();
                break;
            default:
                break;
        }
    }
}
}  // namespace

void setup() {
    Serial.begin(115200);
    for (uint8_t pin : cfg::kMuxSelectPins) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }
    pinMode(cfg::kViperAdcPin, INPUT);
    analogReadResolution(12);
    analogSetPinAttenuation(cfg::kViperAdcPin, ADC_11db);
    resetCapture();

    Serial.println();
    Serial.println(F("Viper X16 isolated raw-sensor test"));
    Serial.println(F("Safety: connect only verified sensor power and USB; leave motor power disconnected."));
    Serial.println(F("SIG/OUT must not exceed 3.3 V at the ESP32 ADC."));
    printHelp();
}

void loop() {
    pollConsole();

    const uint32_t now = millis();
    if (now - lastFrameMs < kFrameIntervalMs) {
        return;
    }
    lastFrameMs = now;
    sampleFrame();
    if (streaming) {
        printFrame();
    }
}
