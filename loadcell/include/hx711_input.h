#pragma once

#include <atomic>
#include <stdint.h>
#include <string.h>

enum class CaptureMode : uint8_t { Two = 2, Four = 4 };
enum class Hx711ReadStatus { Ready, Settling, Timeout };

inline bool parseCaptureMode(const char *text, CaptureMode &mode) {
    if (strcmp(text, "2") == 0) {
        mode = CaptureMode::Two;
    } else if (strcmp(text, "4") == 0) {
        mode = CaptureMode::Four;
    } else {
        return false;
    }
    return true;
}

class Hx711Input {
public:
    template <class Adc, class Clock>
    Hx711ReadStatus read(
        Adc &adc, Clock clock, uint32_t timeoutMs, uint8_t desiredGain, int32_t &raw
    ) {
        if (desiredGain != selectedGain) {
            if (!adc.wait_ready_timeout(timeoutMs)) {
                return Hx711ReadStatus::Timeout;
            }
            adc.set_gain(desiredGain);
            // These trailing clock pulses select the NEXT conversion, not the value just read.
            adc.read();
            selectedGain = desiredGain;
            selectedAt = clock();
            return Hx711ReadStatus::Settling;
        }
        // Cover the datasheet's worst-case settling time at 10 samples/second.
        if (static_cast<uint32_t>(clock() - selectedAt) < 400) {
            return Hx711ReadStatus::Settling;
        }
        if (!adc.wait_ready_timeout(timeoutMs)) {
            return Hx711ReadStatus::Timeout;
        }
        raw = static_cast<int32_t>(adc.read());
        return Hx711ReadStatus::Ready;
    }

private:
    uint8_t selectedGain = 0;
    uint32_t selectedAt = 0;
};

struct CaptureValues {
    CaptureMode mode;
    bool secondary;
    int32_t raw[2];
};

inline uint8_t captureGain(const CaptureValues &values) {
    return values.mode == CaptureMode::Two ? 128 : (values.secondary ? 32 : 64);
}

inline const char *captureLabel(const CaptureValues &values, unsigned amplifier) {
    if (values.mode == CaptureMode::Two) {
        return amplifier == 0 ? "A" : "B";
    }
    return amplifier == 0 ? (values.secondary ? "A2" : "A1") :
        (values.secondary ? "B2" : "B1");
}

class CaptureAcquisition {
public:
    std::atomic<uint8_t> activeMode{0};

    template <class Adc, class Clock>
    Hx711ReadStatus read(
        Adc &first, Adc &second, Clock clock, uint32_t timeoutMs,
        CaptureMode requested, CaptureValues &values
    ) {
        if (requested != mode) {
            mode = requested;
            secondary = false;
        }
        CaptureValues next{mode, secondary, {0, 0}};
        const uint8_t gain = captureGain(next);
        const auto firstStatus = inputs[0].read(first, clock, timeoutMs, gain, next.raw[0]);
        const auto secondStatus = inputs[1].read(second, clock, timeoutMs, gain, next.raw[1]);
        if (firstStatus != Hx711ReadStatus::Ready || secondStatus != Hx711ReadStatus::Ready) {
            activeMode.store(0);
            return firstStatus == Hx711ReadStatus::Timeout || secondStatus == Hx711ReadStatus::Timeout ?
                Hx711ReadStatus::Timeout : Hx711ReadStatus::Settling;
        }
        values = next;
        activeMode.store(static_cast<uint8_t>(mode));
        if (mode == CaptureMode::Four) {
            secondary = !secondary;
        }
        return Hx711ReadStatus::Ready;
    }

private:
    Hx711Input inputs[2];
    CaptureMode mode = CaptureMode::Two;
    bool secondary = false;
};
