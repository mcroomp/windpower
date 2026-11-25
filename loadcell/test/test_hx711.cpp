#include <assert.h>
#include <stdlib.h>
#include <string>

#include "sample_csv.h"

namespace {
uint32_t now = 0;

struct Adc {
    uint8_t currentGain = 32;
    uint8_t nextGain = 32;
    bool ready = true;
    int reads = 0;

    bool wait_ready_timeout(uint32_t) {
        now += 10;
        return ready;
    }

    void set_gain(uint8_t gain) {
        nextGain = gain;
    }

    int32_t read() {
        ++reads;
        const int32_t value = currentGain == 128 ? -128 : currentGain;
        currentGain = nextGain;
        return value;
    }
};

uint32_t clockMs() {
    return now;
}

void switchingDiscardsPreviousChannelAndWaitsForSettling() {
    now = 0;
    Adc adc;
    Hx711Input input;
    int32_t raw = 999;
    assert(input.read(adc, clockMs, 100, 128, raw) == Hx711ReadStatus::Settling);
    assert(raw == 999 && adc.reads == 1);
    now = 409;
    assert(input.read(adc, clockMs, 100, 128, raw) == Hx711ReadStatus::Settling);
    now = 410;
    assert(input.read(adc, clockMs, 100, 128, raw) == Hx711ReadStatus::Ready);
    assert(raw == -128);

    assert(input.read(adc, clockMs, 100, 32, raw) == Hx711ReadStatus::Settling);
    assert(raw == -128);
    now += 400;
    assert(input.read(adc, clockMs, 100, 32, raw) == Hx711ReadStatus::Ready);
    assert(raw == 32);

    assert(input.read(adc, clockMs, 100, 64, raw) == Hx711ReadStatus::Settling);
    now += 400;
    assert(input.read(adc, clockMs, 100, 64, raw) == Hx711ReadStatus::Ready);
    assert(raw == 64);
}

void failedReadAndReplacementRequestNeverPublishWrongMode() {
    now = UINT32_MAX - 100;
    Adc adc;
    Hx711Input input;
    int32_t raw = 999;
    adc.ready = false;
    assert(input.read(adc, clockMs, 100, 128, raw) == Hx711ReadStatus::Timeout);
    assert(adc.reads == 0 && raw == 999);
    adc.ready = true;
    assert(input.read(adc, clockMs, 100, 128, raw) == Hx711ReadStatus::Settling);
    now += 50;
    assert(input.read(adc, clockMs, 100, 32, raw) == Hx711ReadStatus::Settling);
    now += 399;
    assert(input.read(adc, clockMs, 100, 32, raw) == Hx711ReadStatus::Settling);
    ++now;
    assert(input.read(adc, clockMs, 100, 32, raw) == Hx711ReadStatus::Ready);
    assert(raw == 32);
    adc.ready = false;
    assert(input.read(adc, clockMs, 100, 32, raw) == Hx711ReadStatus::Timeout);
}

void captureAlternatesAllFourAndRestartsCleanlyAfterModeChange() {
    now = 0;
    Adc first, second;
    CaptureAcquisition capture;
    CaptureValues values{};
    const auto poll = [&](CaptureMode mode) {
        return capture.read(first, second, clockMs, 100, mode, values);
    };
    assert(poll(CaptureMode::Two) == Hx711ReadStatus::Settling);
    now += 420;
    assert(poll(CaptureMode::Two) == Hx711ReadStatus::Ready);
    assert(captureGain(values) == 128 && values.raw[0] == -128 && values.raw[1] == -128);
    assert(std::string(captureLabel(values, 0)) == "A");
    assert(std::string(captureLabel(values, 1)) == "B");
    assert(poll(CaptureMode::Two) == Hx711ReadStatus::Ready);

    assert(poll(CaptureMode::Four) == Hx711ReadStatus::Settling);
    assert(capture.activeMode.load() == 0);
    now += 420;
    assert(poll(CaptureMode::Four) == Hx711ReadStatus::Ready);
    const CaptureValues queued = values;
    for (unsigned phase = 0; phase < 4; ++phase) {
        assert(values.mode == CaptureMode::Four);
        assert(values.secondary == (phase % 2 != 0));
        assert(captureGain(values) == (values.secondary ? 32 : 64));
        assert(values.raw[0] == captureGain(values) && values.raw[1] == captureGain(values));
        assert(std::string(captureLabel(values, 0)) == (values.secondary ? "A2" : "A1"));
        assert(std::string(captureLabel(values, 1)) == (values.secondary ? "B2" : "B1"));
        assert(poll(CaptureMode::Four) == Hx711ReadStatus::Settling);
        now += 420;
        if (phase == 0) {
            second.ready = false;
            assert(poll(CaptureMode::Four) == Hx711ReadStatus::Timeout);
            assert(capture.activeMode.load() == 0);
            assert(!values.secondary);
            second.ready = true;
        }
        assert(poll(CaptureMode::Four) == Hx711ReadStatus::Ready);
    }
    // Interrupt a pending secondary conversion with a return to two-input capture.
    assert(poll(CaptureMode::Four) == Hx711ReadStatus::Settling);
    assert(poll(CaptureMode::Two) == Hx711ReadStatus::Settling);
    now += 420;
    assert(poll(CaptureMode::Two) == Hx711ReadStatus::Ready);
    assert(values.mode == CaptureMode::Two && captureGain(values) == 128);
    assert(values.raw[0] == -128 && values.raw[1] == -128);
    assert(queued.mode == CaptureMode::Four && captureGain(queued) == 64);
}

void csvPreservesSignedCountsAndPerSampleModes() {
    Sample sample{{1735689600, 123456}, 0, {CaptureMode::Two, false, {-8388608, 8388607}}};
    char row[128];
    const int written = formatSampleCsv(sample, 0, 1, row, sizeof(row));
    assert(written > 0 && static_cast<size_t>(written) < sizeof(row));
    assert(std::string(row) ==
        "1,00:00:00.123,1735689600123456,A,-8388608,128\n");
    assert(formatSampleCsv(sample, 1, 2, row, sizeof(row)) > 0);
    assert(std::string(row) == "2,00:00:00.123,1735689600123456,B,8388607,128\n");
    sample.values = {CaptureMode::Four, false, {17, -42}};
    assert(formatSampleCsv(sample, 0, 3, row, sizeof(row)) > 0);
    assert(std::string(row) == "3,00:00:00.123,1735689600123456,A1,17,64\n");
    assert(formatSampleCsv(sample, 1, 4, row, sizeof(row)) > 0);
    assert(std::string(row) == "4,00:00:00.123,1735689600123456,B1,-42,64\n");
    sample.values.secondary = true;
    assert(formatSampleCsv(sample, 0, 5, row, sizeof(row)) > 0);
    assert(std::string(row) == "5,00:00:00.123,1735689600123456,A2,17,32\n");
    assert(formatSampleCsv(sample, 1, 6, row, sizeof(row)) > 0);
    assert(std::string(row) == "6,00:00:00.123,1735689600123456,B2,-42,32\n");
    char tooSmall[8];
    assert(formatSampleCsv(sample, 1, 6, tooSmall, sizeof(tooSmall)) >=
        static_cast<int>(sizeof(tooSmall)));
}
}

int main() {
    setenv("TZ", "UTC", 1);
    tzset();
    CaptureMode mode = CaptureMode::Two;
    assert(parseCaptureMode("4", mode) && mode == CaptureMode::Four);
    assert(parseCaptureMode("2", mode) && mode == CaptureMode::Two);
    for (const char *invalid : {"", "A128", "0", "3", "02", "4extra", " 4"}) {
        assert(!parseCaptureMode(invalid, mode));
        assert(mode == CaptureMode::Two);
    }
    switchingDiscardsPreviousChannelAndWaitsForSettling();
    failedReadAndReplacementRequestNeverPublishWrongMode();
    captureAlternatesAllFourAndRestartsCleanlyAfterModeChange();
    csvPreservesSignedCountsAndPerSampleModes();
    puts("PASS: two/four-input capture, switching, settling, timeout, and raw CSV tests.");
}
