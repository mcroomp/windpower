#pragma once

#include <inttypes.h>
#include <stdio.h>
#include <sys/time.h>
#include <time.h>

#include "hx711_input.h"

constexpr char sampleCsvHeader[] =
    "_rownum,_time,_time_unix_us,input,raw,gain\n";

struct Sample {
    timeval timestamp;
    uint32_t uptimeMs;
    CaptureValues values;
};

inline int formatSampleCsv(
    const Sample &sample, unsigned amplifier, uint32_t row, char *buffer, size_t capacity
) {
    tm localTime;
    if (localtime_r(&sample.timestamp.tv_sec, &localTime) == nullptr) {
        return -1;
    }
    const uint64_t unixUs = static_cast<uint64_t>(sample.timestamp.tv_sec) * 1000000 +
        sample.timestamp.tv_usec;
    return snprintf(buffer, capacity,
        "%" PRIu32 ",%02d:%02d:%02d.%03ld,%" PRIu64 ",%s,%" PRId32 ",%u\n",
        row, localTime.tm_hour, localTime.tm_min, localTime.tm_sec,
        static_cast<long>(sample.timestamp.tv_usec / 1000), unixUs,
        captureLabel(sample.values, amplifier), sample.values.raw[amplifier],
        static_cast<unsigned>(captureGain(sample.values)));
}
