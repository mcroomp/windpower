#pragma once

#include <stddef.h>
#include <stdint.h>

// Keep credentials in the ignored local header, never in this file.
#if __has_include("local_config.h")
#include "local_config.h"
#endif

#ifndef LOADCELL_WIFI_SSID
#define LOADCELL_WIFI_SSID ""
#endif
#ifndef LOADCELL_WIFI_PASSWORD
#define LOADCELL_WIFI_PASSWORD ""
#endif
#ifndef LOADCELL_CSV_URL
#define LOADCELL_CSV_URL ""
#endif
#ifndef LOADCELL_OTA_PASSWORD
#define LOADCELL_OTA_PASSWORD ""
#endif

namespace config {
constexpr uint8_t liftDataPin = 33;
constexpr uint8_t liftClockPin = 25;
constexpr uint8_t dragDataPin = 34;
constexpr uint8_t dragClockPin = 32;
constexpr uint16_t webPort = 88;
constexpr uint32_t sampleIntervalMs = 100;
constexpr uint32_t hx711TimeoutMs = 100;
constexpr size_t batchSize = 300;
constexpr size_t queueSize = 600;
constexpr uint32_t httpTimeoutMs = 3000;
constexpr uint32_t retryIntervalMs = 5000;
constexpr long utcOffsetSeconds = 3600;
constexpr char ntpServer[] = "pool.ntp.org";

}
