#include <Arduino.h>
#include <HTTPClient.h>
#include <HX711.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>
#include <atomic>
#include <sys/time.h>
#include <time.h>

#include "config.h"
#include "ota.h"
#include "sample_csv.h"

namespace {
constexpr time_t earliestValidTime = 1577836800; // 2020-01-01 UTC
static_assert(config::batchSize % 2 == 0, "CSV batches must contain complete amplifier pairs.");

HX711 liftScale;
HX711 dragScale;
CaptureAcquisition acquisition;
std::atomic<CaptureMode> captureMode{CaptureMode::Two};
WebServer server(config::webPort);
Preferences preferences;
QueueHandle_t samples;
QueueHandle_t latestSample;
std::atomic<uint32_t> sampleIntervalMs{config::sampleIntervalMs};
String destination;
String filename;
String batch;
size_t batchRows = 0;
uint32_t rowNumber = 0;
bool headerPending = true;
uint32_t lastPostAttempt = 0;

[[noreturn]] void halt(const char *message) {
    Serial.println(message);
    for (;;) {
        delay(1000);
    }
}

bool validDestination(const String &url) {
    return url.startsWith("http://") && url.length() > 7 &&
        url.indexOf('#') < 0 && url.indexOf('\r') < 0 &&
        url.indexOf('\n') < 0 && url.indexOf('\'') < 0 &&
        url.indexOf('"') < 0 && url.indexOf('<') < 0 &&
        url.indexOf('>') < 0 && url.indexOf(' ') < 0;
}

String htmlEscape(String value) {
    value.replace("&", "&amp;");
    value.replace("<", "&lt;");
    value.replace(">", "&gt;");
    value.replace("\"", "&quot;");
    value.replace("'", "&#39;");
    return value;
}

bool postCsv(const String &content) {
    if (WiFi.status() != WL_CONNECTED || destination.isEmpty()) {
        Serial.println("CSV upload unavailable: check Wi-Fi and destination.");
        return false;
    }
    HTTPClient http;
    const String url = destination +
        (destination.indexOf('?') >= 0 ? "&filename=" : "?filename=") + filename;
    http.setConnectTimeout(config::httpTimeoutMs);
    http.setTimeout(config::httpTimeoutMs);
    if (!http.begin(url)) {
        Serial.println("CSV upload failed: cannot initialize HTTP request.");
        return false;
    }
    http.addHeader("Content-Type", "text/csv");
    const int status = http.POST(content);
    http.end();
    if (status < 200 || status >= 300) {
        Serial.printf("CSV upload failed: HTTP status/error %d; retaining batch.\n", status);
        return false;
    }
    return true;
}

bool flushBatch(bool includePartial = true) {
    if (filename.isEmpty()) {
        return false;
    }
    if (headerPending) {
        if (!postCsv(sampleCsvHeader)) {
            return false;
        }
        headerPending = false;
    }
    if (batchRows > 0 && (includePartial || batchRows == config::batchSize)) {
        if (!postCsv(batch)) {
            return false;
        }
        batch = "";
        batchRows = 0;
    }
    return true;
}

void startFile(const timeval &timestamp) {
    tm localTime;
    localtime_r(&timestamp.tv_sec, &localTime);
    char name[96];
    // Milliseconds avoid collisions when rotating files within one second.
    snprintf(name, sizeof(name), "%04d%02d%02d_%02d%02d%02d_%03ld_tpdata.csv",
        localTime.tm_year + 1900, localTime.tm_mon + 1, localTime.tm_mday,
        localTime.tm_hour, localTime.tm_min, localTime.tm_sec,
        static_cast<long>(timestamp.tv_usec / 1000));
    filename = name;
    rowNumber = 0;
    headerPending = true;
    Serial.println("CSV file: " + filename);
}

void appendSample(const Sample &sample) {
    for (unsigned amplifier = 0; amplifier < 2; ++amplifier) {
        char row[128];
        const int written = formatSampleCsv(sample, amplifier, rowNumber + 1, row, sizeof(row));
        if (written < 0 || static_cast<size_t>(written) >= sizeof(row)) {
            halt("Cannot format complete CSV sample.");
        }
        ++rowNumber;
        batch += row;
        ++batchRows;
    }
}

void acquireSamples(void *) {
    uint32_t lastWarning = millis() - 10000;
    for (;;) {
        const uint32_t started = millis();
        Sample sample{};
        const auto status = acquisition.read(liftScale, dragScale, millis,
            config::hx711TimeoutMs, captureMode.load(), sample.values);
        if (status == Hx711ReadStatus::Timeout) {
            if (millis() - lastWarning >= 10000) {
                Serial.println("HX711 timeout; no paired readings emitted.");
                lastWarning = millis();
            }
        } else if (status == Hx711ReadStatus::Ready) {
            gettimeofday(&sample.timestamp, nullptr);
            sample.uptimeMs = millis();
            xQueueOverwrite(latestSample, &sample);
            if (sample.timestamp.tv_sec < earliestValidTime) {
                if (millis() - lastWarning >= 10000) {
                    Serial.println("Waiting for NTP time; no timestamped rows emitted.");
                    lastWarning = millis();
                }
            } else if (xQueueSend(samples, &sample, 0) != pdTRUE) {
                if (millis() - lastWarning >= 10000) {
                    Serial.println("CSV sample queue full; dropping new samples.");
                    lastWarning = millis();
                }
            }
        }
        const uint32_t elapsed = millis() - started;
        const uint32_t interval = sampleIntervalMs.load();
        delay(elapsed < interval ? interval - elapsed : 1);
    }
}

String hx711StatusJson() {
    Sample sample;
    const bool available = xQueuePeek(latestSample, &sample, 0) == pdTRUE;
    String result = "{\"requested_mode\":" + String(static_cast<uint8_t>(captureMode.load())) +
        ",\"active_mode\":" + String(acquisition.activeMode.load()) + ",\"last_sample\":";
    if (!available) {
        return result + "null}";
    }
    result += "{\"mode\":" + String(static_cast<uint8_t>(sample.values.mode)) +
        ",\"age_ms\":" + String(static_cast<uint32_t>(millis() - sample.uptimeMs)) +
        ",\"time_synchronized\":" +
        (sample.timestamp.tv_sec >= earliestValidTime ? "true" : "false") + ",\"readings\":[";
    for (unsigned amplifier = 0; amplifier < 2; ++amplifier) {
        if (amplifier != 0) {
            result += ",";
        }
        result += String("{\"input\":\"") + captureLabel(sample.values, amplifier) +
            "\",\"raw\":" + String(sample.values.raw[amplifier]) +
            ",\"gain\":" + String(captureGain(sample.values)) + "}";
    }
    return result + "]}}";
}

void setupWeb() {
    server.on("/", HTTP_GET, [] {
        String page = "<!doctype html><html><head><meta charset='utf-8'>"
            "<title>RAWES Loadcell</title></head><body><h1>RAWES Loadcell</h1>";
        page += "<p>Two HX711 amplifiers. Sample interval: " +
            String(sampleIntervalMs.load()) + " ms. File: " + htmlEscape(filename) + "</p>";
        page += "<form action='/create_new_csv' method='get'>"
            "<label>CSV destination <input type='url' id='CSVStorageURL' "
            "name='url' size='60' value='" + htmlEscape(destination) +
            "' required></label><button>Save and start new CSV</button></form>";
        page += "<form action='/setSamplingRate' method='get'>"
            "<label>Sample interval (ms) <input type='number' name='val' min='10' "
            "max='60000' value='" + String(sampleIntervalMs.load()) +
            "' required></label><button>Save</button></form>";
        page += "<h2>Capture mode</h2><p>Raw ADC counts only; "
            "calibration belongs to the receiver. <a href='/hx711'>Live status</a></p>";
        const bool four = captureMode.load() == CaptureMode::Four;
        page += String("<form action='/set_hx711' method='get'><select name='mode'>"
            "<option value='2'") + (!four ? " selected>" : ">") +
            "2 inputs: A, B (gain 128)</option><option value='4'" +
            (four ? " selected>" : ">") +
            "4 inputs: A1, A2, B1, B2 (gains 64/32)</option></select>"
            "<button>Save mode</button></form>"
            "<p>Four-input mode alternates inputs with settling; readings are not simultaneous.</p>";
        page += "</body></html>";
        server.send(200, "text/html", page);
    });
    server.on("/hx711", HTTP_GET, [] {
        server.send(200, "application/json", hx711StatusJson());
    });
    server.on("/set_hx711", HTTP_GET, [] {
        CaptureMode mode;
        if (!parseCaptureMode(server.arg("mode").c_str(), mode)) {
            server.send(400, "text/plain", "Use mode=2 or mode=4.");
            return;
        }
        if (preferences.putUChar("capture_mode", static_cast<uint8_t>(mode)) != sizeof(uint8_t)) {
            Serial.println("Failed to persist HX711 mode.");
            server.send(500, "text/plain", "Failed to persist HX711 mode.");
            return;
        }
        captureMode.store(mode);
        server.send(202, "application/json", hx711StatusJson());
    });
    server.on("/setSamplingRate", HTTP_GET, [] {
        const String value = server.arg("val");
        bool digitsOnly = !value.isEmpty() && value.length() <= 5;
        for (size_t i = 0; i < value.length(); ++i) {
            digitsOnly = digitsOnly && value[i] >= '0' && value[i] <= '9';
        }
        const long interval = value.toInt();
        if (!digitsOnly || interval < 10 || interval > 60000) {
            server.send(400, "text/plain", "val must be an interval of 10..60000 ms.");
            return;
        }
        if (preferences.putUInt("interval_ms", interval) != sizeof(uint32_t)) {
            Serial.println("Failed to persist sample interval.");
            server.send(500, "text/plain", "Failed to persist sample interval.");
            return;
        }
        sampleIntervalMs.store(interval);
        server.send(200, "text/plain", "Sample interval saved (ms): " + value);
    });
    server.on("/create_new_csv", HTTP_GET, [] {
        const String url = server.arg("url");
        if (!validDestination(url)) {
            server.send(400, "text/plain", "url must be a plain HTTP CSV endpoint.");
            return;
        }
        timeval now;
        gettimeofday(&now, nullptr);
        if (now.tv_sec < earliestValidTime) {
            server.send(503, "text/plain", "Waiting for NTP time.");
            return;
        }
        if (!filename.isEmpty() && !flushBatch()) {
            server.send(503, "text/plain", "Cannot flush old CSV; destination unchanged.");
            return;
        }
        if (preferences.putString("csv_url", url) != url.length()) {
            Serial.println("Failed to persist CSV destination.");
            server.send(500, "text/plain", "Failed to persist CSV destination.");
            return;
        }
        destination = url;
        startFile(now);
        server.send(200, "text/plain", "CSV destination saved; new CSV queued.");
    });
    server.begin();
}
}

void setup() {
    Serial.begin(115200);
    if (!preferences.begin("rawes-loadcell", false)) {
        halt("Cannot open loadcell configuration in NVS.");
    }
    destination = preferences.getString("csv_url", LOADCELL_CSV_URL);
    if (!destination.isEmpty() && !validDestination(destination)) {
        halt("Invalid CSV destination in configuration.");
    }
    const uint32_t interval = preferences.getUInt("interval_ms", config::sampleIntervalMs);
    if (interval < 10 || interval > 60000) {
        halt("Invalid sample interval in NVS.");
    }
    sampleIntervalMs.store(interval);
    const uint8_t mode = preferences.getUChar("capture_mode", 2);
    if (mode != 2 && mode != 4) {
        halt("Invalid capture mode in NVS.");
    }
    captureMode.store(static_cast<CaptureMode>(mode));
    samples = xQueueCreate(config::queueSize, sizeof(Sample));
    latestSample = xQueueCreate(1, sizeof(Sample));
    if (samples == nullptr || latestSample == nullptr || !batch.reserve(config::batchSize * 128)) {
        halt("Cannot allocate CSV buffers.");
    }
    liftScale.begin(config::liftDataPin, config::liftClockPin);
    dragScale.begin(config::dragDataPin, config::dragClockPin);

    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    if (strlen(LOADCELL_WIFI_SSID) == 0) {
        Serial.println("Wi-Fi SSID missing: create include/local_config.h and rebuild.");
    } else {
        WiFi.begin(LOADCELL_WIFI_SSID, LOADCELL_WIFI_PASSWORD);
    }
    configTime(config::utcOffsetSeconds, 0, config::ntpServer);
    setupWeb();
    setupOTA();
    if (xTaskCreatePinnedToCore(acquireSamples, "loadcell_sampling", 4096,
            nullptr, 1, nullptr, 1) != pdPASS) {
        halt("Cannot create loadcell sampling task.");
    }
    Serial.println("RAWES loadcell started; web configuration on port 88.");
}

void loop() {
    handleOTAClient();
    server.handleClient();
    static bool wasConnected = false;
    const bool connected = WiFi.status() == WL_CONNECTED;
    if (connected != wasConnected) {
        Serial.println(connected ? "Wi-Fi connected: " + WiFi.localIP().toString() :
            "Wi-Fi disconnected; CSV upload paused.");
        wasConnected = connected;
    }
    if (filename.isEmpty() && !destination.isEmpty()) {
        timeval now;
        gettimeofday(&now, nullptr);
        if (now.tv_sec >= earliestValidTime) {
            startFile(now);
        }
    }
    if (!filename.isEmpty()) {
        Sample sample;
        while (batchRows < config::batchSize && xQueueReceive(samples, &sample, 0) == pdTRUE) {
            appendSample(sample);
        }
        if ((headerPending || batchRows == config::batchSize) &&
                millis() - lastPostAttempt >= config::retryIntervalMs) {
            lastPostAttempt = millis();
            flushBatch(false);
        }
    }
    delay(2);
}
