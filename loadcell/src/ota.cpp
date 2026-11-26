#include <Arduino.h>
#include <Update.h>
#include <WebServer.h>

#include "config.h"
#include "ota.h"

namespace {
WebServer otaServer(80);
bool enabled = false;
bool uploading = false;
bool completed = false;

bool authorized() {
    return otaServer.authenticate("admin", LOADCELL_OTA_PASSWORD);
}

void serveUploadForm() {
    if (!authorized()) {
        otaServer.requestAuthentication();
        return;
    }
    otaServer.send(200, "text/html",
        "<!doctype html><html><head><title>RAWES firmware update</title></head>"
        "<body><h1>ESP32 firmware update</h1>"
        "<p>Upload the PlatformIO firmware.bin for this board. "
        "A successful update reboots the sensor and discards pending samples.</p>"
        "<form method='post' action='/update' enctype='multipart/form-data'>"
        "<input type='file' name='firmware' accept='.bin' required>"
        "<button>Upload firmware and reboot</button></form></body></html>");
}

void receiveUpload() {
    if (!authorized()) {
        return;
    }
    HTTPUpload &upload = otaServer.upload();
    switch (upload.status) {
    case UPLOAD_FILE_START:
        completed = false;
        uploading = Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH);
        if (!uploading) {
            Update.printError(Serial);
        } else {
            Serial.println("OTA firmware upload started.");
        }
        break;
    case UPLOAD_FILE_WRITE:
        if (uploading && Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
            Update.printError(Serial);
            Update.abort();
            uploading = false;
        }
        break;
    case UPLOAD_FILE_END:
        if (uploading) {
            completed = Update.end(true);
            if (!completed) {
                Update.printError(Serial);
            }
            uploading = false;
        }
        break;
    case UPLOAD_FILE_ABORTED:
        if (uploading) {
            Update.abort();
        }
        uploading = false;
        completed = false;
        Serial.println("OTA upload aborted; not rebooting.");
        break;
    }
}
}

void setupOTA() {
    if (strlen(LOADCELL_OTA_PASSWORD) == 0) {
        Serial.println("OTA disabled: set LOADCELL_OTA_PASSWORD in include/local_config.h.");
        return;
    }
    otaServer.on("/", HTTP_GET, serveUploadForm);
    otaServer.on("/serverIndex", HTTP_GET, serveUploadForm);
    otaServer.on("/update", HTTP_POST, [] {
        if (!authorized()) {
            otaServer.requestAuthentication();
            return;
        }
        otaServer.sendHeader("Connection", "close");
        if (!completed) {
            Serial.println("OTA update failed or no complete firmware received; not rebooting.");
            otaServer.send(400, "text/plain", "Firmware update failed; device not rebooted.");
            return;
        }
        otaServer.send(200, "text/plain", "Firmware installed; rebooting.");
        Serial.println("OTA update complete; rebooting.");
        delay(500);
        ESP.restart();
    }, receiveUpload);
    otaServer.begin();
    enabled = true;
    Serial.println("Password-protected OTA available on port 80 (username: admin).");
}

void handleOTAClient() {
    if (enabled) {
        otaServer.handleClient();
    }
}
