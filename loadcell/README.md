# ESP32 load-cell firmware

Standalone PlatformIO/Arduino firmware for a **LOLIN D32 Pro (ESP32)**.
Extracted from an existing sensor application without its custom library
dependencies, actuator code, or local configuration.

The two HX711 amplifiers, 100 ms default sample interval, 300-row upload batches,
and HTTP port 88 are retained. CSV carries raw ADC counts and acquisition
metadata; calibration and derived quantities belong to the receiver.
Browser-based OTA is retained. Servo motion, pitch/RPM/light-barrier sensing,
and InfluxDB are excluded.
There are no actuator outputs.

## Build

Install PlatformIO Core if it is not already available:

```powershell
uv tool install --default-index https://packagefeedproxy.microsoft.io/pypi/simple/ platformio==6.1.18
```

From this directory:

```powershell
Copy-Item include\local_config.h.example include\local_config.h
# Edit include\local_config.h with your Wi-Fi and receiving server.
pio run
```

The local header is ignored by Git. A build also works without it, but logs
missing Wi-Fi configuration and cannot upload. Original credentials are not copied.
Build output is `.pio\build\lolin_d32_pro\firmware.bin`.
PlatformIO downloads the pinned ESP32 platform and HX711 dependency on first build.

Uploading is a separate, explicit operation; do not connect actuators:

```powershell
pio run --target upload --upload-port COM_PORT
pio device monitor --port COM_PORT --baud 115200
```

Replace `COM_PORT` with the positively identified ESP32 serial port.
USB uploads use 115200 baud for reliability on the observed CH340 link.
Ordinary `pio run` never uploads or opens a monitor.
No filesystem image or external web assets are required.

## Private Wi-Fi and OTA configuration

Store both passwords in **`include/local_config.h`**, which is ignored by Git.
Create it by copying `include/local_config.h.example`, then edit the copy:

- `LOADCELL_WIFI_SSID`: your Wi-Fi network name.
- `LOADCELL_WIFI_PASSWORD`: your Wi-Fi password.
- `LOADCELL_OTA_PASSWORD`: a strong, unique browser-OTA password.
  An empty value disables OTA; the OTA username is `admin`.
- `LOADCELL_CSV_URL`: optional HTTP receiver URL; leave empty to configure it
  later through the sensor's web page.

Never put real values in `include/config.h`, the checked-in example,
`platformio.ini`, documentation, or compiler flags committed to Git.
Rebuild after editing the private header; these credentials are compiled into
the application, not provisioned through the CSV settings page.
Keep generated `.pio/` firmware and `backups/` private too: they can contain
the compiled credentials. Do not force-add ignored files.

## Back up the existing board over USB

Before replacing the old firmware, use the independent read-only utility.
Run these commands from the repository root:

```powershell
uv run --script loadcell\backup_esp32.py --help
uv run --script loadcell\backup_esp32.py --list
uv run --script loadcell\backup_esp32.py --port COM_PORT
```

Replace `COM_PORT` with the USB serial port you positively identified from the
listing (description, USB VID/PID, serial number/location). The utility never
guesses or connects during `--list`. Its inline dependency configuration uses
the Microsoft Python feed. It is independent of the installed PlatformIO version.

Close serial monitors first. Disconnect external mechanisms/actuators and put
the setup in a safe state: entering the bootloader interrupts the old firmware.
The utility resets the selected classic ESP32 into its serial bootloader,
detects the actual flash capacity, reads **all** flash, checks the exact byte
count, then runs esptool's full-image verification against the device.
It never issues flash-write, erase, or eFuse-write commands. It does upload
esptool's temporary helper to RAM, which is not a firmware installation.

Each attempt has a unique directory under Git-ignored `loadcell\backups`.
Only a completed and verified attempt gets `full_flash.bin` and `manifest.json`
with size, SHA-256, tool version, and USB identity. `esptool.log` records tool
output. Read progress is printed and logged at each 5% threshold, based on actual
bytes received, rather than every block. Unbuffered esptool output makes updates
visible during the read; verification is announced as a separate stage.
Failed attempts retain
`.partial.bin` if any bytes were read; do not
treat these as usable backups. The default baud is 115200 for reliability;
`--baud 460800` is available for known-good links.

Afterward, press EN/RESET or reconnect USB to resume the original firmware:
the utility intentionally leaves the device in the bootloader between operations
and after completion so the application cannot change NVS during verification.
If automatic bootloader entry fails, hold BOOT, tap EN, release BOOT, and retry.
Stop if security restrictions prevent reading; do not bypass protections.

The dump includes bootloader, partition table, applications, NVS, and filesystem
contents, **not source code or eFuses**. It may contain passwords: keep the entire
backup directory private, outside Git, and copy it to a safe second location.
An alternative `--output-dir` must also be kept out of version control.
This tool deliberately has no restore operation. Encryption, secure boot, or
changed eFuses can affect restorability; a verified read is not a tested restore.

## Wi-Fi firmware updates (OTA)

Set `LOADCELL_OTA_PASSWORD` in the ignored local configuration header before the
initial USB upload. With an empty password, OTA is disabled and reports this over
serial. Once connected, open `http://<device-ip>/` (port 80), sign in as `admin`
using your configured password, and upload
`.pio\build\lolin_d32_pro\firmware.bin` from a subsequent build.
The sensor validates the image using the ESP32 Update library and reboots only
after a completed update. Failed or aborted uploads report failure and do not
intentionally reboot. The page works without internet access or external scripts.
Configuration remains on port 88; `/serverIndex` also opens the OTA form.

Use the application `firmware.bin`, not a bootloader, filesystem, or merged image.
Keep the same board/partition layout for OTA builds. NVS configuration survives
normal application updates; pending samples do not. OTA is password-protected
on the server, unlike the original browser-only login. Transport is still plain
HTTP, so use only a trusted isolated network. This is not signed-image verification
or an automatic rollback system; keep USB recovery available.

## Wiring and calibration

| Amplifier / CSV input labels | HX711 DOUT | HX711 SCK |
|---|---:|---:|
| A (formerly lift) / `A` or `A1`, `A2` | GPIO 33 | GPIO 25 |
| B (formerly drag) / `B` or `B1`, `B2` | GPIO 34 | GPIO 32 |

Use the ESP32's 3.3 V logic levels and a common ground. Verify your HX711 board's
supply and logic requirements before wiring. GPIO 34 is input-only.
Board selection, pins, timeouts, and UTC offset live in
[include/config.h](include/config.h). Each amplifier has two multiplexed inputs.
No calibration, automatic tare, voltage
conversion, or force conversion is performed on the ESP32.
The receiver must apply calibration for the specific amplifier, input, and gain.
**Verify units, sign, zero, and scale against known loads before relying on
readings.** Gain alone does not determine load-cell sensitivity or zero offset.

## Configuration and CSV

The serial monitor prints the acquired IP. Open `http://<device-ip>:88/` to set
the CSV destination, sample interval, and two/four-input capture mode. These settings persist in NVS and
override build defaults. Wi-Fi credentials remain in the local build header.
The existing configuration helper
[configure_esp_csv.ps1](../scripts/configure_esp_csv.ps1) can set the destination.
It also selects and verifies four-input capture. If the destination already
matches, it preserves the current file rather than requesting a flush before
the receiver starts.
For the Windows launcher, copy `launch_config.cmd.example` to the ignored
`launch_config.cmd` and configure the device URL and MAVLink connection locally.
Alternatively set `LOADCELL_DEVICE_URL` and `LINKHUB_CONNECTION` in the launching
environment. Never commit the local headers, launch configuration, firmware
binaries, flash backups, logs, or device-specific identifiers.

- `GET /create_new_csv?url=<URL-encoded-http-endpoint>` saves the destination and
  rotates the file after flushing the current batch. A failed flush leaves the
  destination unchanged; restore the old receiver before retrying.
- `GET /setSamplingRate?val=100` sets **milliseconds between sample attempts**,
  not Hz. Accepted range: 10..60000 ms. Actual throughput is limited by the
  HX711 conversion rate, timeouts, and channel switching/settling.
- `GET /set_hx711?mode=2` selects the primary input of each amplifier at gain
  128, labeled **A** and **B**. This is the default.
- `GET /set_hx711?mode=4` captures **A1, A2, B1, B2** automatically.
  A1/B1 are the amplifiers' primary HX711 inputs at gain **64**; A2/B2 are
  their secondary HX711 inputs at gain **32**. The HX711 cannot give both
  inputs the same gain. Capture alternates A1/B1, then A2/B2, repeatedly.
  The labels A/B identify physical amplifiers, not HX711 internal channel names.
  Invalid or missing selections return
  HTTP 400 without changing the saved/requested configuration.
- Accepted mode changes return **HTTP 202**, with JSON status. Application is
  asynchronous: the acquisition task discards the old-channel conversion,
  selects the new channel/gain, and waits at least 400 ms for settling.
  This applies to every alternation in four-input mode, reducing the per-input
  rate substantially (typically about one reading/second/input at the default
  interval; actual hardware timing varies). Inputs are not simultaneous.
  No paired CSV sample is emitted while either amplifier is settling.
- `GET /hx711` returns `requested_mode` (2 or 4), `active_mode` (0 while
  settling or after a timeout), and `last_sample`. The last sample contains
  its own `mode`, `age_ms`, `time_synchronized`, and two `readings`, each with
  `input`, signed `raw`, and `gain`. In mode 4 the latest pair alternates
  between A1/B1 and A2/B2; it is not a cache of all four.
  `last_sample` is null before any valid paired reading and retains its actual
  acquisition labels/gain during subsequent mode changes.
  Raw status works without a CSV receiver or NTP synchronization.
- CSV is posted as `text/csv` with `?filename=<timestamp>_tpdata.csv` (or an
  additional query parameter if the destination already has a query).
  For LinkHub, use `http://<host>:8999/v1/csv` or the port of your instance.

The ordered header is:

```csv
_rownum,_time,_time_unix_us,input,raw,gain
```

Each row contains **one input reading**, not one column per input. The same
schema works in both modes without changing the receiver mapping:

```csv
1,12:00:00.100,1795604400100000,A1,-1234,64
2,12:00:00.100,1795604400100000,B1,5678,64
3,12:00:00.700,1795604400700000,A2,-2345,32
4,12:00:00.700,1795604400700000,B2,6789,32
```

In two-input mode the labels are simply `A` and `B`, both with gain 128.
`raw` is a signed 24-bit ADC count, not a force or voltage. Each queued sample
retains its own labels and gain; changing modes cannot relabel older rows.
The old derived lift/drag and unused RPM/angle columns are not emitted.

Row numbers start at one per file. Timestamps are captured after the two
sequential amplifier reads, not when uploading; the two adjacent rows share
that pair-completion timestamp. The ADC inputs are not sampled
simultaneously. CSV requires NTP synchronization.
`_time_unix_us` is the UTC Unix timestamp in integer microseconds, including the
date. `_time` remains `HH:MM:SS.mmm` with the original fixed UTC+1 offset
(no daylight saving adjustment) for LinkHub's existing timing interface.
Filenames include milliseconds to reduce rotation collisions.

The supplied [LinkHub mapping](../linkhub/csv-map.json) matches this raw schema.
Restart a running LinkHub receiver once with the updated mapping before sending
these rows; its mapping is loaded at process startup. Later mode switches do
not require receiver changes. All post-processing,
including calibration and physical-unit conversion, is the receiver's job.

## Focused software tests

The dependency-free native tests exercise both capture modes, repeated four-input
alternation, mode changes during settling, discarded old conversions, settling
including timer wraparound, timeouts without phase advancement, and raw CSV metadata.
From the repository root on this Windows workstation with WSL:

```powershell
New-Item -ItemType Directory -Force tmp\loadcell-tests | Out-Null
wsl.exe --exec g++ -std=c++11 -Wall -Wextra -Werror -Iloadcell/include loadcell/test/test_hx711.cpp -o tmp/loadcell-tests/test_hx711
wsl.exe --exec tmp/loadcell-tests/test_hx711
```

These are software tests with a fake ADC, not a full ESP32 simulation or a
physical calibration test. Also run the PlatformIO firmware build.

## Reliability and limits

Acquisition runs independently of HTTP in a FreeRTOS task; plain-data samples
cross a bounded queue. No Arduino strings are shared between cores.
Missing HX711 conversions and unsynchronized time are reported and do not
produce fabricated rows. HTTP failures, including non-2xx responses, retain the
current batch for retry every five seconds. The 600-pair queue drops new
samples with a rate-limited serial warning when full. This is not durable storage:
power loss discards pending data, and an ambiguous HTTP failure can cause a
duplicate batch on retry. Only full batches are normally sent; file rotation
also flushes a partial batch.

Wi-Fi, the configuration page, and CSV transport are intended for a **trusted
isolated LAN**. The page has no authentication and HTTP is unencrypted.
There is no access-point provisioning mode, TLS, or on-device CSV storage.
Build success does not validate physical wiring, load calibration, Wi-Fi,
NTP, or end-to-end reception; these require a separate bench test.
