# Load-cell firmware agent guide

## Purpose and scope

This directory is an independent PlatformIO project for the CSV-generating ESP32
load-cell sensor. It is not part of the root Python distribution or the LinkHub
Rust workspace. It extracts a two-HX711 sensor application without servo, pitch,
light-barrier, or InfluxDB code. Browser OTA is retained.
Do not reintroduce actuator control.

## Structure

- `platformio.ini`: pinned ESP32 Arduino platform, LOLIN D32 Pro board, HX711
  dependency, and serial-monitor settings. No automatic upload targets.
- `include/config.h`: GPIO assignments, timing/buffer limits, and build defaults.
- `include/hx711_input.h`: two/four-input capture scheduler, input labels/gains,
  old-conversion discard, and 400 ms channel/gain settling.
- `include/sample_csv.h`: plain sample record and ordered raw CSV serialization.
- `test/test_hx711.cpp`: native software tests of switching and CSV behavior.
- `include/local_config.h.example`: template for the ignored local Wi-Fi/CSV
  configuration. Never put real credentials in tracked files.
- `launch_config.cmd.example`: template for ignored Windows launcher addresses.
  Keep local addresses, device identities, and personal paths out of tracked files.
- `src/main.cpp`: setup, HX711 acquisition task, bounded sample queue, CSV
  serialization/batching/HTTP retry, NTP timestamps, NVS configuration, and
  the HTTP configuration page.
- `include/ota.h`, `src/ota.cpp`: password-protected browser firmware upload on
  port 80. Empty local OTA password disables this service. Failed or aborted
  uploads must not reboot or return success.
- `README.md`: build, wiring, protocol, configuration, and runtime limitations.
- `backup_esp32.py`: independent uv inline-dependency utility for USB full-flash
  backup, size checking, and device verification. Explicit USB port required;
  its only permitted device commands are flash-id, read-flash, and verify-flash.
  Dumps/logs/manifests in ignored `backups/` may contain credentials; never upload
  them, commit them, or treat partial reads as verified backups.

## Execution and invariants

The sampling task on core 1 owns both HX711 objects. It reads signed raw values
and timestamps samples, then passes plain-data records through a FreeRTOS queue.
One additional single-element overwrite queue exposes the latest paired raw
reading independently of CSV delivery and NTP.
Arduino `loop()` owns web requests, preferences writes, CSV strings, filenames,
and HTTP uploads. Atomic sample interval and requested/active capture modes
cross this ownership boundary besides the queues. Do not share mutable Arduino
strings between tasks. Web handlers must not access the HX711 objects.

Keep the raw CSV columns in `sample_csv.h` and HTTP `filename` query parameter
in step with `linkhub/csv-map.json`. The receiver owns all calibration and
post-processing; do not convert counts to force on the ESP32.
Preserve per-sample input/gain tags across queued mode changes.
`/set_hx711?mode=2|4` persists a mode request and returns HTTP 202;
`/hx711` distinguishes requested, active, and last sample modes.
Two-input mode emits A/B at gain 128; four-input mode alternates A1/B1 at
gain 64 and A2/B2 at gain 32, settling on every change.
Each paired record produces two CSV rows with a shared pair-completion time
and independent `input,raw,gain` fields. The six-column header works in both
modes. Keep the 300-row batch size even so pairs are never split.
The configuration helper in `scripts/configure_esp_csv.ps1` depends on the
`/create_new_csv?url=...` endpoint and the `CSVStorageURL` input on `/`.
It preserves an unchanged CSV destination and explicitly requests four-input
capture before the ESP LinkHub launcher starts the receiver.
`/setSamplingRate` takes an interval in milliseconds, not a frequency.
NVS keys are `csv_url`, `interval_ms`, and `capture_mode` under
`rawes-loadcell`.

Do not silently substitute readings on timeouts, claim upload success for
non-2xx responses, weaken validation, or assume the inherited calibration fits
new hardware. Runtime limitations and bench checks are documented in the README.

## Validation

Run `pio run --project-dir loadcell` from the repository root for a firmware build.
If PlatformIO is missing, install using the Microsoft package feed command in
the README. Keep build artifacts in the ignored `.pio` directory.
Do not flash, erase NVS, or interact with physical hardware unless requested.
When changing CSV or hardware behavior, update the README and test that behavior
on the bench when authorized; a compiler check alone is not hardware validation.
