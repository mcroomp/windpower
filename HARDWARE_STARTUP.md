# Hardware Startup Investigation

Last updated: 2026-10-27

This file is a chronological hardware-session evidence log for startup-related
swash, attitude, and anti-rotation-motor observations. It is not the owner for
current procedure.

Current owner documents:

- [design/arming.md](design/arming.md) — arm/disarm, safe-off, passive startup
- [design/calibration.md](design/calibration.md) — calibration commands and workflows
- [design/linkhub.md](design/linkhub.md) — LinkHub ownership, journal, and HTTP API
- [README.md#documentation-map](README.md#documentation-map) — documentation ownership map

## Current code-verified state

The statements in this section were rechecked against current code/config:
`calibrate\hw.py`, `calibrate\run.py`, `scripts\rawes.lua`,
`linkhub_client\src\linkhub_client\client.py`,
`tests\sitl\rawes_common_defaults.parm`, and
`hardware\rawes_hardware_defaults.parm`.

- LinkHub is the sole MAVLink owner. `calibrate` is an HTTP-only client.
- Canonical safe-off remains: disarmed, ACRO, `RAWES_MODE=0`,
  `H_FLYBAR_MODE=1`, `H_SV_MAN=0`, `SERVO9_FUNCTION=36`, `H_YAW_TRIM=0`,
  output 9 off, and neutral swash from Lua's disarmed mode-0 RC overrides.
- Passive startup uses the Lua-handled COMMAND_LONG route:
  - `ENTER_GUIDED` = `31010`
  - `ENTER_PASSIVE` = `31011`
  - passive offsets are `RAWES_ROFF`, `RAWES_POFF`, `RAWES_YOFF`
  - `ENTER_PASSIVE.param1` is the yaw-trim seed
- Current code keeps `SERVO9_FUNCTION=36` configured at boot; runtime remapping
  is not part of the supported handoff.
- Current calibration logging is CSV plus LinkHub journal cursor ranges; the
  LinkHub journal is the authoritative transport record.

## Current next diagnostic steps

These steps remain current because they match the owner docs and code:

1. Start the session with auto-detection: `python -m calibrate` or another
   command without `--connection` / `--baud`.
2. Confirm live state read-only before any armed action:
   `python -m calibrate status`, and if needed,
   `python -m calibrate watch attitude --duration 5`.
3. Diagnose startup behavior from the LinkHub journal, comparing at least
   `HEARTBEAT`, `ATTITUDE`, `ATTITUDE_TARGET`, `EXTENDED_SYS_STATE`,
   `STATUSTEXT`, `NAMED_VALUE_FLOAT`, and `SERVO_OUTPUT_RAW`.
4. Diagnose attitude with quaternion/target evidence rather than wrapped Euler
   yaw subtraction alone.
5. Before any armed retest, follow [design/arming.md](design/arming.md): use
   the current passive route, physically secure or disconnect actuators as
   required, and verify safe-off before and after the run.

## Current do-not-repeat rules

These remain current and safety-relevant:

- Do not run an armed passive startup on unsecured stationary hardware.
- Do not assume disarmed means centered swash or motor-off; verify the full
  safe-off invariant.
- Do not reconnect the yaw motor while armed or with accumulated trim.
- Do not change compass/EKF yaw-source parameters during a live armed test.
- Do not use MCP access and shell calibration commands on the same serial link
  concurrently.
- Do not assume `SCR_ENABLE` toggling alone proves a new Lua upload is running;
  reboot and verify live script status before arming.

## Dated historical evidence retained

Everything below is retained as historical evidence, not as current procedure.
These observations were not re-proven from code in this audit, but they remain
relevant to understanding why the current procedure exists.

### 2026-10-27 native-USB passive timeout investigation

- Initial no-argument calibration could not start because the release LinkHub
  binary was older than source. After rebuilding, no-argument discovery selected
  COM5 at 115200 baud. COM5/COM6 were positively identified as native Pixhawk
  USB, `VID:PID=3162:0053`, serial `160031001751343131363538`.
- Diagnostic journal run `a128e395-124c-4cc7-a9e7-1230c9335342` lives under
  `simulation/logs/linkhub/`. All tests used LinkHub's JSON HTTP API; the
  production Node CLI supplied passive orchestration rather than a second
  MAVLink connection.
- Fifty sequential passive-startup parameter reads passed in 67-176 ms.
  A further 100 reads across four concurrent callers passed in 141-478 ms,
  alongside 20 acknowledged `REQUEST_MESSAGE(AUTOPILOT_VERSION)` commands
  in 37-95 ms. The service reported one connection attempt, one clock epoch,
  no reconnect failures, and zero dropped frames.
- Ordinary passive startup was rejected first by `Hardware safety switch`,
  then by `Leaning`. These were explicit negative acknowledgements, not
  transport timeouts. With the operator's secured-hardware, disconnected-drive
  confirmation and separate force-arm authorization, one five-second
  `run passive --force` completed runup, `ENTER_GUIDED`, quiet qualification,
  `ENTER_PASSIVE`, hold, and cleanup. The hardware safety switch was released
  temporarily with the acknowledged `DO_SET_SAFETY_SWITCH_STATE` command and
  restored afterward; `BRD_SAFETY_DEFLT=1` was not changed. No Lua upload or
  reboot occurred.
- A complete 1078-parameter listing showed `ARMING_CHECK` absent and
  `ARMING_SKIPCHK` present. Reading the absent name deterministically timed
  out despite retries. This name was an investigation probe, not an observed
  request from the current passive startup.
- A controlled read-only contention test, sequence range
  `(411891, 417275]`, demonstrated collateral timeouts: an absent-name read
  with a five-second deadline occupied the protocol's shared parameter lock;
  a concurrent `RAWES_MODE` read with a three-second deadline returned
  `operation_timeout` without transmitting its own MAVLink request. After
  the blocker finished, fresh `RAWES_MODE` and `ARMING_SKIPCHK` reads
  succeeded in 116 ms and 98 ms. The journal contains five absent-name
  requests, followed only then by the successful reads. This proves a local
  queue-timeout mechanism, not the cause of the originally reported
  intermittent passive failure. Existing lock-deadline behavior was unchanged.
- Canonical cleanup and shared `verify_safe_off()` passed after every passive
  attempt. Final state was disarmed ACRO, mode 0, flybar 1, manual swash 0,
  output-9 function 36, yaw trim 0, and output 9 off; hardware safety was
  restored with an accepted acknowledgement.
- Next diagnostic step: capture the exact failing JSON request and concurrent
  parameter operations during an actual intermittent passive failure. Do not
  infer USB loss from the generic HTTP `operation_timeout`, or increase
  deadlines without identifying whether time was spent queued or awaiting a
  vehicle response.

#### Post-fix validation

- Removed the protocol client's global parameter transaction lock. Reads,
  writes, batches, and full listings can overlap without a protocol-imposed
  transaction order. Replies remain correlated by target and parameter name;
  writes additionally require the requested numeric value, while preserving
  ArduPilot's native reply type. Stale same-name values no longer falsely
  confirm writes. Retry intervals and operation deadlines were not increased.
  The current contract is owned by [design/linkhub.md](design/linkhub.md).
- The new unrelated-parameter regression failed on the original code and
  passed after the change. All 155 Rust workspace tests passed, including
  coverage of missing single/batch reads and writes, stalled listings,
  overlapping same-name reads, out-of-order same-name write replies, stale
  values, native type preservation, and unchanged deadlines. The release
  binary was rebuilt before hardware validation.
- Post-fix journal run `bc2f2b65-7bbc-4f51-836f-103f5051604b` is under
  `simulation/logs/linkhub/`. Five repetitions of the original contention
  reproducer all passed: `RAWES_MODE` succeeded in 32-117 ms while the absent
  name remained pending for its original five-second timeout. In the first
  repetition, sequence range `(30505, 35500]`, the healthy request was sent
  208 ms after the absent-name request and answered at 254 ms, before the
  absent-name retries at one-second intervals.
- A further 100 parameter reads across four concurrent callers all passed
  while another absent-name read was active: 71-100 ms, median 92 ms.
- After fresh operator confirmation that the mechanism remained secured with
  hazardous drive power disconnected, one additional five-second forced
  passive run completed runup, GUIDED entry, quiet qualification, passive
  capture, hold, and cleanup while six sequential absent-name probes were
  deliberately active. Only those six expected absent-name probes timed out;
  the production passive controller and its cleanup completed successfully.
- Post-fix service status still reported one connection attempt, one clock
  epoch, zero dropped frames, and no link error. Canonical safe-off was
  verified and the temporary safety release was restored with an accepted
  acknowledgement. `BRD_SAFETY_DEFLT` remained 1; no reboot or Lua upload was
  performed.
- This fixes the demonstrated collateral timeout and passes a passive retest
  under contention. It does not prove that every previously reported random
  timeout had this cause; retain the exact request and journal evidence if
  another intermittent failure occurs.

### 2026-10-22 disarmed cyclic neutral-pulse diagnosis

- Auto-detected through the existing LinkHub at `http://127.0.0.1:8999`,
  `tcp:192.168.9.143:5760`, target `1/1`. Initial state was disarmed ACRO,
  `RAWES_MODE=0`, S1/S2/S3 at 1517 us and output 9 at 1000 us.
- With the operator's explicit secured-hardware authorization, the production
  Swash panel held +roll at the configured limit, zero pitch, and 50%
  collective. No travel limits were changed and the vehicle was not armed.
  RC/servo telemetry was temporarily requested at 100 Hz, then restored to
  the observed baseline 10 Hz.
- Canonical journal run `aec19b50-cf44-4e99-834e-98a2f720529c`,
  sequence range `(1421080, 1430249]`, contains 758 RC and servo samples over
  8.053 seconds of settled hold. All 12 transmitted `RAWES_RLL` commands were
  1.0; their gaps remained below the 3-second watchdog.
- RC1 nevertheless alternated between 1900 us (692 samples) and neutral
  1500 us (66 samples). S1 alternated between 1428 and 1515 us, S2 between
  1515 and 1601 us, while S3 stayed at 1515 us. S1 had 122 neutral samples.
  The live `RC_OVERRIDE_TIME` was 3 seconds, not a short expiry.
- Source and an all-writes Lua regression reproduced the cause:
  `run_collective_setup()` wrote RC1/RC2 neutral first and the requested cyclic
  later in every valid tick. Checking only the mock's final channel values
  had hidden these intermediate writes. The local correction moves neutral
  writes into the failure branch; valid ticks write only their final target.
- Stop returned the vehicle to canonical safe-off, with S1/S2/S3 at 1517 us
  and output 9 at 1000 us. The corrected Lua and passive-control tests passed
  (58 tests), including the all-writes regressions that failed before the fix.
- Windows initially reported no serial ports. Later, LinkHub changed to
  `serial:COM4:115200`; COM4/COM5 were positively identified as native Pixhawk
  USB, `VID:PID=3162:0053`, serial `25001C001651333337363133`.
  Read-only listing showed `rawes.lua` at 85132 bytes and `keep_armed.lua` at
  1678 bytes; the corrected local script is 85215 bytes. The operator chose
  manual deployment and subsequently reported the update completed; the agent
  did not upload or reboot. A post-deployment repeat of the same hold remains
  necessary; do not infer hardware verification from the local regression.

### 2026-10-22 script-upload timeout over network bridge

- No-argument calibration connected through the existing LinkHub at
  `tcp:192.168.9.143:5760`, target `1/1`. Windows reported no serial ports.
  The DroneBridge API reported firmware 2.4.1, build 23, and UART 1500000 baud.
  Upload-path policy is owned by [calibration.md](design/calibration.md).
- Node.js queries against the running LinkHub and its canonical journal
  showed the failed uploads stopping at `CreateDirectory /APM/scripts`,
  before any upload job existed. Each operation sent four requests without
  receiving a matching reply.
- Read-only `ListDirectory` probes with source components 190 and 0 also
  received no replies. The transmitted MAVFTP frame matched the standard
  CRC extra (84); a full-length, untruncated MAVLink 2 listing request did
  not recover responses either. Ordinary parameter reads and the vehicle's
  advertised MAVFTP capability worked.
- With the operator's confirmation that the hardware was secured, calibration
  first verified safe-off, then a LinkHub command rebooted the Pixhawk.
  Reconnection was confirmed by a new vehicle clock epoch. The post-reboot
  directory listing still timed out. One delayed NAK for an earlier directory
  creation was observed; this was not a successful listing or upload.
- DroneBridge's GCS-facing `RADIO_STATUS.txbuf` was consistently zero.
  ArduPilot's FTP reply path checks radio buffer availability, but these
  observations do not prove that the flight controller received those same
  reports. Do not treat this as an established cause or inject fabricated
  buffer reports as a fix.
- No script was uploaded or replaced. Final `calibrate disarm` verified the
  full canonical safe-off invariant, including output 9 at 1000 us.
  Next isolation step: compare a directory listing over verified native USB
  with the network path before changing upload deadlines or transfer logic.

#### Second-controller USB comparison

- The operator connected a second Pixhawk for testing. Native USB enumerated
  as COM7/COM8, `VID:PID=3162:0053`, serial `160031001751343131363538`.
  Heartbeat-based discovery selected COM7 at 115200 baud. An isolated instance
  of the same release LinkHub binary served port 9000; the original TCP
  instance on port 8999 was left untouched.
- Node.js calls to the isolated Rust HTTP API successfully listed
  `/APM/scripts` (`rawes.lua`, 80512 bytes) and created the already-existing
  scripts directory (`created=false`, the expected successful result).
- A disposable 98304-byte non-Lua file under `/APM` uploaded successfully in
  15.187 seconds, including the upload engine's remote CRC verification.
  All 415 requests completed with zero retries or timeouts; a directory
  listing confirmed the exact file size. The temporary remote file and
  transfer record were removed afterwards.
- The installed Lua script was not changed, no parameters or modes were
  written, and the test controller remained disarmed in ACRO. The temporary
  USB LinkHub was stopped after the check.
- This validates the current Rust directory and upload implementation on
  native USB. Because the USB test used a second controller, it does not yet
  distinguish a network-bridge fault from an FTP problem on the original
  controller. A same-controller USB/TCP comparison is still needed.

#### Same-controller USB/DroneBridge comparison

- The operator then connected the original controller simultaneously over
  native USB and DroneBridge. USB enumerated as COM4/COM5,
  `VID:PID=3162:0053`, serial `25001C001651333337363133`; heartbeat discovery
  selected COM4 at 115200 baud. Its first USB directory listing also timed out,
  although USB parameter reads worked.
- At the operator's request, canonical safe-off was verified and the controller
  rebooted over USB. Following reconnection, a USB listing succeeded:
  `rawes.lua` was 80595 bytes and `keep_armed.lua` was 1678 bytes.
- A fresh isolated LinkHub connection to DroneBridge then timed out on the
  same directory listing. After 85 focused Lua/control tests passed through
  the canonical pytest CLI, the original calibration upload command was
  retried over verified USB. It again failed before an upload job started;
  both services' transfer registries were empty.
- This sequence shows that USB FTP can recover after reboot and later become
  unresponsive after a network-side FTP attempt. It is consistent with a
  shared flight-controller FTP worker being blocked, but does not establish
  the precise blocking condition. No Lua file was replaced.
- Post-reboot cleanup verified the full safe-off invariant, including output
  9 at 1000 us; final USB and TCP observations both showed disarmed ACRO.
  Temporary diagnostic LinkHub instances were stopped. The remaining physical
  isolation test is USB with DroneBridge disconnected from the controller.

#### Collective watchdog and cleanup parameter timeouts

- A later disarmed collective run reported a MAVLink timeout and failed to
  confirm `RAWES_MODE=0` during cleanup. The active service was connected
  through DroneBridge. Live Lua status subsequently reported mode 0, disarmed.
- Canonical journal evidence in run
  `37727958-4592-42c5-a5f9-d259ca543ebc` showed a single `H_YAW_TRIM` read
  at sequence 145445 with no matching reply. A subsequent read at 153618
  received the zero-valued reply at 153670. The Rust single-parameter read
  and write transactions had no retransmission, unlike command transactions.
- A complete subsequent `calibrate disarm` verified the canonical safe-off
  invariant, including `RAWES_MODE=0`, `H_YAW_TRIM=0`, and output 9 at 1000 us.
  No armed or collective motion test was started during this diagnosis.
- This parameter-loss finding is separate from the earlier MAVFTP
  `FailErrno` code 2 with errno 0; an unspecified filesystem error must not
  be treated as proof that a directory already exists.
- Single-parameter read/write retries were added in the protocol crate, sharing
  the original operation deadline. Three regression tests first reproduced the
  missing retries; all 71 protocol unit tests then passed with the fix.
  The release binary was rebuilt and, with operator approval, LinkHub restarted
  on the same TCP connection. Its readiness check and full canonical safe-off
  verification passed. A further 24 read-only checks of `RAWES_MODE` and
  `H_YAW_TRIM` all returned zero over TCP. No collective or armed retest was
  performed. The fixed service was subsequently stopped at the operator's
  request, and its process exit was confirmed.

### 2026-10-15 direct-USB Lua deployment and observable uploads

- Auto-detection selected `COM5` at 115200 through LinkHub. Windows and
  LinkHub both reported the Pixhawk 6C identity `VID:PID=3162:0053`, serial
  `160031001751343131363538`; the vehicle was disarmed in ACRO.
- The first 80,512-byte `rawes.lua` upload hit the old synchronous 60-second
  HTTP/MAVFTP deadline and left a zero-byte remote file. LinkHub uploads were
  moved to background `ftp_upload` transfer jobs with progress, retry/loss
  statistics, cancellation, a progress-resetting stall deadline, and remote
  CRC verification. The Python client now starts and polls that job.
- After rebuilding and restarting LinkHub, the standard calibration upload
  completed and `/APM/scripts/rawes.lua` listed at 80,512 bytes. The scripting
  engine was restarted, the Pixhawk was rebooted, and no-argument auto-detection
  reselected the same direct-USB interface.
- Final cleanup verified canonical safe-off: disarmed, ACRO, `RAWES_MODE=0`,
  `SERVO9_FUNCTION=36`, `H_YAW_TRIM=0`, `H_FLYBAR_MODE=1`, `H_SV_MAN=0`, and
  output 9 off. Live `DEBUG_FLOAT_ARRAY` traffic after reboot confirmed the Lua
  script was running.

### 2026-10-07 DroneBridge ESP32-C6 Wi-Fi telemetry link

Plugged in an official DroneBridge for ESP32 board (HW v1.2, ESP32-C6, firmware
v2.4.1, USB `VID:PID=303A:1001`, COM7, MAC `10:BD:A3:90:B4:9C`). It was already
wired to the Pixhawk telemetry port: its settings showed MAVLink protocol, UART
57600, GPIO TX 21 / RX 2 / RTS 22 / CTS 23, and its stats counted decoded
MAVLink messages from the autopilot. The Pixhawk side read
`SERIAL1_PROTOCOL=2, SERIAL1_BAUD=57` and the same for `SERIAL2`.

- The factory access point is `192.168.2.1`, which collides with the bench LAN
  (`192.168.2.0/24`, router `192.168.2.1`). The board's Gateway IP was changed to
  `192.168.4.1` once, from the web UI over a phone. The SSID stays
  `DroneBridge for ESP32` (default password `dronebridge`).
- The PC joins the access point with a manual-connect Wi-Fi profile; the ESP's
  web UI/API is `http://192.168.4.1` and its MAVLink is TCP 5760 / UDP 14550.
  LinkHub connects with `--connection tcp:192.168.4.1:5760` (serial discovery does
  not apply). Windows needs Location services enabled for desktop apps before
  `netsh wlan` can scan.
- The ESP injects its own HEARTBEAT (component 68, `MAV_TYPE_ONBOARD_CONTROLLER`,
  autopilot INVALID) and RADIO_STATUS. LinkHub used to let any non-GCS heartbeat
  retarget the link and overwrite mode/armed state, so status flipped to
  `1/68`, STABILIZE and ACTIVE. Fixed: only autopilot heartbeats update link
  status, and the Python client and UI leave non-autopilot heartbeats out of
  vehicle state.
- Read-only verification over Wi-Fi: status ready with target `1/1`, ACRO,
  disarmed; capabilities, `MAV_SYSID` and `SERIAL*` parameter reads; about
  38 kbit/s received with no dropped frames. Station RSSI was weak (-89 dBm), so
  expect range limits; no arming, parameter write or Lua upload was done over
  this link.
- The ESP is on the Pixhawk's `SERIAL2` (TELEM2). At 57600 baud the UART was
  nearly saturated by the ~38 kbit/s stream, so both ends were raised to 460800
  (later raised again to 921600 and then 1500000, see below):
  `SERIAL2_BAUD=460` on the Pixhawk and `baud=460800` in the ESP settings
  (`POST /api/settings` with the form's fields, which saves and reboots the
  board). Gotchas: ArduPilot applies a `SERIALn_BAUD` change only after a reboot
  (the write is accepted and the old speed keeps running), and the ESP is
  powered from the Pixhawk telemetry connector, so rebooting the Pixhawk also
  reboots the ESP and drops its Wi-Fi; rejoin with `netsh wlan connect`.
  Order used: write the Pixhawk parameter, reboot the Pixhawk through LinkHub,
  then set the ESP baud over Wi-Fi (the web UI works independently of the UART,
  so the link is always recoverable). After this the link recovered with no
  dropped frames, 1078 parameters downloaded in about 7 s, and the vehicle was
  disarmed in ACRO with `RAWES_MODE=0`, `H_FLYBAR_MODE=1`, `H_SV_MAN=0` and
  `SERVO9_FUNCTION=36`. `SERIAL1_BAUD` stayed 57.
- File and log downloads over this link (460800 baud, RSSI about -90 dBm).
  First attempt, with the old lock-step MAVFTP read and whole-suffix DataFlash
  re-request: log 17 (1,495,040 B) took 155 s over DataFlash with about 37%
  duplicate packets, and MAVFTP ran at about 2.4 KB/s and then failed with
  `NAK Fail` because the previous download never sent `TerminateSession` and
  ArduPilot refuses to open a new file while a session was active in the last
  3 s (`GCS_FTP.cpp`). Under load about 1.4% of autopilot frames were lost, in
  bursts, and the ESP's own frames about 7%. The ESP also injects its own
  heartbeat (component 68, autopilot INVALID), which LinkHub now ignores for
  target, mode and armed state.
- After moving downloads to transfer jobs (`BurstReadFile`, pipelined hole
  repair, selective DataFlash re-request, `TerminateSession` always sent; see
  [linkhub/README.md](linkhub/README.md#transfers)): MAVFTP 1,495,040 B in
  39 s (about 38 KB/s, close to the 46 KB/s wire limit) with CRC verified; the
  DataFlash download of the same log in 44 s (34 KB/s); the two files were
  byte-identical (SHA-256). Cancelling a transfer mid-way and starting a new
  one immediately works. With FTP and DataFlash running at once both files
  were still identical (FTP 41 s, log 82 s, i.e. about 36 KB/s aggregate) and
  the engines repaired 5 and 16 gaps with 0 duplicate packets. The vehicle
  stayed disarmed.
- Wire rate at 460800: while downloading, LinkHub's `received_bytes` averaged
  about 43.4 KB/s against the 46.08 KB/s UART ceiling (460800 / 10 bits), so
  that link was UART-bound. Idle telemetry was about 5 KB/s.
- Raised to **921600** (`SERIAL2_BAUD=921`, ESP `baud=921600`, same order as
  above; the ESP's `POST /api/settings` takes the full settings JSON from
  `GET /api/settings` with `baud` changed). Link recovered with 0 dropped
  frames, vehicle disarmed in ACRO, safe-off parameters unchanged. Same log 17,
  byte-identical every time: MAVFTP 25 s (59.5 KB/s, was 38), DataFlash 27 s
  (56.1 KB/s, was 34), both at once FTP 29 s and log 59 s (was 41 s and 82 s;
  13-17 timeouts each and 14-49 gaps, all repaired, 0 duplicates). Wire rate
  averaged about 71-73 KB/s, only about 78% of the new 92 KB/s UART ceiling, so
  the UART is no longer the bottleneck (Wi-Fi at about -90 dBm and the ESP are
  the likely limit). Not tried: 1500000 baud, or RTS/CTS flow control (the ESP
  settings expose `gpio_rts`/`gpio_cts`; the Pixhawk side is `BRD_SER2_RTSCTS`,
  currently 2 = auto).
  I expected ArduPilot's one-`LOG_DATA`-per-400-Hz-tick limit to cap DataFlash
  near 36 KB/s on a link without flow control, but it reached 56 KB/s, so that
  limit does not apply to this build or link.
- Raised again to **1500000** (`SERIAL2_BAUD=1500`, ESP `baud=1500000`), same
  procedure; the ESP took about 40 s to rejoin Wi-Fi after its reboot. Link
  recovered with 0 dropped frames and the vehicle stayed disarmed in ACRO with
  safe-off parameters unchanged. Same log 17, byte-identical every time: MAVFTP
  17 s (85.8 KB/s), DataFlash 23 s (64.7 KB/s), both at once FTP 22 s and log
  38 s (the log needed 137 gap repairs; 0 duplicates). MAVFTP wire rate averaged
  about 109 KB/s of the 150 KB/s UART ceiling (73%). Speed by baud for the
  same log, MAVFTP / DataFlash alone: 460800 38 / 34 KB/s, 921600 59.5 /
  56 KB/s, 1500000 85.8 / 64.7 KB/s. DataFlash scales less and loses more
  frames under contention at 1.5M. Left at 1500000. To revert: set the ESP
  baud first or last as above; the Pixhawk side is `SERIAL2_BAUD` (921 or 460).
- High telemetry rates at 1500000 baud (`PUT /v1/mavlink/message-rates`, measured
  from the journal's frame sequence numbers, the sender's `time_boot_ms` and
  LinkHub's arrival time). Starting streams are about 25 Hz for `ATTITUDE`,
  `ATTITUDE_QUATERNION`, `ATTITUDE_TARGET` and `SERVO_OUTPUT_RAW` (4.9 KB/s on
  the wire). Stepping the rates up: 50 Hz, then 3 messages at 100 Hz (15 KB/s),
  then up to 200 Hz (33 KB/s), then up to 300 Hz for 30 s (64 KB/s, 43% of the
  UART): 0 of 45,940 autopilot frames missing, 0 dropped frames, and the
  autopilot emitted at a steady interval (no gaps). The vehicle rejects a
  400 Hz request (`MAV_RESULT_DENIED`) and a 300 Hz request is delivered as
  333 Hz because the interval rounds to 3 ms. One earlier 200 Hz run lost
  0.7% in a single stall of about 150 ms; two later runs at the same rate lost
  nothing. Arrival at LinkHub is bursty (the ESP batches): per-message delivery
  delay above the best case was p99 about 32 ms at 333 Hz, 57 ms at 200 Hz and
  140 ms at the 25 Hz default, so use `time_boot_ms`, not arrival time, for
  timing. With a DataFlash download running at the same time the autopilot
  itself cut the 300 Hz streams to about 56 Hz (hundreds of missed emissions,
  gaps up to about 300 ms, 0.6% frames lost) while `LOG_DATA` took about
  97 KB/s, so high-rate telemetry is only smooth when no bulk transfer is
  running; the UART was at about 69% then, so this is the autopilot's own
  scheduling, not the wire.
- Gotcha: every `MAV2_*` stream parameter is 0 on this vehicle (4.7.1 names
  them `MAVn_*`, not `SRn_*`), so the starting streams are requested at
  runtime by something other than LinkHub and are not restored by a `null`
  (vehicle-default) rate; `null` turned the streams off until the next
  reboot. Restore them with explicit rates, or reboot.
- Yaw rate-PID telemetry (`PID_TUNING`, axis `PID_TUNING_YAW`): ArduPilot only
  sends it for axes enabled in `GCS_PID_MASK` (bit 2 = yaw, value 4; it was 0),
  and then it must be requested with `PUT /v1/mavlink/message-rates`
  (`{"PID_TUNING": 50}`). It arrived at the requested rate (50 Hz: 502 frames in
  10 s, 200 Hz: 1994) for about 2.3 KB/s per 50 Hz of extra wire, with fields
  `desired`, `achieved`, `FF`, `P`, `I`, `D`, `SRate`, `PDmod`. Disarmed on the
  bench `desired` held at about 0.945 rad/s (a heading error), `achieved` was
  gyro noise around 0 (+/-0.004), `P` equalled `ATC_RAT_YAW_P` (0.18) times the
  error, and `FF`, `I` and `D` were 0, so this only proves the path; the yaw
  loop needs an armed run with the motor to be informative. The mask was
  restored to 0 and the message turned off (`0` Hz) afterwards.
- Yaw PID telemetry is now a default: `hardware/rawes_hardware_defaults.parm`
  sets `GCS_PID_MASK` to 4 (yaw only) and calibrate's shared observation loop
  (`watch`, `run`) requests `PID_TUNING` at 4 Hz. `GCS_PID_MASK` was written to 4
  on the vehicle (it persists; `config fix` would also write it). Check:
  `calibrate watch attitude --duration 12` produced 54 `PID_TUNING` frames in
  13.3 s (3.99 Hz), axis yaw only. The 4 Hz request is a runtime interval, so it
  lasts until the Pixhawk reboots and is re-sent by the next calibrate
  observation; it is not restored at the end of a run.

### 2026-10-06 Typed-MAVLink LinkHub verification on bench hardware

Pixhawk native USB (`VID:PID=3162:0053`, serial `160031001751343131363538`,
COM5/COM6), ArduCopter 4.7.1, with the swash servos and the output-9 yaw motor
disconnected and no battery connected (so arming needs `--force`). The running
LinkHub service was replaced after flushing its journal: stopped, release binary
rebuilt, restarted with the same arguments (`serve --connection auto --baud 115200
--port 8999`). The service now speaks the typed-enum JSON contract
([design/linkhub.md](design/linkhub.md#mavlink-value-representation)).

Read-only checks passed: typed link and component status, capabilities, the full
1,078-parameter list, MAVFTP listing of `/APM/scripts`, `calibrate status`, and
`linkhub query` (`armed`, `statustext`, `param`, `show --json`) on the recorded
journal. No frames were dropped.

`run passive --duration 8` without `--force` was refused by ArduPilot:
`Arm: Hardware safety switch` and `Arm: Compass not calibrated`
(`BRD_SAFETY_DEFLT=1`); the cleanup left the vehicle disarmed in ACRO with the full
canonical safe-off state verified. `run passive --duration 8 --force` then passed:
arm accepted, `Runup Complete`, `ENTER_GUIDED` and `ENTER_PASSIVE` (`MAV_CMD_USER_1`
and `USER_2`) accepted, 200 observation rows, disarm accepted, and safe-off
verified again (`H_YAW_TRIM` returned to 0).

Findings: `SYSID_THISMAV` does not exist on 4.7.1 (the parameter is `MAV_SYSID`),
so it is not a valid "parameters ready" probe. `calibrate status` missed the
1 Hz HEARTBEAT on a busy link because `calibrate.messages.read_one` treated
LinkHub's early empty long-poll as a timeout; it now polls until its deadline.
The service was later redeployed with the dialect generated from ArduPilot's own
Copter-4.7.1 definitions; the read-only checks were repeated on that build, the
armed run was not.

### 2026-10-06 ArduCopter 4.7.1 passive-run verification

The flight controller was updated from ArduCopter 4.7.0 beta
(`97775f82`) to official ArduCopter 4.7.1 (`dbe79216`). Live
`AUTOPILOT_VERSION` reported `flight_sw_version=0x040701FF`.

With actuators unplugged, the shared TypeScript CLI successfully completed:

```text
.\rawes run passive --duration 8
```

The LinkHub journal from cursor `v1:332692` showed normal arm acceptance,
channel-8 interlock assertion, ArduPilot `Runup Complete`, accepted
`ENTER_GUIDED` and `ENTER_PASSIVE` commands, repeated `passive_hold` commands
for the bounded run, then canonical cleanup. Final live state was disarmed
ACRO with `RAWES_MODE=0`, neutral swash outputs, `SERVO9_FUNCTION=36`, and
output 9 at 1000 us.

The preceding 4.7.0-beta attempt armed after the Lua channel-8 low-override fix
but never emitted `Runup Complete`, so the required passive-start gate timed
out. The same gate succeeds on the documented 4.7.1 hardware firmware.

### 2026-10-06 LinkHub telemetry-radio discovery verification

Connected the FTDI telemetry radio on COM6 (`VID:PID=0403:6015`, serial
`D30IKLCCA`) and started the rebuilt LinkHub service on port 8999 without a
baud restriction. LinkHub opened COM6 once, rejected 115200 after the expected
read timeout, changed the same open handle to 57600, and received a heartbeat.
Live status reported `serial:COM6:57600`, `ready=true`, no current error, and
zero consecutive failures. A three-second health sample received 336 MAVLink
messages and 13,707 bytes.

The vehicle was observed disarmed in ACRO. This was a read-only,
ground-service verification: no arming command, parameter write, or Lua
deployment was performed. Lua was not uploaded through the FTDI/telemetry
radio, and the complete canonical safe-off parameter set was not reverified
over this lower-bandwidth connection.

### 2026-10-06 Lua deployment through LinkHub

Ran the focused Lua unit surface before deployment: 63 tests passed. Attached
the calibration client to the existing LinkHub service on port 8999. LinkHub
auto-discovery selected `serial:COM4:115200`; both LinkHub and
`serial.tools.list_ports` identified it as native Pixhawk USB
(`VID:PID=3162:0053`, serial `25001C001651333337363133`, location
`1-5:x.2`). The paired COM5 interface reported the same board identity.

Uploaded `scripts/rawes.lua` through LinkHub MAVFTP and verified the remote
`/APM/scripts/rawes.lua` size as 78,806 bytes. Rebooted the flight controller
through LinkHub and allowed automatic reconnection. Post-reboot status showed
the vehicle disarmed in ACRO with `RAWES_MODE=0`, `SERVO9_FUNCTION=36`, and
output 9 at 1000 us. The common disarm cleanup then verified the complete
canonical safe-off state: disarmed, ACRO, `RAWES_MODE=0`, `H_FLYBAR_MODE=1`,
`H_SV_MAN=0`, `SERVO9_FUNCTION=36`, `H_YAW_TRIM=0`, and output 9 at 1000 us.

### 2026-10-06 LinkHub throughput deployment record (preserved factual content)

Rebuilt the LinkHub release binary and restarted only the ground-side service
with its existing arguments: `serve --connection auto --baud 115200 --port 8999
--data-dir E:\repos\windpower\simulation\logs\linkhub --static-dir
E:\repos\windpower\linkhub-ui\dist --no-cache`. Flushed the old journal through
`POST /v1/journal/flush` before stopping its process.

Read-only status checks before and after restart reported `base_mode=81`
(disarmed). Automatic discovery reacquired `serial:COM5:115200`; the new
status exposed approximately 41.9 kbps RX and 168 bps TX. No vehicle reboot,
arming command, parameter write, or Lua deployment was performed. Full
canonical safe-off was not reverified during this service-only operation.

### 2026-10-05 — ground-owned passive handoff

Historical evidence retained from the 2026-10-05 bench validation showed that
current ground-owned passive startup removed the large GUIDED-entry swash step.
In the retained run evidence, the vehicle reached runup complete, cleared
landed state in ACRO, entered GUIDED_NOGPS, passed the quiet-rate gate, and
acknowledged passive hold. The first samples showed only very small swash and
quaternion error, while later spread increased gradually rather than as a step.

This evidence is why [design/arming.md](design/arming.md) now treats the
current passive route as the canonical startup path.

### 2026-10-05 — runtime yaw-output remap failure

Historical evidence retained from the same investigation showed that passive
hold could generate nonzero yaw-controller demand while
`SERVO_OUTPUT_RAW.servo9_raw` stayed at zero after a runtime
`SERVO9_FUNCTION=36` write. The retained source-level conclusion was that the
parameter write changed stored configuration but did not rebuild Copter's live
function-to-channel map.

This evidence is why current code and docs keep `SERVO9_FUNCTION=36` configured
at boot instead of transferring motor ownership during the handoff.

### 2026-10-05 — disconnected yaw-actuator observations

Historical disconnected-actuator runs showed that:

- the internal Motor4/DDFP command path could respond to yaw motion even when a
  disconnected output did not show a useful physical waveform on output 9;
- a stationary body could still accumulate yaw trim when held-heading error
  remained large;
- the stationary LinkHub UI bench route needed an explicit zero yaw-trim seed
  before passive capture.

These observations are the historical basis for the current rule in
[design/arming.md](design/arming.md): the stationary LinkHub UI bench route
must send `ENTER_PASSIVE.param1 = 0`.

### 2026-10-05 — orientation-dependent GUIDED entry transient

Historical evidence retained from a stationary run near a nose-down attitude
showed that a ground-side `DO_SET_MODE` transition could create a transient
GUIDED level-target window before the first ground target arrived. The retained
telemetry showed `ATTITUDE_TARGET` moving toward level and a large swash step
while the body itself stayed still.

This is the historical basis for the current Lua-handled `ENTER_GUIDED`
transition: current procedure does not use ground-side `DO_SET_MODE` for the
passive handoff.
