# Hardware Startup Investigation

Last updated: 2026-10-06

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
  (later raised again to 921600, see below):
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
