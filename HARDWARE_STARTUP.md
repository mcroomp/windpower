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
