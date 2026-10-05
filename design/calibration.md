# calibrate — Hardware Calibration Tool

`calibrate` is an HTTP-only client of LinkHub. LinkHub owns the
Pixhawk serial/UDP connection, MAVLink parsing and transmission, persistent
telemetry archive, stateful MAVLink transactions, and optional Bluetooth motor
connection. `calibrate` provides servo control, motor testing, ESC diagnostics,
arming, DataFlash download, and Lua script management without importing
`pymavlink` or `bleak`. The calibration launcher uses `pyserial` only to list
candidate ports when starting LinkHub automatically.

Lua script deployment remains best over a direct Pixhawk USB connection because
MAVFTP over a SiK radio is too slow for reliable operational updates.

## Connection

```bash
python -m calibrate
python -m calibrate <verb> [args]
```

When the default local endpoint is not running, calibration starts the release
LinkHub binary, scans serial ports and standard baud rates, and stops that child
service on exit. Use `--connection COM7 --baud 57600` to select a known link, or
`--server` to use an already-running LinkHub.

The default environment is intentionally limited to the hardware/calibration stack,
so `uv run python -m calibrate --help` starts quickly. Install the optional simulation
environment only when needed:

```bash
uv sync --dev --extra simulation
```

The workstation uses `UV_NO_SYNC=1`, so `uv run` does not re-check or modify the
environment on every command. Run `uv sync` explicitly when dependencies change.

LinkHub reports ready after the first heartbeat. Check
`GET /v1/mavlink/status` for the selected connection, target IDs, counters, and
the current journal cursor.

`--force` / `-f` skips interactive confirmation prompts (safe for scripted use at low
throttle).

---

## Output channel mapping

| Output | Component | SERVO_FUNCTION |
|--------|-----------|---------------|
| 1 | S1 — right-rear swashplate servo | 33 (Motor1) |
| 2 | S2 — left-rear swashplate servo | 34 (Motor2) |
| 3 | S3 — front/elevator swashplate servo | 35 (Motor3) |
| 9 | GB4008 anti-rotation motor (AUX 1) | 36 (Motor4) |

The physical layout is HR3-120. ArduPilot has no separate HR3 mixer selection:
`H_SW_TYPE=3` selects H3-120, while `SERVO1/2/3_REVERSED=1` and
`H_SW_COL_DIR=1` implement the front-elevator arrangement. The Pixhawk arrow
and vehicle +X both point toward the centre of gravity (`AHRS_ORIENTATION=0`).
The physical-airframe overrides are in
[`hardware/rawes_hardware_defaults.parm`](../hardware/rawes_hardware_defaults.parm).

Swashplate PWM range: 1000 µs (min) … 1500 µs (neutral) … 2000 µs (max). The heli mixer
hard-codes this range on the swash servos — `SERVOn_MIN/MAX` writes are silently
overwritten on every output tick. Use `swash range <min> <max>` (which writes
`H_COL_MIN/H_COL_MAX`) to limit physical swash travel.

The measured RAWES final-output limits are 1257–1777 µs. The fitted configuration
uses `SERVO1/2/3_TRIM=1517`, `H_COL_MIN=1342`, `H_COL_MAX=1657`, and
`H_CYC_MAX=1000` (10°). A complete native oscillation measured 1260–1774 µs,
including the configured 3 µs guard band.

Motor PWM range: 1000 µs (off) … 2000 µs (full throttle). `SERVO9_MIN/MAX` is the
limiter you want here.

---

## CLI shape

```
python -m calibrate [--server URL] [--force] <verb> [args...]
```

Two long-running verbs (`run`, `watch`) handle anything time-bounded and always log
to `simulation/logs/calibrate/<verb>_<name>_YYYYMMDD_HHMMSS.csv` (gitignored). The
rest are one-shot.

---

## Long-running verbs

### `run <name> [--duration N] [--trim K=V,...] [--force]`

Activate a Lua mode (via `RAWES_MODE`) → arm via MAVLink → stream observation rows
to console + CSV → safety shutdown on exit. ESC or Ctrl-C aborts cleanly. Without
`--duration`, the session is unbounded; abort with ESC/Ctrl-C.
Normal ArduPilot pre-arm checks apply by default. `--force` explicitly bypasses
them for secured, disconnected bench hardware; shutdown remains unchanged and
always restores disarmed safe-off state.

Every safe-off path leaves `SERVO9_FUNCTION=0`, so the DShot yaw-motor output
is unassigned after arm/disarm tests and every run exit. A later passive run
keeps it unassigned during arming and restores Motor4/DDFP ownership only after
passive target capture, when yaw control actually begins.

**Modes (`<name>`):**

| Name | `RAWES_MODE` | Uses yaw motor output? |
|---|---|---|
| `passive` | 3 | yes — `run_yaw_trim` observer sets H_YAW_TRIM each tick |
| `steady` | 1 | observer active |
| `landing` | 4 | no |

`--trim` keys, applies to all modes:

| Key | NVF sent | Meaning |
|---|---|---|
| `tlon` | `RAWES_TLN` | cyclic trim longitudinal |
| `tlat` | `RAWES_TLT` | cyclic trim lateral |
| `thr`  | `RAWES_THR` | IC thrust [0..1] (passive only) |

`run` uses current FC parameters as-is and does not apply per-run parameter
overrides. Yaw is regulated by the servo-readback trim observer in rawes.lua —
calibrate `RAWES_YAW_SLP` (slope) from a bench measurement.

```bash
# Bench check: hold IC swashplate, observer active, 30 s
python -m calibrate --server http://127.0.0.1:8999 run passive --duration 30 --force --trim tlon=0.02,thr=0.342

# Steady-flight bench, unbounded (ESC to stop)
python -m calibrate --server http://127.0.0.1:8999 run steady
```

To use the external Bluetooth motor that drives the rotor on the test stand,
add `--rotor-motor`. The command scans for a device whose name starts with
`BLDC`, connects over the Nordic UART Service, and leaves the motor stopped:

```bash
python -m calibrate --server http://127.0.0.1:8999 run passive --rotor-motor \
  --rotor-speed 10 --rotor-direction cw
```

During the passive run, `m` starts or stops the external rotor motor, `[` and
`]` decrease or increase its speed by 5 percentage points, and `0` sends an
emergency stop. Every exit path (timer, ESC, Ctrl-C, startup failure, or remote
stop) sends `S:0;E:0;B:1` before disconnecting Bluetooth and performing the
normal vehicle safety shutdown.

The Bluetooth motor module can also be tested independently of the Pixhawk.
With no run duration it only connects, sends the safe-stop command, and
disconnects:

```bash
uv run python -m calibrate.bldc_ble
```

Spinning the test-stand motor requires an explicit bounded duration:

```bash
uv run python -m calibrate.bldc_ble --run-seconds 5 --speed 10 --direction cw
```

Normal completion, connection errors, and Ctrl-C all pass through the same
emergency-stop and disconnect cleanup.

### `watch <stream> [--duration N]`

Read-only observation; never changes vehicle state, never arms. Default duration 10 s.

| Stream | Subscribes to | Row columns |
|---|---|---|
| `servos` | RC_CHANNELS (SERVO_OUTPUT_RAW) | t, s1..s8 |
| `esc` | ESC_TELEMETRY_1_TO_4/5_TO_8 | t, rpm, voltage, current, temperature |
| `text` | STATUSTEXT | t, severity, text |
| `attitude` | ATTITUDE | t, roll, pitch, yaw, ωx, ωy, ωz (all deg / deg-s) |
| `power` | BATTERY_STATUS / SYS_STATUS | t, vbat_v, current_a, power_w |

```bash
python -m calibrate --server http://127.0.0.1:8999 watch servos --duration 15
python -m calibrate --server http://127.0.0.1:8999 watch attitude --duration 60
python -m calibrate --server http://127.0.0.1:8999 watch text
```

---

## One-shot verbs

### `status`
Vehicle snapshot: armed state, flight mode, battery, EKF flags, SERVO_OUTPUT_RAW for
all active outputs, plus pass/fail tables for key stack params, interlock/DShot path,
and yaw control gains.

### `set <name> <value>` / `get <name> [<name> ...]`
Read or write ArduPilot parameters. `get` accepts multiple names so a diagnostic
session can read a group through one connection. `set` verifies via read-back and flags
silent rejects (writes that the FC ACKs but doesn't apply, e.g. swash-channel
`SERVOn_MIN/MAX`).

```bash
python -m calibrate --server http://127.0.0.1:8999 set H_COL_MAX 1700
python -m calibrate --server http://127.0.0.1:8999 get RAWES_MODE H_SV_MAN H_FLYBAR_MODE
```

`config check` gets the complete parameter table in one stateful request and
compares it locally. `config fix` submits all differences through
LinkHub's batch parameter operation and then gets the table once more to
verify every requested value. MAVLink still applies each parameter
individually; the batch avoids one HTTP round trip and one sequential wait per
parameter.

The standalone `arm` command uses normal ArduPilot pre-arm checks, remains
armed only for `--duration` seconds (five seconds by default), and always
requests disarm and safe-off restoration afterward. `arm --force` explicitly
uses ArduPilot's force-arm magic value to bypass pre-arm checks for secured,
disconnected bench hardware. Normal arm remains the default, and forced arm is
still duration-bounded with the same guaranteed disarm cleanup.

### `swash`
Three forms:

```bash
swash <coll%> [lon%] [lat%]    # HR3-120 physical mixer manual drive (-100..+100)
swash range <min_us> <max_us>  # writes H_COL_MIN / H_COL_MAX (heli mixer respects these)
swash neutral [n]              # drive S1/S2/S3 (or n) to 1500 us
swash fit-range <min> <max> --cyclic N --allow-full-range
                                 # empirically fit the final PWM envelope
```

### `servo`
Three forms:

```bash
servo <ch> <pwm>                            # raw PWM; ch1-3 all disconnect, then restore
servo mode <name|0..5> [--duration N]       # native mode; oscillate requires explicit override
servo sweep [--duration N]                  # safe raw-PWM sweep within H_COL_MIN/MAX
servo hold <ch> <pwm> [--duration N]        # hold; ch1-3 disconnect; swash stays disarmed
```

Native mode names are `automated` (0), `passthrough` (1), `max` collective (2),
`zero` thrust collective (3), `min` collective (4), and `oscillate` (5).
ArduPilot's native oscillation exercises maximum configured cyclic at minimum
and maximum collective, so it requires disconnected servos and the explicit
`--allow-full-range` flag. `servo sweep` instead disconnects all three swash
functions and performs a raw-PWM HR3 sweep bounded by live `H_COL_MIN/MAX`,
restoring neutral and the original functions afterward. All servo setup
commands require a disarmed vehicle.

`swash fit-range` is for disconnected servos. It centers all three servo trims
at the midpoint of the requested final PWM range, sets the requested
`H_CYC_MAX`, runs complete native oscillation cycles, and adjusts
`H_COL_MIN/MAX` from measured `SERVO_OUTPUT_RAW` extrema until the requested
range (with a default 3 us margin) is satisfied. Optional controls are
`--iterations 1..5` and `--margin N`.

### Interactive ACRO manual mode

```bash
run acro-manual [--duration N]
```

This selects ArduPilot ACRO and `RAWES_MODE=2`, verifies
`H_FLYBAR_MODE=1`, seeds normalized roll/pitch/collective at `0/0/0.5`,
then arms. Arrow keys adjust roll and pitch by 0.05; `-` and `=` adjust
collective by 0.05 without Shift. Lua latches each NVP setpoint and refreshes
the short-lived RC override every 10 ms. The live table keeps RC1–RC4 and
mixed S1–S3 PWM telemetry enabled and refreshes four times per second.
Selecting mode 2 outside ACRO, without flybar passthrough, or without a
complete three-axis seed immediately disarms.
`IM_ACRO_COL_EXP=0` is also required so normalized collective remains linear
and matches the GUIDED throttle convention.
On exit, the run command sets `RAWES_MODE=0`, requests normal MAVLink disarm,
falls back to force-disarm when ArduPilot rejects an in-flight disarm, and then
applies the canonical safe-off state. That state is: confirmed disarmed,
`RAWES_MODE=0`, `H_SV_MAN=0`, ACRO RC passthrough selected by
`H_FLYBAR_MODE=1`, and `SERVO9_FUNCTION=0` so the yaw motor is unassigned.
While disarmed in mode 0,
Lua refreshes RC1/RC2 at their configured trims and computes the RC3 value that
places the reversed swash mixer at its 1500-us center. The overrides are cleared
as soon as the vehicle arms or RAWES enters another mode.

### `motor`
GB4008 throttle test via `MAV_CMD_DO_MOTOR_TEST`. Prompts above 5% unless `--force`.

```bash
motor <pct> [--duration N]    # default 5 s
motor off
```

### `arm [--duration N] [--force]`
Arm through MAVLink for a bounded interval, then disarm and apply canonical
safe-off. `--force` explicitly bypasses ArduPilot pre-arm checks.

### `disarm` / `reboot`
Every confirmed normal or force disarm selects the canonical safe-off state:
`RAWES_MODE=0`, `H_SV_MAN=0`, ArduPilot ACRO, `H_FLYBAR_MODE=1`, and
`SERVO9_FUNCTION=0`.
The `disarm` command tries normal disarm first and automatically falls back to
force-disarm when ArduPilot rejects disarming from an active flight state.
Lua's disarmed mode-0 RC overrides hold the swash neutral without using an
ArduPilot manual servo setup mode.
The mode change happens only after disarm is confirmed so it cannot cause an
armed control transition.

### `battery monitor off|on [type]`

Disable battery monitoring for USB-only bench work with `battery monitor off`.
Restore it with `battery monitor on`; the default type `4` is analog voltage and
current, matching the current hardware configuration. An alternate monitor type
can be supplied to `on`. ArduPilot marks `BATT_MONITOR` reboot-required, so the
command reports that a reboot is needed; calibrate does not reboot automatically.
Restore battery monitoring before reconnecting or operating from a battery.

### `script upload <file>` / `script list` / `script remove <name>`
Lua FS over MAVLink FTP. `upload` writes to `/APM/scripts/<basename>` and then
toggles `SCR_ENABLE 1→0→1` to restart the scripting engine (no reboot needed).
Use this command only through a direct USB connection; do not upload Lua over
the SiK radio.

### `logs list` / `logs fetch [--id N] [--dir D]`

LinkHub owns the complete DataFlash protocol transaction: `LOG_ENTRY`
collection, chunk requests, out-of-order reassembly, retries, timeouts, and
`LOG_REQUEST_END`. `logs list` shows available controller logs. `logs fetch`
downloads the latest log by default or a selected ID to
`simulation/logs/calibrate/` unless `--dir` is supplied.

### `config show` / `config apply`
Diff the live FC params against shared parm defaults:
`tests/sitl/copter-heli.parm` + `tests/sitl/rawes_common_defaults.parm` +
`hardware/rawes_hardware_defaults.parm`
(excluding SITL-only and hardware calibration params). `show` prints an
`[OK]`/`[DIFF]`/`[FAIL]` table without changes; `apply` writes every `[DIFF]`.

---

## Logging

Every `run` and `watch` session writes a CSV under `simulation/logs/calibrate/`
(gitignored). Header is `# key: value` comments capturing the verb, mode/stream
name, duration, trim/gain dicts, run-start timestamps (local + UTC), and a snapshot
of relevant AP params. Data section is plain CSV.

LinkHub owns the lossless raw MAVLink journal for all clients. Calibration
does not write a duplicate MAVLink JSONL. Each CSV records its LinkHub start
cursor, and the command prints the final cursor range. Query that range with
the native `linkhub query` commands documented in `design/linkhub.md`.

For `run`, the CSV now also captures:
- Lua diagnostic NVFs: `YFF_*` and `OL_*`
- `ATTITUDE_TARGET` state when emitted by ArduPilot
- `PID_TUNING` state when emitted by ArduPilot
- Actual (`ATTITUDE_QUATERNION`) and target (`ATTITUDE_TARGET.q`) attitude
  quaternions (`mav_att_q_*` / `mav_att_target_q_*`), plus the quaternion
  attitude error (`mav_att_qerr_*`): `conj(q_actual) (x) q_target`, its total
  rotation angle (`mav_att_qerr_deg`), and a yaw-only deviation
  (`mav_att_qerr_yaw_deg`, via `2*atan2(z, w)`). The Lua heading/yaw lock is
  implemented as a quaternion attitude target under the hood (AP's
  `set_target_angle_and_rate_and_throttle` converts the Euler args to a
  quaternion before handing off to the attitude controller), so comparing
  quaternions directly avoids the +-180 deg wraparound ambiguity that
  differencing the two Euler yaw columns has near the wrap boundary.

Neither `ATTITUDE_QUATERNION` (#31) nor `ATTITUDE_TARGET` (#83) rides along
with the legacy `EXTRA1` `REQUEST_DATA_STREAM` group on ArduCopter -- `run`
explicitly requests both via `MAV_CMD_SET_MESSAGE_INTERVAL` at 25 Hz. Without
that explicit request, `mav_att_q_*`/`mav_att_target_q_*`/`mav_att_qerr_*`
stay empty even while a GUIDED angle target is actively held (verify with
`linkhub query <journal> types` -- if a message type never appears at all,
it's a missing stream/interval request, not a decode bug; see
`design/linkhub.md`). `ATTITUDE_TARGET`
also only appears at all once the vehicle is
actually in `GUIDED`/`GUIDED_NOGPS` and Lua is driving an angle target.
`run passive` waits for ground-side qualification, then asks Lua to capture the
onboard AHRS quaternion. Target-quaternion and quaternion-error columns are
populated once that hold is engaged.

### Interactive passive attitude hold

`run passive` arms and completes heli runup in ACRO with `RAWES_MODE=0`, applies
neutral cyclic plus passive collective until `EXTENDED_SYS_STATE` confirms that
Copter's landed state is clear, stages passive thrust with absolute hold
disabled, and then enters `GUIDED_NOGPS`. The complete startup and safe-off
sequence is owned by [arming.md](arming.md).
During this staging phase Lua commands zero body rate plus thrust only; it does
not send an Euler angle target and passive yaw trim remains inhibited. The
ground waits for an `ACTIVE` heartbeat, fresh attitude-rate telemetry, and a
continuous quiet interval after any EKF yaw-alignment event. It then sends
`RAWES_PEN=1`; Lua captures the onboard AHRS quaternion and enables absolute
attitude and yaw hold. Ground-side qualification can be tuned per run with
`--settle-rate-deg-s`, `--settle-time`, and `--settle-timeout` without changing
or uploading Lua.
Keyboard offsets are composed relative to the captured
quaternion as `q_target = q_initial * q_relative`; they are not added to its
Euler angles. During the run:

- Left/Right changes relative target roll by 5 degrees.
- Up/Down changes relative target pitch by 5 degrees.
- `,`/`.` (the `<`/`>` keys) changes relative target yaw by -/+5 degrees.
- Relative roll and pitch keyboard travel is limited to +/-30 degrees.
- `-`/`=` changes held thrust by 0.05 within `[0,1]`.
- Space resets relative roll/pitch/yaw offsets to zero; it does not recapture
  the fixed Lua-owned anchor.
- Yaw remains at the captured heading until changed with `,`/`.`.
- ESC exits, disarms, and leaves `RAWES_MODE=0`.

For a text-only interactive protocol trace, add `--protocol-debug`:

```bash
python -m calibrate --server http://127.0.0.1:8999 run passive --protocol-debug
```

This suppresses the 3D window. Before each keyboard command it prints the
latest actual and target quaternions, quaternion error, and swash PWM, followed
by every transmitted `NAMED_VALUE_FLOAT`. Relative attitude state is sent as
`RAWES_ROFF/POFF/YOFF`; `RAWES_PEN` is sent only after the ground qualification
gate passes. ArduPilot receives angle targets with zero rate feed-forward after
capture and its native attitude/rate loops determine the commanded motion.

For an automatic spinning-hardware exercise, add `--auto-sequence` and select
the dwell per step with `--step-hold`:

```bash
python -m calibrate --server http://127.0.0.1:8999 run passive \
  --protocol-debug --auto-sequence --step-hold 5
```

The bounded sequence performs onboard attitude capture, +/-5 degree roll and
pitch commands with a baseline return after each, then +/-0.05 collective
commands with baseline returns. Every target is sent once; Lua passes the
target to ArduPilot with zero rate feed-forward, and ArduPilot determines the
motion through its native attitude and rate controllers. Normal safety shutdown
runs automatically after the final dwell.

### Agent and automation access

Agents and automation call LinkHub's HTTP API directly. There is no
calibration MCP server or intermediate channel service. Stateful generic
operations live under `/v1/mavlink`; RAWES-specific sequencing and safety
policy remain in the calibration client.

The command automatically opens a live PyVista window using the existing
RAWES hub/blade renderer and swashplate inset. The live view renders the four
physical blades individually and omits the translucent high-speed rotor-disc
anti-flicker aid. The main view overlays the
actual rotor orientation in blue and the commanded target orientation in
yellow. The inset reconstructs collective and cyclic plate motion from live
S1/S2/S3 PWM. The HUD shows actual/target attitude, quaternion error, swash
PWM, yaw-motor PWM, thrust, and rotor speed. A visible orange GB4008 rotor
turns opposite the commanded motor-shaft RPM. The blade speed uses ESC
telemetry when available and otherwise uses the commanded motor speed divided
by the 10:1 motor-to-rotor ratio. Hub translation follows live
`LOCAL_POSITION_NED`. Arrow, yaw, thrust, and Escape keys work directly in
this window; closing it also ends the run and invokes the normal safety
shutdown.

Actual position, orientation, target orientation, swash position, and rotor
speed are linearly interpolated between MAVLink updates (with rotation matrices
re-orthonormalized) so the display remains smooth rather than stepping at the
telemetry rate.

The quaternion is sent atomically as `RAWES_QW/QX/QY/QZ`. Lua converts the
complete quaternion to the equivalent Euler triplet only at the final
`set_target_angle_and_rate_and_throttle` boundary because that is the attitude
API exposed by ArduPilot scripting.

`--roll`, `--pitch`, and `--yaw` can override individual captured angles for
the initial target. The live table refreshes four times per second and shows
actual roll/pitch, target roll/pitch, target thrust, yaw, quaternion error,
swash PWM, and yaw-motor PWM.

`PID_TUNING` caveat: ArduPilot may suppress these messages when `GCS_PID_MASK=0`.
calibrate requests `PID_TUNING`, but the FC must still be configured to emit it.

### `GCS_PID_MASK` (ArduCopter)

`GCS_PID_MASK` is documented in the canonical ArduPilot parameter file:
`tests/sitl/copter-heli.parm`.

Use that `.parm` file as the single source of truth for bit assignments, default, and common values.

Important: `PID_TUNING` in Copter reports rate-loop internals (plus AccelZ), not
the outer attitude-angle controller internals.

Useful for offline analysis.

---

## GB4008 motor constants (used in `watch esc` derivations)

| Constant | Value | Source |
|----------|-------|--------|
| Kv | 66 RPM/V | EMAX spec |
| Pole configuration | see SERVO_BLH_POLES | verified against known RPM |
| Gear ratio | 10:1 | Hardware |
| Kt (motor shaft) | 0.144 N·m/A | Derived: 60/(2π×66) |
| eRPM → motor RPM | ÷ (SERVO_BLH_POLES/2) | pole-pairs |
| eRPM → rotor RPM | ÷ (SERVO_BLH_POLES/2 × 10) | apply gear ratio |

---

## Typical calibration sequence

```bash
# 0. Survey ports (first time)
curl http://127.0.0.1:8999/v1/mavlink/status

# 1. Diff against canonical params; apply if needed
python -m calibrate --server http://127.0.0.1:8999 config show
python -m calibrate --server http://127.0.0.1:8999 config apply
python -m calibrate --server http://127.0.0.1:8999 reboot

# 2. Verify live state
python -m calibrate --server http://127.0.0.1:8999 status

# 3. Swashplate neutral + mixing check (one-shot, no arming)
python -m calibrate --server http://127.0.0.1:8999 swash neutral
python -m calibrate --server http://127.0.0.1:8999 swash 50 0 0
python -m calibrate --server http://127.0.0.1:8999 swash 0 0 50

# 4. Limit swash travel if servos can't take full range
python -m calibrate --server http://127.0.0.1:8999 swash range 1300 1700
python -m calibrate --server http://127.0.0.1:8999 set H_CYC_MAX 1000

# 5. Swash motion check -- one complete native ArduPilot cycle
python -m calibrate --server http://127.0.0.1:8999 servo sweep --duration 12

# 6. Motor spin check
python -m calibrate --server http://127.0.0.1:8999 motor 5 --duration 5
python -m calibrate --server http://127.0.0.1:8999 watch esc --duration 10

# 7. Connect the Pixhawk directly by USB, then upload updated Lua and verify
python -m calibrate --server http://127.0.0.1:8999 script upload scripts/rawes.lua
python -m calibrate --server http://127.0.0.1:8999 script list

# 8. Quiet armed bench check
python -m calibrate --server http://127.0.0.1:8999 run passive --duration 30 --trim tlon=0.02,thr=0.342

# 9. Passive hold check with current controller settings
python -m calibrate --server http://127.0.0.1:8999 run passive --duration 60 --trim tlon=0.02,thr=0.342
```
