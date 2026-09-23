# calibrate — Hardware Calibration Tool

`calibrate` (top-level package, run as `python -m calibrate`, or via `calibrate.cmd`
on Windows) connects to the Pixhawk 6C over USB (or SiK radio)
and provides servo control, motor testing, ESC diagnostics, arming, and Lua script
upload — all over MAVLink, with no arming required for most commands.

Lua script deployment is the exception to normal radio operation: upload scripts
only over a direct Pixhawk USB connection. MAVLink FTP over the SiK radio is too
slow for reliable operational updates. Disconnect the radio MCP connection,
connect USB, upload and verify the script manually, reboot or restart scripting,
then reconnect the MCP over the normal radio link.

## Connection

```bash
python -m calibrate                              # auto-detect port
python -m calibrate --port COM7
python -m calibrate --port COM7 --baud 57600     # SiK radio
python -m calibrate --port COM7 <verb> [args]    # non-interactive
```

If `--port` is omitted, the tool scans all COM ports and connects to the first one
that responds with a MAVLink heartbeat (tries 115200, then 57600/38400/19200/9600 as
fallbacks). Use `ping` to survey ports without connecting.

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
python -m calibrate [--port P] [--baud B] [--force] <verb> [args...]
```

Two long-running verbs (`run`, `watch`) handle anything time-bounded and always log
to `simulation/logs/calibrate/<verb>_<name>_YYYYMMDD_HHMMSS.csv` (gitignored). The
rest are one-shot.

---

## Long-running verbs

### `run <name> [--duration N] [--trim K=V,...]`

Activate a Lua mode (via `RAWES_MODE`) → arm via `RAWES_ARM` → stream observation rows
to console + CSV → safety shutdown on exit. ESC or Ctrl-C aborts cleanly. Without
`--duration`, the session is unbounded (5-min `RAWES_ARM`); abort with ESC/Ctrl-C.

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
python -m calibrate --port COM7 run passive --duration 30 --trim tlon=0.02,thr=0.342

# Steady-flight bench, unbounded (ESC to stop)
python -m calibrate --port COM7 run steady
```

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
python -m calibrate --port COM7 watch servos --duration 15
python -m calibrate --port COM7 watch attitude --duration 60
python -m calibrate --port COM7 watch text                       # default 10 s
```

---

## One-shot verbs

### `status`
Vehicle snapshot: armed state, flight mode, battery, EKF flags, SERVO_OUTPUT_RAW for
all active outputs, plus pass/fail tables for key stack params, interlock/DShot path,
and yaw control gains.

### `set <name> <value>` / `get <name>`
Read or write a single ArduPilot parameter. `set` verifies via read-back and flags
silent rejects (writes that the FC ACKs but doesn't apply, e.g. swash-channel
`SERVOn_MIN/MAX`).

```bash
python -m calibrate --port COM7 set H_COL_MAX 1700
python -m calibrate --port COM7 get RAWES_MODE
```

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
On exit, the run command sets `RAWES_MODE=0` permanently so Lua releases
RC1–RC3, turns the motor output off, requests disarm through Lua's always-active
`RAWES_ARM` handler, and selects ArduPilot ACRO after a disarmed heartbeat.
If Lua does not confirm disarm within one second, calibrate immediately sends a
force-disarm instead of waiting on a normal disarm that may be rejected while
ArduPilot does not consider the vehicle landed.
This is the canonical safe-off state: disarmed, Lua control released,
`H_SV_MAN=3` centered/zero-thrust swash setup mode, and virtual-flybar-disabled
behavior provided by `H_FLYBAR_MODE=0`. Before any run, calibrate restores
`H_SV_MAN=0` so the automated heli mixer is active.

### `motor`
GB4008 throttle test via `MAV_CMD_DO_MOTOR_TEST`. Prompts above 5% unless `--force`.

```bash
motor <pct> [--duration N]    # default 5 s
motor off
```

### `arm [--duration N]`
Set stack arm state + send `RAWES_ARM=N*1000` (default 10 s). Doesn't touch `RAWES_MODE` — use
`run <name>` if you also want to activate a Lua mode.

### `disarm` / `reboot` / `ping [baud]`
Every confirmed normal or force disarm selects the canonical safe-off state:
`RAWES_MODE=0`, `H_SV_MAN=3`, ArduPilot ACRO, and `H_FLYBAR_MODE=0`
virtual-flybar-disabled behavior.
The mode change happens only after disarm is confirmed so it cannot cause an
armed control transition. `ping` doesn't open a connection.

### `script upload <file>` / `script list` / `script remove <name>`
Lua FS over MAVLink FTP. `upload` writes to `/APM/scripts/<basename>` and then
toggles `SCR_ENABLE 1→0→1` to restart the scripting engine (no reboot needed).
Use this command only through a direct USB connection; do not upload Lua over
the SiK radio. The MCP's normal COM6/57600 radio connection is suitable for
control and diagnosis, not script deployment.

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
`analysis/mavlink_jsonl_query.py types <log>.mavlink.jsonl` -- if a message
type never appears at all, it's a missing stream/interval request, not a
decode bug; see `analysis/mavlink_jsonl_query.md` for full usage -- it is
the first-line tool for diagnosing any problematic run). `ATTITUDE_TARGET`
also only appears at all once the vehicle is
actually in `GUIDED`/`GUIDED_NOGPS` and Lua is driving an angle target.
`run passive` captures the current roll/pitch/yaw and immediately uses it as
the initial target, so its target-quaternion and quaternion-error columns are
populated once the hold is engaged.

### Interactive passive attitude hold

`run passive` arms and completes heli runup in ACRO with `RAWES_MODE=0`, stages
passive thrust with absolute hold disabled, and then enters `GUIDED_NOGPS`.
During this staging phase Lua commands zero body rate plus thrust only; passive
yaw trim remains inhibited. The ground waits for an `ACTIVE` heartbeat and a
three-second quiet interval after any EKF yaw-alignment event, captures the
settled `ATTITUDE_QUATERNION`, and sends `RAWES_PEN=1` to enable absolute
attitude and yaw hold. This prevents EKF in-flight yaw resets from triggering
the GB4008 while the stationary hardware is already physically on target.
Keyboard offsets are composed relative to the captured
quaternion as `q_target = q_initial * q_relative`; they are not added to its
Euler angles. During the run:

- Left/Right changes relative target roll by 5 degrees.
- Up/Down changes relative target pitch by 5 degrees.
- `,`/`.` (the `<`/`>` keys) changes relative target yaw by -/+5 degrees.
- Relative roll and pitch keyboard travel is limited to +/-30 degrees.
- `-`/`=` changes held thrust by 0.05 within `[0,1]`.
- Space sends the one-shot `RAWES_YIC=-1000` capture command. Lua samples the
  onboard AHRS roll/pitch/yaw and commits that attitude directly, avoiding
  ground-telemetry latency in a read-and-echo reset.
- Yaw remains at the captured heading until changed with `,`/`.`.
- ESC exits, disarms, and leaves `RAWES_MODE=0`.

For a text-only interactive protocol trace, add `--protocol-debug`:

```bash
python -m calibrate --port COM6 --baud 57600 run passive --protocol-debug
```

This suppresses the 3D window. Before each keyboard command it prints the
latest actual and target quaternions, quaternion error, and swash PWM, followed
by every transmitted `NAMED_VALUE_FLOAT`. Space is intentionally a single
`RAWES_YIC=-1000` packet; normal arrow/yaw updates remain atomic
`RAWES_QW/QX/QY/QZ` target updates. ArduPilot receives angle targets with zero
rate feed-forward and its native attitude/rate loops determine the commanded
motion.

For an automatic spinning-hardware exercise, add `--auto-sequence` and select
the dwell per step with `--step-hold`:

```bash
python -m calibrate --port COM6 --baud 57600 run passive \
  --protocol-debug --auto-sequence --step-hold 5
```

The bounded sequence performs onboard attitude capture, +/-5 degree roll and
pitch commands with a baseline return after each, then +/-0.05 collective
commands with baseline returns. Every target is sent once; Lua passes the
target to ArduPilot with zero rate feed-forward, and ArduPilot determines the
motion through its native attitude and rate controllers. Normal safety shutdown
runs automatically after the final dwell.

### AI/MCP access

The local stdio MCP server exposes the complete calibrate command dispatcher
through one persistent MAVLink connection. VS Code configuration is checked in
at `.vscode/mcp.json` and defaults to COM6 at 57600 baud.

Run it directly when testing the server outside VS Code:

```bash
.venv/Scripts/python.exe -m calibrate.mcp_server --port COM6 --baud 57600
```

The MCP process owns one persistent `RawesGCS` connection. It does not launch
the calibrate command line. Typed tools call the shared calibration library
functions directly: connection management, hardware status, parameters,
swash, servo, motor, run modes, telemetry watch, logs, Lua scripts,
configuration, arm/reboot, and emergency disarm. Printed library output is
captured into the tool result so it cannot corrupt the MCP stdio protocol.
Passive startup holds `RAWES_MODE=0` in ACRO during the configured traditional-
heli RSC runup interval. Only after runup does it enter GUIDED_NOGPS, capture
the current quaternion, seed the passive target, and select `RAWES_MODE=3`.
This prevents GUIDED's landed/runup branch from flattening the internal
roll/pitch target before passive control begins.
Hardware operations are serialized and protected by an independent process
watchdog. One-shot tools have a 30-second deadline. Intentionally long tools
(`run_mode`, `watch`, `motor`, logs, script, and config) have a 600-second
deadline. `run_mode` and `watch` accept a per-call `timeout_s` override. When a
deadline expires, the watchdog first requests cooperative stop so the normal
safety shutdown can run. If the operation is still blocked 10 seconds later,
the watchdog terminates the MCP process with exit code 124. Configure these
defaults with `--command-timeout`, `--long-command-timeout`, and
`--timeout-grace`; `.vscode/mcp.json` specifies the project defaults explicitly.

`stop_operation` interrupts an active `run_mode` at its next observation-loop
iteration and lets the normal mode-0, Lua-disarm, motor-off, ACRO safe-off
lifecycle complete while keeping the MCP server running. It is lock-free, so a
blocked hardware operation cannot prevent the stop request. `shutdown_server`
first requests the same cancellation, waits for the active operation to release
the connection, force-disarms to confirm hardware safety, closes COM6, returns
its final result, and then terminates the stdio MCP process. By default it
refuses to terminate if disarm cannot be confirmed. Use
`shutdown_server(force=true)` only after independently confirming the hardware
is safe; it closes the connection and exits even without disarm confirmation.
VS Code can then start a fresh MCP process containing updated code without a
window reload.

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
python -m calibrate ping

# 1. Diff against canonical params; apply if needed
python -m calibrate --port COM7 config show
python -m calibrate --port COM7 config apply   # if any [DIFF] shown
python -m calibrate --port COM7 reboot

# 2. Verify live state
python -m calibrate --port COM7 status

# 3. Swashplate neutral + mixing check (one-shot, no arming)
python -m calibrate --port COM7 swash neutral
python -m calibrate --port COM7 swash 50 0 0     # all servos rise equally?
python -m calibrate --port COM7 swash 0 0 50     # lateral differential?

# 4. Limit swash travel if servos can't take full range
python -m calibrate --port COM7 swash range 1300 1700
python -m calibrate --port COM7 set H_CYC_MAX 1000

# 5. Swash motion check -- one complete native ArduPilot cycle
python -m calibrate --port COM7 servo sweep --duration 12

# 6. Motor spin check
python -m calibrate --port COM7 motor 5 --duration 5
python -m calibrate --port COM7 watch esc --duration 10

# 7. Connect the Pixhawk directly by USB, then upload updated Lua and verify
python -m calibrate --port <USB_COM_PORT> script upload scripts/rawes.lua
python -m calibrate --port <USB_COM_PORT> script list

# 8. Quiet armed bench check
python -m calibrate --port COM7 run passive --duration 30 --trim tlon=0.02,thr=0.342

# 9. Passive hold check with current controller settings
python -m calibrate --port COM7 run passive --duration 60 --trim tlon=0.02,thr=0.342
```
