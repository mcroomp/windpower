# calibrate — Hardware Calibration Tool

`calibrate` (top-level package, run as `python -m calibrate`, or via `calibrate.cmd`
on Windows) connects to the Pixhawk 6C over USB (or SiK radio)
and provides servo control, motor testing, ESC diagnostics, arming, and Lua script
upload — all over MAVLink, with no arming required for most commands.

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
| 1 | S1 — swashplate 0 deg (East) | 33 (Motor1) |
| 2 | S2 — swashplate 120 deg | 34 (Motor2) |
| 3 | S3 — swashplate 240 deg | 35 (Motor3) |
| 9 | GB4008 anti-rotation motor (AUX 1) | 36 (Motor4) |

Swashplate PWM range: 1000 µs (min) … 1500 µs (neutral) … 2000 µs (max). The heli mixer
hard-codes this range on the swash servos — `SERVOn_MIN/MAX` writes are silently
overwritten on every output tick. Use `swash range <min> <max>` (which writes
`H_COL_MIN/H_COL_MAX`) to limit physical swash travel.

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
swash <coll%> [lon%] [lat%]    # H3-120 forward mixer manual drive (-100..+100)
swash range <min_us> <max_us>  # writes H_COL_MIN / H_COL_MAX (heli mixer respects these)
swash neutral [n]              # drive S1/S2/S3 (or n) to 1500 us
```

### `servo`
Three forms:

```bash
servo <ch> <pwm>                            # raw PWM; ch1-3 all disconnect, then restore
servo mode <name|0..5> [--duration N]       # run a native H_SV_MAN mode, then restore
servo sweep [--duration N]                  # native ArduPilot all-swash test; default 12 s
servo hold <ch> <pwm> [--duration N]        # hold; ch1-3 disconnect; swash stays disarmed
```

Native mode names are `automated` (0), `passthrough` (1), `max` collective (2),
`zero` thrust collective (3), `min` collective (4), and `oscillate` (5). The command
requires a disarmed vehicle, prints live S1/S2/S3 PWM telemetry, and restores the
previous setting afterward. `servo sweep` is an alias for `servo mode oscillate`.
Direct and hold commands temporarily disconnect all three swash functions before
sending raw PWM. Changing `H_SW_TYPE` is neither required nor a way to disable the
swash mixer.

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
Self-explanatory. `ping` doesn't open a connection.

### `script upload <file>` / `script list` / `script remove <name>`
Lua FS over MAVLink FTP. `upload` writes to `/APM/scripts/<basename>` and then
toggles `SCR_ENABLE 1→0→1` to restart the scripting engine (no reboot needed).

### `config show` / `config apply`
Diff the live FC params against shared parm defaults:
`tests/sitl/copter-heli.parm` + `tests/sitl/rawes_common_defaults.parm`
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
actually in `GUIDED`/`GUIDED_NOGPS` and Lua is driving an angle target (e.g.
`run passive --hold`, or `steady`/`pumping`) -- a bare `run passive` without
`--hold`/an IC-seeded IC still logs `mav_att_q_*` (actual attitude) but only
gets a non-empty target/`qerr` once the hold is engaged.

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
| eRPM → rotor RPM | ÷ (SERVO_BLH_POLES/2 × 80/44) | apply gear ratio |

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

# 7. Upload updated Lua and verify
python -m calibrate --port COM7 script upload scripts/rawes.lua
python -m calibrate --port COM7 script list

# 8. Quiet armed bench check
python -m calibrate --port COM7 run passive --duration 30 --trim tlon=0.02,col=-0.15

# 9. Passive hold check with current controller settings
python -m calibrate --port COM7 run passive --duration 60 --trim tlon=0.02,thr=0.342
```
