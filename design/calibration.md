# calibrate.py — Hardware Calibration Tool

`simulation/scripts/calibrate.py` connects to the Pixhawk 6C over USB (or SiK radio)
and provides servo control, motor testing, ESC diagnostics, arming, and Lua script
upload — all over MAVLink, with no arming required for most commands.

## Connection

```bash
python simulation/scripts/calibrate.py                              # auto-detect port
python simulation/scripts/calibrate.py --port COM7
python simulation/scripts/calibrate.py --port COM7 --baud 57600     # SiK radio
python simulation/scripts/calibrate.py --port COM7 <verb> [args]    # non-interactive
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
| 4 | GB4008 anti-rotation motor (MAIN OUT 4) | 0 (Lua-owned via SRV_Channels) |

Swashplate PWM range: 1000 µs (min) … 1500 µs (neutral) … 2000 µs (max). The heli mixer
hard-codes this range on the swash servos — `SERVOn_MIN/MAX` writes are silently
overwritten on every output tick. Use `swash range <min> <max>` (which writes
`H_COL_MIN/H_COL_MAX`) to limit physical swash travel.

Motor PWM range: 800 µs (off) … 2000 µs (full throttle). `SERVO4_MIN/MAX` is the
limiter you want here.

---

## CLI shape

```
calibrate.py [--port P] [--baud B] [--force] <verb> [args...]
```

Two long-running verbs (`run`, `watch`) handle anything time-bounded and always log
to `simulation/logs/calibrate/<verb>_<name>_YYYYMMDD_HHMMSS.csv` (gitignored). The
rest are one-shot.

---

## Long-running verbs

### `run <name> [--duration N] [--trim K=V,...] [--gain K=V,...]`

Activate a Lua mode (via `SCR_USER6`) → arm via `RAWES_ARM` → stream observation rows
to console + CSV → safety shutdown on exit. ESC or Ctrl-C aborts cleanly. Without
`--duration`, the session is unbounded (5-min `RAWES_ARM`); abort with ESC/Ctrl-C.

**Modes (`<name>`):**

| Name | `SCR_USER6` | Takes SERVO4? | Accepts `--gain`? |
|---|---|---|---|
| `passive` | 3 | yes (pins at 800 µs) | no |
| `yaw` | 2 | yes (Lua yaw PID drives SERVO4) | yes (full PID + filters + servo limits) |
| `steady` | 1 | no | no |
| `pumping` | 5 | no | no |
| `landing` | 4 | no | no |

`--trim` keys (rad), applies to all modes:

| Key | NVF sent | Meaning |
|---|---|---|
| `tlon` | `RAWES_TLN` | cyclic trim longitudinal |
| `tlat` | `RAWES_TLT` | cyclic trim lateral |
| `col`  | `RAWES_COL` | IC collective |

`--gain` keys (only for `yaw`; values written to ArduPilot params, restored on exit):

| Key | AP param | Notes |
|---|---|---|
| `p`,`i`,`d`,`ff`,`imax` | `ATC_RAT_YAW_*` | Yaw rate PID gains |
| `trim` | `H_YAW_TRIM` | Feedforward throttle |
| `flte`,`fltt`,`fltd` | `ATC_RAT_YAW_FLT*` | Target / error / derivative filters |
| `accelmax` | `ATC_ACCEL_Y_MAX` | Yaw accel limit |
| `servo_min`,`servo_max` | `SERVO4_MIN/MAX` | Motor PWM range cap |

```bash
# Bench check: hold IC swashplate + motor off, 30 s
python calibrate.py --port COM7 run passive --duration 30 --trim tlon=0.02,col=-0.15

# Yaw PID tuning session with safety cap on motor max
python calibrate.py --port COM7 run yaw --duration 60 --gain p=0.015,imax=0.7,servo_max=1100 --trim tlon=0.02,col=-0.15

# Steady-flight bench, unbounded (ESC to stop)
python calibrate.py --port COM7 run steady
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
python calibrate.py --port COM7 watch servos --duration 15
python calibrate.py --port COM7 watch attitude --duration 60
python calibrate.py --port COM7 watch text                       # default 10 s
```

---

## One-shot verbs

### `status`
Vehicle snapshot: armed state, flight mode, battery, EKF flags, SERVO_OUTPUT_RAW for
all active outputs, and a pass/fail table for key RAWES parameters + tail PID values.

### `set <name> <value>` / `get <name>`
Read or write a single ArduPilot parameter. `set` verifies via read-back and flags
silent rejects (writes that the FC ACKs but doesn't apply, e.g. swash-channel
`SERVOn_MIN/MAX`).

```bash
python calibrate.py --port COM7 set H_COL_MAX 1700
python calibrate.py --port COM7 get SCR_USER6
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
servo <ch> <pwm>                            # write raw PWM via MAV_CMD_DO_SET_SERVO
servo sweep <ch> [--step-ms N]              # slow sweep 1500 -> 2000 -> 1000 -> 1500
servo hold <ch> <pwm> [--duration N]        # arm + hold ch at pwm for N s (default 60)
```

### `motor`
GB4008 throttle test via `MAV_CMD_DO_MOTOR_TEST`. Prompts above 5% unless `--force`.

```bash
motor <pct> [--duration N]    # default 5 s
motor off
```

### `arm [--duration N]`
Set ACRO + send `RAWES_ARM=N*1000` (default 10 s). Doesn't touch `SCR_USER6` — use
`run <name>` if you also want to activate a Lua mode.

### `disarm` / `reboot` / `ping [baud]`
Self-explanatory. `ping` doesn't open a connection.

### `script upload <file>` / `script list` / `script remove <name>`
Lua FS over MAVLink FTP. `upload` writes to `/APM/scripts/<basename>` and then
toggles `SCR_ENABLE 1→0→1` to restart the scripting engine (no reboot needed).

### `config show` / `config apply`
Diff the live FC params against `simulation/scripts/rawes_params.json`. `show` prints
a `[OK]`/`[DIFF]`/`[FAIL]` table without changes; `apply` writes every `[DIFF]`.

---

## Logging

Every `run` and `watch` session writes a CSV under `simulation/logs/calibrate/`
(gitignored). Header is `# key: value` comments capturing the verb, mode/stream
name, duration, trim/gain dicts, run-start timestamps (local + UTC), and a snapshot
of relevant AP params. Data section is plain CSV.

Useful for offline analysis (e.g. `simulation/scripts/analyze_tuning.py` for yaw PID
runs).

---

## GB4008 motor constants (used in `watch esc` derivations)

| Constant | Value | Source |
|----------|-------|--------|
| Kv | 66 RPM/V | EMAX spec |
| Pole configuration | 24N22P (11 pole-pairs) | EMAX spec |
| Gear ratio | 80:44 = 1.818 | Hardware |
| Kt (motor shaft) | 0.144 N·m/A | Derived: 60/(2π×66) |
| eRPM → motor RPM | ÷ 11 | pole-pairs |
| eRPM → rotor RPM | ÷ 20 | ÷ (11 × 80/44) |

---

## Typical calibration sequence

```bash
# 0. Survey ports (first time)
python calibrate.py ping

# 1. Diff against canonical params; apply if needed
python calibrate.py --port COM7 config show
python calibrate.py --port COM7 config apply   # if any [DIFF] shown
python calibrate.py --port COM7 reboot

# 2. Verify live state
python calibrate.py --port COM7 status

# 3. Swashplate neutral + mixing check (one-shot, no arming)
python calibrate.py --port COM7 swash neutral
python calibrate.py --port COM7 swash 50 0 0     # all servos rise equally?
python calibrate.py --port COM7 swash 0 0 50     # lateral differential?

# 4. Limit swash travel if servos can't take full range
python calibrate.py --port COM7 swash range 1300 1700
python calibrate.py --port COM7 set H_CYC_MAX 1000

# 5. Flap deflection measurement -- sweep each servo
python calibrate.py --port COM7 servo sweep 1
python calibrate.py --port COM7 servo sweep 2

# 6. Motor spin check
python calibrate.py --port COM7 motor 5 --duration 5
python calibrate.py --port COM7 watch esc --duration 10

# 7. Upload updated Lua and verify
python calibrate.py --port COM7 script upload simulation/scripts/rawes.lua
python calibrate.py --port COM7 script list

# 8. Quiet armed bench check
python calibrate.py --port COM7 run passive --duration 30 --trim tlon=0.02,col=-0.15

# 9. Yaw PID tuning
python calibrate.py --port COM7 run yaw --duration 60 \
    --gain p=0.015,imax=0.7,servo_max=1100 \
    --trim tlon=0.02,col=-0.15
```
