# calibrate.py — Hardware Calibration Tool

`simulation/scripts/calibrate.py` connects to the Pixhawk 6C over USB (or SiK radio)
and provides servo control, motor testing, ESC diagnostics, arming, and Lua script
upload — all over MAVLink, with no arming required for most commands.

## Connection

```bash
python simulation/scripts/calibrate.py                          # auto-detect port
python simulation/scripts/calibrate.py --port COM3
python simulation/scripts/calibrate.py --port COM4 --baud 57600   # SiK radio
python simulation/scripts/calibrate.py --port COM4 <command>      # non-interactive
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
| 1 | S1 — swashplate 0 deg (East) | 33 |
| 2 | S2 — swashplate 120 deg | 34 |
| 3 | S3 — swashplate 240 deg | 35 |
| 4 | GB4008 anti-rotation motor (MAIN OUT 4) | H_TAIL_TYPE=3 (DDFP CW, no sign flip — US-convention rotor) |

Swashplate PWM range: 1000 µs (min) … 1500 µs (neutral) … 2000 µs (max).
Motor PWM range: 800 µs (off) … 2000 µs (full throttle).

---

## Commands

### `status`
Comprehensive vehicle snapshot: armed state, flight mode, battery voltage/current/cells,
EKF flags (attitude/velocity/position), SERVO_OUTPUT_RAW for all active outputs, and a
pass/fail table for key RAWES parameters + tail PID values.

```bash
python calibrate.py --port COM4 status
```

---

### `servo <n> <pwm>`
Set output `n` (1–16) to a raw PWM value directly, bypassing all mixing.
Works while disarmed (`MAV_CMD_DO_SET_SERVO`).

```bash
python calibrate.py --port COM4 servo 1 1500    # S1 to neutral
python calibrate.py --port COM4 servo 4 1100    # motor output low
```

---

### `neutral [n]`
Set output `n` to 1500 µs. Omit `n` to set all three swashplate servos (1, 2, 3).

```bash
python calibrate.py --port COM4 neutral         # S1+S2+S3 to 1500
python calibrate.py --port COM4 neutral 2       # S2 only
```

---

### `swash <coll_%> [lon_%] [lat_%]`
Set S1/S2/S3 via the H3-120 forward mixer. Values are −100 to +100 (percent of full throw).
Useful for verifying mixing geometry before flight.

```bash
python calibrate.py --port COM4 swash 50          # 50% collective, no cyclic
python calibrate.py --port COM4 swash 0 0 30      # 30% lateral tilt only
python calibrate.py --port COM4 swash -30 20 0    # -30% collective, 20% longitudinal
```

Collective-only (`lon=lat=0`) should move all three servos by equal amounts.
Pure lateral or longitudinal tilt should show the correct H3-120 differential pattern.

---

### `sweep <n> [step_ms]`
Slowly sweep output `n` through 1500 → 2000 → 1000 → 1500 at `step_ms` ms per µs step
(default 5 ms = ~5 s full sweep). Use with an angle gauge to measure flap deflection vs PWM.

```bash
python calibrate.py --port COM4 sweep 1          # S1, 5 ms/step
python calibrate.py --port COM4 sweep 2 10       # S2, slower (10 ms/step)
```

Ctrl-C aborts and returns the servo to neutral.

---

### `motor <pct>` / `motor off`
Run `MAV_CMD_DO_MOTOR_TEST` on output 4 (GB4008) at `pct` percent throttle for 5 s.
Works while disarmed. Prompts for confirmation above 30% unless `--force` is set.

```bash
python calibrate.py --port COM4 motor 5           # 5% — audible spin check
python calibrate.py --port COM4 motor 10 --force  # 10%, no prompt
python calibrate.py --port COM4 motor off         # stop immediately
```

---

### `monitor [seconds]`
Stream live ESC telemetry at ~4 Hz for `seconds` seconds (default 10).
Columns: elapsed time, eRPM, motor RPM, rotor RPM, current (A), shaft torque (N·m),
voltage (V), ESC temperature (°C).

```bash
python calibrate.py --port COM4 monitor 30
```

Use while running motor tests to validate RPM, current draw, and torque estimates.

---

### `listen [seconds]`
Stream all STATUSTEXT messages and armed-state changes. No time limit unless `seconds`
is given; Ctrl-C stops early.

```bash
python calibrate.py --port COM4 listen            # indefinite
python calibrate.py --port COM4 listen 30         # 30 s
```

Useful after changing `SCR_USER6` to watch Lua startup messages.

---

### `mode <0|1|2|4|5>`
Set `SCR_USER6` to the given Lua mode number, then listen 10 s for STATUSTEXT confirmation.

| Value | Mode    | Description |
|-------|---------|-------------|
| 0     | none    | Passive: no RC overrides; periodic logging of STATUSTEXT / NV floats only. **Default on boot.** |
| 1     | steady  | Steady flight at 50 Hz: cyclic altitude hold (`RAWES_ALT`) + VZ PI collective. Owns Ch1/Ch2/Ch3. **No yaw control** -- AP's `ATC_RAT_YAW` tail PID is disabled in hardware config (`SERVO4_FUNCTION=0`); needs Lua-side yaw integration before yaw drift is countered. |
| 2     | yaw     | Manual Lua yaw PID: holds yaw rate = 0 by driving `SERVO4` directly via `set_output_pwm_chan_timeout`. Also holds cyclic trim (`RAWES_TLN`/`RAWES_TLT`) and IC collective (`RAWES_COL`), with the cyclic rotated by accumulated `gyro:z()` so the world-frame disk-tilt direction stays constant as the body yaws. **No SERVO4_FUNCTION shuffle needed** -- the hardware default is already `SERVO4_FUNCTION=0` so Lua owns SERVO4 exclusively. |
| 4     | landing | Reserved -- not yet implemented in `rawes.lua`. Will run the landing controller (`reel_in` -> `descent` -> `final_drop` triggered by `RAWES_SUB=1`). |
| 5     | pumping | De Schutter pumping cycle at 50 Hz: phase advance is ground-driven via `NAMED_VALUE_FLOAT("RAWES_SUB", N)` (0=hold, 1=reel_out, 2=transition, 3=reel_in, 4=transition_back); TensionPID on collective using `RAWES_TEN` feedback; cyclic altitude hold via `RAWES_ALT`. **No yaw control** (AP tail PID disabled). |

```bash
python calibrate.py --port COM4 mode 1           # steady mode
python calibrate.py --port COM4 mode 0           # back to passive
```

---

### `arm [seconds]`
Set ACRO mode, send `RAWES_ARM=<ms>` (default 10 s countdown), then display a live table
of elapsed time, armed state, yaw rate, SERVO4 PWM, and STATUSTEXT for the duration.

```bash
python calibrate.py --port COM4 arm              # 10 s arm window
python calibrate.py --port COM4 arm 30           # 30 s arm window
```

Lua arms the vehicle and starts a disarm countdown; re-sending `RAWES_ARM` refreshes it.

---

### `hold <pwm> [seconds]`
Arm via `RAWES_ARM`, then hold output 4 (GB4008) at a fixed PWM for `seconds` seconds
(or indefinitely until Ctrl-C). Temporarily sets `SERVO4_FUNCTION=0` to release the
output from ArduPilot mixing, and restores it on exit.

```bash
python calibrate.py --port COM4 hold 900         # ~8% indefinitely
python calibrate.py --port COM4 hold 1000 20     # hold for 20 s
```

Use to characterise motor/ESC at a constant setpoint without Lua running.

---

### `yawmanual [seconds] [key=value ...]`
Arm via `RAWES_ARM`, set `SCR_USER6=2` (MODE_YAW), and log live tuning data
to `tuning_<ts>.csv` until the seconds timer expires or ESC is pressed. The
Lua's manual yaw PID drives SERVO4 directly (bypassing ArduPilot's DDFP
mixer); the command also pins cyclic + collective at the IC operating
point so the bench rig sees the full flight-mode actuator set.

**Per-run param overrides** (restored on exit, normal or aborted):

| Key | Maps to | Notes |
|-----|---------|-------|
| `p`, `i`, `d`, `ff`, `imax` | `ATC_RAT_YAW_*` | Yaw rate PID gains |
| `trim` | `H_YAW_TRIM` | Feedforward |
| `flte`, `fltt`, `fltd` | `ATC_RAT_YAW_FLT*` | Target / error / derivative filters |
| `accelmax` | `ATC_ACCEL_Y_MAX` | Yaw accel limit |
| `servo_min`, `servo_max` | `SERVO4_MIN/MAX` | Motor PWM range cap (drop SERVO4_MAX to limit motor authority during early tuning) |

**Cyclic + collective trim** (sent as `NAMED_VALUE_FLOAT` so `rawes.lua`
MODE_YAW's `set_trim_ic_rc_overrides` holds them at the IC and rotates
the cyclic by accumulated `gyro:z()` so the world-frame disk-tilt
direction stays constant as the body yaws):

| Key | NVF | Units |
|-----|-----|-------|
| `tlon` | `RAWES_TLN` | rad (cyclic trim longitudinal) |
| `tlat` | `RAWES_TLT` | rad (cyclic trim lateral) |
| `col`  | `RAWES_COL` | rad (IC collective) |

```bash
# 30 s session, lower max throttle as a safety cap, gentler P:
python calibrate.py --port COM4 yawmanual 30 p=0.015 i=0 servo_max=1100

# Tune yaw with IC operating point loaded:
python calibrate.py --port COM4 yawmanual 60 tlon=0.02 tlat=0.05 col=-0.15
```

---

### `disarm`
Send `MAV_CMD_COMPONENT_ARM_DISARM` (param1=0) and clear all RC overrides.
Waits up to 10 s for the disarmed heartbeat.

```bash
python calibrate.py --port COM4 disarm
```

---

### `reboot`
Send `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN`. Reconnect in ~5 s.

```bash
python calibrate.py --port COM4 reboot
```

---

### `config [--apply]`
Diff the current ArduPilot parameters against the canonical RAWES values in
`simulation/scripts/rawes_params.json`. Without `--apply`, prints a preview table
(`[OK]` / `[DIFF]` / `[FAIL]`) and makes no changes. With `--apply`, writes all
differing parameters and reboots.

```bash
python calibrate.py --port COM4 config            # preview only
python calibrate.py --port COM4 config --apply    # write + reboot
```

All expected values are read from `rawes_params.json` (keys: `key`, `tail_pid`).
Entries with `"expected": null` are informational only and are not written.

---

### `getparam <name>` / `setparam <name> <value>`
Read or write a single ArduPilot parameter.

```bash
python calibrate.py --port COM4 getparam ATC_RAT_YAW_P
python calibrate.py --port COM4 setparam SCR_USER6 1
```

---

### `ftp-list`
List files currently on the SD card in `/APM/scripts/` via MAVLink FTP.

```bash
python calibrate.py --port COM4 ftp-list
```

Requires `pymavlink >= 2.4` with the `mavftp` module. If unavailable, use Mission Planner
→ Config → MAVFtp instead.

---

### `ftp-upload <file> [--no-restart]`
Upload a `.lua` file to `/APM/scripts/` via MAVLink FTP, then restart the scripting engine
by toggling `SCR_ENABLE` (no reboot needed). Pass `--no-restart` to skip the restart.

```bash
python calibrate.py --port COM4 ftp-upload simulation/scripts/rawes.lua
python calibrate.py --port COM4 ftp-upload simulation/scripts/rawes.lua --no-restart
```

The remote path is always `/APM/scripts/<basename>`. The scripting restart takes ~1 s;
the Lua script reloads from SD and begins executing immediately.

---

### `ftp-remove <file>`
Remove a file from `/APM/scripts/` via MAVLink FTP.

```bash
python calibrate.py --port COM4 ftp-remove rawes.lua
```

---

### `release [n]` / `restore [n]`
Temporarily set `SERVO{n}_FUNCTION=0` to release output `n` (default: output 4, the
GB4008) from ArduPilot mixing so it can be driven manually with `servo`. `restore`
sets `SERVO{n}_FUNCTION` back to the value saved by the most recent `release`.

```bash
python calibrate.py --port COM4 release          # release output 4
python calibrate.py --port COM4 servo 4 1000     # drive it manually
python calibrate.py --port COM4 restore          # give it back to ArduPilot
```

---

### `ping [baud]`
Scan all COM ports and report which ones respond to a MAVLink heartbeat.
Tries `baud` (default 115200), then lower fallback rates.

```bash
python calibrate.py ping                          # no connection needed
python calibrate.py ping 57600
```

---

## GB4008 motor constants (used in monitor)

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
python calibrate.py --port COM4 config
python calibrate.py --port COM4 config --apply   # if any [DIFF] shown

# 2. Verify live state after reboot
python calibrate.py --port COM4 status

# 3. Swashplate neutral + mixing check
python calibrate.py --port COM4 neutral
python calibrate.py --port COM4 swash 50 0 0    # all servos rise equally?
python calibrate.py --port COM4 swash 0 0 50    # S1 differential?

# 4. Flap deflection measurement — sweep each servo, record angle vs PWM
python calibrate.py --port COM4 sweep 1
python calibrate.py --port COM4 sweep 2
python calibrate.py --port COM4 sweep 3

# 5. Motor spin check
python calibrate.py --port COM4 motor 5
python calibrate.py --port COM4 monitor 10

# 6. Upload updated Lua script
python calibrate.py --port COM4 ftp-upload simulation/scripts/rawes.lua
python calibrate.py --port COM4 ftp-list        # verify file appeared

# 7. Test arm / Lua mode
python calibrate.py --port COM4 mode 1          # set steady mode
python calibrate.py --port COM4 arm 30          # arm for 30 s, watch yaw rate + SERVO4
```
