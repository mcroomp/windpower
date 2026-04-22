# calibrate.py — Hardware Calibration Tool

`simulation/scripts/calibrate.py` connects to the Pixhawk 6C over USB (or SiK radio)
and provides servo control, motor testing, ESC diagnostics, and Lua script upload —
all over MAVLink, with no arming required.

## Connection

```bash
python simulation/scripts/calibrate.py                      # default COM4 115200, interactive
python simulation/scripts/calibrate.py --port COM3
python simulation/scripts/calibrate.py --port COM4 --baud 57600   # SiK radio
python simulation/scripts/calibrate.py --port COM4 <command>      # non-interactive
```

`--force` / `-f` skips interactive confirmation prompts (safe for scripted use at low throttle).

---

## Output channel mapping

| Output | Component | SERVO_FUNCTION |
|--------|-----------|---------------|
| 1 | S1 — swashplate 0 deg (East) | 33 |
| 2 | S2 — swashplate 120 deg | 34 |
| 3 | S3 — swashplate 240 deg | 35 |
| 4 | GB4008 anti-rotation motor (MAIN OUT 4) | H_TAIL_TYPE=4 (DDFP) |

PWM range: 1000 µs (min) … 1500 µs (neutral) … 2000 µs (max).

---

## Commands

### `status`
Print current `SERVO_OUTPUT_RAW` for all active outputs.

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

### `diag`
Full motor / AM32 / DShot diagnostic dump. Runs automatically through 7 checks:

1. **Parameter audit** — verifies `SERVO4_FUNCTION`, `SERVO_BLH_MASK`, `SERVO_BLH_OTYPE`,
   `H_TAIL_TYPE`, `BRD_SAFETYENABLE`, `H_RSC_MODE`, and other DShot-chain params;
   flags likely misconfigurations with plain-English explanations.
2. **Safety/arming state** — decodes heartbeat: armed, safety switch, mode, system status.
3. **SERVO_OUTPUT_RAW** — what ArduPilot is actually sending to output 4.
4. **Motor test ACK** — sends 5% motor test and reports whether it was accepted or rejected.
5. **ESC telemetry** — listens 3 s for `ESC_TELEMETRY_1_TO_4`; if received, reports:
   eRPM → motor RPM → rotor RPM (via 80:44 gear, 11 pole-pairs),
   current → shaft torque (via Kt = 0.144 N·m/A). If absent, explains why.
6. **STATUSTEXT drain** — captures ArduPilot error/warning messages emitted during the test.
7. **SYS_STATUS motor health bit** — whether ArduPilot considers motor outputs healthy.

```bash
python calibrate.py --port COM4 diag
```

**Common failure causes diagnosed:**
- `SERVO_BLH_MASK` not including bit 3 → DShot not enabled on output 4
- `SERVO_BLH_OTYPE < 4` → PWM mode, not DShot
- `H_TAIL_TYPE != 4` → motor not mapped as DDFP tail
- `BRD_SAFETYENABLE=1` → safety switch not pressed
- `H_RSC_MODE=0` → RSC gating motor output
- No ESC telemetry → ESC not powered, DShot signal wire not connected to output 4,
  or ESC not responding. Note: `SERVO_BLH_BDSHOT` does not exist in ArduPilot 4.6+;
  bidirectional DShot is auto-detected when `SERVO_BLH_OTYPE >= 4`.

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

### `scripts`
List files currently on the SD card in `/APM/scripts/` via MAVLink FTP.

```bash
python calibrate.py --port COM4 scripts
```

Requires `pymavlink >= 2.4` with the `mavftp` module. If unavailable, use Mission Planner
→ Config → MAVFtp instead.

---

### `upload <file> [--no-restart]`
Upload a `.lua` file to `/APM/scripts/` via MAVLink FTP, then restart the scripting engine
by toggling `SCR_ENABLE` (no reboot needed). Pass `--no-restart` to skip the restart.

```bash
python calibrate.py --port COM4 upload simulation/scripts/rawes.lua
python calibrate.py --port COM4 upload simulation/scripts/rawes.lua --no-restart
```

The remote path is always `/APM/scripts/<basename>`. The scripting restart takes ~1 s;
the Lua script reloads from SD and begins executing immediately.

---

## GB4008 motor constants (used in diag / monitor)

| Constant | Value | Source |
|----------|-------|--------|
| Kv | 66 RPM/V | EMAX spec |
| Pole configuration | 24N22P (11 pole-pairs) | EMAX spec |
| Gear ratio | 80:44 = 1.818 | Hardware |
| Kt (motor shaft) | 0.144 N·m/A | Derived: 60/(2π×66) |
| eRPM → motor RPM | ÷ 11 | pole-pairs |
| eRPM → rotor RPM | ÷ 20 | ÷ (11 × 80/44) |

---

### `config`
Set and validate the complete RAWES DShot parameter set, then reboot. Auto-detects
Heli/Copter vs Rover frame and applies the correct values. Covers every DShot-chain
param including `BRD_SAFETY_DEFLT=0` (required — safety switch blocks outputs).

```bash
python calibrate.py --port COM4 config
```

Prints a pass/fail table for every parameter. Reboots on success; reports `[FAIL]` and
skips reboot if any value is wrong.

| Frame | `SERVO9_FUNCTION` | DShot protocol param |
|-------|-------------------|---------------------|
| Heli/Copter | 94 (Script 1 / Lua) | `SERVO_BLH_OTYPE=5` |
| Rover | 36 (Motor4) | `MOT_PWM_TYPE=6` |

Always set: `SERVO9_MIN/MAX/TRIM=1000/2000/1000`, `SERVO_BLH_MASK=256`,
`SERVO_BLH_POLES=22`, `SERVO_BLH_BDMASK=0`, `SERVO_DSHOT_ESC=3`,
`BRD_IO_DSHOT=0`, `BRD_SAFETY_DEFLT=0`, `ARMING_CHECK=0`.

---

## Typical calibration sequence

```bash
# 0. Apply full DShot configuration (first time or after EEPROM wipe)
python calibrate.py --port COM4 config

# 1. Verify parameter chain
python calibrate.py --port COM4 diag

# 2. Swashplate neutral + mixing check
python calibrate.py --port COM4 neutral
python calibrate.py --port COM4 swash 50 0 0    # all servos rise equally?
python calibrate.py --port COM4 swash 0 0 50    # S1 differential?

# 3. Flap deflection measurement — sweep each servo, record angle vs PWM
python calibrate.py --port COM4 sweep 1
python calibrate.py --port COM4 sweep 2
python calibrate.py --port COM4 sweep 3

# 4. Motor spin check
python calibrate.py --port COM4 motor 5
python calibrate.py --port COM4 monitor 10

# 5. Upload updated Lua script
python calibrate.py --port COM4 upload simulation/scripts/rawes.lua
python calibrate.py --port COM4 scripts         # verify file appeared
```
