# DShot Configuration — Pixhawk 6C + REVVitRC AM32 + GB4008

## What is DShot?

DShot (Digital Shot) is a digital ESC protocol that replaces analogue PWM.
Instead of pulse width, it sends a 16-bit digital frame per cycle:
11 bits of throttle value (0–2047), 1 telemetry request bit, 4 CRC bits.

**Bidirectional DShot** (also called EDT — Extended DShot Telemetry) adds a
return path: the ESC sends eRPM data back on the same signal wire between
command frames (half-duplex). ArduPilot uses this for RPM sensing
(`RPM1_TYPE=5`) without a separate telemetry wire.

---

## ArduPilot DShot on Pixhawk 6C — IOMCU vs FMU

The Pixhawk 6C has two output processors:

| Processor | Outputs | DShot support |
|-----------|---------|---------------|
| IOMCU (IO, STM32F1) | MAIN OUT 1–8 | Requires `BRD_IO_DSHOT=1` (ArduPilot 4.5+) |
| FMU (STM32H7) | AUX OUT 1–6 (outputs 9–14) | One-way and bidirectional DShot natively — no extra param needed |

**Motor is wired to AUX OUT 1 (output 9, FMU).** `BRD_IO_DSHOT` is not needed.

---

## Confirmed Working Parameters (ArduPilot 4.6.3, Pixhawk 6C, Rover frame)

Confirmed working 2026-04-17: motor spins at 20% via DO_MOTOR_TEST.

### Frame

| Parameter | Value | Notes |
|-----------|-------|-------|
| `FRAME_CLASS` | 1 | Rover ground frame — required for DO_MOTOR_TEST (Heli frame blocks it) |
| `FRAME_TYPE` | 0 | Default |

### Motor output (AUX OUT 1 = output 9)

| Parameter | Value | Notes |
|-----------|-------|-------|
| `SERVO9_FUNCTION` | 36 | Motor4 — maps AUX OUT 1 to GB4008 |
| `SERVO9_MIN` | 1000 | DShot min (must not be 1100 default — clamps signal) |
| `SERVO9_MAX` | 2000 | DShot max |
| `SERVO9_TRIM` | 1000 | DShot idle |
| `MOT_PWM_TYPE` | 6 | DShot300 — Rover frame uses this instead of `SERVO_BLH_OTYPE` |

### BLHeli / DShot subsystem

| Parameter | Value | Notes |
|-----------|-------|-------|
| `SERVO_BLH_MASK` | 256 | Bit 8 = output 9 — registers AUX OUT 1 with BLHeli subsystem |
| `SERVO_BLH_BDMASK` | 0 | **One-way DShot** — do NOT set to 256 until AM32 EDT is enabled on ESC; bidir without EDT causes bad-frame accumulation and suppresses motor output |
| `SERVO_BLH_OTYPE` | 5 | DShot300 — used by BLHeli passthrough / test features |
| `SERVO_BLH_POLES` | 22 | GB4008 24N22P (11 pole-pairs); default 14 is WRONG |
| `SERVO_BLH_TRATE` | 10 | Telemetry request rate 10 Hz |
| `SERVO_BLH_AUTO` | 0 | Manual mask config |
| `SERVO_DSHOT_ESC` | 3 | ESC type = AM32 (not 1=BLHeli32) |
| `SERVO_DSHOT_RATE` | 0 | 1 kHz command rate (default) |

### Safety / arming

| Parameter | Value | Notes |
|-----------|-------|-------|
| `BRD_SAFETY_DEFLT` | 0 | Safety switch disabled — outputs live immediately on boot |
| `BRD_IO_DSHOT` | 0 | Not needed — motor is on AUX OUT (FMU), not MAIN OUT (IOMCU) |
| `ARMING_CHECK` | 0 | Skip pre-arm checks (bench use) |

### RPM telemetry (NOT YET ACTIVE — requires AM32 EDT)

| Parameter | Value | Notes |
|-----------|-------|-------|
| `RPM1_TYPE` | 0 | Currently disabled — set to 5 after enabling AM32 EDT |
| `SERVO_BLH_BDMASK` | 0 | Currently disabled — set to 256 after enabling AM32 EDT |

---

## RPM index mapping

With `SERVO_BLH_MASK=256` (only output 9 registered), the GB4008 is
BLHeli motor #1 → index 0 in `ESC_TELEMETRY_1_TO_4` → feeds `RPM1`.

If you add more ESCs to the mask, they map to RPM2, RPM3 etc. in
bitmask order (lowest bit first).

---

## Enabling AM32 EDT (to activate RPM telemetry)

The REVVitRC AM32 ESC must have **Extended DShot Telemetry (EDT)** enabled
before bidir DShot is turned on in ArduPilot. Setting `SERVO_BLH_BDMASK=256`
without EDT active causes ArduPilot to accumulate bad frames on the receive
window and eventually suppress the motor output entirely.

**Step 1 — Enable EDT on the ESC (do this first):**
1. Connect Pixhawk to Mission Planner via USB
2. Go to **Optional Hardware → ESC Calibration / BLHeli**
3. Click **Connect** — ArduPilot puts the ESC into BLHeli passthrough mode
4. In the AM32 configurator, enable **Extended Telemetry** (or "Bidirectional DShot")
5. Click **Write**, then power-cycle the ESC (disconnect battery briefly)

**Step 2 — Enable bidir in ArduPilot (only after Step 1):**
```bash
python calibrate.py --port COM4 setparam SERVO_BLH_BDMASK 256
python calibrate.py --port COM4 setparam RPM1_TYPE 5
```

**Step 3 — Verify:**
```bash
python calibrate.py --port COM4 arm
python calibrate.py --port COM4 monitor 10
```

Expected columns: `elapsed  eRPM  motor_RPM  rotor_RPM  current_A  torque_Nm  voltage_V  temp_C`

At idle (minimum throttle): eRPM > 0, current ~0.5–2 A.

---

## ESC Wiring

```
Pixhawk AUX OUT 1 (output 9, 3-pin servo header)
  Pin 1 (S)  ─────────────────────────  ESC signal input
  Pin 2 (+)  (servo rail, not used)     NOT connected to ESC power
  Pin 3 (-)  ─────────────────────────  ESC signal ground (common GND)

4S LiPo battery  ─── XT30 ──────────── ESC power input (separate wire)
```

The ESC gets power from the battery directly (XT30/XT60).
The DShot signal runs on Pin 1 (S) with Pin 3 (-) as reference ground.
Both connections are required — missing the ground wire means no DShot signal.

---

## DShot Arming Behaviour

In DShot mode, the ESC auto-arms from the first valid DShot packet.
No PWM arming sequence is needed. The ESC:
1. Powers up, listens for DShot frames
2. Detects DShot300 from the signal line
3. Arms immediately when it receives a valid command

When ArduPilot is disarmed, it sends DShot value 0 (motor stop command).
When ArduPilot arms, it starts sending throttle-proportional DShot values.

---

## GB4008 Motor Constants

| Constant | Value | Source |
|----------|-------|--------|
| Kv | 66 RPM/V | EMAX spec |
| Pole config | 24N22P = 11 pole-pairs | EMAX spec |
| Gear ratio | 80:44 = 1.818 | Hardware |
| Kt (motor shaft) | 0.144 N·m/A | 60 / (2π × 66) |
| eRPM → motor RPM | ÷ 11 | pole-pairs |
| eRPM → rotor RPM | ÷ 20 | ÷ (11 × 80/44) |

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| Motor worked then stopped, no error | `SERVO_BLH_BDMASK=256` set without AM32 EDT | Set `SERVO_BLH_BDMASK=0`; enable AM32 EDT first before re-enabling |
| Motor silent, no beep | DShot signal not reaching ESC | Check signal wire on AUX OUT 1 pin 1; check common GND on pin 3 |
| Motor test command sent but motor silent | `BRD_SAFETY_DEFLT=1` blocking outputs | Set `BRD_SAFETY_DEFLT=0`, reboot |
| Motor silent, `SERVO9_MIN=1100` | Default min clamps 800 µs signal | Set `SERVO9_MIN=1000` |
| DO_MOTOR_TEST rejected ("Disabled on heli") | ArduPilot heli frame blocks motor test | Switch to Rover frame (`FRAME_CLASS=1`), reboot |
| `RPM1` always 0 | AM32 EDT not enabled on ESC | Enable via BLHeli passthrough in Mission Planner, then set `SERVO_BLH_BDMASK=256`, `RPM1_TYPE=5` |
| `SERVO_BLH_BDSHOT` not found | Removed in ArduPilot 4.6+ | Use `SERVO_BLH_BDMASK` instead |
| No ESC beep on power-up | ESC not receiving DShot or not powered | Check battery connected; check signal + GND wires |

---

## Flight Mode Parameters (ArduPilot Heli Frame, SERVO9_FUNCTION=94)

These parameters apply when the Pixhawk is configured as a helicopter and `rawes.lua`
controls the GB4008 counter-torque motor via `SRV_Channels`. Differs from bench/Rover
config above in frame type, `SERVO9_FUNCTION`, `SERVO9_MIN/TRIM`, and DShot type param.

Confirmed working in SITL stack tests (`test_lua_yaw_trim`, `test_lua_flight_steady`).

| Parameter | Value | Notes |
|-----------|-------|-------|
| `SERVO9_FUNCTION` | 94 | Script 1 — Lua writes GB4008 PWM via `SRV_Channels` (Heli/flight mode) |
| `SERVO9_MIN` | 800 | PWM off — 800 us = motor off; default 1100 clamps signal, motor stays silent |
| `SERVO9_MAX` | 2000 | PWM max |
| `SERVO9_TRIM` | 800 | trim = off (800 us); rawes.lua `_set_throttle_pct(0)` outputs 800 us |
| `SERVO_BLH_MASK` | 256 | Bit 8 = output 9 — registers AUX OUT 1 with BLHeli subsystem |
| `SERVO_BLH_OTYPE` | 5 | DShot300 (Heli/Copter frame uses this; Rover uses `MOT_PWM_TYPE=6`) |
| `SERVO_BLH_POLES` | 22 | GB4008 24N22P (11 pole-pairs) — default 14 is wrong |
| `SERVO_BLH_TRATE` | 10 | Telemetry request rate 10 Hz |
| `SERVO_BLH_AUTO` | 0 | Manual mask config |
| `SERVO_BLH_BDMASK` | 0 | One-way DShot initially — set to 256 only after AM32 EDT is enabled |
| `SERVO_DSHOT_ESC` | 3 | ESC type = AM32 (REVVitRC) |
| `SERVO_DSHOT_RATE` | 0 | 1 kHz command rate (default) |
| `BRD_IO_DSHOT` | 0 | Not needed — motor is on AUX OUT (FMU), not MAIN OUT (IO) |
| `BRD_SAFETY_DEFLT` | 0 | Safety switch disabled — outputs live immediately on boot |
| `RPM1_TYPE` | 0 | Disabled initially — set to 5 after enabling AM32 EDT on ESC |
| `RPM1_MIN` | 0 | No minimum filter |

Use `calibrate.py bench-mode` to switch to `SERVO9_FUNCTION=36` (Rover frame, `MOT_PWM_TYPE=6`)
for bench testing. Use `calibrate.py flight-mode` to restore `SERVO9_FUNCTION=94` (Heli frame).
