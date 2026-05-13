# Motor Control — GB4008 + REVVitRC AM32

> **ESC hardware note:** REVVitRC 50A ships with IFlight_50A firmware (`IFLIGHT_F051` target, STM32F051 MCU).
> It was flashed to AM32 as an upgrade. MCU signature 0x1F06, EEPROM at 0x7C00.

## Current Setup: Standard PWM

The GB4008 is currently controlled via **standard PWM** on **MAIN OUT 4 (output 4, IOMCU)**.
ArduPilot's `ATC_RAT_YAW` PID (`H_TAIL_TYPE=4`, DDFP CCW) drives SERVO4 directly.
No DShot, no BLHeli subsystem, no RPM telemetry is active.

**H_TAIL_TYPE** — relevant values for this project:

| Value | Mode | Output behaviour |
|-------|------|-----------------|
| 0 | Servo | Bidirectional servo; SERVO4_TRIM=1500 µs neutral; no sign flip |
| 3 | DDFP CW | Unidirectional motor, CW; positive PID → more throttle |
| **4** | **DDFP CCW** | Unidirectional motor, CCW; applies ×−1 so CW-drift (neg PID) → more throttle |

Use **4** (CCW) for the GB4008: CW hub drift produces a negative yaw-error PID output; the CCW sign flip converts that to positive throttle — GB4008 spins up and counters the drift.  Type 3 (CW) would clamp the negative PID to zero, leaving the motor off.

---

## Confirmed Working Parameters (ArduPilot 4.6.3, Pixhawk 6C)

### Flight mode (Heli frame)

| Parameter | Value | Notes |
|-----------|-------|-------|
| `H_TAIL_TYPE` | 4 | DDFP CCW — ArduPilot yaw PID output routed to SERVO4 |
| `SERVO4_MIN` | 800 | Motor off at 800 µs |
| `SERVO4_MAX` | 2000 | Motor full throttle at 2000 µs |
| `SERVO4_TRIM` | 800 | Trim = off (motor off at neutral stick) |
| `BRD_SAFETY_DEFLT` | 0 | Safety switch disabled — outputs live immediately on boot |
| `RPM1_TYPE` | 0 | Disabled — no ESC telemetry with PWM |

### Bench / motor-test mode (Rover frame)

Switch to Rover frame (`FRAME_CLASS=1`) to use `DO_MOTOR_TEST` (heli frame blocks it).

| Parameter | Value | Notes |
|-----------|-------|-------|
| `FRAME_CLASS` | 1 | Rover ground frame — required for DO_MOTOR_TEST |
| `SERVO4_FUNCTION` | 36 | Motor4 — maps MAIN OUT 4 to GB4008 |
| `SERVO4_MIN` | 1000 | PWM min |
| `SERVO4_MAX` | 2000 | PWM max |
| `SERVO4_TRIM` | 1000 | PWM idle |
| `BRD_SAFETY_DEFLT` | 0 | Safety switch disabled |
| `ARMING_CHECK` | 0 | Skip pre-arm checks (bench use) |

---

## ESC Wiring

```
Pixhawk MAIN OUT 4 (output 4, 3-pin servo header)
  Pin 1 (S)  ─────────────────────────  ESC signal input
  Pin 2 (+)  (servo rail, not used)     NOT connected to ESC power
  Pin 3 (-)  ─────────────────────────  ESC signal ground (common GND)

4S LiPo battery  ─── XT30 ──────────── ESC power input (separate wire)
```

Both signal (Pin 1) and ground (Pin 3) connections are required.

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

## Troubleshooting (PWM)

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| Motor silent, no beep | Signal not reaching ESC | Check signal wire on MAIN OUT 4 pin 1; check common GND on pin 3 |
| Motor test command sent but motor silent | `BRD_SAFETY_DEFLT=1` blocking outputs | Set `BRD_SAFETY_DEFLT=0`, reboot |
| Motor silent, `SERVO4_MIN=1100` | Default min clamps low-end signal | Set `SERVO4_MIN=800` (flight) or `SERVO4_MIN=1000` (bench) |
| DO_MOTOR_TEST rejected ("Disabled on heli") | Heli frame blocks motor test | Switch to Rover frame (`FRAME_CLASS=1`), reboot |

---

## Future: DShot (not currently active)

DShot is a digital ESC protocol that replaces PWM with 16-bit digital frames.
**Bidirectional DShot (EDT)** adds RPM telemetry back on the same wire.

The REVVitRC AM32 ESC supports DShot300 and EDT. The hardware is ready; only
software configuration is needed to enable it.

### ArduPilot DShot on Pixhawk 6C — IOMCU vs FMU

| Processor | Outputs | DShot support |
|-----------|---------|---------------|
| IOMCU (IO, STM32F1) | MAIN OUT 1–8 | Requires `BRD_IO_DSHOT=1` (ArduPilot 4.5+) |
| FMU (STM32H7) | AUX OUT 1–6 (outputs 9–14) | Native DShot — no extra param needed |

MAIN OUT 4 (IOMCU) requires `BRD_IO_DSHOT=1` to enable DShot.

### Steps to enable DShot (when ready)

**Step 1 — Enable EDT on the ESC first:**
1. Connect Pixhawk to Mission Planner via USB
2. Go to **Optional Hardware → ESC Calibration / BLHeli**
3. Click **Connect** — ArduPilot puts the ESC into BLHeli passthrough mode
4. In the AM32 configurator, enable **Extended Telemetry** (or "Bidirectional DShot")
5. Click **Write**, then power-cycle the ESC

**Step 2 — Enable DShot in ArduPilot:**

| Parameter | Value | Notes |
|-----------|-------|-------|
| `BRD_IO_DSHOT` | 1 | Enable DShot on IOMCU (MAIN OUT) — reboot required |
| `SERVO_BLH_MASK` | 8 | Bit 3 = output 4 — register MAIN OUT 4 with BLHeli subsystem |
| `SERVO_BLH_OTYPE` | 5 | DShot300 |
| `SERVO_BLH_POLES` | 22 | GB4008 24N22P (11 pole-pairs); default 14 is wrong |
| `SERVO_BLH_TRATE` | 10 | Telemetry request rate 10 Hz |
| `SERVO_BLH_AUTO` | 0 | Manual mask config |
| `SERVO_DSHOT_ESC` | 3 | ESC type = AM32 (REVVitRC) |
| `SERVO_DSHOT_RATE` | 0 | 1 kHz command rate (default) |

**Step 3 — Enable bidir RPM telemetry (only after EDT enabled on ESC):**
```bash
python calibrate.py --port COM4 setparam SERVO_BLH_BDMASK 8
python calibrate.py --port COM4 setparam RPM1_TYPE 5
```

> Do NOT set `SERVO_BLH_BDMASK=8` before EDT is active on the ESC — ArduPilot
> will accumulate bad frames and suppress motor output entirely.

**Step 4 — Verify:**
```bash
python calibrate.py --port COM4 arm
python calibrate.py --port COM4 monitor 10
```
Expected: `elapsed  eRPM  motor_RPM  rotor_RPM  current_A  torque_Nm  voltage_V  temp_C`

### RPM index mapping (when DShot active)

With `SERVO_BLH_MASK=8` (only output 4 registered), the GB4008 is
BLHeli motor #1 → index 0 in `ESC_TELEMETRY_1_TO_4` → feeds `RPM1`.
