# Motor Control — GB4008 + REVVitRC AM32

> **ESC hardware note:** REVVitRC 50A ships with IFlight_50A firmware (`IFLIGHT_F051` target, STM32F051 MCU).
> It was flashed to AM32 as an upgrade. MCU signature 0x1F06, EEPROM at 0x7C00.

## Current Setup: bidirectional DShot300 on AUX 1 (RPM telemetry)

The GB4008 runs **bidirectional DShot300 on AUX 1 (FMU output 9 = SERVO9)** for
RPM-only telemetry (no EDT). See [DShot on the FMU](#dshot-on-the-fmu-aux-outputs)
below for the full parameter set, the timer-group rationale for why the motor
moved off MAIN OUT 4, and the **BIDIR channel constraint** that forced AUX 1
instead of AUX 2.

The **standard-PWM configuration below** (MAIN OUT 4, `H_TAIL_TYPE=3`) remains the
documented fallback. ArduPilot's `ATC_RAT_YAW` PID (`H_TAIL_TYPE=3`, DDFP CW)
drives the motor output directly in any wiring.

**H_TAIL_TYPE** — relevant values for this project:

| Value | Mode | Output behaviour |
|-------|------|-----------------|
| 0 | Servo | Bidirectional servo; SERVO4_TRIM=1500 µs neutral; no sign flip |
| **3** | **DDFP CW** | Unidirectional motor; positive PID → more throttle (NO sign flip) |
| 4 | DDFP CCW | Unidirectional motor; applies ×−1 (sign-flipped path) |

Use **3** (CW) for the GB4008: under the US-convention rotor (CCW from above, see [CLAUDE.md](../CLAUDE.md) "Rotor spin direction"), body drifts CCW (`gyro:z() < 0`) → yaw error = `−gyro:z()` > 0 → positive PID output → positive SERVO4 throttle → motor counters the drift.  Type 4 (CCW with ×−1 flip) would clamp the positive PID to zero, leaving the motor off.

---

## Confirmed Working Parameters (ArduPilot 4.6.3, Pixhawk 6C)

### Flight mode (Heli frame)

| Parameter | Value | Notes |
|-----------|-------|-------|
| `H_TAIL_TYPE` | 3 | DDFP CW (no sign flip) — ArduPilot yaw PID output routed to SERVO4. US-convention rotor: positive PID error (from CCW body drift) → positive throttle. |
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
| Pole config | see SERVO_BLH_POLES | verified against known RPM |
| Gear ratio | 80:44 = 1.818 | Hardware |
| Kt (motor shaft) | 0.144 N·m/A | 60 / (2π × 66) |
| eRPM → motor RPM | ÷ (SERVO_BLH_POLES/2) | pole-pairs |
| eRPM → rotor RPM | ÷ (SERVO_BLH_POLES/2 × 80/44) | apply gear ratio |

---

## Troubleshooting (PWM)

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| Motor silent, no beep | Signal not reaching ESC | Check signal wire on MAIN OUT 4 pin 1; check common GND on pin 3 |
| Motor test command sent but motor silent | `BRD_SAFETY_DEFLT=1` blocking outputs | Set `BRD_SAFETY_DEFLT=0`, reboot |
| Motor silent, `SERVO4_MIN=1100` | Default min clamps low-end signal | Set `SERVO4_MIN=800` (flight) or `SERVO4_MIN=1000` (bench) |
| DO_MOTOR_TEST rejected ("Disabled on heli") | Heli frame blocks motor test | Switch to Rover frame (`FRAME_CLASS=1`), reboot |

---

## DShot on the FMU (AUX outputs)

DShot is a digital ESC protocol that replaces the analog PWM pulse with 16-bit
digital frames. **Bidirectional DShot** streams eRPM back on the same wire (this
is all we use — RPM only). **EDT (Extended DShot Telemetry)** is a *separate*
layer on top that adds temperature/voltage/current; we do NOT enable it.

The GB4008 anti-rotation motor is driven by **bidirectional DShot300 on AUX 1**
(the FMU PWM connector, pin 1), which ArduPilot exposes as **SERVO9 / output 9**.
The swashplate servos stay on MAIN OUT 1–3 (IOMCU, standard PWM).

> **Why AUX 1, not AUX 2?** Bidirectional DShot needs input-capture DMA, which on
> the `Pixhawk6C-bdshot` firmware is only wired on the **odd** FMU timer channels.
> From `hwdef/Pixhawk6C-bdshot/hwdef.dat`, only **AUX 1, 3, 5, 7** carry the
> `BIDIR` flag (TIM1_CH1, TIM1_CH3, TIM4_CH3, TIM5_CH1); AUX 2/4/6/8 are DShot-out
> only and return no telemetry. The motor was moved from AUX 2 → **AUX 1** (same
> TIM1 group, adjacent pin) so RPM telemetry works. **This requires the
> `Pixhawk6C-bdshot` firmware variant** — the stock `Pixhawk6C` build has no
> bidir-DShot support at all.

### Why AUX 2 (FMU) and not MAIN OUT 4 (IOMCU) — the timer-group rule

Every RCOutput channel belongs to a hardware timer, and **all active channels
sharing one timer must use the same output protocol** — you cannot mix a PWM
servo and a DShot motor in the same timer group.

**IOMCU groups (STM32F103, from `hwdef/iomcu/hwdef.inc`):**

| Timer | Channels |
|-------|----------|
| TIM2 | MAIN 1, 2 |
| **TIM4** | **MAIN 3, 4** |
| TIM3 | MAIN 5, 6, 7, 8 |

The swashplate servo S3 sits on **MAIN 3** (PWM) and the motor was on **MAIN 4** —
both in **TIM4**. Enabling DShot on MAIN 4 would force the whole TIM4 group to
DShot and break the PWM servo on MAIN 3. This is the "only one protocol per
timer group" conflict.

**FMU groups (STM32H743, from `hwdef/Pixhawk6C-bdshot/hwdef.dat`):**

| Timer | Channels | Bidir-capable (RPM) |
|-------|----------|---------------------|
| **TIM1** | **AUX 1, 2, 3, 4** (SERVO 9–12) | AUX 1, AUX 3 |
| TIM4 | AUX 5, 6 (SERVO 13–14) | AUX 5 |
| TIM5 | AUX 7, 8 (SERVO 15–16) | AUX 7 |

The motor is on **AUX 1** (TIM1) — a bidir-capable channel whose group-mates
(AUX 2, 3, 4) are unused, so the whole TIM1 group runs DShot with nothing to
conflict. Bonus: **FMU DShot is native — no `BRD_IO_DSHOT`** (that param only
enables DShot on the IOMCU MAIN outputs).

### Output-channel remap (heli tail motor MAIN 4 → AUX 1)

The DDFP tail motor is heli `Motor4` (`SRV_Channel::k_motor4 = 36`). Move it off
SERVO4 onto SERVO9:

| Parameter | Value | Notes |
|-----------|-------|-------|
| `SERVO9_FUNCTION` | 36 | Motor4 (heli DDFP tail) on AUX 1 |
| `SERVO4_FUNCTION` | 0 | Free the old MAIN OUT 4 |
| `SERVO9_MIN` | 1000 | DShot throttle floor (motor off) |
| `SERVO9_MAX` | 2000 | DShot throttle ceiling (full) |
| `SERVO9_TRIM` | 1000 | Idle = off |

### Enable bidirectional DShot — RPM only, no EDT

`SERVO_BLH_OTYPE` overrides the output protocol for the channels selected by
`SERVO_BLH_MASK`; this is the standard recipe for DShot on a non-multicopter-motor
output. The mask is a per-channel bitmask (bit N = channel N+1), so **output 9 =
bit 8 = 256**. Bidirectional DShot returns eRPM with `SERVO_DSHOT_ESC = 1`
(plain AM32) — EDT (`= 3`) is only needed for the *extra* telemetry types.

| Parameter | Value | Notes |
|-----------|-------|-------|
| `SERVO_BLH_MASK` | 256 | Bit 8 = Channel9 — register AUX 1 with the BLHeli/DShot subsystem |
| `SERVO_BLH_OTYPE` | 5 | DShot300 (0:None 1:OneShot 2:OneShot125 3:Brushed 4:DShot150 **5:DShot300** 6:DShot600 7:DShot1200) |
| `SERVO_BLH_AUTO` | 0 | Manual mask config (do not auto-add copter motors) |
| `SERVO_BLH_BDMASK` | 256 | Bidir DShot ON, Channel9 — returns eRPM |
| `SERVO_BLH_POLES` | 22 | poles; eRPM ÷ (SERVO_BLH_POLES/2) = mech RPM. Default 14 is wrong |
| `SERVO_DSHOT_ESC` | 1 | ESC type = **AM32, RPM only, no EDT** (0:None **1:BLHeli32/Kiss/AM32** 2:BLHeli_S/BlueJay 3:AM32/Kiss **+EDT** 4:BLHeli_S/BlueJay+EDT) |
| `SERVO_DSHOT_RATE` | 0 | 1 kHz fixed command rate (safe for the 400 Hz heli loop) |
| `RPM1_TYPE` | 5 | ESC Telemetry Motors Bitmask |
| `RPM1_ESC_MASK` | 256 | Bit 8 = Channel9 — which ESC's eRPM feeds `RPM1` |

> **`SERVO_DSHOT_ESC = 1`, not 3.** Value 3 selects AM32 **with EDT**, which only
> works after EDT firmware config is written to the ESC. For RPM-only we keep `1`.
> (The old note that "3 = AM32" was wrong.)
>
> **Requires the `Pixhawk6C-bdshot` firmware** and bidirectional DShot enabled in
> the AM32 configurator on the ESC. `SERVO_BLH_MASK`, `SERVO_BLH_OTYPE`,
> `SERVO_BLH_BDMASK`, `SERVO_BLH_POLES` are all `@RebootRequired`.

**RPM index mapping:** with only output 9 registered, the GB4008 is the sole
BLHeli motor → its eRPM feeds `RPM1` (selected by `RPM1_ESC_MASK = 256`).

### Later: add EDT (extended telemetry: temp / voltage / current)

Only if the extra fields are wanted on top of RPM:

1. Enable Extended Telemetry in the AM32 configurator (via BLHeli passthrough in
   Mission Planner) and power-cycle the ESC.
2. Set `SERVO_DSHOT_ESC = 3` (AM32 + EDT) and `SERVO_BLH_TRATE = 10`.

> Do NOT switch to EDT firmware config on the ESC without also setting
> `SERVO_DSHOT_ESC = 3` — mismatched EDT state corrupts the telemetry frames.
