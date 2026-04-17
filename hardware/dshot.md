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
| IOMCU (IO, STM32F1) | MAIN OUT 1–8 | One-way **and bidirectional** DShot — requires `BRD_IO_DSHOT=1` (ArduPilot 4.5+) |
| FMU (STM32H7) | AUX OUT 1–6 (outputs 9–14) | One-way and bidirectional DShot natively |

**Key point:** ArduPilot 4.5 added bidirectional DShot support to the IOMCU on Pixhawk 6C.
`BRD_IO_DSHOT=1` enables this. MAIN OUT 4 supports bidirectional DShot with `SERVO_BLH_BDMASK=8`.
Moving to AUX OUT is NOT required for telemetry on Pixhawk 6C 4.5+.

Source: ArduPilot docs (common-dshot-escs.html) + ArduPilot Discourse
"Bi-directional DShot on IOMCU - it's here!" (4.5 release discussion).

---

## Confirmed Working Parameters (ArduPilot 4.6.3, Pixhawk 6C)

| Parameter | Value | Notes |
|-----------|-------|-------|
| `BRD_IO_DSHOT` | 1 | Enable DShot on IO processor — critical |
| `SERVO_BLH_MASK` | 8 | Bit 3 = output 4 — enables DShot on MAIN OUT 4 |
| `SERVO_BLH_BDMASK` | 8 | Bit 3 = output 4 — bidirectional DShot on MAIN OUT 4 |
| `SERVO_BLH_OTYPE` | 5 | DShot300 protocol |
| `SERVO_BLH_POLES` | 22 | GB4008 24N22P motor (11 pole-pairs); default 14 is WRONG |
| `SERVO_BLH_AUTO` | 1 | Auto-configure BLHeli outputs |
| `SERVO_BLH_TRATE` | 10 | Telemetry request rate 10 Hz |
| `SERVO_DSHOT_ESC` | 1 | ESC type = BLHeli32 / Kiss / AM32 |
| `SERVO_DSHOT_RATE` | 0 | 1 kHz command rate (default) |
| `RPM1_TYPE` | 5 | RPM sensor = ESC telemetry (bidirectional DShot) |
| `SERVO4_FUNCTION` | 36 | DDFP tail — maps output 4 to GB4008 yaw motor |

Note: `SERVO_BLH_BDSHOT` does not exist in ArduPilot 4.6+. Bidirectional DShot
is controlled by `SERVO_BLH_BDMASK` only.

---

## AM32 ESC — Extended Telemetry (EDT)

The REVVitRC AM32 ESC must have **Extended DShot Telemetry (EDT)** enabled for
ArduPilot to receive RPM back over the bidirectional DShot wire.

AM32 ESCs do NOT always have EDT enabled by default. It must be configured via
the ESC's firmware configuration tool.

### How to enable EDT on AM32 via ArduPilot BLHeli Passthrough

ArduPilot has a BLHeli passthrough feature that lets you configure the ESC over
the DShot signal wire without opening the ESC or using a separate USB programmer.

**Steps:**
1. Connect Pixhawk to Mission Planner via USB
2. Go to **Optional Hardware → ESC Calibration / BLHeli**
3. Click **Connect** — ArduPilot puts the ESC into passthrough mode
4. The AM32 configurator (or BLHeli32 Suite) will enumerate the ESC
5. Enable **Extended Telemetry** (or "Bidirectional DShot") in the ESC settings
6. Write settings, disconnect passthrough

After EDT is enabled on the AM32 firmware, `ESC_TELEMETRY_1_TO_4` MAVLink
messages will appear and `RPM1` will report actual motor eRPM.

### Verifying telemetry is working

```bash
# After arming with battery connected:
python calibrate.py --port COM4 monitor 10
```

Expected output columns: `elapsed  eRPM  motor_RPM  rotor_RPM  current_A  torque_Nm  voltage_V  temp_C`

At idle (motor running at minimum throttle): eRPM > 0, current ~ 0.5-2 A.

---

## ESC Wiring

```
Pixhawk MAIN OUT 4 (3-pin servo header)
  Pin 1 (S)  ─────────────────────────  ESC signal input
  Pin 2 (+)  (servo rail 8.5 V)         NOT connected to ESC power
  Pin 3 (-)  ─────────────────────────  ESC signal ground (common GND)

4S LiPo battery  ─── XT30 ──────────── ESC power input (separate wire)
```

The ESC gets power from the battery directly (XT30/XT60).
The DShot signal runs on Pin 1 (S) with Pin 3 (-) as reference ground.
Both connections are required — missing the ground wire means no DShot signal.

---

## DShot Arming Behaviour

In DShot mode, the ESC auto-arms from the first valid DShot packet.
**No PWM arming sequence** (1000 us hold) is needed. The ESC:
1. Powers up, listens for DShot frames
2. Detects DShot300 from the signal line
3. Arms immediately when it receives a valid command

When ArduPilot is disarmed, it sends DShot value 0 (motor stop command).
The ESC is powered and listening, but the motor does not spin.
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
| ESC beeps on power-up but no telemetry | AM32 EDT not enabled | Enable via BLHeli passthrough in Mission Planner |
| No ESC beep, motor silent | DShot signal not reaching ESC | Check signal wire on MAIN OUT 4 pin 1; check common ground on pin 3 |
| ESC beeps but motor doesn't spin when armed | `H_RSC_MODE` or DDFP gating | Check `H_TAIL_TYPE=4`, `H_RSC_MODE=1`, RSC runup timer |
| `SERVO_BLH_BDSHOT` not found | Removed in ArduPilot 4.6+ | Use `SERVO_BLH_BDMASK=8` instead |
| `RPM1` always 0 | EDT not enabled on ESC, or `RPM1_TYPE != 5` | Enable AM32 EDT; verify `RPM1_TYPE=5` |
| Motor test rejected ("Disabled on heli") | ArduPilot heli frame blocks motor test | Use `calibrate.py arm` + `hold` or `monitor` instead |
