# ArduPilot Implementation — RAWES Integration Notes

## Overview

ArduPilot's Traditional Helicopter frame is the closest match for RAWES. The swashplate hardware interface is mechanically compatible, but the RAWES control architecture, rotor type, and tethered flight dynamics differ from a standard helicopter in ways that require careful integration planning.

---

## What Works Transparently

ArduPilot outputs collective + cyclic as PWM to S1/S2/S3, which tilt the swashplate. It does not know or care what is downstream. The 3-servo swashplate interface is identical to a standard helicopter — ArduPilot's swashplate mixer (H3-120 or H4-90 type) can drive the RAWES swashplate directly.

---

## Where the Helicopter Model Breaks Down

### 1. Kaman Flap Adds Inner-Loop Lag

In a standard helicopter:
```
swashplate moves → blade pitch changes immediately
```

In RAWES:
```
swashplate moves → flap deflects → aerodynamic moment builds
  → blade elastically twists → pitch changes
```

This is a second-order dynamic with its own time constant. The flap controller PID (Weyel 2025) exists specifically to manage this inner loop. ArduPilot's attitude PID sees additional phase lag in the response and will need significant detuning — if naively configured it will oscillate.

### 2. Rotor Speed Controller (RSC) Assumption Is Wrong

ArduPilot's RSC assumes a *powered* rotor and commands throttle to hold RPM. RAWES rotor is wind-driven: RPM is determined by wind speed and blade pitch, not a motor. Options:

- Set RSC to passthrough or autorotation mode
- Repurpose the RSC motor output for the GB4008 counter-torque motor (which does need closed-loop speed control, just not for rotor RPM)

### 3. Cyclic Phase Angle May Be Off

ArduPilot applies a fixed 90° phase shift for gyroscopic precession (standard helicopter assumption). For RAWES, the effective phase between swashplate input and disk tilt response may differ due to:

- Flap lag time constant
- Blade elastic compliance
- Wind-driven (not powered) rotor dynamics

If the phase is incorrect, cyclic inputs will couple into the wrong axis. This needs to be measured empirically or derived from the simulation model.

### 4. Tether Changes Vehicle Dynamics Entirely

ArduPilot models a free-flying vehicle. RAWES is tethered:

- Tether tension acts as a position constraint and restoring moment
- Attitude response is fundamentally different from a free-flying helicopter
- ArduPilot's position and velocity controllers will fight the tether

The tether force is not modeled in the current simulation (only approximated as a stabilizing moment). This is a known gap — see CLAUDE.md model limitations.

---

## Control Architecture

The RAWES control hierarchy has three levels. ArduPilot maps to the outermost level only:

```
ArduPilot (trajectory / attitude commands)        ← level A
        ↓
  Flap controller — Weyel PID                     ← level B
  (reference blade pitch → swashplate PWM)
        ↓
  RAWES physics (tethered, wind-driven rotor)      ← level C
```

ArduPilot has no visibility into levels B or C. The flap controller must run as a separate inner loop.

---

## Integration Approaches

### Option 1 — ArduPilot helicopter frame, heavy PID tuning
Use ArduPilot Traditional Helicopter frame and tune attitude PIDs to absorb the flap lag.

| Pros | Cons |
|------|------|
| Lowest implementation effort | Phase margin will be poor — fragile |
| No custom code | Cyclic phase error unaddressed |
| Standard GCS/RC support | RSC repurposing is awkward |

Suitable for: initial bench testing and swashplate servo verification only.

### Option 2 — Companion computer intercepts attitude commands *(recommended first step)*
ArduPilot handles RC input, IMU fusion, failsafes, and servo PWM output. A companion computer (Raspberry Pi / Jetson) intercepts attitude commands via MAVLink, runs the Weyel flap controller, and forwards corrected swashplate commands back to ArduPilot via MAVLink servo override or a custom MAVROS node.

```
RC transmitter
    ↓
ArduPilot (IMU, RC, failsafes, MAVLink)
    ↓ attitude setpoint (MAVLink)
Companion computer
    ↓ runs Weyel flap controller (Python/C++)
    ↓ servo override (MAVLink)
ArduPilot
    ↓ PWM
S1, S2, S3 → swashplate
```

| Pros | Cons |
|------|------|
| Clean separation of concerns | Added latency in the control loop |
| Flap controller in Python (matches thesis code) | Companion computer adds weight/complexity |
| ArduPilot retains failsafe authority | MAVLink servo override rate-limited (~50 Hz) |
| No ArduPilot firmware modification | |

### Option 3 — Custom ArduPilot firmware with flap controller as inner loop
Implement the Weyel flap controller directly inside ArduPilot as a new frame type or library module. ArduPilot natively outputs reference pitch commands that the flap controller converts to swashplate PWM at full loop rate.

| Pros | Cons |
|------|------|
| Tightest integration, lowest latency | High implementation complexity |
| Full ArduPilot feature set | Requires C++ firmware development |
| Proper solution for production use | Long iteration cycle |

---

## Recommended Implementation Path

1. **Phase 1 — Swashplate verification (Option 1)**
   Configure ArduPilot as a 3-servo helicopter (H3-120 or H4-90 swashplate). Verify servo directions, travel limits, and collective/cyclic mixing on the bench. Confirm DS113MG response and UBEC sizing under load.

2. **Phase 2 — Flap controller integration (Option 2)**
   Port Weyel thesis Python flap controller to run on a companion computer. Use MAVLink COMMAND_LONG or HIL_ACTUATOR_CONTROLS to inject corrected swashplate commands. Test with rotor on ground at low RPM.

3. **Phase 3 — SITL validation**
   Run ArduPilot SITL with a custom RAWES dynamics plugin (based on the thesis simulation model). Validate attitude response and cyclic phase before flying.

4. **Phase 4 — Custom firmware (Option 3)**
   Once the control architecture is validated in SITL, port the flap controller into ArduPilot firmware for production use.

---

## ArduPilot Configuration Starting Points

### Swashplate and RSC

| Parameter | Value | Reason |
|-----------|-------|--------|
| `FRAME_CLASS` | 11 (Heli) | Traditional helicopter frame |
| `H_SWASH_TYPE` | H4-90 | 4-blade, 90° blade spacing — closest match |
| `H_COL_MAX` | TBD | Limit collective to keep flap loads in linear regime |
| `H_CYC_MAX` | TBD | Limit cyclic amplitude to ≤15° rotor tilt |
| `H_RSC_MODE` | 2 (passthrough) | Wind-driven rotor — no RSC throttle control needed |
| `H_PHANG` | TBD | Cyclic phase angle — needs empirical tuning |
| `SERVO1_FUNCTION` | 33 (Motor1 / S1) | Swashplate servo S1 |
| `SERVO2_FUNCTION` | 34 (Motor2 / S2) | Swashplate servo S2 |
| `SERVO3_FUNCTION` | 35 (Motor3 / S3) | Swashplate servo S3 |
| `SERVOx_MIN/MAX` | TBD | Set to prevent servos reaching mechanical stops |
| `ATC_RAT_RLL_P/I/D` | TBD | Detune significantly to absorb flap lag |
| `ATC_RAT_PIT_P/I/D` | TBD | Same |

> `H_SWASH_TYPE` = H4-90 assumes 4 push-rods at 90° spacing driven by 3 servos. Verify this matches the actual swashplate geometry — may need H3-120 if the lower ring uses a 3-point 120° servo arrangement driving 4 push-rods.

### GB4008 Anti-Rotation Motor (Tail Rotor)

The GB4008 counter-torque motor maps directly onto ArduPilot's DDFP (Direct Drive Fixed Pitch) tail rotor type. ArduPilot's yaw rate controller outputs to the tail channel, which modulates GB4008 motor speed to hold world-frame yaw rate = 0. This is the correct setpoint: the tether runs down the axle, so any world-frame rotation of the electronics assembly would twist it.

The Pixhawk IMU sits on the electronics assembly (the non-rotating part), so it directly measures what the yaw controller needs to regulate — no separate sensor required.

| Parameter | Value | Reason |
|-----------|-------|--------|
| `H_TAIL_TYPE` | 3 or 4 | DDFP CW or CCW — match to actual rotor spin direction (TBD) |
| `SERVO4_FUNCTION` | 36 (Motor4) | Assigns output 4 to the DDFP tail motor (GB4008 ESC) |
| `SERVO4_MIN` | 1000 | ESC minimum PWM |
| `SERVO4_MAX` | 2000 | ESC maximum PWM |
| `SERVO4_TRIM` | ~1150 | ESC idle PWM — base speed before yaw modulation; tune to provide steady counter-torque at cruise rotor RPM |
| `H_TAIL_SPEED` | 0.5–0.7 | Normalised baseline motor speed (0–1); start at 0.5, increase if motor cannot hold against bearing drag at operating RPM |
| `H_DDFP_THST_EXPO` | 0.65 | Thrust curve exponent — linearises motor torque response |
| `H_COL2YAW` | TBD | Collective-to-yaw feedforward: when collective changes, rotor drag torque changes and GB4008 must compensate; tune to minimise yaw kick during collective inputs |
| `ATC_RAT_YAW_P` | 0.20 | Starting value — increase if electronics drift, decrease if yaw wags |
| `ATC_RAT_YAW_I` | 0.05 | Absorbs steady-state bearing friction load |
| `ATC_RAT_YAW_D` | 0.0 | Start at zero; add only if underdamped oscillation appears |
| `ATC_RAT_YAW_FF` | 0.0 | Feedforward — tune from log data after initial hover |
| `ATC_ANG_YAW_P` | ≥3.5 | Angle controller — keep at default minimum |

**Tuning notes:**

- `SERVO4_TRIM` sets the idle torque. At operating rotor RPM the bearing drag is roughly constant, so the integrator (`ATC_RAT_YAW_I`) will wind up to that level. Setting trim near the steady-state operating point reduces how much integrator wind-up is needed and improves transient response.
- The cyclic push-rod reactions create a periodic disturbance at blade-pass frequency (~18 Hz at 28 rad/s with 4 blades). The attitude loop runs at 400 Hz so bandwidth is not a concern, but avoid high `ATC_RAT_YAW_D` which would amplify this high-frequency content.
- `H_COL2YAW` is the most important feedforward to tune early: every collective step changes rotor drag torque, which immediately disturbs electronics yaw. A well-tuned feedforward eliminates this coupling before the PID has to react.

---

## Open Items

- [ ] Determine correct `H_SWASH_TYPE` for 3-servo lower ring driving 4 push-rods
- [ ] Confirm rotor spin direction (CW or CCW from above) to set `H_TAIL_TYPE` = 3 or 4
- [ ] Determine `H_PHANG` — measure or simulate cyclic phase lag from flap dynamics
- [ ] Tune `SERVO4_TRIM` and `H_TAIL_SPEED` against measured bearing friction at operating RPM
- [ ] Tune `H_COL2YAW` to suppress yaw kick during collective changes
- [ ] Decide companion computer hardware (weight budget, latency requirements)
- [ ] Port Weyel Python flap controller to companion computer
- [ ] Design MAVLink interface between ArduPilot and flap controller

---

## References

- Weyel 2025 — flap controller PID, state space model, simulation parameters
- [physical_design.md](physical_design.md) — swashplate geometry, servo specs, power architecture
- [flapmodel.md](flapmodel.md) — full mathematical model
- ArduPilot Traditional Helicopter docs — https://ardupilot.org/copter/docs/traditional-helicopter-connecting-apm.html
- ArduPilot Tail Rotor Setup — https://ardupilot.org/copter/docs/traditional-helicopter-tailrotor-setup.html
- ArduPilot helicopter parameters — https://ardupilot.org/copter/docs/parameters-Copter.html
