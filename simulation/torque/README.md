# Counter-Torque Motor Simulation

Isolated simulation and ArduPilot stack test for the RAWES anti-rotation motor system.

## Physical Background

### The Problem

The RAWES rotor (blades + outer hub shell) spins freely in autorotation driven by wind.
The *stationary* inner assembly — flight controller, battery, servo linkages — is mounted
on bearings inside the spinning shell.  The inner assembly must maintain a fixed heading
while the outer shell spins; otherwise the tether wraps and heading control is lost.

### The Actuator

**Motor:** EMAX GB4008, 66 KV, 90T, 24N22P brushless gimbal motor (hollow shaft)
**ESC:**   REVVitRC 50A AM32, 3–6S, servo PWM / DSHOT, firmware AM32

The motor stator is fixed to the stationary inner assembly.  The motor rotor is geared to the
spinning outer rotor hub via an **80:44 spur gear** (hub side has 80 teeth; motor pinion has
44 teeth).  The motor therefore spins at `80/44 ≈ 1.82×` the rotor hub speed.

Because the motor is geared to the spinning rotor hub, driving the motor produces a
counter-torque on the inner assembly that cancels the bearing drag, keeping the inner assembly
at a fixed heading.  The work the motor does is overcoming bearing and swashplate friction —
in a frictionless system the inner assembly would stay stationary on its own inertia.

### Motor Model

The GB4008 behaves as a standard DC motor:

```
τ = τ_stall × max(0, throttle − ω_motor / ω₀)

τ_stall  = V_bat / (KV_rad × R)
         = 15.2 / (6.91 × 7.5) ≈ 0.293 N·m   (at full throttle, zero speed)

ω₀       = KV_rad × V_bat = 6.91 × 15.2 ≈ 105 rad/s   (no-load speed at full throttle)
```

The motor only produces torque when `throttle > ω_motor / ω₀` (above the back-EMF
threshold).  At nominal axle speed (28 rad/s), `ω_motor ≈ 51 rad/s` and the back-EMF
threshold is `51/105 ≈ 48.5%`.  **Equilibrium throttle is ≈ 75%.**

### Hub Yaw Dynamics

```
I_hub × ψ̈ = Q_bearing + Q_motor

Q_bearing = k_bearing × (ω_axle − ψ̇)           [viscous bearing drag]
Q_motor   = −(80/44) × τ_motor(throttle, ω_rel)  [reaction on hub, maintains heading]
ω_rel     = (ω_axle − ψ̇) × (80/44)              [motor speed relative to hub]
```

Default parameters:

| Symbol        | Value          | Source                         |
|---------------|----------------|--------------------------------|
| `I_hub`       | 0.007 kg·m²    | ~1 kg stationary assembly, r≈0.12 m |
| `k_bearing`   | 0.005 N·m·s/rad| Tunable; start here           |
| `τ_stall`     | 0.293 N·m      | From hardware specs (R=7.5 Ω) |
| `ω₀`          | 105 rad/s      | KV × V_bat                    |
| Gear ratio    | 80/44 ≈ 1.818  | Motor faster than axle         |

### Gear Efficiency Analysis

At nominal operating point (ω_axle = 28 rad/s, k_bearing = 0.005):

| Gear ratio | Motor speed % | Throttle | Current | Input power | Efficiency |
|-----------|---------------|----------|---------|-------------|------------|
| 1.0  (no gear) | 26.7% | 74.4% | 0.97 A | 10.9 W | 35.8% |
| 1.818 (80:44)  | 48.5% | 74.7% | 0.53 A |  6.0 W | 64.9% |
| 3.191 (max)    | 85.0% | 100%  | 0.30 A |  4.6 W | 85.0% |

The current 80:44 gear gives 65% efficiency with 25% throttle headroom for disturbances.
A higher ratio (e.g. 3:1) would give 80%+ efficiency but leave almost no headroom.

---

## ArduPilot Integration

### Control Mode

**ACRO mode** with neutral yaw stick.  ACRO commands a desired yaw *rate* (zero with
neutral stick).  The yaw rate PID outputs a tail-rotor command (Ch4) to null the sensed
yaw rate.

**H_TAIL_TYPE = 0** (servo-type output): Ch4 output is symmetric around neutral PWM
(1500 µs), which the mediator maps to the equilibrium throttle (74.7%).  This means
ArduPilot's PID only needs to correct *deviations* from equilibrium — no integrator
wind-up required for steady-state torque balance.

**Biased throttle mapping:**
```
pwm ≤ 1500 µs:  throttle = trim × (pwm − 1000) / 500
pwm > 1500 µs:  throttle = trim + (1 − trim) × (pwm − 1500) / 500
trim = equilibrium_throttle(ω_axle, params) ≈ 0.747
```

### EKF Initialization (Startup Hold)

The mediator runs a 10-second startup phase before enabling hub dynamics:

1. Sends a slow 5°/s yaw spin to give the EKF gyro and compass data
2. EKF aligns tilt + yaw (compass-based) in ≈4 s
3. Hub is locked at ψ = 0 during startup (no dynamics)
4. Hub psi at startup end is preserved to avoid attitude discontinuity
5. Dynamics phase begins: motor counter-rotation + bearing friction active

### Key Parameters

| Parameter           | Value   | Purpose                              |
|---------------------|---------|--------------------------------------|
| `ARMING_SKIPCHK`    | 0xFFFF  | Skip all pre-arm checks              |
| `H_RSC_MODE`        | 1       | CH8 passthrough — instant runup      |
| `H_TAIL_TYPE`       | 0       | Servo output, symmetric ±1500 µs     |
| `H_COL2YAW`         | 0       | No collective→yaw feedforward        |
| `ATC_RAT_YAW_P`     | 0.001   | Very small P — trim does steady-state|
| `ATC_RAT_YAW_I/D`   | 0       | No integrator (trim handles it)      |
| `EK3_SRC1_YAW`      | 1       | Compass yaw source                   |
| `COMPASS_USE`        | 1       | Compass enabled                      |

---

## Test Suite

All stack tests require Docker (same ArduPilot SITL as main tests).
Unit tests run natively on Windows with the existing venv.

### Running

```sh
# Unit tests (Windows native, no Docker)
simulation/tests/unit/.venv/Scripts/python.exe -m pytest simulation/torque/test_model.py -v

# All stack tests — counter-torque tests are unified with flight stack tests
bash sim.sh test-stack -v

# Counter-torque tests only
bash sim.sh test-stack -v -k torque_armed

# Single stack test
bash sim.sh test-stack -v -k test_yaw_regulation
```

### Test Files

| File | Profile | Description | Pass criterion |
|------|---------|-------------|----------------|
| `test_model.py`         | — | 9 unit tests for `model.py` (no SITL) | All pass |
| `test_yaw_regulation.py`| `constant` | Nominal RPM, hold yaw steady | `|ψ̇| < 1°/s` after 40 s |
| `test_slow_rpm.py`      | `slow_vary` | ±18% RPM at 0.05 Hz (20 s period) | `|ψ̇| < 2°/s` after 40 s |
| `test_fast_rpm.py`      | `fast_vary` | ±18% RPM at 0.25 Hz (4 s period) | `|ψ̇| < 3°/s` after 40 s |
| `test_gust_recovery.py` | `gust` | 20% RPM spike at t=10 s for 5 s | `|ψ̇| < 2°/s` after 40 s |
| `test_pitch_roll.py`    | `pitch_roll` | Hub tilts ±5°/±3° at const RPM | `|ψ̇| < 2°/s` after 40 s |
| `test_lua_yaw_trim.py`  | `constant` | **Lua feedforward running inside SITL** | `|ψ̇| < 1°/s` after 40 s |

### Profiles (mediator_torque.py --profile)

| Name        | ω_axle(t)                              | Hub tilt                  |
|-------------|----------------------------------------|---------------------------|
| `constant`  | 28 rad/s constant                      | Flat                      |
| `slow_vary` | 28 + 5·sin(2π·0.05·t)                 | Flat                      |
| `fast_vary` | 28 + 5·sin(2π·0.25·t)                 | Flat                      |
| `gust`      | 28 (nominal); ×1.20 for t∈[10, 15] s | Flat (20% — motor authority limit) |
| `pitch_roll`| 28 rad/s constant                      | roll ±5°@0.08 Hz, pitch ±3°@0.05 Hz |

---

## Visualiser

```sh
# Auto-discovers all torque_telemetry*.json in simulation/logs/
simulation/tests/unit/.venv/Scripts/python.exe simulation/torque/visualize_torque.py

# Specific file
simulation/tests/unit/.venv/Scripts/python.exe simulation/torque/visualize_torque.py \
    simulation/logs/torque_telemetry_slow_vary.json
```

**Controls:** `Space` play/pause · `←/→` step · `+/-` speed · `N`/`B` next/prev file · drag orbit

**Scene elements:**
- Hub cylinder with alternating black/white top quadrants (rotate with ψ; tilts for pitch_roll test)
- Stationary axle rod with 4-blade rotor indicator spinning at ω_rotor (speed varies for slow/fast/gust)
- Amber yaw pointer showing hub heading
- Green throttle bar with yellow equilibrium mark at 74.7%
- Sensor/motor panel (left): gyro, accel, attitude sent to ArduPilot + Ch4 PWM, torques
- Scrolling event log with colour-coded threshold crossings
- File counter top-centre showing current test and PASS/FAIL

---

## File Map

```
simulation/torque/
├── model.py                Physics: hub yaw dynamics, GB4008 motor, RK4
├── torque_telemetry.py     TorqueTelemetryFrame, Recorder, JSONSource
├── torque_test_utils.py    Shared observation loop + assertions (used by stack tests)
├── mediator_torque.py      ArduPilot SITL mediator — profiles, sensor packets, lua_mode
├── test_model.py           9 unit tests (native Windows, no Docker)
├── visualize_torque.py     3D PyVista visualiser (30 fps, hardware OpenGL)
└── scripts/
    ├── lua_defaults.parm   SITL param overrides for Lua test (minimal, avoids SIGFPE)
    └── check_params.py     Debug utility: connects to SITL and reads params

simulation/scripts/
└── rawes.lua               Unified Lua controller (SCR_USER7: 0=none, 1=flight, 2=yaw, 3=both)

simulation/tests/stack/
├── conftest.py             Unified fixtures: acro_armed* + torque_armed* + TorqueStackContext
├── test_yaw_regulation.py  Constant RPM yaw hold (mediator adaptive trim)
├── test_slow_rpm.py        Slow sinusoidal RPM variation
├── test_fast_rpm.py        Fast sinusoidal RPM variation
├── test_gust_recovery.py   Gust shock + recovery (20% overspeed)
├── test_pitch_roll.py      Pitch/roll oscillation with yaw regulation
└── test_lua_yaw_trim.py    Lua feedforward controller running inside ArduPilot SITL
```

---

## Lua Feedforward Controller

`test_lua_yaw_trim.py` validates the **hardware-equivalent control architecture** where
the feedforward computation runs inside ArduPilot rather than in the mediator.

### How it works

```
mediator_torque.py (--lua-mode)     ArduPilot SITL
────────────────────────────────    ──────────────────────────────────────
Pure physics engine only:           rawes.lua (SCR_USER7=2):
  hub yaw dynamics                    motor_rpm = battery:voltage(0)
  bearing friction                      (mediator encodes RPM as voltage)
  motor torque                        trim = compute_trim(motor_rpm)
  sends battery.voltage=motor_rpm     yaw_corr = -Kp × gyro:z()
  (encodes motor RPM for Lua)         throttle = trim + yaw_corr
  reads SERVO9 (Ch9) for motor ──── SRV_Channels:set_output_pwm(94, pwm)
```

### SITL RPM channel

Motor RPM cannot be read via `RPM:get_rpm()` in SITL because ArduPilot's JSON
physics backend does not parse the `"rpm"` field (it has no entry in the JSON
keytable).  As a workaround the mediator encodes motor RPM as battery voltage:

- Mediator sends `"battery": {"voltage": motor_rpm}` (e.g., ~486)
- Lua reads `battery:voltage(0)` and interprets values > 50 as motor RPM
- A real 4S battery is 12.6 V which is < 50, so this is safe in normal tests

**On hardware** (Pixhawk 6C + AM32 with DSHOT bidirectional):
```lua
-- Replace the battery voltage hack with:
local motor_rpm = RPM:get_rpm(0)  -- DSHOT telemetry from AM32 ESC
-- RPM1_TYPE = 5 (DSHOT), RPM1_ESC = 9 (Ch9)
```
DSHOT is preferred over PWM because it provides **bidirectional telemetry on the
same wire** — motor RPM comes back from the ESC automatically via back-EMF
measurement without needing a separate RPM sensor.

### Docker setup for Lua

The Docker image bakes these into `copter-heli.parm` (the SITL defaults file):
```
SCR_ENABLE  1    — enable Lua scripting subsystem
RPM1_TYPE  10    — SITL: read rpm from JSON (unused; workaround uses battery.voltage)
RPM1_MIN    0
```

EEPROM is deleted before the Lua test starts (`wipe_eeprom=True` in `_torque_stack`)
so the fresh defaults take effect without requiring `--wipe` (which crashes this
ArduPilot build with SIGFPE).

### Result

15/15 tests pass including `test_lua_yaw_trim` at **0.27°/s** max yaw rate — identical
accuracy to the mediator-controlled baseline, confirming the Lua feedforward is working.

---

## Expected Hardware Implementation

### Mechanical Integration

The GB4008 motor mounts on the stationary hub frame with its shaft aligned to the
axle axis.  The 80:44 spur gear meshes between the motor pinion (44T) and a gear
ring on the axle (80T).  The motor stator is bolted directly to the stationary frame —
any torque the motor produces on its rotor creates an equal and opposite torque on
the frame.

The hollow shaft of the GB4008 allows the tether attachment point to pass through the
motor centre, keeping the tether connection on the central axis.

### ESC and Signal Chain

```
Pixhawk 6C  →  RC Output Ch4  →  REVVitRC AM32 ESC  →  GB4008 motor
```

- **PWM protocol:** Standard 50 Hz servo PWM (1000–2000 µs) or DSHOT600
- **ArduPilot parameter:** `H_TAIL_TYPE = 0` (servo), `H_RSC_MODE = 1` (CH8 passthrough)
- **Motor interlock:** CH8 must be HIGH (2000 µs) via RC transmitter or GCS override

In flight, the pilot's yaw stick sets a desired yaw *rate*.  With neutral stick (no yaw
command), ArduPilot's ATC_RAT_YAW PID drives Ch4 to hold ψ̇ = 0.  The ESC converts
Ch4 PWM to a motor speed proportional to the PWM value.

### Tuning Procedure (Hardware)

1. **Measure k_bearing:** Spin axle at known RPM with motor disconnected; measure torque
   on hub with a torque wrench or force gauge at known radius.
   → Update `HubParams.k_bearing` in `model.py`.

2. **Verify equilibrium throttle:** At nominal autorotation RPM, command motor until
   ψ̇ = 0 with no PID active.  Record the throttle percentage.
   → Compare to `equilibrium_throttle()` prediction; adjust `k_bearing` if they disagree.

3. **Set trim:** Update `--trim-throttle` (mediator) or H_TRIM_THROTTLE (hardware) to
   the measured equilibrium throttle so neutral stick = exact equilibrium.

4. **Tune ATC_RAT_YAW_P:** Increase from 0.001 until yaw disturbance rejection is fast
   enough without oscillation.  The trim handles steady state; P only corrects transients.
   With the GB4008 at R=7.5 Ω, the stability limit at 400 Hz is P ≈ 2.7.

5. **Run hardware test profiles** (equivalent to the stack tests):
   - `slow_vary`: manually vary motor RPM; check yaw stays < 2°/s
   - `gust`: briefly grab axle (impulse) and release; measure recovery time
   - `pitch_roll`: tilt hub manually; check yaw hold is unaffected

### Battery and Power

The GB4008 at equilibrium draws approximately:
```
I_eq = τ_stall × throttle_eq × KV_rad / (1 − throttle_eq×back_emf) ≈ 0.53 A
P_eq ≈ 6 W
```
The 4S 450 mAh LiPo provides roughly 450 mAh / 0.53 A ≈ 50 minutes of continuous
anti-rotation at nominal autorotation RPM — well within flight duration limits.

### Known Limitations of the Simulation

1. **Bearing friction model is viscous (linear)** — real bearings have Coulomb + viscous
   components; k_bearing must be measured, not assumed.
2. **Motor model ignores iron losses** — actual efficiency at high RPM will be lower.
3. **Single-body hub** — push-rod reaction forces from blade pitch servos are ignored.
4. **No reel dynamics** — winch tension changes are not modelled in this sub-simulation.
5. **ArduPilot yaw PID tuning** — the current P=0.001 is correct for the simulation
   (which has fast motor response); hardware may need higher P since the motor response
   is limited by ESC update rate and motor inductance.
