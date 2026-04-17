# RAWES -- Rotary Airborne Wind Energy System

A tethered autorotating rotor kite that harvests wind energy through a pumping cycle.
This repository contains the ArduPilot flight controller model, physics simulation, and
all documentation for the RAWES hardware and control system.

---

## What Is This System?

Imagine a large spinning rotor -- like a helicopter rotor, but with no engine. Instead of an
engine driving the blades, the **wind drives them**. As the wind blows, the rotor spins
automatically (this is called autorotation, the same principle a helicopter uses when its
engine fails to glide safely to the ground).

Now attach a long cable to the bottom of that rotor and connect the other end to a winch on
the ground. As the rotor climbs and pulls the cable out, the tension in the cable spins a
generator on the ground -- **generating electricity**. Reel the cable back in at low power
cost, let the rotor climb again, and repeat. This is a Rotary Airborne Wind Energy System.

Our system has four blades, each two metres long, spinning at roughly 270 RPM at an altitude
of 50 metres. Four small servo motors tilt a mechanical plate inside the hub (a swashplate)
to control the blade pitch -- exactly as a helicopter controls its rotor, but entirely from
the ground via trailing-edge flaps.

Key distinction from a drone: **no motor drives rotation** -- wind does. Control is entirely
through blade pitch, actuated indirectly via trailing-edge flaps on each blade.

### Key Parameters

| Parameter          | Value         |
|--------------------|---------------|
| Blade count        | 4 (90 deg apart) |
| Blade length       | 2000 mm       |
| Total rotor radius | ~2500 mm      |
| Rotor mass         | 5 kg          |
| Blade airfoil      | SG6042        |
| Tether diameter    | 1.9 mm (Dyneema SK75) |
| Max tether length  | 300 m         |
| Min altitude       | 10 m          |
| Tether attachment  | Bottom of axle|
| Anti-rotation motor| EMAX GB4008 66KV, 80:44 spur gear |
| Servos S1/S2/S3    | DS113MG V6.0 |
| Flight controller  | Holybro Pixhawk 6C |
| Battery            | 4S LiPo 15.2V, 450 mAh |

---

## Simulation Overview

The simulation is a digital twin -- a complete mathematical replica of the physical system
running inside a computer. It has three interconnected layers:

```
+-------------------------------------------------------------+
|  Flight Controller (ArduPilot SITL)                         |
|  The "brain" -- decides how to tilt the rotor               |
|  to follow a target path or hold altitude                   |
+---------------------------+---------------------------------+
                            | Servo commands (4x per step)
                            v
+-------------------------------------------------------------+
|  Aerodynamics + Control Bridge  (Python)                    |
|  Translates servo positions -> blade pitch angles           |
|  Calculates lift, drag, and moments from the wind           |
|  Simulates the swashplate mechanism                         |
+---------------------------+---------------------------------+
                            | Forces and moments (6 values per step)
                            v
+-------------------------------------------------------------+
|  Physics Engine  (Python RK4)                               |
|  Rigid-body dynamics: gravity, inertia, tether              |
|  Updates position, velocity, orientation                    |
|  400 times per second                                       |
+-------------------------------------------------------------+
```

---

## Documentation Map

### Hardware

| File | Description |
|------|-------------|
| [hardware/design.md](hardware/design.md) | Full assembly layout, rotor geometry, blade design (SG6042), swashplate, Kaman servo flap mechanism (US3217809), anti-rotation motor, electronics and power architecture |
| [hardware/components.md](hardware/components.md) | Detailed component specs: GB4008 motor, REVVitRC ESC, AM32 firmware, DS113MG servos, SiK radio, RP3-H receiver, Boxer M2 transmitter |
| [hardware/flap_sensor_bench.md](hardware/flap_sensor_bench.md) | Bench measurement system for swashplate-to-flap deflection characterisation (ESP32 + MPU-6050 WiFi rig + manual digital level procedure) |

### Theory

| File | Description |
|------|-------------|
| [theory/pumping_cycle.md](theory/pumping_cycle.md) | De Schutter et al. 2018 -- pumping cycle OCP, state variables, aerodynamics (Eq. 25-31), structural constraints, system parameters (Table I) |
| [theory/orbit_mechanics.md](theory/orbit_mechanics.md) | Beaupoil 2026 -- orbit characteristics, gyroscopic angular momentum, five control design requirements from orbit mechanics, pumping cycle energy results |
| [theory/flap_dynamics.md](theory/flap_dynamics.md) | Weyel 2025 thesis summary -- flap state-space model, feed-forward + PID controller, N4SID identification, performance results |

### System / Flight Stack

| File | Description |
|------|-------------|
| [system/stack.md](system/stack.md) | Complete flight control reference -- system architecture (3-node diagram), ground planner, winch controller, Pixhawk orbit tracker (rawes.lua), yaw trim (rawes.lua), ArduPilot configuration, startup/arming sequence, EKF3 GPS fusion analysis, Lua API constraints |

### Simulation

| File | Description |
|------|-------------|
| [simulation/README.md](simulation/README.md) | Simulation architecture, module summary, coordinate frames, sensor design, initial state, running tests, analysis tools index |
| [simulation/internals.md](simulation/internals.md) | Sensor design, controller functions, dynamics model, aero model (SkewedWakeBEM), tether, pumping cycle architecture, known gaps |
| [simulation/history.md](simulation/history.md) | Phase 2 and Phase 3 M3 decisions -- why SkewedWakeBEM, collective passthrough fix, EKF altitude unreliability, test results |
| [simulation/torque_model.py](simulation/torque_model.py) | Counter-torque hub yaw physics model -- HubParams, GB4008 motor torque, RK4 integrator, equilibrium throttle |
| [simulation/aero/deschutter.md](simulation/aero/deschutter.md) | De Schutter 2018 equation-level validation -- maps Eq. 25-31 to implementation, C_{D,T} derivation, beta diagnostic, known gaps vs SkewedWakeBEM |

---

## Running Tests

Tests run in three sequential stages. Always run them in order.

```bash
# Stage 1 -- Unit tests (Windows, no Docker, ~460 tests, ~65 s)
bash sim.sh test-unit -q

# Stage 2 -- Simtests (Windows, no Docker, ~29 tests, ~5 min)
bash sim.sh test-simtest -q

# Stage 3 -- Stack tests (Docker, ArduPilot SITL)
bash sim.sh test-stack -v
```

See CLAUDE.md for the full workflow, Docker setup, and troubleshooting.

---

## Current Status

Phase 3, Milestone 3 -- pumping cycle stack test passed. Physics, closed-loop
controller, and ArduPilot SITL integration are all validated. Next milestone
covers GB4008 yaw configuration and writing the full rawes_params.parm.

Full phase plan, milestone checklist, and known test status: [CLAUDE.md](CLAUDE.md)
