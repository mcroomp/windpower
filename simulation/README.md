# RAWES Simulation

End-to-end flight simulation for the **Rotary Airborne Wind Energy System (RAWES)** —
a 4-blade tethered autorotating rotor kite controlled via trailing-edge flaps through an
H3-120 swashplate. Combines ArduPilot SITL (flight controller logic), MBDyn (6-DOF rigid body physics),
and a Python mediator (aerodynamics + sensor simulation).

---

## Architecture

```
  ┌──────────────────────────────────────────────────────────────────────┐
  │                     RAWES Simulation System                         │
  │                                                                      │
  │  ┌─────────────┐   UDP 9002 (servos)    ┌───────────────────────┐   │
  │  │             │ ─────────────────────► │                       │   │
  │  │ ArduPilot   │                        │    mediator.py        │   │
  │  │ SITL        │ ◄───────────────────── │                       │   │
  │  │ (heli frame)│   UDP 9003 (state)     │  - swashplate.py      │   │
  │  └─────────────┘                        │  - aero.py (BEM)      │   │
  │         ▲                               │  - sensor.py          │   │
  │         │ MAVLink                       └───────┬───────────────┘   │
  │  ┌──────┴──────┐                               │  UNIX sockets      │
  │  │ Mission     │                  /tmp/rawes_forces.sock (forces →)  │
  │  │ Planner /   │                  /tmp/rawes_state.sock  (← state)   │
  │  │ MAVProxy    │                               │                    │
  │  └─────────────┘                        ┌──────▼───────────────┐   │
  │                                          │      MBDyn           │   │
  │                                          │  6-DOF rigid body    │   │
  │                                          │  + gravity           │   │
  │                                          │  + tether spring     │   │
  │                                          │  rotor.mbd           │   │
  │                                          └──────────────────────┘   │
  └──────────────────────────────────────────────────────────────────────┘
```

### Data Flow (400 Hz loop)

```
  SITL → [servo PWM 1000-2000µs, 16 ch]
       → swashplate.h3_inverse_mix(S1,S2,S3) → collective + tilt
       → aero.compute_forces() → [Fx Fy Fz Mx My Mz] (ENU)
       → mbdyn.send_forces()   → MBDyn integrates one step (RK)
       → mbdyn.recv_state()    → [pos vel R omega] (ENU)
       → sensor.compute()      → IMU + NED position/velocity
       → sitl.send_state()     → SITL JSON physics state
```

---

## File Reference

| File | Role |
|------|------|
| `mediator.py` | Main event loop — bridges all subsystems |
| `mbdyn_interface.py` | UNIX socket client for MBDyn |
| `sitl_interface.py` | UDP JSON interface to ArduPilot SITL |
| `swashplate.py` | H3-120 swashplate mixing + cyclic pitch |
| `aero.py` | Simplified BEM rotor aerodynamics |
| `sensor.py` | IMU / GPS sensor simulation (ENU → NED) |
| `airfoil_gen.py` | Generates `mbdyn/SG6042.c81` airfoil table |
| `mbdyn/rotor.mbd` | MBDyn input deck (hub body, tether, streams) |
| `mbdyn/rawes.set` | MBDyn parameter definitions (included by rotor.mbd) |
| `run_sim.sh` | One-command sim launcher |
| `install.sh` | WSL2 Ubuntu dependency installer |

---

## Prerequisites

### Required

| Software | Version | Install |
|----------|---------|---------|
| MBDyn | 1.7.x+ | `install.sh` clones from `public.gitlab.polimi.it/DAER/mbdyn` |
| Python 3 | 3.10+ | Ubuntu: `apt install python3` |
| NumPy | 1.21+ | `pip3 install numpy` |
| ArduPilot SITL | 4.5+ | See ArduPilot docs |

### Optional (recommended)

| Software | Purpose |
|----------|---------|
| SciPy | Advanced analysis |
| Matplotlib | Post-processing plots |
| MAVProxy | CLI ground station |
| Mission Planner | GUI ground station (Windows) |

---

## Quick Start

```bash
# Step 1: Install dependencies (WSL2 Ubuntu, one time)
#   install.sh will print any sudo commands you need to run manually
bash simulation/install.sh
#   A Python venv is created at simulation/.venv — no sudo needed for pip

# Step 2: Start MBDyn + mediator
#   run_sim.sh activates the venv automatically
./simulation/run_sim.sh

# Step 3: Start ArduPilot SITL (in a separate terminal — run_sim.sh prints the command)
cd ~/ardupilot
./Tools/autotest/sim_vehicle.py \
    --vehicle ArduCopter \
    --frame heli \
    --custom-location=51.5074,-0.1278,50,0 \
    --sitl-instance-args='-f json:127.0.0.1' \
    --no-rebuild
```

To activate the venv manually for running individual scripts:
```bash
source simulation/activate_venv.sh
python3 simulation/mbdyn_interface.py   # smoke test
```

The mediator will print status logs. Once SITL connects, you should see
position updates at 1 Hz.

---

## Port Layout

| Port | Protocol | Direction | Purpose |
|------|----------|-----------|---------|
| 9002 | UDP | SITL → mediator | Servo PWM outputs (16 channels) |
| 9003 | UDP | mediator → SITL | Physics state (JSON) |
| 14550 | UDP | SITL → Mission Planner | MAVLink telemetry |
| 5760 | TCP | SITL → MAVProxy | MAVLink ground station |
| `/tmp/rawes_forces.sock` | UNIX | mediator → MBDyn | Aerodynamic wrench (6 × float64) |
| `/tmp/rawes_state.sock` | UNIX | MBDyn → mediator | Hub state (18 × float64) |

---

## How to Connect Mission Planner

1. Open Mission Planner on Windows.
2. In the top-right connection selector, choose **UDP**.
3. Port: **14550**.
4. Click **Connect**.
5. SITL will appear as an ArduCopter helicopter frame at 50 m altitude.

Or use MAVProxy:
```bash
mavproxy.py --master=tcp:127.0.0.1:5760 --console --map
```

---

## Physical Model Summary

| Parameter | Value |
|-----------|-------|
| Blades | 4, 90° apart |
| Blade length | 2.0 m |
| Rotor radius | 2.5 m (incl. hub) |
| Chord | 0.15 m |
| Airfoil | SG6042 |
| Rotor mass | 5 kg |
| Rotor inertia (Iz) | 10.0 kg·m² |
| Nominal spin rate | 28 rad/s (λ=7 at 10 m/s wind) |
| Tether attachment | 0.5 m below hub CoM |
| Tether spring k | 300 N/m |
| Initial altitude | 50 m |
| Design wind | 10 m/s East |

### Coordinate Systems

```
  MBDyn:       ENU  →  X=East, Y=North, Z=Up
  ArduPilot:   NED  →  X=North, Y=East, Z=Down

  Conversion: ap_N = mbd_Y,  ap_E = mbd_X,  ap_D = -mbd_Z
```

### Swashplate (H3-120)

- S1 at 0° (East reference)
- S2 at 120°
- S3 at 240°
- Servo channels: ArduPilot outputs 1, 2, 3 → S1, S2, S3
- ESC channel: output 4 → GB4008 anti-rotation motor

---

## Known Limitations / Future Work

### Current limitations

1. **Fixed wind field** — ambient wind is constant vector set at launch.
   Wind turbulence, shear, and gusts are not modelled.

2. **Simplified BEM aerodynamics** — no tip losses, no wake swirl, no blade
   flap/lag dynamics. The BEM model gives reasonable first-order thrust and
   torque but will not capture vortex ring state or dynamic inflow.

3. **Rotor spin not modelled in MBDyn** — MBDyn integrates hub body translation
   and tilt but the actual rotor angular momentum (gyroscopic effects) is
   approximated. The spin rate is held near 28 rad/s via the aero model
   rather than a full rotor spin equation of motion.

4. **No tether dynamics** — the tether is a massless spring-rod. Catenary
   shape, tether drag, and reel-in/reel-out dynamics are not modelled.

5. **Anti-rotation motor is open-loop** — GB4008 torque is proportional to
   ESC input; no closed-loop speed control within the mediator.

6. **Fixed-step integration** — MBDyn uses a fixed 2.5 ms step. If the
   aerodynamic forces vary rapidly (e.g., stall transient), the step size
   may need reduction.

### TODO for future upgrades

- [ ] Implement full rotor spin equation of motion in MBDyn (6th DOF)
- [ ] Add von Karman turbulence model to wind field
- [ ] BEM with tip-loss factor (Prandtl) and dynamic inflow (Peters-He)
- [ ] Full Weyel 2025 flap controller (feed-forward + PID) in mediator.py
- [ ] Tether catenary model with winch reel-in/reel-out control
- [ ] Hardware-in-the-loop (HIL) testing with physical Pixhawk 6C
- [ ] ArduPilot parameter file for RAWES helicopter frame configuration
- [ ] Matplotlib real-time plot of rotor trajectory
- [ ] Log to MAVLink binary (.tlog) for post-processing in Mission Planner

---

## References

1. Felix Weyel (2025) — "Modeling and Closed Loop Control of a Cyclic Pitch
   Actuated Rotary Airborne Wind Energy System", Bachelor's Thesis, Uni Freiburg.

2. De Schutter, Leuthold, Diehl (2018) — "Optimal Control of a Rigid-Wing
   Rotary Kite System for Airborne Wind Energy".

3. US Patent US3217809 (Kaman/Bossler, 1965) — Servo-flap rotor control system.

4. MBDyn Manual — https://www.mbdyn.org/documentation/
