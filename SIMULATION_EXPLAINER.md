# Simulating a Rotary Wind Energy Kite — A Plain-English Guide

## What Are We Building?

Imagine a large spinning rotor — like a helicopter rotor, but with no engine. Instead of an engine driving the blades, the **wind drives them**. As the wind blows, the rotor spins automatically (this is called *autorotation*, the same principle a helicopter uses when its engine fails to glide safely to the ground).

Now attach a long cable to the bottom of that rotor and connect the other end to a winch on the ground. As the rotor climbs and pulls the cable out, the tension in the cable spins a generator on the ground — **generating electricity**. Reel the cable back in at low power cost, let the rotor climb again, and repeat. This is a **Rotary Airborne Wind Energy System**, or RAWES.

Our system has four blades, each two metres long, spinning at roughly 270 RPM at an altitude of 50 metres. Four small servo motors tilt a mechanical plate inside the hub (a *swashplate*) to control the blade pitch, steering the rotor and controlling lift — exactly as a helicopter controls its rotor, but entirely from the ground.

---

## What Does the Simulation Do?

The simulation is a **digital twin** — a complete mathematical replica of the physical system running inside a computer. Before we build and fly hardware, we can:

- Test the rotor in any wind condition
- Try different control strategies
- Crash it virtually (and learn from it at zero cost)
- Measure forces and stresses that would be difficult or impossible to instrument on a real device
- Run weeks of flight time in minutes

The simulation has three interconnected layers:

```
┌─────────────────────────────────────────────────────┐
│  Flight Controller (ArduPilot SITL)                 │
│  The "brain" — decides how to tilt the rotor        │
│  to follow a target path or hold altitude           │
└───────────────────┬─────────────────────────────────┘
                    │  Servo commands (4× per step)
                    ▼
┌─────────────────────────────────────────────────────┐
│  Aerodynamics + Control Bridge  (Python)            │
│  Translates servo positions → blade pitch angles    │
│  Calculates lift, drag, and moments from the wind   │
│  Simulates the swashplate mechanism                 │
└───────────────────┬─────────────────────────────────┘
                    │  Forces and moments (6 values per step)
                    ▼
┌─────────────────────────────────────────────────────┐
│  Physics Engine  (Python RK4)                       │
│  Rigid-body dynamics: gravity, inertia, tether      │
│  Updates position, velocity, orientation            │
│  400 times per second                               │
└─────────────────────────────────────────────────────┘
```

Each of these three layers runs simultaneously and exchanges data 400 times per second — fast enough to capture every significant motion of the rotor.

---

## What Can You Vary?

The simulation is fully parameterised. You can change any of the following and immediately see the effect:

### Wind Conditions
| What you change | What it tests |
|----------------|---------------|
| Wind speed (e.g. 5 m/s vs 15 m/s) | Operating range, power output at different wind speeds |
| Wind direction | How well the rotor tracks crosswind vs headwind |
| Turbulence (gusts) | Control stability under real-world wind variability |
| Wind shear (faster at altitude) | Effect of the logarithmic wind profile on the pumping cycle |

### Rotor and Blade Properties
| What you change | What it tests |
|----------------|---------------|
| Blade length (1.5 m → 2.0 m) | Scaling laws, swept area, tip speed |
| Blade chord and airfoil | Lift/drag trade-offs |
| Number of blades (3 vs 4) | Torque smoothness, efficiency |
| Rotor mass and inertia | Jump-takeoff energy, control response speed |

### Control Settings
| What you change | What it tests |
|----------------|---------------|
| Collective pitch (all blades together) | Overall lift, altitude hold |
| Cyclic pitch (tilt direction and amount) | Steering, trajectory shaping |
| PID controller gains | How tightly the rotor tracks a reference pitch |
| MPC trajectory target | Long-range path optimisation, pumping cycle efficiency |

### Tether and Power Cycle
| What you change | What it tests |
|----------------|---------------|
| Tether length (10 m → 300 m) | Altitude ceiling, cable weight effect |
| Reel-out speed | Power extraction rate, optimal tip-speed ratio |
| Reel-in strategy | Net cycle efficiency |

---

## Can the Results Be Shown Graphically?

Yes. The simulation produces several useful categories of visualisation.

---

### 1. Force and Aerodynamic Plots (Already Working)

The current simulation stack produces the state and force data needed for time-series plots of the rotor during flight.

**Example — Forces on each blade during one rotation cycle:**

Typical plots would show thrust, aerodynamic moments, hub velocity, rotor tilt, and estimated rotor spin versus time.

This tells you whether the current controller and aerodynamic model are producing reasonable loads, whether the rotor is remaining within the expected operating envelope, and how effectively the system is converting wind into tether pull.

---

### 2. Three-Dimensional Flight Path Plots (Already Working)

The simulation tracks the full 3D position of the rotor over time and can plot the trajectory in space.

**Example — RAWES trajectory under three different pitch inputs:**

You can plot the rotor position in 3D, or project it onto horizontal and vertical planes, to see whether a given control strategy keeps the rotor airborne, causes it to descend, or produces the intended steering response.

Running the same comparison at different wind speeds shows the envelope within which the system can operate.

---

### 3. Time Histories For Controller And State Debugging

Because the mediator publishes and consumes the full hub state each step, you can inspect:

- hub position, velocity, and attitude over time
- estimated rotor spin rate
- swashplate collective and cyclic commands
- aerodynamic wrench components `[Fx, Fy, Fz, Mx, My, Mz]`
- synthetic IMU outputs sent back to ArduPilot

These plots are useful for debugging controller behavior, spotting sign errors or frame-conversion mistakes, and checking whether the simplified aerodynamic model is driving the rigid-body model in a physically reasonable way.

---

### 4. Real-Time 3D Animation (Future)

A Blender-based 3D animation of the rotor flying is possible once the simulation produces stable flight trajectories. The workflow would record hub position and orientation per step and import it into Blender for rendering.

---

## Example: What a "Wind Speed Sweep" Study Looks Like

To understand the power curve of the system across its operating range, you would:

1. **Set up** a parameter file with wind speed as a variable
2. **Run** the simulation at 5, 6, 7, ..., 15 m/s — each run takes a few minutes of computer time
3. **Extract** the average tether tension and reel-out speed from each run
4. **Calculate** the power: `P = tension × reel-out speed`
5. **Plot** power vs wind speed → this is the **power curve**

The same approach works for any design parameter. Change blade length from 1.5 m to 2.0 m and re-run the sweep — the power curve shifts. This is how you optimise the design on a computer before cutting any material.

---

## Summary: What the Simulation Gives You

| Question | How simulation answers it |
|----------|--------------------------|
| Will it fly in X m/s wind? | Run physics simulation, observe if altitude is maintained |
| How much power does it generate? | Integrate tether tension × reel speed over a pumping cycle |
| Is the controller stable? | Plot tracking error under disturbances |
| What happens in a gust? | Inject a step change in wind speed, observe recovery |
| How does it take off? | Simulate jump takeoff: pre-spin → collective pitch spike |
| What are peak tether forces? | Read peak tension from simulation, compare to cable rating |
| Does longer blade = more power? | Sweep blade length, compare power curves |
| Can I see it? | Blender 3D animation or 3D trajectory plots |

The simulation is not a final answer — real flight will always reveal things a model cannot predict. But it is the fastest, cheapest, and safest way to test ideas, find failure modes, and build confidence before flying hardware.

---

*Document prepared 2026-03-26, updated 2026-03-28. System: RAWES 4-blade tethered autorotating rotor kite. Physics: Python RK4 (dynamics.py). Aerodynamics + control bridge: Python. Flight controller: ArduPilot SITL. Design reference: De Schutter et al. (2018) and current project simulation documents.*
