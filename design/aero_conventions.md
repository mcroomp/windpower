# Aero Interface Conventions

This document owns the **windpower-side contract** with the external
`dynbem` rotor model. The live aero implementation is not vendored into
this repository, so the verified source of truth is how windpower builds
`RotorInputs`, consumes the returned result, and checks signs in tests.

For system ownership, see [flight_stack.md](flight_stack.md). For where
the aero call sits inside the runtime, see [simulation.md](simulation.md).

## Current runtime usage

- `simulation\physics_core.py` and `tests\simtests\simtest_runner.py`
  create the live model with `dynbem.create_aero(...)`.
- The production/default model key in this repo is `quasi_static`.
- Alternative model keys such as `oye`, `pitt_peters`, and `vpm` appear
  only when a test or tool opts into them explicitly.
- The call shape used throughout the repo is:
  `result, state = aero.step(inputs, state, dt)`.

Older docs that describe an in-repo `compute_forces()` API or a local
`RotorOutput` type are obsolete.

## `RotorInputs` fields windpower currently passes

Every current call site in this repo populates these fields:

| Field | Meaning in windpower | Evidence |
|---|---|---|
| `collective_rad` | Physical blade collective in radians, after the single thrust→collective mapping boundary. | `simulation\physics_core.py`, unit-test probes |
| `tilt_lon` | Longitudinal cyclic command; **positive means nose-down disk**. | `tests\unit\test_cyclic_direction.py` |
| `tilt_lat` | Lateral cyclic command; **positive means roll-right disk**. | `tests\unit\test_cyclic_direction.py` |
| `R_hub` | Body-to-NED rotation matrix. `R_hub[:, 2]` is body-z (down through the disk) in NED. | `simulation\dynamics.py`, `simulation\sensor.py` |
| `v_hub_world` | Hub velocity in NED. | `simulation\physics_core.py` |
| `wind_world` | Ambient air-velocity vector in NED. Example call sites use `[0, 10, 0]` for eastward flow. | `simulation\config.py`, `tests\simtests\test_generate_ic.py` |
| `omega_rad_s` | Rotor spin rate in rad/s. | `simulation\physics_core.py` |
| `rho_kg_m3` | Air density scalar. | `simulation\physics_core.py`, `tests\unit\_aero_probe.py` |

Current windpower call sites do **not** pass a `t` field.

## Result fields windpower consumes

The repo relies on these result members today:

| Field | How windpower uses it | Evidence |
|---|---|---|
| `F_world` | Net aerodynamic force in NED. Passed into dynamics and logged into telemetry. | `simulation\physics_core.py`, `simulation\mediator.py` |
| `m_hub_world` | Net hub moment in NED/world coordinates. Passed into `RigidBodyDynamics.step()` and transformed to body frame in sign tests. | `simulation\physics_core.py`, `tests\unit\test_roll_sign_chain.py` |
| `Q_spin` | Scalar aerodynamic spin torque for the rotor-speed ODE. | `simulation\physics_core.py`, `envelope\point_mass.py` |
| `M_spin` | Separate spin-axis moment vector. Used by some tests/analysis helpers, but not added directly in `PhysicsCore`'s rigid-body moment sum. | `tests\unit\test_swashplate_aero.py`, `tests\unit\test_telemetry_aero_columns.py` |

`tests\unit\test_telemetry_aero_columns.py` documents the current
windpower assumption explicitly: the live aero result exposes
`F_world`, `m_hub_world`, `Q_spin`, and `M_spin`.

## Frame conventions

- World frame is always **NED** (`x=north`, `y=east`, `z=down`).
- `R_hub` maps **body → NED** (`v_ned = R_hub @ v_body`).
- Convert a world-frame moment or force back to body coordinates with
  `R_hub.T @ vector`.
- `R_hub[:, 2]` is the body z-axis, which points **down through the
  rotor disk** in the RAWES FRD convention.

## Verified sign conventions

Current unit tests pin down the cyclic sign chain:

- `tilt_lon > 0` produces a **negative body-frame pitch moment**
  (nose-down). Verified by `tests\unit\test_cyclic_direction.py` and
  `tests\unit\test_roll_sign_chain.py`.
- `tilt_lat > 0` produces a **positive body-frame roll moment**
  (roll-right). Verified by the same tests.
- To check those signs, transform the returned world-frame moment with:
  `M_body = R_hub.T @ result.m_hub_world`.

For near-level hover/landing cases, upward aerodynamic force appears as
`F_world[2] < 0` in NED; that convention is locked in by
`tests\unit\test_hover_sign.py`. For arbitrary disk attitudes, inspect
the full vector in world or body coordinates rather than assuming a
simple `-F_world[2]` scalar captures the whole thrust picture.

## Wind convention

Wind vectors in this repo are treated as **ambient air-velocity vectors
expressed in NED**, not “wind-from” compass headings. Examples:

- `np.array([0.0, 10.0, 0.0])` = eastward flow
- `np.array([0.0, -10.0, 0.0])` = westward flow

This convention is what current config defaults, simtests, and envelope
tools actually pass into `RotorInputs`.

## Integration notes inside windpower

- `PhysicsCore` advances aero state with `aero.step(inputs, state, dt)`.
- Rotor spin is integrated separately through `dynbem.step_omega()` using
  `Q_spin`.
- `RigidBodyDynamics` receives `F_world`, `m_hub_world`, and the scalar
  `omega_spin`; gyroscopic coupling is handled in the rigid-body model,
  not by adding `M_spin` directly in `PhysicsCore`.
- `simulation\mediator.py` logs aero force/moment columns from the same
  result object into `telemetry_columns.py`'s schema.

## Practical checks

When investigating sign bugs, use these tests first:

- `tests\unit\test_cyclic_direction.py`
- `tests\unit\test_roll_sign_chain.py`
- `tests\unit\test_hover_sign.py`
- `tests\unit\test_telemetry_aero_columns.py`

## Reference rotors and papers

- `simulation\rotor_definitions\beaupoil_2026.yaml` is the active RAWES rotor.
- `simulation\rotor_definitions\de_schutter_2018.yaml` stores the De Schutter
  reference geometry and airfoil coefficients (3 blades, 3.1 m radius, 1.6 m
  root cutout, 0.125 m chord, aspect ratio 12, `CD_structural` 0.021);
  `tests\unit\test_rotor_definition.py` loads it and checks the derived
  geometry.
- `archive\aero\aero_deschutter.py` is the archived in-repo strip-theory
  implementation, kept only for offline comparison. It is not used by
  `PhysicsCore`, `PhysicsRunner`, or `mediator.py`.
- De Schutter J., Leuthold R., Diehl M. (2018), "Optimal Control of a
  Rigid-Wing Rotary Kite System for Airborne Wind Energy"
  ([PDF](../documents/DeSchutter2018.pdf)). The repo's pumping logic is a
  ground-side phase machine (`groundstation\pumping_planner.py`), not an
  optimal-control solver.
- Weyel F. (2025), "Modeling and Closed Loop Control of a Cyclic Pitch
  Actuated Rotary Airborne Wind Energy System"
  ([PDF](../documents/Bachelorarbeit_Felix_Weyel.pdf)). The reproduction scripts
  are standalone under `felix\` (`simulate.py` targets Figures 10, 11, 14 and
  15); the simulation runtime does not import them.