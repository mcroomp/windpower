# RAWES Simulation Internals

This document owns the simulation runtime internals: the physical-world
model, sensor and actuator stand-ins, test adapters, and the lockstep
interface. For the repository-level ownership map, see
[../README.md#documentation-map](../README.md#documentation-map).

## Runtime boundaries

- The mediator is the simulated physical world plus the lockstep adapter.
  It is not a flight controller, MAVLink router, or production ground
  planner.
- ArduPilot owns estimation, modes, Lua behavior, attitude/rate control,
  and servo mixing.
- LinkHub is the sole MAVLink owner and journal. The mediator writes raw
  physics telemetry only; the SITL harness later enriches it with LinkHub
  observations.
- Production pumping/landing policy lives in `groundstation\`, not in
  `simulation\`.
- `simulation\` may contain stand-ins for hardware that does not exist yet
  (`GovernedWinchNode`, `VirtualComms`, torque-only mediators), but those
  remain simulation-only components.

See [flight_stack.md](flight_stack.md) for ground/AP command contracts,
[aero_conventions.md](aero_conventions.md) for the aero-model boundary,
and [sitl_testing.md](sitl_testing.md) for stack-harness ownership.

## Runtime module map

| Area | Current source of truth | Responsibility |
|---|---|---|
| Shared physics step | `simulation\physics_core.py` | Owns the shared step used by the mediator and by in-process simtests: aero call, tether, rigid-body dynamics, rotor-speed ODE, optional hub-motor ODE, kinematic handoff, and simulation time. |
| Rigid-body integrator | `simulation\dynamics.py` | RK4 integration of `pos`, `vel`, `R`, and world-frame `omega` in NED, with gravity applied internally. |
| Tether | `simulation\tether.py` | Two-ended cable contract, straight spring-damper, Tallak quasi-static equilibrium, and diagnostic constant load. |
| Sensor plants | `simulation\sensor.py` | `PhysicalSensor` and `SpinSensor`; converts physics state into the IMU/GPS-style packet sent to SITL or simtests. |
| Frame helpers | `simulation\frames.py` | `build_orb_frame`, `build_gps_yaw_frame`, `build_vel_aligned_frame`, and `T_ENU_NED`. |
| Swashplate / collective mapping | `simulation\swashplate.py`, `simulation\param_defaults.py` | H3-120 servo mixing and the single thrust↔collective mapping boundary derived from the rotor YAML plus `.parm` files. |
| Portable control math | `simulation\controller.py` | Geometry helpers mirrored to Lua, simtest-facing cyclic wrappers, and the yaw-trim observer. Not a production flight stack. |
| SITL lockstep transport | `simulation\sitl_interface.py`, `simulation\mediator_base.py`, `simulation\mediator.py` | Servo-packet decode, state reply, lockstep loop, physics hosting, telemetry/events output, optional winch-command socket. |
| Torque-only stack path | `simulation\mediator_torque.py`, `simulation\torque_model.py` | Standalone yaw/counter-torque test path with fixed hub kinematics and GB4008 motor model. |
| Startup / initial conditions | `simulation\kinematic.py`, `simulation\ic.py`, `tests\simtests\test_generate_ic.py` | Kinematic startup override plus the generated steady-state initial-condition JSON. |
| Winch stand-ins | `simulation\winch.py`, `simulation\winch_node.py` | Motion-profile helpers and the simulated winch-node firmware stand-in. |
| Test-only comms adapters | `simulation\comms.py`, `simulation\unified_ground.py` | In-process telemetry/command latency model plus direct/Lua adapters for simtests. |
| Ground-side production policy | `groundstation\pumping_planner.py`, `groundstation\landing_planner.py`, `groundstation\unified_ground.py`, `groundstation\winch_protocol.py` | Production pumping/landing state machines, NVF/MAVLink marshalling, and the cable-side winch protocol. |
| Simtest AP adapters | `tests\simtests\simtest_runner.py`, `tests\common\mock_ardupilot.py` | Shared `PhysicsRunner`, Lua-backed and Python-backed AP equivalents, and test telemetry writing. |
| Telemetry schema | `simulation\telemetry_columns.py`, `simulation\telemetry_csv.py` | Canonical CSV column order and typed row I/O. |

## Full SITL path

1. `SITLInterface.recv_servos()` blocks on the next ArduPilot JSON-backend
   servo packet and records the packet-declared `frame_rate` and
   `frame_count`.
2. `mediator.py` decodes swash outputs with
   `ardupilot_h3_120_inverse()` and maps ArduPilot collective `[0..1]`
   into physical collective radians with `collective_out_to_rad()`.
3. `PhysicsCore.step()` advances tether, aero state, rotor speed,
   rigid-body dynamics, optional hub-motor dynamics, and kinematic
   release logic.
4. `PhysicalSensor.compute()` and `SpinSensor.measure()` produce the
   state packet returned to SITL.
5. The mediator writes physics-only telemetry plus `events.jsonl`.
   It never reads MAVLink to decorate those rows.

Two extra boundaries exist around that loop:

- If `--winch-cmd-port` is enabled, the mediator hosts a
  `GovernedWinchNode` and exchanges only `WinchCommand` /
  `WinchTelemetry` payloads across that socket.
- If no external ground-side driver is attached, `simulation.config`
  currently builds a trivial `HoldPlanner()` only. Pumping and landing
  phase logic remain outside the mediator.

## In-process simtest path

- `tests\simtests\simtest_runner.py::PhysicsRunner` is a thin wrapper
  around the same `PhysicsCore` used by the mediator.
- `tests\common\mock_ardupilot.py::MockArdupilot` provides:
  - a Lua backend (`RawesLua` + `GuidedAttitudeController`) for script
    parity tests;
  - Python-backed pumping approximation and landing prototype
    (`_PumpingPythonMode`, `_LandingPythonMode`) for fast closed-loop
    simtests.
  The Python pumping cyclic law is not currently identical to Lua's absolute
  angle path; see [flight_stack.md](flight_stack.md#49-simulation-mapping).
- `simulation\comms.py::VirtualComms` and
  `simulation\unified_ground.py::DirectComms` are simtest-only
  helpers. The production MAVLink adapter is
  `groundstation.unified_ground.GcsComms`; Lua unit tests use it too, with the
  Lua harness as the `gcs`.

## Timing and lockstep

Avoid blanket “the simulation runs at 400 Hz” statements; the current
code has multiple clocks:

- `SITLInterface.dt()` is always `1 / frame_rate`, where `frame_rate`
  comes from the latest SITL servo packet header.
- `SITLInterface` falls back to 400 Hz only before the first packet
  arrives.
- The current stack-test overlay sets `SIM_RATE_HZ 1200` in
  `tests\sitl\rawes_sitl_defaults.parm`, so full SITL typically advances
  at 1/1200 s per physics frame.
- Many in-process simtests intentionally use `DT = 1/400` and run the
  mock AP loop at 50 Hz (`tests\common\mock_ardupilot.py::AP_HZ`).

Document the caller-owned `dt`, not a single repository-wide timestep.

## Sensor model

`simulation\sensor.py::PhysicalSensor` is the only hub sensor model.
Its verified current behavior is:

- `R_hub` is treated as the full body-to-NED attitude matrix.
- `rpy` is extracted directly from `R_hub` (ZYX Euler order).
- `gyro_body = R_hub.T @ omega_world`; no rotor-spin stripping is done in
  the sensor.
- `accel_body = R_hub.T @ (accel_world_ned - gravity_ned)`.
- Position and velocity remain NED quantities relative to `home_ned_z`.
- `SpinSensor` is a separate measurement channel for `omega_spin`.

This keeps the physical-world model honest: the anti-rotation behavior is
represented in dynamics, not faked in the sensor outputs.

## Controller and guidance helpers

`simulation\controller.py` is a library of portable math plus simtest
wrappers. It is not a separate production controller stack.

| Helper | Current role |
|---|---|
| `compute_bz_tether()` | Unit vector from hub toward anchor. |
| `compute_bz_altitude_hold()` | Stateless body-z target from current position, target elevation, tension feedforward, and gravity compensation. |
| `update_plane_azimuth()` | Low-pass reference azimuth used by the altitude/elevation hold helpers. |
| `slerp_body_z()` | Rate-limited body-z interpolation. |
| `compute_rate_cmd()` / `compute_rate_cmd_sqrt()` | Body-z error → body-rate command helpers. |
| `AltitudeHoldController` / `ElevationHoldController` | Simtest-side wrappers built on the portable helpers. |
| `HeliCyclicController` | ArduPilot-style inner rate loop plus the swashplate servo lag model used by `PhysicsRunner`. |
| `YawTrimObserver` | Python port of the Lua yaw-trim observer. Cross-checked by `tests\unit\test_yaw_trim_parity.py`. |

The repo’s AP-equivalent closed loops live in `arduloop\` and
`tests\common\mock_ardupilot.py`.

## PhysicsCore responsibilities

`PhysicsCore` currently owns these submodels and boundaries:

- `RigidBodyDynamics` for translation, attitude, and world-frame body
  rate.
- `Tether.evaluate()` for both endpoint reactions; `PhysicsCore` applies the
  kite-end force at the actual attachment offset and computes its moment.
- The live external aero model via `dynbem.create_aero(...)`; the
  default model key is `quasi_static`, with opt-in alternatives selected
  only by callers/tests.
- Rotor-speed integration through `dynbem.step_omega()` using
  aerodynamic `Q_spin`.
- Optional GB4008/yaw-motor state through `simulation\torque_model.py`
  when a caller supplies `yaw_throttle`.
- `KinematicStartup` release logic, including the debug-only `nul`
  kinematic aero mode.
- The observable boundary exposed through `hub_observe()`.

Rotor-spin inertia is resolved by `resolve_i_spin_kgm2()` in
`simulation\rotor_physics.py`: an explicit `I_spin_kgm2` in the rotor definition
wins; when it is null the value is derived from blade mass and the spinning
hub shell only. The stationary inner assembly is excluded. The GB4008 stator is
fixed to that assembly and its rotor is geared to the spinning hub
(`simulation\torque_model.py`), so motor torque acts between the two bodies
rather than as an external couple.

The live aero result fields that windpower code consumes are `F_world`,
`m_hub_world`, `Q_spin`, and (in some tests/analysis) `M_spin`.
`PhysicsCore` passes `F_world` and `m_hub_world` into the rigid-body
step and threads `omega_spin` separately into the dynamics for
gyroscopic coupling.

## Two-ended tether physics

`simulation/tether.py` owns `TetherInputs`, `TetherResult`, `TetherProperties`,
and the structural `Tether` protocol. One cable evaluation supplies both the
aircraft load and the winch load; there are not separate winch and kite tether
simulations.

Inputs describe the anchor and aircraft attachment positions and velocities
in NED, deployed **unstretched** length, payout rate (positive reel-out),
uniform wind, air density, and gravity. Material properties belong to the
cable model; deployed length belongs to the winch plant and is supplied through
`PhysicsCore.step(rest_length=..., payout_rate_ms=...)`. If payout rate is not
supplied, the core derives it from that step's length change and `dt`.

The physics core constructs the actual aircraft endpoint:

```text
offset = axle_attachment_length * R[:, 2]
attachment_position = hub_position + offset
attachment_velocity = hub_velocity + omega_world x offset
moment_on_hub = offset x force_on_hub
```

Both models use this geometry. The old centre-based stretch calculation has
been removed; the IC generator now chooses rest length using the actual
attachment and resolves force balance against its direction. Regenerate the
IC through the owner workflow in [testing.md](testing.md#steady-state-ic-ownership),
not by hand-editing its JSON.

`TetherResult` contains forces **on the attached bodies**, the sampled line
positions, arc length, extension, endpoint tension magnitudes, slack state,
elastic energy, dissipated power where modelled, and solve diagnostics.
`PhysicsCore.tether_result` is the authoritative snapshot.
`tension_now` means kite-end tension; `anchor_tension` means the ideal
winch load-cell magnitude. The scalar tensions need not have a fixed ordering.
For quasi-static equilibrium:

```text
force_on_hub + force_on_anchor = integrated_drag_and_weight
```

The mediator feeds only `anchor_tension` to the winch sensor/governor.
Mechanical drum power uses that measured local tension times reel speed.
The ground command/winch wire protocol is unchanged, and line shape is not
an additional ground-controller sensor. Physics CSVs retain `tether_tension`
for the kite end and add `tether_anchor_tension` for the winch end. This is a
physics-only column, not a Lua named-value input/output; NVF maps are unchanged.

### Model selection

The mediator config key `tether_model` and the `PhysicsCore`/`PhysicsRunner`
constructor keyword select:

| Key | Implementation | Scope |
|---|---|---|
| `tallak_quasi_static` (default) | `TallakQuasiStaticTether` | Extensible suspended-line equilibrium, transverse aerodynamic drag, line weight, shape, and distinct endpoint reactions. |
| `spring_damper` | `SpringDamperTether` | Explicit simplified alternative: original straight, tension-only `k=EA/chord_length` law, zeta 0.05, extension-only damping. Relative radial endpoint velocity minus payout drives damping. No line drag, weight, or belly. |
| `constant_tension` | `ConstantTensionTether` | Diagnostic fixed load, selected with `tether_constant_tension_n`; no distributed loads or elastic dynamics. |

Unknown names and invalid inputs raise errors rather than selecting a default.
Tests may inject a `Tether` implementation with `tether_override`.
`DEFAULT_TETHER_MODEL` in `simulation/tether.py` is the shared default for
mediator configuration, the physics core, and simtest runners/helpers.

The better model is named after Tallak Tveide's
[TetherDragODESolver](https://github.com/tallakt/TetherDragODESolver), which
motivated the quasi-static equilibrium approach. Our implementation is
independently written: it solves a wind/gravity-loaded extensible cable
between prescribed endpoints, not the Julia solver's rotating-line ODE.
It does **not** apply a universal quarter-drag multiplier.

At 16 segments by default, segment-midpoint tension and elastic strain determine
segment length/direction. A three-component anchor-force solve closes the
endpoint position; an outer iteration updates drag from local transverse
relative wind and strained segment length. Weight uses mass per unstretched
length. Endpoint velocities are linearly interpolated along material stations.
Axial material slip does not directly contribute to this normal-drag law.
Warm starts reuse the previous converged forces/shape, not physical dynamic
state. Failed endpoint or load convergence raises `TetherConvergenceError`;
there is no silent straight-line fallback.

This version has no line inertia, transverse waves, material hysteresis/damping,
ground contact, axial skin friction, rupture, or payout momentum flux. A loaded
line can support tension even when the endpoint chord is shorter than its rest
length. The shape assumes a freely suspended line; it must not be interpreted
as a ground-contact solution. Collinear, vertically slack zero-wind
configurations can fail the endpoint solve; ground-level slack deployment is
not a supported startup condition. Flight tests and unrelated physics/telemetry
fixtures should start from the shared airborne IC, not a slack line that would
need to lie on the ground. The IC is a straight-line force-balance seed, checked
against the active cable model by `test_ic_force_balance`.
Quasi-static shape changes are instantaneous, so gust and low-tension transient
fidelity is not claimed. Solve time must be benchmarked for the intended
lockstep rate; there is no hard real-time 1200 Hz deadline guarantee.

Analytical, mesh-convergence, operating-envelope, and flight regression tests
are described in [testing.md](testing.md#tether-model-validation).

## Winch and ground-side boundaries

The ground/winch split is now explicit:

- `groundstation\pumping_planner.py` and
  `groundstation\landing_planner.py` own the production pumping and
  landing state machines.
- `groundstation\winch_protocol.py` owns the cable-side wire protocol
  (`WinchCommand`, `WinchTelemetry`).
- `simulation\winch_node.py::GovernedWinchNode` is a stand-in for the
  fast local winch firmware that does not exist yet.
- `simulation\winch.py` contains two helpers:
  - `WinchController`: target-length motion profile helper.
  - `GovernedWinchController`: tension-governed, jerk-limited velocity
    controller used by the node stand-in.

For the node controller, the requested velocity is
`cruise + kp_tension*(filtered_measured_tension - target_tension)`, then
limited by speed, acceleration, and jerk. The separate length-target helper
(`WinchController`) sets a cruise speed from the tension error in the
direction of the remaining length and follows a trapezoidal profile. Both
command reel speed, not airborne collective.

No hub position, altitude, or attitude is part of the cable-side winch
protocol.

## Yaw motor plant

`simulation/torque_model.py` owns the simulated finite-torque speed governor:

```text
omega_target = throttle * rpm_scale
Q = clamp(esc_kp * (omega_target - omega_motor), +/-esc_q_max)
J_total = hub_inertia / gear_ratio^2 + motor_inertia
d(omega_motor)/dt = Q / J_total
psi_dot = -omega_rotor + omega_motor / gear_ratio
```

Its equilibrium is `throttle_eq = omega_rotor*gear_ratio/rpm_scale`.
`mediator_torque.py` maps PWM linearly from `MOTOR_PWM_MIN..MOTOR_PWM_MAX`
to throttle `[0,1]`. Constants live in `HubParams` / `torque_model.py`. This
is a simulation model, not a measured model of the hardware ESC. Hardware
identity and wiring are in [hardware.md](hardware.md); the Lua trim observer
is in [flight_stack.md](flight_stack.md#5-yaw--torque-compensation).

## Initial conditions

`simulation\ic.py` is the single source of truth for the generated
steady-state initial condition JSON:

- canonical file: `simulation\steady_state_starting.json`;
- generator: `tests\simtests\test_generate_ic.py::test_create_ic`;
- consumers: `simulation\config.py`, simtests, the SITL stack, and
  analysis tools.

The current IC payload includes more than `pos` / `vel` / `R0`: it also
carries `R0_kinematic`, `R0_orbit`, `orbit_bz`, `eq_thrust`,
`coll_eq_rad`, and trim cyclic values. Torque-only stack tests do not
use this path; they use the separate setup in `mediator_torque.py`.

## Telemetry and logs

- `simulation\telemetry_columns.py` is the master ordered schema.
- `simulation\telemetry_csv.py::TelRow` is the typed row object and the
  canonical CSV reader/writer.
- Stack runs preserve the mediator’s raw physics log (named
  `telemetry.physics.csv` by the harness) and later write an enriched
  `telemetry.csv` by sampling LinkHub/MAVLink observations into the
  `mavlink_async` columns.
- Simtests usually write `telemetry.csv` directly via `TelRow` helpers;
  they do not run the LinkHub enrichment step.

Keep schema changes centralized in `telemetry_columns.py`; the rest of
the simulation and analysis code derives from that file.

## Out of scope for this document

- Flight-stack contracts, modes, and NVF semantics: [flight_stack.md](flight_stack.md)
- Aero API, frames, and signs: [aero_conventions.md](aero_conventions.md)
- SITL orchestration, artifacts, and diagnosis: [sitl_testing.md](sitl_testing.md)
- EKF gating and startup evidence: [EKF_GATING.md](EKF_GATING.md) and
  [sitl_testing.md](sitl_testing.md#flight-timeline-anchors)
