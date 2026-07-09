# RAWES - Agent Guide (Short Index)

This file is intentionally short. It gives agents a quick working summary and points to the canonical docs.
Detailed design and implementation content lives in `design/*.md` and module-level docs.

## Project Snapshot

RAWES is a tethered, 4-blade autorotating rotor kite (no drive motor on the rotor).
Wind drives autorotation; cyclic steers; tether tension during reel-out drives a ground generator.

Current focus:
- Fix stack regression in `test_lua_flight_steady_sitl` first.
- After steady stack is stable, validate pumping and landing stack tests.

## Read Order (for agents)

1. `design/flight_stack.md` (system behavior and control ownership)
2. `design/simulation.md` (simulation internals and module responsibilities)
3. `design/sitl_testing.md` (stack workflow and diagnosis)
4. Topic-specific docs from the ownership map below

## Documentation Ownership (Single Source of Truth)

Use the primary doc for each topic. Other docs should link, not restate.

| Topic | Primary doc | Supporting docs |
|---|---|---|
| Flight architecture, mode ownership, AP/Lua boundaries | `design/flight_stack.md` | `design/tension_collective_control_loop.md`, `design/GUIDED_CONTROL_LOOPS.md` |
| Simulation internals (physics, sensors, controller plumbing, module map) | `design/simulation.md` | `simulation/README.md`, code docstrings |
| SITL stack workflow, lockstep, diagnosis procedure | `design/sitl_testing.md` | `simulation/analysis/diagnose_sitl.py` usage text |
| Aero interfaces and conventions | `design/aero_conventions.md` | `design/aero.md` |
| EKF gating and GPS yaw bring-up | `design/EKF_GATING.md` | `design/ekf_const_pos_mode.md` |
| ArduPilot heli PID behavior | `design/ardupilot_pids.md` | `design/GUIDED_CONTROL_LOOPS.md` |
| Swashplate geometry and sign mapping | `design/ardupilot_swashplate.md` | `simulation/swashplate.py` |
| Hardware assembly and components | `design/hardware.md` | `design/components.md`, `design/dshot.md`, `design/flap_sensor_bench.md` |
| Testing taxonomy and Lua/Python test conventions | `design/testing.md` | `simulation/pytest.ini` |
| Milestones and decisions history | `design/history.md` | this file (summary only) |

## Core Invariants (summary)

- Frames: simulation physics runs NED world + FRD body.
- body_z: rotor axis points down through disk in NED conventions.
- AP interface: ground sends commanded tension + target altitude (+ phase/substate), not measured tension.
- AP control split: orientation feedforward from commanded tension; altitude PID sets collective.
- Stack tests must validate real stack behavior (no simulation-only stabilizing hacks).
- Use GUIDED mode for flight behavior under test.

For signs, frame details, EKF gating, and mixer conventions, read the primary docs in the ownership table.

## Workflow Rules

- Do not use git history (`git log`, `git show`, `git blame`) for diagnosis unless user asks.
- For failing simtest/stack test, validate telemetry/log quality before root-cause analysis.
- Keep telemetry schema centralized in `simulation/telemetry_columns.py`.
- Keep `controller.py` aligned with `simulation/scripts/rawes.lua` behavior.
- Keep `simulation/scripts/rawes_test_surface.lua` exports in sync with needed Lua test symbols.

## Test Entry Points

- Unit: `bash test.sh unit ...`
- Simtest: `bash test.sh simtest ...`
- Stack (Docker): `bash test.sh stack ...`

Use `design/sitl_testing.md` for stack-specific run/diagnose flow.

## File Placement Rules

- Temporary and working files must go in `tmp/` at repo root.
- One-off diagnostic scripts belong in `simulation/tests/oneoff/`.
- Reusable analysis tooling belongs in `simulation/analysis/`.

## Agent Editing Policy for Docs

When updating documentation:
- Edit the primary owner doc for the topic.
- In other docs, keep only short context + link to the owner doc.
- Avoid duplicating long parameter tables or algorithm walkthroughs across multiple files.
- If ownership changes, update this map first.
