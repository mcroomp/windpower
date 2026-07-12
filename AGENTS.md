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
- When roll and pitch appear together as paired values (params, tuple returns,
  unpacking, CSV columns, helper args), always use `roll, pitch` order.
  Do not introduce `pitch, roll` ordering unless an external interface
  explicitly requires it; if so, add an inline comment at that boundary.

## SCR_USER Mapping (Agent Critical)

SCR_USER1..6 are NO LONGER USED by rawes.lua. Replaced by script-generated
RAWES_* parameters (param:add_table key 77, prefix "RAWES_"). Canonical mapping:

| Old SCR_USER | New param    | Default | Purpose                            |
|---|---|---|---|
| SCR_USER1    | RAWES_YAW_SLP | 0      | Yaw motor slope [RPM/µs], 0=bench default |
| SCR_USER2    | RAWES_KP_ALT  | 0.010  | Altitude P gain                    |
| SCR_USER3    | RAWES_KI_ALT  | 0.001  | Altitude I gain                    |
| SCR_USER4    | RAWES_KD_VZ   | 0.040  | Vertical-speed damping             |
| SCR_USER5    | RAWES_KP_EL   | 2.5    | In-plane (elevation) position rate-P gain  |
| SCR_USER6    | RAWES_MODE    | 0      | Mode selector (0=none,1=steady,3=passive,4=landing) |
| *(new)*      | RAWES_KP_AZ   | 0.5    | Crosswind (azimuth) position rate-P gain   |

Runtime overrides for crosswind gains via NAMED_VALUE_FLOAT:
- `RAWES_CWP` overrides `_cw_rate_kp` (in-plane/elevation, defaults from RAWES_KP_EL)
- `RAWES_CWA` overrides `_cw_rate_kp_az` (crosswind/azimuth, defaults from RAWES_KP_AZ)
- `RAWES_CWD` overrides `_cw_rate_kd` (D term, default 0)
- `RAWES_CWM` overrides `_cw_rate_max` (saturation, default 0.6)

Set RAWES_MODE per-test; other RAWES_* are in rawes_common_defaults.parm.

For signs, frame details, EKF gating, and mixer conventions, read the primary docs in the ownership table.

## Workflow Rules

- Do not use git history (`git log`, `git show`, `git blame`) for diagnosis unless user asks.
- Do not preserve backward-compatibility parameters, fields, aliases, or shims when making code changes.
- Assume no external callers: prefer a clean cutover and remove legacy paths in the same change to avoid debt.
- Do not make tests pass by making gates easier or introducing hacks unless the user has explicitly asked for it.
- For failing simtest/stack test, validate telemetry/log quality before root-cause analysis.
- Keep telemetry schema centralized in `simulation/telemetry_columns.py`.
- When changing telemetry columns: update `TelRow` fields in `telemetry_csv.py`, NVF maps in `torque_test_utils.py`, and row-write dicts in `mediator.py` in the same commit. Mismatch causes `AttributeError` in `TelRow.to_dict()` at runtime.
- Keep `controller.py` aligned with `simulation/scripts/rawes.lua` behavior.
- Keep `simulation/scripts/rawes_test_surface.lua` exports in sync with needed Lua test symbols.

## Test Entry Points

There are three tiers, each with a different scope and runtime:

| Tier | Command | Marker | Notes |
|---|---|---|---|
| Unit | `.venv/Scripts/python.exe -m pytest simulation/tests/unit` | (none) | Fast; no physics sim |
| Simtest | `.venv/Scripts/python.exe -m pytest simulation/tests/simtests` | `simtest` | Python physics loop; seconds–minutes |
| Stack | `bash test.sh stack [-n N]` | `sitl` | ArduPilot SITL in Docker |

Stack-test execution rule (agent-critical):
- For any test under `simulation/tests/sitl/**`, ALWAYS use `bash test.sh stack ...`.
- DO NOT run SITL tests with host-side pytest commands like
    `.venv/Scripts/python.exe -m pytest simulation/tests/sitl/...`.
    Those bypass the Docker stack harness and can fail with host-path issues
    (for example `/ardupilot/scripts` not existing on Windows host).
- For a single SITL test, use:
    `bash test.sh stack -n 1 -k <test_name>`
    Example:
    `bash test.sh stack -n 1 -k test_pumping_cycle_lua_sitl`

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

**If a mistake was caused by stale documentation, fix that documentation in the same commit
as the code fix.** The test for "stale" is: would a future agent reading only the docs make
the same mistake? If yes, the doc is stale and must be updated before the session closes.
This applies to:
- Design docs (`design/*.md`) describing parameters, modes, or control architecture.
- `AGENTS.md` workflow rules and invariants.
- Parm files (`*.parm`) that document defaults.
- Repo memory files (`/memories/repo/`) that summarise past decisions.
Do NOT wait for the user to notice. Fix the doc immediately when the stale reference is identified.
