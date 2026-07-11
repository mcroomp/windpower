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

Treat this mapping as contract-level. Keep it consistent across:
- `simulation/scripts/rawes.lua`
- `simulation/tests/sitl/rawes_common_defaults.parm`
- design/hardware docs that reference SCR_USER fields.

Canonical mapping:
- `SCR_USER1` -> yaw motor slope override (`YAW_RPM_PER_US`), 0 means use bench default.
- `SCR_USER2` -> Lua `KP_ALT`.
- `SCR_USER3` -> Lua `KI_ALT`.
- `SCR_USER4` -> Lua `KD_VZ`.
- `SCR_USER5` -> Lua `RATE_KP_OUTER`.
- `SCR_USER6` -> `RAWES_MODE` selector.

Do not repurpose `SCR_USER2..5` for anchor/slew or other runtime values.
Anchor/slew are NAMED_VALUE_FLOAT inputs (`RAWES_SLW`, `RAWES_ANN`, `RAWES_ANE`, `RAWES_AND`).

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
