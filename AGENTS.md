# RAWES - Agent Guide (Short Index)

This file is intentionally short. It gives agents a quick working summary and points to the canonical docs.
Detailed design and implementation content lives in `design/*.md` and module-level docs.

## Project Snapshot

RAWES is a tethered, 4-blade autorotating rotor kite (no drive motor on the rotor).
Wind drives autorotation; cyclic steers; tether tension during reel-out drives a ground generator.

Current focus:
- Run `bash test.sh stack -n 1 -k test_lua_flight_steady_sitl` to validate the steady flight SITL stack.
- After steady stack passes, validate pumping and landing stack tests.

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
- Controller layer (Lua + Python mock) works in thrust [0..1]. Physics layer works in collective_rad.
  Single conversion point: `thrust_to_coll_rad()` in `simulation/param_defaults.py`.
  Never convert thrust→rad→thrust in a roundtrip; compute in thrust and map once at the physics boundary.
- Stack tests must validate real stack behavior (no simulation-only stabilizing hacks).
- Use GUIDED mode for flight behavior under test.
- When roll and pitch appear together as paired values (params, tuple returns,
  unpacking, CSV columns, helper args), always use `roll, pitch` order.
  Do not introduce `pitch, roll` ordering unless an external interface
  explicitly requires it; if so, add an inline comment at that boundary.

## RAWES_* Parameter Mapping

All Lua configuration uses RAWES_* script-generated params (param:add_table key 77, prefix "RAWES_").
Canonical parameter list (mirrors `param:add_param` calls in rawes.lua):

| Param         | Default | Purpose                                             |
|---|---|---|
| RAWES_MODE    | 0       | Mode selector (0=none,1=steady,3=passive,4=landing) |
| RAWES_YAW_SLP | 0       | Yaw motor slope [RPM/µs], 0=bench default           |
| RAWES_KP_ALT  | 0.0263  | Altitude P gain [thrust/m]                          |
| RAWES_KI_ALT  | 0.0026  | Altitude I gain [thrust/m·s]                        |
| RAWES_KD_VZ   | 0.105   | Vertical-speed damping [thrust/(m/s)]               |
| RAWES_KP_EL   | 2.5     | In-plane (elevation) position rate-P [rad/s per m]  |
| RAWES_KP_AZ   | 0.5     | Crosswind (azimuth) position rate-P [rad/s per m]   |
| RAWES_KD_EL   | 0.0     | In-plane position rate-D [rad/s per (m/s)]          |
| RAWES_CWMAX   | 0.6     | Position rate saturation [rad/s]                    |
| RAWES_SLW     | 0.40    | Elevation/body_z slew rate limit [rad/s]            |
| RAWES_TEL_HZ  | 2.0     | Diagnostic NVF telemetry emission rate [Hz]         |
| RAWES_YFF_MAX | 0.7     | Yaw trim clamp upper bound [throttle]               |
| RAWES_YFF_TAU | 0.3     | Yaw trim low-pass time constant [s]                 |
| RAWES_TRP     | 2.0     | Tension feedforward ramp time constant [s]          |

Ground→Lua NAMED_VALUE_FLOAT interface (not AP params):

| NVF key    | Purpose                                                          |
|---|---|
| RAWES_ALT  | Target altitude [m] above anchor                                 |
| RAWES_TEN  | Target/feed-forward tether tension [N]                           |
| RAWES_SUB  | Substate (0=hold,1=reel_out,2=transition,3=reel_in,4=transition_back) |
| RAWES_ARM  | Optional disarm timer [ms until forced disarm]                   |
| RAWES_ANN  | Anchor North from EKF origin [m]                                 |
| RAWES_ANE  | Anchor East from EKF origin [m]                                  |
| RAWES_AND  | Anchor Down from EKF origin [m]                                  |
| RAWES_THR  | IC thrust [0..1] (passive seed; committed atomically with RIC/PIC) |
| RAWES_RIC  | IC roll [rad]                                                    |
| RAWES_PIC  | IC pitch [rad]                                                   |
| RAWES_YIC  | Optional fixed yaw target [rad] for MODE_PASSIVE                 |

Set RAWES_MODE per-test; other RAWES_* are in rawes_common_defaults.parm.

For signs, frame details, EKF gating, and mixer conventions, read the primary docs in the ownership table.

## DShot Setup (Agent Critical)

Use this as the quick contract-level reference for the yaw-motor ESC path.
Canonical long-form owner doc is `design/dshot.md`.

Active hardware-default parameters (from `simulation/tests/sitl/rawes_common_defaults.parm`):
- `SERVO9_FUNCTION=36` (Motor4 on output 9), `SERVO9_MIN=1000`, `SERVO9_MAX=2000`, `SERVO9_TRIM=1000`
- `SERVO_BLH_MASK=256` (output 9)
- `SERVO_BLH_BDMASK=256` (bidirectional DShot on output 9)
- `SERVO_BLH_AUTO=0` (manual masks)
- `SERVO_BLH_OTYPE=5` (DShot300)
- `SERVO_BLH_POLES=22`
- `SERVO_DSHOT_ESC=1` (AM32 telemetry decode)
- `SERVO_DSHOT_RATE=0`
- `BRD_IO_DSHOT=0` (FMU output path)
- `RPM1_TYPE=5`, `RPM1_ESC_MASK=256` (ESC telemetry routed from output 9)

SITL behavior:
- These BLHeli/DShot params are intentionally excluded from SITL boot verification
    (`simulation/tests/sitl/stack_utils.py` -> `SITL_UNSUPPORTED_PARAMS`) because
    ArduCopter-heli SITL does not compile the BLHeli backend and drives output 9 as PWM.

## Workflow Rules

- Do not use git history (`git log`, `git show`, `git blame`) for diagnosis unless user asks.
- Do not preserve backward-compatibility parameters, fields, aliases, or shims when making code changes.
- Assume no external callers: prefer a clean cutover and remove legacy paths in the same change to avoid debt.
- Do not make tests pass by making gates easier or introducing hacks unless the user has explicitly asked for it.
- For failing simtest/stack test, validate telemetry/log quality before root-cause analysis.
- Keep telemetry schema centralized in `simulation/telemetry_columns.py`.
- When changing telemetry columns: edit `COLUMN_GROUPS` in `telemetry_columns.py` (the
  single source — `COLUMN_SPECS` and `COLUMNS` are derived from it automatically), update
  `TelRow` fields in `telemetry_csv.py`, NVF maps in `torque_test_utils.py`, and row-write
  dicts in `mediator.py` in the same commit. Mismatch causes `AttributeError` in
  `TelRow.to_dict()` at runtime.
- Keep `controller.py` aligned with `simulation/scripts/rawes.lua` behavior.
- Keep `simulation/scripts/rawes_test_surface.lua` exports in sync with needed Lua test symbols.
- `_PumpingPythonMode` in `simulation/tests/common/mock_ardupilot.py` is a mechanical translation
  of `rawes.lua do_steady_loop_inner()`. Variable names mirror Lua. When changing Lua altitude PID
  logic, update the Python in the same commit. Key state that must stay in sync:
  `_tension_for_bz` is a RAMPED value (τ=RAWES_TRP≈2 s) toward `_tension_cmd_n` — not a step.
  Missing this ramp caused tether slack on phase transitions (reel-out→reel-in tension change).
- Prefer module-level imports in Python. Avoid `import` statements inside functions or methods
  unless the import is genuinely optional (e.g. heavy optional dependency). Lazy imports that
  exist only to work around circular imports are a sign of bad architecture — fix the circular
  dependency by refactoring (e.g. extract a shared module, invert the dependency) rather than
  papering over it with a local import.

## Test Entry Points

There are three tiers, each with a different scope and runtime:

| Tier | Command | Marker | Notes |
|---|---|---|---|
| Unit | `.venv/Scripts/python.exe -m pytest simulation/tests/unit` | (none) | Fast; no physics sim |
| Simtest | `.venv/Scripts/python.exe -m pytest simulation/tests/simtests` | `simtest` | Python physics loop; seconds–minutes |
| Stack | `bash test.sh stack [-n N]` | `sitl` | ArduPilot SITL in Docker |

Stack-test execution rule (agent-critical):
- For any test under `simulation/tests/sitl/**`, ALWAYS use `bash test.sh stack -n 4 ...`.
- DO NOT run SITL tests with host-side pytest commands like
    `.venv/Scripts/python.exe -m pytest simulation/tests/sitl/...`.
    Those bypass the Docker stack harness and can fail with host-path issues
    (for example `/ardupilot/scripts` not existing on Windows host).
- For a single SITL test, use:
    `bash test.sh stack -n 1 -k <test_name>`
    Example:
    `bash test.sh stack -n 1 -k test_pumping_cycle_lua_sitl`
    Do NOT pipe through `tail` or `grep` — the failure summary is printed last
    and piping will truncate or hide it.

After a SITL run, **do not re-run to see the error**. The failure summary is
printed at the end of the `test.sh` output (the `=== FAILURES ===` section shows
the tail of each failure, including the assertion error). The full output is also
saved — read it directly if needed:
    `Get-Content simulation/logs/<test_name>/worker.log | Select-String "ERROR|CRITICAL|Traceback|assert|FAIL" | Select-Object -First 30`

## Visualization

Flight telemetry (pumping, steady, passive SITL runs):
```
.venv/Scripts/python.exe simulation/viz3d/visualize_3d.py simulation/logs/<test_name>/telemetry.csv
```
Example — most recent pumping SITL run:
```
.venv/Scripts/python.exe simulation/viz3d/visualize_3d.py simulation/logs/test_pumping_cycle_lua_sitl/telemetry.csv
```

Counter-torque motor telemetry (torque SITL runs):
```
.venv/Scripts/python.exe simulation/viz3d/visualize_torque.py simulation/logs/<test_name>/telemetry.csv
```
Example — yaw regulation run:
```
.venv/Scripts/python.exe simulation/viz3d/visualize_torque.py simulation/logs/test_yaw_regulation_sitl/telemetry.csv
```

Controls (both visualizers): Space = play/pause, Left/Right = step frame, +/- = speed.

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
