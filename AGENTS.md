# RAWES - Agent Guide (Short Index)

This file is intentionally short. It gives agents a quick working summary and points to the canonical docs.
Detailed design and implementation content lives in `design/*.md` and module-level docs.

## Project Snapshot

RAWES is a tethered, 4-blade autorotating rotor kite (no drive motor on the rotor).
Wind drives autorotation; cyclic steers; tether tension during reel-out drives a ground generator.

## Repository Layout

The repo root is a single Python distribution (`pyproject.toml`, name `rawes`) containing
9 first-party top-level packages, installed in editable mode
(`pip install -e . --no-deps`, done by `setup.cmd`/`setup.sh`). Import them as plain dotted
packages (`from simulation.controller import ...`, `from analysis.flight_log import ...`) —
there are no `sys.path.insert()` hacks anywhere in the codebase.

| Package | Contents |
|---|---|
| `simulation/` | Physics/aero/EKF-adjacent simulation runtime, Lua/Python flight-stack modules (`mediator.py`, `controller.py`, `param_defaults.py`, `swashplate.py`, `frames.py`, `winch.py`, `winch_node.py`, `comms.py` (`VirtualComms`), ...), `requirements.txt`, `Dockerfile`, `logs/` |
| `groundstation/` | Genuinely-production ground-station/flight-planner code, split out from `simulation/`: `pumping_planner.py`, `landing_planner.py`, `gcs.py` (`RawesGCS` MAVLink client), `rawes_modes.py` (protocol constants), `mavlink_log.py`, `ekf_flags.py`, `winch_protocol.py` (`WinchCommand`/`WinchTelemetry` wire format), `unified_ground.py` (`GcsComms` production comms adapter). Distinction: code that will genuinely run on the real ground station/flight planner lives here; code that stands in for not-yet-built hardware (e.g. the winch-node control loop in `simulation/winch.py`) stays in `simulation/` |
| `arduloop/` | Self-contained Python port of ArduPilot's traditional-heli attitude/rate-control stack (used by the in-process mock ArduPilot) |
| `calibrate/` | Bench calibration REPL/tooling for hardware bring-up |
| `envelope/` | Flight-envelope map computation (`compute_map.py`) and related analysis |
| `analysis/` | Post-run diagnosis/report scripts (all read `simulation/logs/{test_name}/...`) |
| `viz3d/` | 3D telemetry playback and torque visualizers |
| `scripts/` | Deployed Lua flight scripts (`rawes.lua`, `rawes_test_surface.lua`) and standalone runtime scripts (`sitl_bench.py`, `query_hardware.py`) |
| `tests/` | All test suites: `tests/unit`, `tests/simtests`, `tests/sitl` (Docker/SITL), `tests/hil`, `tests/oneoff`, `tests/common` |

Non-package top-level directories: `design/` (owner docs), `documents/`, `hardware/`,
`presentations/`, `felix/`, `am32config/` (ESC config tool, separate `package.json`), `tmp/`
(scratch/working files only).

`simulation/logs/` is the single log root for every test tier (unit fixtures, simtests, and
SITL stack runs all write there) — it did not move when the other packages were promoted to
top-level.

## Read Order (for agents)

1. `design/flight_stack.md` (system behavior and control ownership)
2. `design/simulation.md` (simulation internals and module responsibilities)
3. `design/sitl_testing.md` (stack workflow and diagnosis)
4. Topic-specific docs from the ownership map below

## Code Search: Prefer ast-grep over grep/ripgrep

`ast-grep` (CLI: `ast-grep`, alias `sg`) is installed and available in this workspace.
For searching *source code* (Python, Lua), prefer it over `grep`/`rg`/text-based
`grep_search` because it matches on AST structure, so it ignores comments/strings and
is indentation/formatting agnostic. Still use plain text search for non-code files
(docs, `.parm` files, logs, config).

Basic invocation:
```
ast-grep run -p '<PATTERN>' [-l <LANG>] [PATHS...]
```
- `-p/--pattern`: the AST pattern to match (see below).
- `-l/--lang`: language (`python`, `lua`, etc). Optional — ast-grep infers language
  from file extension when scanning a directory, but set it explicitly when
  scanning a single file whose extension is ambiguous or when using `--stdin`.
- `PATHS`: files or directories to search (defaults to `.`).
- `-A/-B/-C <N>`: lines of context after/before/around a match (like grep).
- `-r/--rewrite <FIX>`: rewrite matched code (combine with `-i` for interactive
  confirmation, or `-U` to apply all rewrites unattended — treat `-U` as a
  hard-to-reverse bulk edit, confirm intent before running it).
- `--json[=pretty|stream|compact]`: structured output for programmatic use.

Pattern syntax (tree-sitter based):
- Meta-variables capture a single AST node: `$NAME`, `$ARGS`, `$X` (uppercase by convention).
- `$$$NAME` captures zero or more nodes (e.g. a variable-length argument list or
  statement block).
- Patterns must be syntactically valid (partial) code in the target language — write
  the pattern the way you'd write real code, using meta-variables where content varies.

Examples used/verified in this repo:
```
# Find all calls to a function across the simulation/ package
ast-grep run -p 'thrust_to_coll_rad($$$ARGS)' simulation

# Find a Python function definition (any body) in one file
ast-grep run -p 'def $NAME($$$ARGS):
    $$$BODY' simulation/param_defaults.py

# Find Lua function definitions in a script
ast-grep run -p 'function $NAME($$$ARGS)
  $$$BODY
end' -l lua scripts/rawes.lua
```

When to still use grep/`grep_search`: matching exact substrings/regex in prose,
`.parm`/`.md`/`.yml`/log files, or when you need to match across code+comments+strings
uniformly (e.g. searching for a TODO string or a parameter name that may appear in
comments).

Searching OUTSIDE this workspace (e.g. a separate `C:\repos\ardupilot` checkout):
the `grep_search`/`file_search`/`semantic_search` tools are scoped to this workspace
folder and silently return "No matches found" (a generic VS Code search-exclusion
message) for paths outside it — this does NOT mean the pattern is genuinely absent,
it means the tool couldn't search there at all. For any path outside the current
workspace folder, go straight to a terminal command (`grep`/`sed`/`rg` via
`run_in_terminal`) instead of retrying the workspace-scoped search tools.

## Documentation Ownership (Single Source of Truth)

Use the primary doc for each topic. Other docs should link, not restate.

| Topic | Primary doc | Supporting docs |
|---|---|---|
| Flight architecture, mode ownership, AP/Lua boundaries | `design/flight_stack.md` | `design/tension_collective_control_loop.md`, `design/GUIDED_CONTROL_LOOPS.md` |
| Simulation internals (physics, sensors, controller plumbing, module map) | `design/simulation.md` | `simulation/README.md`, code docstrings |
| SITL stack workflow, lockstep, diagnosis procedure | `design/sitl_testing.md` | `analysis/diagnose_sitl.py` usage text |
| SITL IC-start timeline and event anchors | `design/sitl_flight_timeline.md` | `design/sitl_testing.md`, `tests/sitl/flight/conftest.py` |
| Aero interfaces and conventions | `design/aero_conventions.md` | `design/aero.md` |
| EKF gating and GPS yaw bring-up | `design/EKF_GATING.md` | `design/ekf_const_pos_mode.md` |
| ArduPilot heli control-loop behavior | `design/GUIDED_CONTROL_LOOPS.md` | `design/flight_stack.md` |
| Swashplate geometry and sign mapping | `simulation/swashplate.py` | `design/flight_stack.md` |
| Hardware assembly and components | `design/hardware.md` | `design/components.md`, `design/dshot.md`, `design/flap_sensor_bench.md` |
| Testing taxonomy and Lua/Python test conventions | `design/testing.md` | `pyproject.toml` (`[tool.pytest.ini_options]`) |
| Milestones and decisions history | `design/history.md` | this file (summary only) |

Parameter-reference ownership note:
- Canonical place for ArduPilot parameter defaults and inline explanations is `tests/sitl/copter-heli.parm`.
- Canonical place for RAWES_* parameter defaults and inline explanations is `tests/sitl/rawes_common_defaults.parm`.
- If a parameter explanation changes, update the owning `.parm` file first; other docs should link to it instead of duplicating bitmasks/tables.

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

## RAWES_* Parameter Reference

The full RAWES_* script-generated parameter table, the NAMED_VALUE_FLOAT/INT
wire interface, and the RAWES_MODE → vehicle-API mapping are owned by
`design/flight_stack.md` (see its "RAWES_\* script-generated parameters" table
and §4.2b–4.5) — do not duplicate those tables here. Live defaults for
non-RAWES_* AP params are in `tests/sitl/rawes_common_defaults.parm`; set
`RAWES_MODE` per-test.

For signs, frame details, EKF gating, and mixer conventions, read the primary docs in the ownership table.

## DShot Setup (Agent Critical)

Canonical owner doc: `design/dshot.md` (full parameter tables, wiring, RPM
conversion). One SITL-specific gotcha not covered there: BLHeli/DShot params
(`SERVO9_*`, `SERVO_BLH_*`, `SERVO_DSHOT_*`, `RPM1_*`) are intentionally
excluded from SITL boot verification (`tests/sitl/stack_utils.py` ->
`SITL_UNSUPPORTED_PARAMS`) because ArduCopter-heli SITL does not compile the
BLHeli backend and drives output 9 as plain PWM — see `design/sitl_testing.md`.

## Workflow Rules

- Do not use `wsl` to run anything directly. `test.sh` is the only script that uses WSL, and it
  already contains the logic to re-invoke itself inside WSL when needed (for Docker access).
  Run `bash test.sh ...` (or `./test.sh ...`) from Git Bash directly — do not wrap it in
  `wsl -e bash -lc "..."` or run other commands (pytest, analysis scripts, git, etc.) via `wsl`.
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
- Keep `controller.py` aligned with `scripts/rawes.lua` behavior.
- Keep `scripts/rawes_test_surface.lua` exports in sync with needed Lua test symbols.
- `_PumpingPythonMode` in `tests/common/mock_ardupilot.py` is a mechanical translation
  of `rawes.lua do_steady_loop_inner()`. Variable names mirror Lua. When changing Lua altitude PID
  logic, update the Python in the same commit. Key state that must stay in sync:
  `_tension_for_bz` is a RAMPED value (τ=RAWES_TRP≈2 s) toward `_tension_cmd_n` — not a step.
  Missing this ramp caused tether slack on phase transitions (reel-out→reel-in tension change).
- Prefer module-level imports in Python. Avoid `import` statements inside functions or methods
  unless the import is genuinely optional (e.g. heavy optional dependency). Lazy imports that
  exist only to work around circular imports are a sign of bad architecture — fix the circular
  dependency by refactoring (e.g. extract a shared module, invert the dependency) rather than
  papering over it with a local import.
- Python 3.12+ is the project floor and should be treated as the baseline. Prefer modern Python
  features when they improve clarity and reduce boilerplate (for example `match`/`case`, `X | Y`
  union types, and 3.12 generic/type-alias syntax) rather than avoiding them for backward-
  compatibility with older interpreters.
- `/tmp` is NOT one shared filesystem on this box — Git Bash and WSL2 (used for
  `docker`/`test.sh stack`) each have their own separate `/tmp`, and native Windows
  executables invoked from Git Bash can't resolve `/tmp/...` paths at all. Full
  gotcha writeup (redirection ownership, `cygpath -w` conversion, diagnosis tips)
  is in `design/sitl_testing.md`.

## Test Entry Points

There are three tiers, each with a different scope and runtime:

| Tier | Command | Marker | Notes |
|---|---|---|---|
| Unit | `.venv/Scripts/python.exe -m pytest tests/unit` | (none) | Fast; no physics sim |
| Simtest | `.venv/Scripts/python.exe -m pytest tests/simtests` | `simtest` | Python physics loop; seconds–minutes |
| Stack | `bash test.sh stack [-n N]` | `sitl` | ArduPilot SITL in Docker |

SITL IC-start timeline rule (agent-critical):
- For SITL flight diagnosis, use one shared timeline anchored at the IC-start flow.
- Treat `t_sim` with the `kinematic_exit` event as the canonical phase boundary for
  release-to-flight comparisons across steady/passive/pumping/landing stack tests.
- Canonical definition and per-phase markers live in `design/sitl_flight_timeline.md`.

Stack-test execution rule (agent-critical):
- For any test under `tests/sitl/**`, ALWAYS use `bash test.sh stack -n 4 ...`.
- DO NOT run SITL tests with host-side pytest commands like
    `.venv/Scripts/python.exe -m pytest tests/sitl/...`.
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

Long-running test commands (agent-critical, efficiency): unit/simtest/stack runs can
take 1-5+ minutes. Do not pipe a possibly-backgrounded command through `tail`/`grep`,
and do not poll `get_terminal_output` in a tight loop — full guidance (why piping
hides output, why polling wastes calls, and the preferred iterate-in-isolation
pattern) is in `design/sitl_testing.md`.

## Visualization

Flight telemetry (pumping, steady, passive SITL runs):
```
.venv/Scripts/python.exe viz3d/visualize_3d.py simulation/logs/<test_name>/telemetry.csv
```
Example — most recent pumping SITL run:
```
.venv/Scripts/python.exe viz3d/visualize_3d.py simulation/logs/test_pumping_cycle_lua_sitl/telemetry.csv
```

Counter-torque motor telemetry (torque SITL runs):
```
.venv/Scripts/python.exe viz3d/visualize_torque.py simulation/logs/<test_name>/telemetry.csv
```
Example — yaw regulation run:
```
.venv/Scripts/python.exe viz3d/visualize_torque.py simulation/logs/test_yaw_regulation_sitl/telemetry.csv
```

Controls (both visualizers): Space = play/pause, Left/Right = step frame, +/- = speed.

## File Placement Rules

- Temporary and working files must go in `tmp/` at repo root.
- One-off diagnostic scripts belong in `tests/oneoff/`.
- Reusable analysis tooling belongs in `analysis/`.

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
