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
end' -l lua simulation/scripts/rawes.lua
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
| SITL stack workflow, lockstep, diagnosis procedure | `design/sitl_testing.md` | `simulation/analysis/diagnose_sitl.py` usage text |
| SITL IC-start timeline and event anchors | `design/sitl_flight_timeline.md` | `design/sitl_testing.md`, `simulation/tests/sitl/flight/conftest.py` |
| Aero interfaces and conventions | `design/aero_conventions.md` | `design/aero.md` |
| EKF gating and GPS yaw bring-up | `design/EKF_GATING.md` | `design/ekf_const_pos_mode.md` |
| ArduPilot heli control-loop behavior | `design/GUIDED_CONTROL_LOOPS.md` | `design/flight_stack.md` |
| Swashplate geometry and sign mapping | `simulation/swashplate.py` | `design/flight_stack.md` |
| Hardware assembly and components | `design/hardware.md` | `design/components.md`, `design/dshot.md`, `design/flap_sensor_bench.md` |
| Testing taxonomy and Lua/Python test conventions | `design/testing.md` | `simulation/pytest.ini` |
| Milestones and decisions history | `design/history.md` | this file (summary only) |

Parameter-reference ownership note:
- Canonical place for ArduPilot parameter defaults and inline explanations is `simulation/tests/sitl/copter-heli.parm`.
- Canonical place for RAWES_* parameter defaults and inline explanations is `simulation/tests/sitl/rawes_common_defaults.parm`.
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

RAWES mode -> vehicle control API (current `rawes.lua` behavior):

| RAWES_MODE | Mode | Vehicle API used |
|---|---|---|
| 0 | none | none |
| 3 | passive | `vehicle:set_target_rate_and_throttle` for thrust-only seed (`RAWES_THR` without `RAWES_RIC/PIC`); `vehicle:set_target_angle_and_rate_and_throttle` once full IC seed is present |
| 1 | steady | `vehicle:set_target_angle_and_rate_and_throttle` |

Steady command basis (`RAWES_MODE=1`):
- roll/pitch: derived from `bz_altitude_hold(rel, el_rad, tension_n, az_ref)` and converted by `bz_ned_to_roll_pitch(...)`.
- throttle: altitude PID around IC thrust (`RAWES_THR` as trim/seed), with vertical-speed damping and thrust slew limiting before sending to ArduPilot.

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
- `/tmp` is NOT one shared filesystem on this box — Git Bash (mingw/MSYS2, the default terminal)
  and WSL2 (used for `docker`/`test.sh stack`) each have their own separate `/tmp`:
  - A bare `> /tmp/foo.log` redirection in a Git Bash command writes to Git Bash's own `/tmp`
    (really `C:\Users\<user>\AppData\Local\Temp\foo.log` — check with `cygpath -w /tmp/foo.log`).
  - A command run via `wsl -e bash -lc "... > /tmp/foo.log"` writes inside WSL's `/tmp`
    (`\\wsl$\...\tmp\foo.log` from Windows, or `/tmp/foo.log` from *inside* another `wsl -e` call)
    — NOT reachable from a later plain Git Bash `cat /tmp/foo.log`.
  - If the redirection is written OUTSIDE the `wsl -e bash -lc "..."` quoted string (e.g.
    `wsl -e bash -lc "cmd" > /tmp/foo.log`), it's the OUTER (Git Bash) shell that owns the
    redirect, not WSL — easy to mix up.
  - A native Windows executable (e.g. `.venv/Scripts/python.exe`) invoked from Git Bash does NOT
    understand `/tmp/...` paths passed as arguments (it's a Windows process, not MSYS2-aware) —
    convert with `cygpath -w /tmp/foo.log` first, or it'll fail with `FileNotFoundError` even
    though `ls /tmp/foo.log` (from Git Bash) shows the file existing.
  - Rule of thumb: know which shell environment (Git Bash vs WSL) is actually creating/reading
    a `/tmp` path before assuming a file exists or is missing; don't conclude "no output was
    produced" just because a naive `cat`/`find` from the wrong shell doesn't see it.

## Test Entry Points

There are three tiers, each with a different scope and runtime:

| Tier | Command | Marker | Notes |
|---|---|---|---|
| Unit | `.venv/Scripts/python.exe -m pytest simulation/tests/unit` | (none) | Fast; no physics sim |
| Simtest | `.venv/Scripts/python.exe -m pytest simulation/tests/simtests` | `simtest` | Python physics loop; seconds–minutes |
| Stack | `bash test.sh stack [-n N]` | `sitl` | ArduPilot SITL in Docker |

SITL IC-start timeline rule (agent-critical):
- For SITL flight diagnosis, use one shared timeline anchored at the IC-start flow.
- Treat `t_sim` with the `kinematic_exit` event as the canonical phase boundary for
  release-to-flight comparisons across steady/passive/pumping/landing stack tests.
- Canonical definition and per-phase markers live in `design/sitl_flight_timeline.md`.

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

Long-running test commands (agent-critical, efficiency):
- Unit and simtest runs can take 1-5+ minutes. Do NOT pipe them through `| tail -N` when
  running in a mode that may background the command — a slow/idle command piped through
  `tail` produces no output until the pipeline's stdout closes, so a background poll via
  `get_terminal_output` just returns the same stale snapshot every time (wastes calls,
  looks like a hang). Run the bare command first; only pipe through `tail`/`grep` after
  confirming the run is fast enough to complete synchronously, or redirect to a file
  (`... > /tmp/out.log 2>&1`) and read/grep the file instead.
- Once a command has been moved to background, do not repeatedly call `get_terminal_output`
  in a tight loop — it will not return new content until the process actually produces more
  output or exits. Wait for the automatic completion notification instead of polling.
- Do NOT call `get_terminal_output` immediately after a command moves to background "just to
  check progress". It returns a byte-limited tail of the WHOLE terminal scrollback, not just
  the new command's output — if the new command has only printed a little so far, the tail
  can still be dominated by leftover output from earlier unrelated commands in the same
  terminal, which looks like stale/wrong output but is really just "not enough new output yet
  to push the old stuff out of the tail window". End the turn and wait for the automatic
  completion notification instead; only poll if genuinely unsure whether the process is hung
  after a long silence.
- Prefer `bash test.sh stack -n 1 -k <test_name>` (single test) while iterating; only widen
  to `-n 4`/full suite once the targeted test is confirmed passing, to keep turnaround short.

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
