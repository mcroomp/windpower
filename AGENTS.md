# RAWES - Agent Guide (Short Index)

This file is intentionally short. It gives agents a quick working summary and points to the canonical docs.
Detailed design and implementation content lives in `design/*.md` and module-level docs.

## Project Snapshot

RAWES is a tethered, 4-blade autorotating rotor kite (no drive motor on the rotor).
Wind drives autorotation; cyclic steers; tether tension during reel-out drives a ground generator.

## Agreed Runtime Architecture

- The mediator is the simulated physical world and lockstep adapter only: dynamics,
  aero, tether, wind, sensors, simulated hardware plants, actuator application,
  physics events, and raw physics telemetry.
- ArduPilot owns estimation, modes, attitude/rate control, Lua behavior, and servo
  mixing. LinkHub is the sole MAVLink owner and journal.
- Production ground-control policy runs separately from the mediator and lives in
  `groundstation/`; tests may host it in-process but must use production command
  boundaries. Simulation-only hardware stand-ins remain in `simulation/`.
- The SITL harness owns process orchestration, timeouts, artifacts, and post-run
  enrichment. It combines mediator physics telemetry with LinkHub observations
  after the run; the mediator must not consume MAVLink merely to decorate CSV rows.

## Repository Layout

The repo root is a single Python distribution (`pyproject.toml`, name `rawes`) containing
9 first-party top-level packages, installed in editable mode by `uv sync`.
Import them as plain dotted
packages (`from simulation.controller import ...`, `from analysis.flight_log import ...`) —
there are no `sys.path.insert()` hacks anywhere in the codebase.

| Package | Contents |
|---|---|
| `simulation/` | Physics/aero/EKF-adjacent simulation runtime, Lua/Python flight-stack modules (`mediator.py`, `controller.py`, `param_defaults.py`, `swashplate.py`, `frames.py`, `winch.py`, `winch_node.py`, `comms.py` (`VirtualComms`), ...), `requirements.txt`, `Dockerfile`, `logs/` |
| `groundstation/` | Genuinely-production ground-station/flight-planner code, split out from `simulation/`: `pumping_planner.py`, `landing_planner.py`, `rawes_modes.py` (protocol constants), `mavlink_log.py`, `ekf_flags.py`, `winch_protocol.py` (`WinchCommand`/`WinchTelemetry` wire format), `unified_ground.py` (`GcsComms` production comms adapter). Distinction: code that will genuinely run on the real ground station/flight planner lives here; code that stands in for not-yet-built hardware (e.g. the winch-node control loop in `simulation/winch.py`) stays in `simulation/` |
| `arduloop/` | Self-contained Python port of ArduPilot's traditional-heli attitude/rate-control stack (used by the in-process mock ArduPilot) |
| `calibrate/` | Bench calibration REPL/tooling for hardware bring-up |
| `envelope/` | Flight-envelope map computation (`compute_map.py`) and related analysis |
| `analysis/` | Post-run diagnosis/report scripts (all read `simulation/logs/{test_name}/...`) |
| `viz3d/` | 3D telemetry playback and torque visualizers |
| `scripts/` | Deployed Lua flight scripts (`rawes.lua`, `rawes_test_surface.lua`) and standalone runtime scripts (`sitl_bench.py`) |
| `tests/` | All test suites: `tests/unit`, `tests/simtests`, `tests/sitl` (Docker/SITL), `tests/oneoff`, `tests/common` |

Non-package top-level directories: `design/` (owner docs), `documents/`, `hardware/`,
`presentations/`, `felix/`, `am32config/` (ESC config tool, separate `package.json`),
`linkhub/` (standalone Rust transport and diagnostic gateway), `linkhub-ui/` (compiled
TypeScript/Three.js browser control and telemetry UI served by LinkHub), `tmp/`
(scratch/working files only).

`simulation/logs/` is the single log root for every test tier (unit fixtures, simtests, and
SITL stack runs all write there) — it did not move when the other packages were promoted to
top-level.

Use `uv` as the standard Python environment and dependency manager for local work:
`uv sync --dev` provisions the lightweight hardware environment and `uv run ...` executes
commands. Use `uv sync --dev --extra simulation` for the full scientific stack. Do not
install project dependencies with pip or build a packaged installer for the channel service.
On this Windows workstation, `UV_NO_SYNC=1` is configured as a persistent user environment
variable, so ordinary `uv run ...` commands do not perform dependency synchronization.
Run `uv sync` explicitly after changing dependencies or pulling a lockfile update.

## Task-scoped reading (for agents)

Do not preload the design library or read every document in sequence. Start with
the relevant entry below and read only sections needed for the task; follow
cross-links when the work actually crosses an ownership boundary.

| Work area | Read directly |
|---|---|
| Flight modes, Lua behavior, ground/AP command contracts | [flight_stack.md](design/flight_stack.md) |
| Arm/disarm, passive startup, any hardware operation | [arming.md](design/arming.md); also [HARDWARE_STARTUP.md](HARDWARE_STARTUP.md) before touching hardware |
| Calibration commands, recording, Lua deployment | [calibration.md](design/calibration.md), plus the safety documents above for hardware work |
| LinkHub service, HTTP API, journal queries | [LinkHub README](linkhub/README.md); [architecture](design/linkhub.md) for ownership, cursor, protocol, or rate-policy changes |
| Browser UI | [UI README](linkhub-ui/README.md); read LinkHub architecture only when changing its API boundary |
| Python HTTP client | [client README](linkhub_client/README.md) |
| Simulation physics, sensors, actuator plants, mediator | [simulation.md](design/simulation.md) |
| Unit/simtest conventions, Lua/Python parity | [testing.md](design/testing.md) |
| Docker/SITL execution, diagnosis, or IC-start timeline (`kinematic_exit`, `t_rel`) | [sitl_testing.md](design/sitl_testing.md) |
| ArduPilot attitude/rate internals or Python port | [GUIDED_CONTROL_LOOPS.md](design/GUIDED_CONTROL_LOOPS.md) or [ArduLoop README](arduloop/README.md), according to the implementation being changed |
| Aero axes, signs, units, `dynbem` interface | [aero_conventions.md](design/aero_conventions.md) |
| GPS/yaw aiding failure, `const_pos_mode` | [EKF_GATING.md](design/EKF_GATING.md) |
| Airframe geometry, components, swash mapping, yaw motor / DShot / RPM | [hardware.md](design/hardware.md) |

The hardware-session log contains observations and next diagnostic steps, not
authority to override current safety procedures. Update it after each hardware
session. Update the arming owner in the same change whenever a source or test
finding changes a critical ArduPilot arm/disarm fact.

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

## Pylance Responsiveness

- Issue Pylance MCP/LSP requests serially. In particular, do not batch multiple
  `textDocument/diagnostic` calls in parallel: concurrent diagnostics can stall the
  Pylance MCP bridge until its request deadline even though each request completes
  quickly on its own.
- `pyrightconfig.json` is the source of truth for analyzed project roots. Before
  blaming workspace size for a timeout, query Pylance's workspace root, effective
  settings, and user-file list, then retry one diagnostic serially.
- If even one serial Pylance request times out, restart Pylance before retrying.
  Do not send more concurrent requests to a server that is already unresponsive.

## Documentation Ownership (Single Source of Truth)

Use the primary doc for each topic. Other docs should link, not restate.

The canonical ownership map and complete document list live in
[README.md -- Documentation Map](README.md#documentation-map); the
Task-scoped reading table above routes directly to the owner for a task.
Keep both current rather than maintaining another table here.
Module READMEs own orientation and local usage; design owners hold detailed
behavior. Historical evidence must not override a current owner procedure.

Parameter-reference ownership note:
- Canonical place for ArduPilot parameter defaults and inline explanations is `tests/sitl/copter-heli.parm`.
- Canonical place for RAWES_* parameter defaults and inline explanations is `tests/sitl/rawes_common_defaults.parm`.
- If a parameter explanation changes, update the owning `.parm` file first; other docs should link to it instead of duplicating bitmasks/tables.

## MAVLink Log Diagnosis (Agent Critical)

For ANY problematic hardware or SITL run, query LinkHub's canonical journal
with `linkhub query` before writing one-off parsing code. Use the native
`types`, `show`, `count`, `stats`, `armed`, `statustext`, `nvf`, `param`, and
`diagnostics` subcommands; pipe `show --json` into `jq` for composed analysis.
See `design/linkhub.md` for the full interface. New runs must not export a
duplicate `mavlink.jsonl`.

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
- Canonical hardware safe-off is one invariant across every normal/forced
  arm-disarm cycle and every calibration run exit: confirmed disarmed,
  `RAWES_MODE=0`, ACRO, `H_FLYBAR_MODE=1`, `H_SV_MAN=0`,
  `SERVO9_FUNCTION=36` (DDFP mapping established at boot), `H_YAW_TRIM=0`,
  output 9 verified off, and neutral swash outputs from Lua's disarmed mode-0
  neutral hold. ArduPilot forces DDFP off while disarmed; runtime
  `SERVO9_FUNCTION` writes do not rebuild the live output map. Cleanup paths
  must converge on `_set_safe_off_state()` and must not unassign/reassign the
  motor function at runtime.
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

For signs, frame details, EKF gating, and mixer conventions, use the direct links
in Task-scoped reading; do not preload unrelated references.

## DShot Setup (Agent Critical)

Canonical owner doc: [design/hardware.md](design/hardware.md) (wiring, RPM
conversion, SITL exclusion); parameter values live in
`tests/sitl/rawes_common_defaults.parm`. BLHeli/DShot params
(`SERVO9_*`, `SERVO_BLH_*`, `SERVO_DSHOT_*`, `RPM1_*`) are intentionally
excluded from SITL boot verification (`tests/sitl/stack_utils.py` ->
`SITL_UNSUPPORTED_PARAMS`) because ArduCopter-heli SITL does not compile the
BLHeli backend and drives output 9 as plain PWM — see `design/sitl_testing.md`.

## Workflow Rules

- **Critical — do not bypass broken tooling.** If a required build, test runner,
  language server, MCP query, or other project tool does not work as expected,
  stop the task and diagnose the tool failure first. Do not substitute a weaker
  tool, infer the missing result, skip the validation, or continue through an
  alternate path merely to make progress. Restore the intended tool and rerun the
  original operation. If the failure cannot be understood and fixed, stop and ask
  the user to investigate rather than bypassing it.
- **Critical — the agent may run in either Windows or WSL, but every Docker
  operation must run through WSL; never use or probe Docker Desktop's native
  Windows engine.** Do not invoke `wsl` manually. When launched from Windows,
  `test.sh` and the Docker subcommands of `setup.sh` automatically re-invoke
  themselves inside WSL; when already in WSL, they run there directly. Use
  `bash test.sh ...` or `bash setup.sh build` from the current environment and
  let the scripts choose the Docker execution path. Do not wrap commands in
  `wsl -e bash -lc "..."`.
- **Critical — run only one Cargo command at a time for `linkhub/`.** Cargo
  commands share `linkhub/target`; overlapping checks, tests, or builds contend
  for its lock and can appear stuck while duplicating expensive dependency
  compilation. Before starting any Cargo command, inspect active shell sessions
  and Cargo/rustc process trees. If an earlier task-owned Cargo command is stale
  or superseded, stop its specific owning shell/process tree and confirm it has
  exited before launching the replacement. Never start a second Cargo command
  merely because the first has not printed output, and never kill Cargo/rustc
  processes by name or disturb unrelated editor/rust-analyzer processes.
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
| Unit | `uv run python -m pytest tests/unit` | (none) | Fast; no physics sim |
| Simtest | `uv run python -m pytest tests/simtests` | `simtest` | Python physics loop; seconds–minutes |
| Stack | `bash test.sh stack [-n N]` | `sitl` | ArduPilot SITL in Docker |

## Hardware Calibration Connection

- On the first hardware operation in a conversation, run `python -m calibrate`
  without `--port` or `--baud` so it auto-detects the active Pixhawk connection.
- After a successful scan, reuse the detected port and baud for subsequent
  one-shot commands in that conversation/session only.
- Never assume a port or baud from an earlier session. If the remembered
  connection fails, fall back immediately to `python -m calibrate` without
  connection parameters instead of trying guessed ports.
- An agent may upload `scripts/rawes.lua` automatically only when the selected
  port is positively identified as the flight controller's native direct-USB
  interface. Verify the selected port with `serial.tools.list_ports` and require
  board-specific USB identity (VID/PID plus device serial/location), not merely
  a `COM` name or the fact that the adapter itself uses USB. The current Pixhawk
  6C native interface enumerates as `VID:PID=3162:0053` with device serial
  `160031001751343131363538` (COM7/COM8 interfaces). Reconfirm this metadata
  each hardware session; do not assume the port assignment persists.
- Never upload Lua through a SiK/telemetry radio, generic USB-serial adapter, or
  any connection whose metadata is missing or ambiguous. If direct USB cannot
  be proven, stop before deployment and give the operator the exact upload
  command to run manually.
- Before an automatic direct-USB upload, run the focused Lua/control tests and
  require them to pass. After upload, verify the remote script size, reboot,
  auto-detect again without connection arguments, and restore canonical
  safe-off before any armed test.

Where test logs land (agent-critical — do not guess this):
- Every tier writes to `simulation/logs/<name>/` (params.json, simtest.log/worker.log,
  telemetry.csv).
- `<name>` is the **pytest test *function* name** (`request.node.name`), NOT the
  test *file* name and NOT a `-k` substring — these often differ (e.g.
  `test_pump_cycle_lua.py`'s live test is `def test_lua_pumping_unified(...)`, so
  its logs are in `simulation/logs/test_lua_pumping_unified/`). Check the actual
  `def test_...(` line (or the test's own printed `log:` line) before assuming a path.

SITL IC-start timeline rule (agent-critical):
- For SITL flight diagnosis, use one shared timeline anchored at the IC-start flow.
- Treat `t_sim` with the `kinematic_exit` event as the canonical phase boundary for
  release-to-flight comparisons across steady/passive/pumping/landing stack tests.
- Canonical definition and per-phase markers live in [Flight timeline anchors](design/sitl_testing.md#flight-timeline-anchors).

Stack-test execution rule (CRITICAL, non-negotiable):
- For any test under `tests/sitl/**`, ALWAYS use `bash test.sh stack -n 4 ...`.
- NEVER run SITL tests with host-side pytest commands like
    `uv run python -m pytest tests/sitl/...`.
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

When the user asks to "show me the visualization" (or similar) with no
mention of exporting/saving, run the interactive tool as a plain foreground
command with **no `--export` flag** — a real display is available in this
environment, so the tool opens its own interactive window directly; do not
default to generating a GIF/PNG file instead. Only pass `--export` when the
user explicitly asks to save/export a file.

Always pass the explicit telemetry path `simulation/logs/<test_name>/telemetry.csv`
for the specific test just run/discussed — don't guess a filename or omit the
path.

Flight telemetry (pumping, steady, passive SITL runs):
```
uv run python viz3d/visualize_3d.py simulation/logs/<test_name>/telemetry.csv
```
Example — most recent pumping SITL run:
```
uv run python viz3d/visualize_3d.py simulation/logs/test_pumping_cycle_lua_sitl/telemetry.csv
```

Counter-torque motor telemetry (torque SITL runs):
```
uv run python viz3d/visualize_torque.py simulation/logs/<test_name>/telemetry.csv
```
Example — yaw regulation run:
```
uv run python viz3d/visualize_torque.py simulation/logs/test_yaw_regulation_sitl/telemetry.csv
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
- If ownership changes, update the [README ownership map](README.md#documentation-map)
  first and keep the task-scoped links above in sync.

**If a mistake was caused by stale documentation, fix that documentation in the same commit
as the code fix.** The test for "stale" is: would a future agent reading only the docs make
the same mistake? If yes, the doc is stale and must be updated before the session closes.
This applies to:
- Design docs (`design/*.md`) describing parameters, modes, or control architecture.
- `AGENTS.md` workflow rules and invariants.
- Parm files (`*.parm`) that document defaults.
- Repo memory files (`/memories/repo/`) that summarise past decisions.
Do NOT wait for the user to notice. Fix the doc immediately when the stale reference is identified.
