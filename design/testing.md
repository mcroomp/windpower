# Testing

This document owns the Windows-native test taxonomy, simtest conventions, and
Lua/Python test support paths. Full-stack Docker execution for `tests/sitl/`
lives in [sitl_testing.md](sitl_testing.md); this file links to that workflow
instead of restating it.

## Current test tiers

| Tier | Paths | Runtime | Markers | Notes |
|---|---|---|---|---|
| Unit | `tests/unit/` | Windows-native Python | default, optional `expensive` | Fastest feedback; no Docker |
| Simtest | `tests/simtests/` | Windows-native Python | `simtest` | Full physics loop, writes logs under `simulation/logs/` |
| SITL stack | `tests/sitl/` | Docker via `bash test.sh stack ...` | `sitl` | Owned by [sitl_testing.md](sitl_testing.md) |

The exact current inventory should be queried from the repo rather than copied
into a long static table:

```powershell
uv run python -m pytest --collect-only -q tests\unit
uv run python -m pytest --collect-only -q tests\simtests
```

List SITL tests through the Docker runner ([sitl_testing.md](sitl_testing.md)),
never host-side pytest. `tests/envelope/` and `tests/hil/` are separate suites
not covered by the commands above; `tests/hil/` talks to real hardware and has
its own prerequisites.

## Running unit and simtests

From the repo root:

Python dependencies are declared only in `pyproject.toml` and resolved in `uv.lock`.
Use `uv add` (with the appropriate `--dev` or `--optional` selection) to change
dependencies, and commit both files. Setup, CI, and Docker consume the lockfile
with `uv sync --locked`; there are no separate requirements files.

```powershell
uv sync --dev
uv sync --dev --extra simulation   # required for physics-heavy simtests

# Unit tests
uv run python -m pytest tests\unit -q
uv run python -m pytest tests\unit -m "not expensive" -q

# Simtests
uv run python -m pytest tests\simtests -q
uv run python -m pytest tests\simtests\test_generate_ic.py::test_create_ic -s

# Combined Windows-native run
uv run python -m pytest tests\unit tests\simtests -q
```

Do **not** run `tests/sitl/` with host-side `pytest`; use
[sitl_testing.md](sitl_testing.md).

The streaming wrapper is also available as
`uv run python simulation/run_tests.py --filter summary tests/unit -q`.
It launches pytest through the project uv environment without synchronizing;
run `uv sync --dev --extra simulation` explicitly after dependency changes.

## Python linting

Python 3.14 is the minimum runtime. Local setup upgrades within the minor selected
by `.python-version`; see [runtime setup](../README.md#running-tests).
CI and simulation containers pin their own Python patch releases.
Development dependencies include Ruff, with configuration in `pyproject.toml`.
The **Modernization + simplification** profile selects Ruff's default correctness
rules (`E4`, `E7`, `E9`, `F`), pyupgrade (`UP`), and flake8-simplify (`SIM`),
targeting Python 3.14. It does not impose a formatter gate.

```powershell
uv run ruff check .
uv run ruff check . --fix   # safe fixes only; review changes before committing
uv run python -m pytest tests/lint --run-lint -m lint -v
```

Archived code, scratch files, and generated run logs are excluded. Protocol
enum names retain their generated spelling; narrowly documented inline
exceptions preserve optional-dependency skips, ArduPilot gain names, and file
handles whose ownership spans multiple calls.

`setup.cmd` and the default `setup.sh` setup/build flow fail before UI/LinkHub
builds if Ruff or Pyright fails. CI's lint job gates both Python tests and the stack image
build. Docker `build` and `build-lite` commands lint a read-only mount of the
repository with the image's Python, Ruff, and Pyright before reporting success, including
cache-hit builds.

Ruff and Pyright are tests under `tests/lint/`, marked `lint` and skipped by
default. Pass `--run-lint` to enable them; `-m lint` alone does not enable them.
CI's dedicated lint job explicitly runs this category and gates downstream jobs.
Ordinary unit/simtest sessions, collection-only invocations, and Docker stack
workers do not launch static checks. The stack runner's pre-worker build check
still validates the image/source combination once before starting workers.
Lint/type errors, warnings, or missing tools fail the enabled lint tests.

For standalone type diagnostics, install development dependencies and the simulation
extra (`uv sync --dev --extra simulation`), then run `uv run pyright --warnings`.
The standard-mode configuration in `pyrightconfig.json` also enables
`reportMissingParameterType` as an error: function parameters must have explicit
annotations (apart from inferred `self`/`cls` and contextual callbacks).
The `simulation` and `groundstation` packages use strict checking, including
unknown-type diagnostics. Check either package with `uv run pyright simulation`
or `uv run pyright groundstation`; run the full-project check after changing
shared contracts.
It gates builds, CI, and opt-in lint tests with no baseline error allowance.
Gates pass the active interpreter explicitly
so Docker checks never resolve packages from a mounted Windows virtual environment.
Simulation dependencies
include NeuralFoil for the SG6042 polar analysis.
Pyright uses the same Node.js as npm builds. Windows setup provides it locally;
see [runtime setup](../README.md#running-tests) for standalone command usage.
Use the major in `.node-version` rather than installing `pyright[nodejs]`.
Shared helpers used across test tiers belong in `tests/common/`; stack workers
do not synchronize `tests/unit/`.

## Markers and timeouts

`pyproject.toml` currently defines:

| Setting | Current value |
|---|---|
| Global pytest timeout | `600` seconds |
| `lint` marker | repository-wide Ruff/Pyright checks; skipped unless `--run-lint` is passed |
| `simtest` marker | full physics simulation |
| `sitl` marker | Docker-backed ArduPilot SITL test |
| `expensive` marker | loop-heavy unit test that is useful to exclude during quick local runs |

`tests/simtests/conftest.py` also auto-applies `pytest.mark.timeout(600)` to any
simtest that does not declare its own timeout explicitly.

Pumping tests retain three full cycles at 400 Hz and the 600-second default
timeout. Elastic cable/drum substeps must not multiply aero/inflow evaluations:
those remain at the caller's frame rate (see
[PhysicsCore responsibilities](simulation.md#physicscore-responsibilities)).
`tests/unit/test_torque_winch.py` checks stiff-mode convergence, aero call
counts, and 400/1600 Hz flight convergence for quasi-static, Pitt-Peters and
Oye aero. `tests/unit/test_tether_models.py` checks that moving stretched
lines converge without a long under-relaxation tail while matching independent
equilibria. No solver tolerances or flight safety gates are relaxed.

## Unit tests (`tests/unit/`)

The unit suite is intentionally broad and file-granular. Prefer collection
output and the current files under `tests\unit\` over a hand-maintained table.

Two unit tests are worth calling out explicitly because they catch subtle
integration bugs that simple round-trip checks miss:

- `tests/unit/test_swashplate_servo_directions.py` validates physical servo
  directions for cyclic commands.
- `tests/unit/test_roll_sign_chain.py` and
  `tests/unit/test_cyclic_direction_mapping.py` guard sign and ordering
  conventions through the control chain.

### Tether model validation

The cable contract and model limitations are owned by
[simulation.md](simulation.md#two-ended-tether-physics).

`tests/unit/test_tether_models.py` checks the original spring constitutive
law, payout-relative damping, elastic energy/force consistency, slack,
analytical gravity-only catenaries, endpoint reaction balance, quadratic
transverse drag, and zero drag for parallel flow. It checks 8/16/32-segment
convergence and sweeps 50/100/200 m spans, nominal 30/300 N preload, and
0/10/15 m/s crosswind. Operating-envelope endpoint forces must agree with the
32-segment reference within 1% plus 0.02 N. Warm-start wind steps must agree
with independent cold solves; failed solves must raise explicitly. Newton
regressions check recorded SciPy endpoint forces, oversized-step backtracking,
singular/nonfinite/stalled steps, and preservation of the warm-start cache on
failure.

Boundary tests verify the shared Tallak default, attachment position/rotational velocity, attachment
moment, distinct kite/winch readings in telemetry, and local-load-cell drum
energy accounting.

```powershell
uv run python -m pytest tests\unit\test_tether_models.py tests\unit\test_tether_stability.py -q
uv run python -m pytest tests\simtests\test_steady_flight_lua.py::test_tallak_tether_lua_flight -q
```

For isolated tether throughput and CPU hotspots, run
`uv run python -m analysis.profile_tether`. It compares steady, moving,
cold-start, and sagging solves, 8/16/32-segment moving solves, and the spring
model. Unprofiled repeated timings are separate from the moving 16-segment
`cProfile` capture. Results, latency percentiles, convergence diagnostics,
and profiler artifacts go to `simulation/logs/tether_profile/`. Inputs are
prebuilt; these rates exclude input construction and the rest of the physics
loop, so they are not full-stack throughput or a real-time guarantee.

The Tallak flight smoke test runs the real Lua controller for 10 simulated
seconds. It requires no bad events, 50--496 N tension, altitude within 2 m
of its starting point, and horizontal radius below 5 m. This is not a
gust-transient or long-duration qualification. The original 90-second Lua
steady-flight and pumping-cycle tests run the Tallak default as full-length
regressions. The aero-telemetry column test also uses the shared airborne IC,
and verifies that startup is taut and above ground before checking its original
aero-column assertions. It does not test ground contact or slack-line deployment.

## Simtests (`tests/simtests/`)

All simtests are current, repo-local scenarios. They use the `simtest` marker
and run on the host Python environment, not in Docker.

### Current scenario files

| File | Current purpose |
|---|---|
| `test_generate_ic.py` | Generates and verifies `simulation\steady_state_starting.json` |
| `test_steady_flight.py` | Python-controlled steady flight from the generated IC |
| `test_steady_flight_lua.py` | In-process `rawes.lua` steady flight from the same IC |
| `test_steady_flight_ic_angle_only.py` | Minimal guided-angle steady-flight replay using IC attitude only |
| `test_steady_flight_offplane.py` | Off-plane steady-flight boundedness from a rotated launch plane |
| `test_pump_cycle_unified.py` | Python pumping-cycle controller plus coupled torque-driven winch |
| `test_pump_cycle_lua.py` | Lua pumping-cycle mirror using the same coupled drum |
| `test_landing.py` | Python landing prototype; currently skipped pending controller rework |
| `test_landing_lua.py` | Lua landing prototype; currently skipped (Lua landing mode 4 is not implemented) |
| `test_sensor_closed_loop.py` | Sensor consistency tests; the sustained closed-loop case is currently skipped for rotation-matrix drift |
| `test_ground_liftoff.py` | Bounded tensioned Lua takeoff to at least 50 m payout and established inclined steady-flight orientation |
| `test_yaw_regulation_lua.py` | Closed-loop Lua yaw-trim observer against the torque plant |

### Shared simtest machinery

| File | Role |
|---|---|
| `tests/simtests/conftest.py` | registers `simtest`, applies timeout defaults, provides `simtest_log` |
| `tests/simtests/simtest_ic.py` | loads `steady_state_starting.json` |
| `tests/simtests/simtest_runner.py` | `PhysicsRunner` wrapper around `PhysicsCore` |
| `simulation/simtest_log.py` | `SimtestLog` and `BadEventLog`; writes `params.json` and `simtest.log` under `simulation/logs/<test_name>/` |

### Steady-state IC ownership

`simulation\steady_state_starting.json` has one writer and many readers:

- **Only**
  `tests\simtests\test_generate_ic.py::test_create_ic`
  writes the file.
- IC-based unit/simtests must load it through
  `tests\simtests\simtest_ic.py`.

Regenerate it after a change that affects the equilibrium state:

```powershell
uv run python -m pytest tests\simtests\test_generate_ic.py::test_create_ic -s
```

## Lua/Python test infrastructure

The in-process Lua test surface is shared across unit tests and simtests.

| File | Current role |
|---|---|
| `simulation/rawes_lua_harness.py` | `RawesLua` host-side harness for `scripts/rawes.lua` |
| `scripts/rawes_test_surface.lua` | exports `_rawes_fns` and other test-visible Lua internals |
| `tests/common/mock_ardupilot.py` | shared adapter for Lua-backed and Python-equivalent control paths |
| `groundstation/rawes_modes.py` | canonical RAWES mode/substate constants and helpers |

Keep these in sync:

- When `scripts/rawes.lua` gains a new module-level local that tests need,
  export it from `scripts/rawes_test_surface.lua` in the same change.
- Keep `_PumpingPythonMode` altitude/tension logic aligned with Lua. Its cyclic
  path differs: Python computes body-z-error rate targets, Lua sends absolute
  angles with zero rate inputs. Use Lua-backed tests and SITL to
  test deployed cyclic behavior (see
  [flight_stack.md](flight_stack.md#49-simulation-mapping)).

## Simtest outputs and telemetry

The `simtest_log` fixture creates `simulation\logs\<test_name>\` and currently
stores:

- `params.json` from `simulation.simtest_log.SimtestLog.dump_params_json()`;
- `simtest.log` when the test writes a summary;
- `telemetry.csv` when the test calls `write_telemetry(...)`;
- any scenario-specific extra artifacts the test writes beside them.

`tests/common/mock_ardupilot.py` rate-gates telemetry with
`_tel_every_from_env()`, which reads `RAWES_TEL_HZ` and defaults to `20 Hz`.

### Takeoff scenario limits

`test_ground_liftoff.py` is an enabled simulation regression, not a hardware
takeoff procedure. It starts with a preloaded elastic tether and a pre-spun rotor,
with the winch governed throughout launch: there is no slack/free-spool phase.
The launch tension target is 100 N throughout, not a hard lower bound; a load
around 50 N is acceptable. Both the winch and flight stack
receive that command, not physics-truth measured tension. Actual slack (zero
tension) and the tether breaking load are hard failures.

The test stops immediately on slack, overload, ground contact, non-finite
altitude/load, failure to lift off within 20 simulated seconds, or failure to
reach the takeoff-to-steady transition within 60 simulated seconds. Tension
limits include every physics substep, not just logged frames. It saves the
exact terminal frame and stop reason before asserting failure.

Takeoff requests a 50-degree downwind lean above the 2 m clearance height.
Handoff requires at least 8 m payout, a steady body-axis target tilted at
least 30 degrees, alignment within 8 degrees, and body rates at most
0.5 rad/s. Success requires tracking the steady controller's
inclined body-axis target (at least 30 degrees from down, within 5 degrees,
body rates at most 0.5 rad/s) continuously for 0.2 seconds, with at least
50 m of unstretched tether paid out. Short-tether alignment is not success.
The commanded altitude follows a 25-degree tether elevation, with a 3 m
minimum and length capped at 50 m for the altitude calculation. After liftoff
the winch receives a 0.8 m/s payout cruise. It stops when both requirements
hold; there is no 100 m payout requirement or post-success flight. Reaching
50 m starts a 45-second orientation-settling deadline. The overall simulation
bound is 150 seconds and the wall-clock timeout is 900 seconds. Progress
telemetry is saved every 10 simulated seconds as well as at termination.

The scenario explicitly selects `SpringDamperTether`, a straight elastic cable;
other scenarios retain the default quasi-static cable. Preload uses the actual axle attachment
distance, not hub-centre distance.

The winch receives only the production cable-side command and anchor-end load-cell
boundary. Its finite-inertia torque plant and the elastic tether are stepped
together; physical assumptions are owned by [simulation.md](simulation.md#winch-and-ground-side-boundaries).
Do not substitute hub-truth length tracking, a slack launch, or a constant-force
tether to make the test pass.

## Telemetry schema change checklist

`simulation/telemetry_columns.py` is the schema source of truth. Its current
authoritative structure is `COLUMN_GROUPS`; `COLUMNS`, `COLUMN_SOURCES`, and
`ASYNC_MAV_COLUMNS` are derived from it.

When the schema changes:

1. Update `simulation/telemetry_columns.py` (`COLUMN_GROUPS` first).
2. Update `simulation/telemetry_csv.py` so `TelRow` still matches the schema.
3. Update explicit row construction sites, especially
   `tests/sitl/torque/torque_test_utils.py`.
4. Update mediator row writers and any simtest-side telemetry overrides.
5. If the change touches async MAVLink-enriched fields, update
   `analysis/enrich_sitl_telemetry.py` as well.

If these drift apart, failures usually surface only when a test actually writes
telemetry.

## Common testing pitfall: symmetric round-trip bugs

Round-trip tests alone are not enough for bidirectional transforms.

If both the forward and inverse path swap the same axes or parameters,
`forward(x) -> inverse(...) == x` can still pass while the real actuator outputs
are wrong. That is why the suite keeps directional assertions such as
`test_swashplate_servo_directions.py` in addition to inverse/round-trip checks.

When adding a new transform with an inverse, include at least one test that
checks the physically meaningful intermediate result, not just the round trip.
