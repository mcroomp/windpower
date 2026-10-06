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
uv run python -m pytest --collect-only -q tests\sitl
```

## Running unit and simtests

From the repo root:

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

## Markers and timeouts

`pyproject.toml` currently defines:

| Setting | Current value |
|---|---|
| Global pytest timeout | `600` seconds |
| `simtest` marker | full physics simulation |
| `sitl` marker | Docker-backed ArduPilot SITL test |
| `expensive` marker | loop-heavy unit test that is useful to exclude during quick local runs |

`tests/simtests/conftest.py` also auto-applies `pytest.mark.timeout(600)` to any
simtest that does not declare its own timeout explicitly.

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
| `test_pump_cycle_unified.py` | Python pumping-cycle controller plus governed winch |
| `test_pump_cycle_lua.py` | Lua pumping-cycle mirror of the unified pumping test |
| `test_landing.py` | Python landing sequence with landing planner and winch |
| `test_landing_lua.py` | Lua landing-mode mirror of the Python landing test |
| `test_sensor_closed_loop.py` | PhysicalSensor consistency during sustained closed-loop flight |
| `test_ground_liftoff.py` | Takeoff-to-steady transition scenario; currently marked skipped while behavior is investigated |
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
- `_PumpingPythonMode` in `tests/common/mock_ardupilot.py` is a mechanical port
  of the Lua steady-loop behavior; when Lua altitude/tension logic changes,
  update the Python port in the same commit.

## Simtest outputs and telemetry

The `simtest_log` fixture creates `simulation\logs\<test_name>\` and currently
stores:

- `params.json` from `simulation.simtest_log.SimtestLog.dump_params_json()`;
- `simtest.log` when the test writes a summary;
- `telemetry.csv` when the test calls `write_telemetry(...)`;
- any scenario-specific extra artifacts the test writes beside them.

`tests/common/mock_ardupilot.py` rate-gates telemetry with
`_tel_every_from_env()`, which reads `RAWES_TEL_HZ` and defaults to `20 Hz`.

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
