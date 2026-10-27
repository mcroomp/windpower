# SITL stack testing

This document owns the current workflow for `tests/sitl/`: running the Docker
stack, understanding its artifacts, and doing first-pass diagnosis. Windows-native
unit tests and simtests are covered by [testing.md](testing.md).

## Current entry points

Use the real Bash scripts from the repo root:

| Task | Command |
|---|---|
| Build the stack image | `bash setup.sh build` |
| Run one stack selection | `bash test.sh stack -n 1 -k test_name` |
| Run the full stack suite | `bash test.sh stack -n 4` (effective concurrency capped at 2) |
| Profile lockstep timing | `bash test.sh stack -n 1 --profile-lockstep -k test_name` |

Important current behavior verified from `test.sh` and `setup.sh`:

- `test.cmd` is only a thin wrapper around `bash.exe test.sh %*`; document and
  use `bash test.sh`, not the wrapper.
- `bash setup.sh build-lite` builds the lightweight runtime target **without**
  ArduPilot or LinkHub. It is useful for non-stack container work, but it is
  **not** sufficient for `tests/sitl/`.
- `bash test.sh stack ...` is the only supported runner for the stack suite.
  Do **not** run `tests/sitl/` with host-side `pytest`.

## Current runner contract

The stack runner is file-parallel, not test-function-parallel.

Verified behavior from `test.sh`:

- It discovers `tests/sitl/**/test_*.py` and runs one ephemeral container per
  matching **file**.
- `-n` is an upper bound only. The runner currently caps effective concurrency
  to **2** lockstep stacks because higher parallelism causes host scheduling
  stalls and UDP loss.
- `test_linkhub_stress_sitl.py` runs exclusively so the transport benchmark
  measures LinkHub rather than contention from another stack.
- Before collection the runner calls `bash setup.sh build` and probes the image
  for Python 3.12+, `arducopter-heli`, and `linkhub`.
- Each selected file gets an outer wall-clock timeout through
  `RAWES_STACK_TEST_TIMEOUT_S` (default `600` seconds).

## Architecture facts the harness assumes

- ArduPilot SITL, the mediator, and LinkHub run inside the per-test container.
- The physical mediator and ArduPilot are in lockstep at `SIM_RATE_HZ=1200`,
  i.e. three physics frames per 400 Hz ArduPilot control-loop tick.
- LinkHub is the **only** MAVLink owner and journal. The mediator does not read
  MAVLink merely to decorate telemetry, and the stack does **not** export a
  duplicate transport log.
- Post-run enrichment is owned by the harness: mediator physics telemetry is
  copied to `telemetry.physics.csv`, then
  `analysis/enrich_sitl_telemetry.py` samples LinkHub observations onto that
  timeline to produce the canonical `telemetry.csv`.

## Current per-test artifacts

Per-test artifacts land under `simulation\logs\<test_name>\`.

Common current artifacts:

| Artifact | Source |
|---|---|
| `worker.log` | `test.sh` worker stdout/stderr summary |
| `sitl.log` | ArduPilot SITL process |
| `gcs.log` | stack-side GCS/setup helpers |
| `linkhub.log` | LinkHub service process |
| `linkhub\` | copied LinkHub journal root for the run |
| `arducopter.log` | ArduPilot text process/crash output, when produced |
| `dataflash.BIN` | Recovered binary DataFlash log, when produced |

Mediator-backed fixtures also add:

| Artifact | Source |
|---|---|
| `mediator.log` | mediator process |
| `events.jsonl` | mediator event log |
| `telemetry.physics.csv` | raw mediator physics telemetry before enrichment |
| `telemetry.csv` | enriched telemetry sampled with LinkHub observations |

## Diagnosis workflow

Always query the canonical LinkHub journal first:

```powershell
linkhub query simulation\logs\<test_name>\linkhub types
linkhub query simulation\logs\<test_name>\linkhub diagnostics
linkhub query simulation\logs\<test_name>\linkhub statustext
```

Decision order:

1. Check the artifacts are from this run and complete, then confirm from the
   journal / enriched telemetry that the EKF was healthy and GPS-aided at
   `kinematic_exit` (details: [EKF_GATING.md](EKF_GATING.md)).
2. Check the hand-off: IC position, disk tilt, and rotor RPM at release.
3. Only if both pass, analyse post-release controller or physics behavior.

A missing sample or field is missing evidence, not a pass.

Other entry points:

| Task | Command |
|---|---|
| Raw LinkHub journal inspection | `linkhub query simulation\logs\<test_name>\linkhub show --json` |
| Visualize telemetry | `visualize.cmd simulation\logs\<test_name>\telemetry.csv` |

Before trusting an `analysis/` helper, check which artifacts it actually reads.

Journal records and `linkhub_client` messages carry MAVLink enumerations and
bitmasks as names, not numbers: the client decodes them to
`linkhub_client.messages` types (`MavCmd`, `MavResult`, `MavModeFlag`,
`EkfStatusFlags`, ...; bitmasks are frozensets, test with `flag in
message.base_mode`). Raw journal fields from `linkhub query ... show --json`
use `{"type": "MAV_X"}` objects and `"A | B"` strings; decode them with
`decode_message(RawMessage(...))`. Fixture STATUSTEXT log lines print the severity name
(`[sev=MAV_SEVERITY_INFO]`).

## Lockstep protocol: one non-negotiable rule

The physics worker must reply to **every** SITL servo packet. Missing one reply
stalls ArduPilot permanently.

Related current facts:

- `gcs.sim_now()` is simulation time derived from LinkHub observations, not wall
  clock.
- `sim_sleep(N)` waits `N` simulation seconds; the physics loop must continue to
  service lockstep packets during that wait.

For lower-level lockstep ownership, see
[simulation.md](simulation.md).

## SITL scripting-thread starvation

Lua `vehicle:*` bindings must take ArduPilot's scheduler semaphore, which the
main loop releases only while waiting for the next IMU sample. In lockstep
SITL, each scheduler delay advances at least one physics frame:

- At `SIM_RATE_HZ=1200` a frame is 0.83 ms, so the main loop still waits for
  later samples with the semaphore released and Lua gets to run.
- At 400 Hz one frame is the whole 2.5 ms loop, so the semaphore is almost
  never free: Lua calls block for seconds, RC overrides time out, and heli
  runup restarts.

Keep `SIM_RATE_HZ=1200` in `tests/sitl/rawes_sitl_defaults.parm` and the torque
boot params; the mediator follows the servo-packet frame rate. Do not raise or
disable `RC_OVERRIDE_TIME` to hide such stalls. This is a SITL-only effect.

The static GUIDED holds (the entry hold after `ENTER_GUIDED` and the passive
hold after `ENTER_PASSIVE`) never poll `vehicle:get_mode()`. They call
`vehicle:set_target_angle_and_rate_and_throttle()` only when the target changes
(at most every 50 ms) and otherwise once per second as a keepalive inside
`GUID_TIMEOUT` (3 s). This keeps semaphore-taking calls to a minimum.

## Kinematic hold: current shared implementation

The canonical time anchors for the hold and release are defined in
[Flight timeline anchors](#flight-timeline-anchors) below.

Current shared implementation, verified from `tests/sitl/flight/conftest.py` and
`simulation/config.py`:

| Path | Current role |
|---|---|
| `simulation/kinematic.py` | central trajectory math (`make_smooth_trapezoid_traj`, `make_linear_traj`, `KinematicStartup`) |
| `simulation/mediator.py` | production wiring of the kinematic startup into the lockstep mediator |
| `tests/sitl/flight/conftest.py::_ic_trapezoid_stack` | the only IC-start flight-fixture entry point |

Exact durations, gains, and hand-off timings live in those code paths and can
drift. Do not duplicate them here; read `_ic_trapezoid_stack` and
`simulation\config.py` when a change depends on the current numeric values.

## Current stack setup sequence

The setup sequence still needs to finish inside the hold:

1. Connect GCS and request message rates.
2. Wait for the parameter subsystem.
3. Verify boot parameters.
4. Wait for EKF tilt alignment.
5. Arm.
6. Confirm the intended guided mode before yielding to the test.

The IC-start flight fixtures then move Lua through the passive/steady hand-off
described under [Flight timeline anchors](#flight-timeline-anchors).

## Flight timeline anchors

These anchors apply to IC-start flight stacks (`tests/sitl/flight/`: steady,
passive, pumping, landing, and GPS bring-up). They do not apply to
Windows-native unit tests, simtests, or torque-only stacks.

Use, in order:

1. `t_sim` from the telemetry/event stream (raw traceability to logs and
   fixture timing);
2. the row/event where `note == "kinematic_exit"`;
3. `t_rel = t_sim - t_kin_exit`.

Rules:

- Use `t_rel` for release-to-flight comparisons across runs.
- If `kinematic_exit` is missing, the run is invalid for post-release flight
  diagnosis until the telemetry/event path is fixed.
- Every diagnosis reports raw `t_sim`, derived `t_rel`, and whether the event is
  before or after `kinematic_exit`. For a first divergence, also report the
  comparison window in `t_rel` and the artifact used (`telemetry.csv`,
  `events.jsonl`, or the LinkHub journal).

Field names to use in analysis (schema in `simulation/telemetry_columns.py`):
`t_sim`, `sitl_time`, `phase`, `note`, `omega_rotor`,
`mav_att_{roll,pitch,yaw}_deg`, `mav_att_target_{roll,pitch,yaw}_deg`,
`ekf_pos_{x,y,z}`.

Sequence: `t_sim = 0` starts the mediator and kinematic hold; arm/setup
finishes inside the hold and Lua enters PASSIVE before release; the mediator
logs `kinematic_exit` (`t_rel = 0`). For the steady hand-off,
`tests/sitl/flight/test_lua_flight_steady_sitl.py` waits `_PASSIVE_SETTLE_S`,
then promotes `RAWES_MODE` from PASSIVE to STEADY and sends `RAWES_ALT`.
Numeric timings live in `_ic_trapezoid_stack` and `simulation/config.py`.

## DShot/BLHeli parameters excluded from SITL verification

`tests/sitl/stack_utils.py` currently excludes these hardware-only parameters
from SITL boot verification because ArduCopter-heli SITL does not compile the
BLHeli / bidirectional-DShot backend:

- `SERVO_BLH_MASK`
- `SERVO_BLH_BDMASK`
- `SERVO_BLH_AUTO`
- `SERVO_BLH_OTYPE`
- `SERVO_BLH_POLES`
- `SERVO_BLH_TRATE`
- `BRD_IO_DSHOT`

The yaw motor still appears in telemetry and control flow through the simulated
stack; only the hardware ESC backend itself is absent.

## Windows / WSL path gotcha

On the supported workstation, Docker access is WSL-only. Invoke `bash test.sh`
or `bash setup.sh build` from Git Bash and let those scripts re-enter WSL
themselves. Do not wrap them manually in `wsl.exe`.
