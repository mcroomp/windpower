# SITL Testing

Everything you need to **run, diagnose, and reason about RAWES SITL stack tests**
— the full-stack (ArduPilot + Lua) Docker tests under
[simulation/tests/sitl/](../simulation/tests/sitl/). Read this whenever you are
editing or diagnosing a SITL stack test; it is intentionally kept out of the
always-on [AGENTS.md](../AGENTS.md) context.

> **Scope.** This doc covers SITL **stack** tests only. Windows-native unit tests
> and simtests (no Docker, no ArduPilot/Lua) are covered by
> [design/testing.md](testing.md) and the Running Tests section of
> [AGENTS.md](../AGENTS.md).

---

## When to use this doc

Pull this doc into context when you are:

- running a SITL stack test (`bash test.sh stack ...`);
- diagnosing a SITL failure (`diagnose_sitl.py`, `analyse_run.py`, EKF/GPS gating);
- touching the kinematic-hold trajectory, the IC-start fixtures, or `_run_acro_setup`;
- debugging the SITL lockstep protocol or the GPS/EKF startup path.

---

## Running SITL stack tests

**Stack tests require Docker. Never mix with the Windows venv.** Each stack test
runs in its own fresh Docker container, one per test file.

| Task | Command |
|------|---------|
| Stack test (single) | `bash test.sh stack -n 1 -k test_foo` |
| Stack test (full) | `bash test.sh stack -n 8` |

- **NEVER call `docker exec` directly to run stack tests. Use `bash test.sh stack`.**
- Use the Bash tool directly — never `wsl.exe`. Always absolute paths.
- The Docker image is built by `bash setup.sh build` (with ArduPilot) or
  `bash setup.sh build-lite` (without). Container lifecycle:
  `bash test.sh start|stop|sync|shell|exec`.
- Stack test logs land in `simulation/logs/{test_name}/` —
  `mediator.log`, `sitl.log`, `gcs.log`, `telemetry.csv`, `arducopter.log`.
  Suite summary: `simulation/logs/suite_summary.json`.
- **`internal_controller` MUST be `False` for all full-stack flight tests** — the
  whole point is to validate that ArduPilot + Lua actually fly the vehicle.
- **SITL must run as close to hardware as possible.** Find and fix root causes; do
  NOT paper over failures with simulation-only hacks. Confirm with the user before
  adding any override. `base_k_ang` is diagnostic-only and defaults to 0.

---

## Post-run diagnosis workflow

**ALWAYS run `diagnose_sitl.py` FIRST after ANY SITL stack run, before making any
decision.**

```
.venv/Scripts/python.exe simulation/analysis/diagnose_sitl.py <test_name>
```

It answers the two gating questions in order:

- **CHECK 1 — was the EKF healthy (GPS-aiding) at the kinematic exit?** If not, a
  dataflash blocker chain (GPS 3D fix → GPS pre-arm checks `XKF4.GPS` → `ORGN`
  origin → `XKF4.SS` nav_filter_status) pinpoints the first broken link. When the
  EKF is stuck in `const_pos_mode`/`AID_NONE` it enumerates every
  `readyToUseGPS()` gate (PosXY source, validOrigin, tiltAlignComplete,
  **yawAlignComplete**, delAngBiasLearned, gpsGoodToAlign, gpsDataToFuse) to name
  the exact blocking condition. Without a healthy EKF the Lua can never capture
  GPS, so nothing downstream is trustworthy — everything after the exit gate is
  ignored.
- **CHECK 2 — at kinematic exit, are we at the IC position, disk tilt (body_z),
  and rotor RPM** (vs `steady_state_starting.json`)?

Decision order: if CHECK 1 fails, fix the EKF/GPS path first. If CHECK 1 passes
but CHECK 2 fails, fix the kinematic hand-off first. Only when both pass is a
post-release flight failure a real controller/physics bug — then move on to
`analyse_run.py`.

**Run `analyse_run.py` only after `diagnose_sitl.py` passes both gates.** It loads
all log sources (telemetry CSV, mavlink.jsonl, mediator.log, arducopter.log) into
a unified `FlightLog` and prints a single bucketed report.

```
.venv/Scripts/python.exe simulation/analysis/analyse_run.py <test_name>   # --bucket 10 coarse, --bucket 1 frame-level
```

**Fix telemetry/logging before diagnosing physics.** If telemetry columns are
zero/missing/wrong (e.g. `tether_m=0`, phase never changes), fix the logging bug
first — diagnosing from bad telemetry produces wrong conclusions.

### Other diagnosis entry points

| Task | Command |
|------|---------|
| Pump cycle diagnosis | `.venv/Scripts/python.exe simulation/analysis/pump_diagnosis.py --test test_pump_cycle_unified --bucket 1` |
| Landing diagnosis | `.venv/Scripts/python.exe simulation/analysis/analyse_landing.py [--test test_landing_lua_sitl] [--bucket 2]` |
| Visualize result | `visualize.cmd simulation/logs/<test_name>/telemetry.csv` |
| EKF gating reference | [design/EKF_GATING.md](EKF_GATING.md), [design/ekf_const_pos_mode.md](ekf_const_pos_mode.md) |

---

## SITL lockstep protocol — key rule

The physics worker must reply to **every** SITL servo packet without exception —
skipping a reply causes ArduPilot to stall permanently. `gcs.sim_now()` returns
`time_boot_ms/1000` from the most recently processed MAVLink message, not
wall-clock time. `sim_sleep(N)` waits N sim-seconds; the physics loop must keep
running during the wait. Full reference:
[design/simulation.md § SITL Lockstep Protocol](simulation.md).

---

## Kinematic hold timeline

The **kinematic hold** (a.k.a. kinematic startup phase) is the artificial,
physics-free trajectory that brings the hub to the IC operating point and holds
it there while the EKF aligns on GPS. It exists for one reason: to leave the EKF
**healthy and GPS-aiding** (out of `const_pos_mode`/`AID_NONE`, with
`yawAlignComplete` latched) by the time real physics takes over at *kinematic
exit*. Nothing downstream is trustworthy until that gate passes — see
[design/EKF_GATING.md](EKF_GATING.md) and the `diagnose_sitl.py` CHECK 1/CHECK 2
contract above.

> **Single source of truth.** There is exactly **one** central kinematic-hold
> implementation. The trajectory math lives in
> [simulation/kinematic.py](../simulation/kinematic.py); the production loop
> ([simulation/mediator.py](../simulation/mediator.py)) and every Windows-native
> unit/simtest build their trajectory from it. Every SITL **flight** fixture that
> must *start at the IC* goes through the single shared helper
> `_ic_trapezoid_stack` in
> [simulation/tests/sitl/flight/conftest.py](../simulation/tests/sitl/flight/conftest.py).
> **Do not** fork or re-derive a kinematic trajectory inside a test. If a test
> needs to start at the IC, call the shared fixture; if it needs a different
> profile, change the shared implementation (and this doc), do not copy it.

---

## 1. Components (the one implementation)

| Layer | Symbol | File | Role |
|-------|--------|------|------|
| Trajectory factory | `make_smooth_trapezoid_traj()` | [kinematic.py](../simulation/kinematic.py) | C1-continuous (raised-cosine) trapezoid ending exactly at `pos0` with zero velocity |
| Trajectory factory | `make_linear_traj()` / `compute_launch_position()` | [kinematic.py](../simulation/kinematic.py) | Constant-velocity fallback path (used only when `kinematic_cruise_speed == 0`) |
| Driver | `KinematicStartup` | [kinematic.py](../simulation/kinematic.py) | Wraps a `traj_fn(t)->(pos,vel)` (+ optional `R_fn(t)->R`); `state_at(t)` returns the held kinematic state |
| Production wiring | mediator startup block | [mediator.py](../simulation/mediator.py) (`_kin_duration`, `make_smooth_trapezoid_traj`, `KinematicStartup`) | Builds the trajectory from config and feeds the SITL sensor stream |
| Config knobs | `startup_damp_seconds`, `kinematic_cruise_speed`, `kinematic_accel_s`, `kinematic_decel_s`, `kinematic_vel_ramp_s`, `kinematic_aero_mode` | [config.py](../simulation/config.py) | Default profile; overridden per-fixture |
| Central IC fixture | `_ic_trapezoid_stack` | [flight/conftest.py](../simulation/tests/sitl/flight/conftest.py) | The **only** entry point for "start at the IC" SITL flight tests |
| SITL setup sequence | `_run_acro_setup` (6 steps) / `_acro_stack` | [stack_infra.py](../simulation/tests/sitl/stack_infra.py) | Connect → params → EKF tilt align → arm → confirm GUIDED_NOGPS |

The trapezoid path is selected whenever `kinematic_cruise_speed > 0`; otherwise
the linear fallback path is used. With dual GPS (`EK3_SRC1_YAW=2`, RELPOSNED
heading) yaw is known from the first fix, so the motion exists only to give the
EKF **velocity observability** during the hold — not to align yaw.

---

## 2. Canonical IC-start timeline (trapezoid)

This is the profile set by `_ic_trapezoid_stack` and shared by the steady,
ic-passive, and pumping fixtures. Parameters: `startup_damp_seconds = 60`,
`kinematic_cruise_speed = 1.0 m/s`, `kinematic_accel_s = 5`,
`kinematic_decel_s = 5`, `kinematic_aero_mode = "nul"`, `_arm_at_sim_s = 8`.
Time is measured from mediator start at `speedup = 1`.

```
 t (s)  phase / event
 ------  --------------------------------------------------------------
 0       kinematic hold begins; hub at launch_pos (back-computed so the
         trapezoid ends exactly at pos0). Lua not yet active.
 0..5    accelerate 0 -> 1 m/s along the IC yaw heading (raised cosine).
 ~6      GPS first fix; EKF3 origin set.
 ~8      arm (after EKF tilt alignment); SCR_USER6=3 (MODE_PASSIVE) set;
         IC attitude commanded via nul-aero cyclic during the hold.
 5..55   cruise at 1 m/s along the IC heading (constant velocity).
 ~34     GPS fuses: delAngBiasLearned converges, readyToUseGPS() passes,
         const_pos_mode clears. (yawAlignComplete must latch by here.)
 55..60  decelerate 1 -> 0 m/s, arriving EXACTLY at pos0 at rest.
 60      KINEMATIC EXIT: physics takes over. Test promotes SCR_USER6 3 -> 1
         (MODE_STEADY) and steady guidance becomes active.
 60+     free flight under ArduPilot + Lua.
```

Why this shape:

- **Ends at `pos0` with zero velocity.** `launch_pos` is back-computed from the
  integrated speed profile so there is **no residual position/velocity error** at
  release — GPS aiding engages with nothing to shock the EKF. (A constant-velocity
  drift used to leave the hub ~58 m from `pos0` and jolt the filter when aiding
  finally engaged.)
- **Raised-cosine ramps** give continuous acceleration (no jerk step) at every
  phase boundary.
- **Level frame yawed to the IC heading.** The hold starts at roll=pitch=0 yawed
  to the IC heading; the IC roll/pitch is slewed in later via the `nul`-aero
  cyclic (it cannot apply yaw), keeping the EKF pre-arm seed level and consistent.
- **MODE_PASSIVE during the hold.** `SCR_USER6=3` is set right after arm so the
  Lua commands the IC attitude as a GUIDED angle target (IC roll/pitch from
  `RAWES_RIC`/`RAWES_PIC` + yaw captured at entry, with **zero rate
  feed-forward**) plus IC collective via throttle. The `nul`-aero integrates
  that angle command so the disk slews to the IC tilt during the hold.

---

## 3. Where the timeline is parameterized

Not every SITL test uses the 60 s canonical profile. The duration and ramp
windows are config-driven; only the *implementation* is shared:

| Test / fixture | `startup_damp_seconds` | Notes |
|----------------|------------------------|-------|
| `_ic_trapezoid_stack` (steady / ic-passive / pumping) | 60 | Canonical trapezoid, `cruise=1.0` |
| `guided_nogps_armed_landing_lua` | 65 | Trapezoid + `kinematic_vel_ramp_s` tail; body_z capture gated by `KINEMATIC_SETTLE_MS` |
| `test_kinematic_gps_sitl` | 160 | Long hold for GPS-fusion timing studies |
| `config.py` default | 30 | **Linear** fallback path (`kinematic_cruise_speed=0`) for non-IC stacks |

> Note: the `_run_acro_setup` docstring still mentions a "30 s" damping window;
> that is the legacy default, not the IC-start value. The IC fixtures override
> `startup_damp_seconds` to 60. The authoritative duration for any given test is
> the value in its fixture/extra-config, not the docstring.

---

## 4. SITL setup sequence (runs inside the hold)

`_run_acro_setup` must complete its six steps **inside** the kinematic window so
the hub is still being held when GPS aligns:

1. Connect GCS; request telemetry streams; motor interlock LOW (CH8=1000).
2. Wait for the param subsystem.
3. Verify boot params via MAVLink read-back.
4. Wait for EKF tilt alignment (FAIL HARD if it never arrives).
5. Arm with `force=True` (interlock low → arm → raise CH8 to 2000).
6. Confirm GUIDED_NOGPS mode.

The arm at `t ~ 8 s` is deliberately early so the IC attitude is commanded as
soon as possible, giving the `nul`-aero the full remaining hold window to slew
the disk to the IC orientation before release.

---

## 5. Verification

After any change, confirm the hold still delivers a healthy EKF and a clean
hand-off:

```
bash test.sh stack -n 1 -k test_lua_flight_steady_sitl
.venv/Scripts/python.exe simulation/analysis/diagnose_sitl.py test_lua_flight_steady_sitl
```

- **CHECK 1** — EKF GPS-aiding (out of `const_pos_mode`, `yawAlignComplete`
  latched) **before** the 60 s release.
- **CHECK 2** — at exit, hub is at IC position, disk tilt (body_z), and rotor RPM
  vs `steady_state_starting.json`.

The Windows-native guard for the trajectory math is
[simulation/tests/unit/test_startup_trajectory.py](../simulation/tests/unit/test_startup_trajectory.py)
(`make_smooth_trapezoid_traj` ends at `pos0` with zero velocity, continuous accel).

---

## 6. Keeping this doc in sync

**This doc is the canonical reference for SITL stack testing and the kinematic
hold timeline. If you change any of the SITL workflow, the diagnosis entry
points, the lockstep contract, or the kinematic-hold timeline, update this doc in
the same commit** (and update the short pointer in [AGENTS.md](../AGENTS.md) only
if the *summary* changes). Specifically, update the relevant section whenever you
change any of:

- the SITL run/diagnose commands (`test.sh stack`, `diagnose_sitl.py`,
  `analyse_run.py`) or the CHECK 1 / CHECK 2 contract;
- the SITL lockstep protocol;
- the central trajectory implementation in
  [simulation/kinematic.py](../simulation/kinematic.py)
  (`make_smooth_trapezoid_traj`, `make_linear_traj`, `KinematicStartup`,
  `compute_launch_position`);
- the mediator wiring that builds the trajectory in
  [simulation/mediator.py](../simulation/mediator.py);
- the shared IC fixture `_ic_trapezoid_stack` or the kinematic config keys
  (`startup_damp_seconds`, `kinematic_cruise_speed`, `kinematic_accel_s`,
  `kinematic_decel_s`, `kinematic_vel_ramp_s`, `kinematic_aero_mode`,
  `_arm_at_sim_s`) in any fixture or in [config.py](../simulation/config.py);
- the `_run_acro_setup` six-step sequence or the arm timing.

Any SITL test that needs to **start at the IC must reuse the single shared
implementation** (`_ic_trapezoid_stack` → `kinematic.py`). Do not add a second
kinematic-hold path; extend the shared one and record the change here.
