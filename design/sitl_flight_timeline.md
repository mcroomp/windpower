# SITL Flight Timeline (IC-Start Canonical)

This document defines the canonical timeline used for SITL flight analysis when
stacks start from the shared IC flow.

Scope:
- SITL stack flight tests under `simulation/tests/sitl/flight/`.
- Steady, passive, pumping, and landing variants that use IC-start fixtures.

Out of scope:
- Windows-native unit/simtests.
- Non-flight SITL infrastructure checks.

## Why this exists

All SITL flight comparisons must use the same event anchors. Without one
canonical timeline, we can mis-attribute divergence to controller behavior when
it is actually a phase-alignment artifact.

## Canonical time anchors

Use these anchors in this order:

1. Absolute simulation time: `t_sim` from telemetry CSV.
2. Kinematic release boundary: row/event where `note == "kinematic_exit"`.
3. Relative flight time: `t_rel = t_sim - t_kin_exit`.

Rules:
- Use `t_rel` for any release-to-flight comparison (simtest vs SITL, run-vs-run,
  first-divergence windows, reaction windows).
- Do not use wall clock, container uptime, or MAVLink receive order as primary
  comparison axes.
- If `kinematic_exit` is missing, treat the run as invalid for flight-phase
  root-cause analysis until telemetry/event logging is fixed.

## Standard phase timeline (IC-start stacks)

This is the expected sequence for IC-start SITL flight stacks.

1. `t_sim = 0`: mediator start; kinematic startup begins.
2. Kinematic hold window: EKF alignment and IC approach while physics hand-off is
   gated.
3. `note == "kinematic_exit"`: one-shot release marker; this is `t_rel = 0`.
4. Post-release guided flight: controller/physics behavior under test.

Notes:
- Exact hold duration is fixture-configured and may vary by test.
- Phase duration differences do not change the anchor rule: always compare using
  `t_rel` based on `kinematic_exit`.

## Pre-release event matrix (major columns)

Use this table format for steady SITL timeline analysis up to kinematic exit.
Each row is a milestone, and each major subsystem has its own column.

| Time marker | Kinematic / trajectory | GPS | EKF | Vehicle mode / arm | RAWES / Lua | Analysis note |
|---|---|---|---|---|---|---|
| `t_sim = 0`, `t_rel < 0` | Kinematic startup begins | No guaranteed fix yet | Startup alignment state | Disarmed | `RAWES_MODE=0` expected | Run initialization |
| Early hold (`t_rel << 0`) | Trapezoid accel/cruise/decel progressing | `GPS_RAW_INT` transitions 0 -> 1 -> 6 | `EKF_STATUS` transitions toward aiding-ready flags | GUIDED_NOGPS setup starts, then arm pending | Passive pipeline preparing IC seed | Confirm bring-up order |
| Mid hold (`t_rel < 0`) | Kinematic still active | Fix 6 expected to be stable | Origin set, then GPS aiding active | Armed, GUIDED_NOGPS stable | `RAWES_MODE=3` (passive), IC ready status | Verify pre-release stability |
| Late hold (`t_rel -> 0-`) | Near release target state (at IC pose/vel) | Fix remains stable | Aiding-ready flags stable | Armed/mode unchanged | Passive hold commands continue | Ensure no last-second state jumps |
| `note == kinematic_exit`, `t_rel = 0` | Kinematic handoff ends | Should still be locked | Should still be aiding | Armed/mode continuous across handoff | Steady logic eligible to take control | Start post-release comparisons |

Minimum fields to populate per row:
- Time: `t_sim`, `t_rel`.
- GPS: `fix_type`, `satellites_visible`.
- EKF: `EKF_STATUS_REPORT.flags` and key STATUSTEXT milestones.
- Mode/arm: HEARTBEAT `custom_mode` and armed bit.
- RAWES/Lua: mode transitions and passive/steady status text.

## Required analysis convention

For SITL flight diagnostics, report timing in both forms:
- `t_sim` for raw traceability.
- `t_rel` for behavioral comparison.

When reporting first divergence, include:
- first-divergence `t_rel` bucket,
- pre-window and post-window definitions in `t_rel`,
- whether events are before or after `kinematic_exit`.

## Data sources and marker ownership

Canonical marker producers/consumers:
- Producer: `simulation/mediator.py` writes `note = "kinematic_exit"` and event log entry.
- Shared fixture path: `simulation/tests/sitl/flight/conftest.py` (`_ic_trapezoid_stack`).
- Primary diagnosis workflow: `design/sitl_testing.md` and `simulation/analysis/diagnose_sitl.py`.

## Validation checklist (after timeline-related changes)

1. Run one IC-start SITL flight test.
2. Confirm telemetry has a single `kinematic_exit` marker.
3. Confirm analysis scripts compute finite `t_rel` values post-release.
4. Confirm first-divergence output references `t_rel` (not only absolute time).

Suggested command:
- `bash test.sh stack -n 1 -k test_lua_flight_steady_sitl`
