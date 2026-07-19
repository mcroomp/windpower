# SITL Flight Timeline (IC-Start Canonical)

This document defines the canonical timeline used for SITL flight analysis when
stacks start from the shared IC flow.

Scope:
- SITL stack flight tests under `tests/sitl/flight/`.
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

## Field naming contract (docs vs code)

Use the exact field names below when writing analysis notes/scripts.

Telemetry CSV (`simulation/logs/<test>/telemetry.csv`):
- `t_sim`, `note`, `phase`, `omega_rotor`
- `mav_att_roll_deg`, `mav_att_pitch_deg`, `mav_att_yaw_deg`
- `mav_att_target_roll_deg`, `mav_att_target_pitch_deg`, `mav_att_target_yaw_deg`
- `ekf_pos_x`, `ekf_pos_y`, `ekf_pos_z`

MAVLink JSONL (`simulation/logs/<test>/mavlink.jsonl`):
- `GPS_RAW_INT.fix_type`, `GPS_RAW_INT.satellites_visible`
- `EKF_STATUS_REPORT.flags`
- `HEARTBEAT.custom_mode`, `HEARTBEAT.base_mode` (armed bit lives in `base_mode`)
- `STATUSTEXT.text`

Not valid in current telemetry CSV schema:
- `mav_fix_type`, `mav_satellites_visible`, `mav_ekf_flags`, `mav_mode`, `mav_armed`
- `roll_rad`, `pitch_rad`

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

## Canonical event matrix (reference run)

Reference artifacts:
- `simulation/logs/test_lua_flight_steady_sitl/telemetry.csv`
- `simulation/logs/test_lua_flight_steady_sitl/mavlink.jsonl`
- `simulation/logs/test_lua_flight_steady_sitl/events.jsonl`

Anchor:
- `t_kin_exit = 60.003 s` from telemetry/event marker `note == "kinematic_exit"`

Use this as the compact reference table for this run.

| `t_start` (s) | Kind | Milestone | Evidence |
|---:|---|---|---|
| 0.000 | observed | Mediator startup and 60 s kinematic profile start | `events.jsonl: startup`, `events.jsonl: kinematic_config.total_s=60` |
| 8.860-8.960 | observed | EKF yaw aligned and GPS reaches 3D fix (`fix_type=6`) | `STATUSTEXT "EKF3 IMU0 yaw aligned"`, `GPS_RAW_INT.fix_type=6` |
| 14.000 (nominal) | scheduled | Arm gate reached in fixture | `conftest.py _arm_at_sim_s=14.0` |
| 14.000+ (nominal) | scheduled | PASSIVE IC seeds scheduled, including tilt targets (`RAWES_RIC/PIC`) | `conftest.py gcs.send_message(NamedValueFloat(...)) RAWES_THR/RIC/PIC`, `RAWES_MODE=3` |
| 24.077 | observed | EKF starts GPS aiding | `STATUSTEXT "EKF3 IMU0 is using GPS"` |
| 26.748 | observed | Tilt target first visible at AP interface | `telemetry.csv: |mav_att_target_pitch_deg| >= 1` |
| 26.848 | observed | Physical tilt response begins (real attitude moves) | `telemetry.csv: |mav_att_pitch_deg| >= 5` |
| 27.178 | observed | Large tilt achieved | `telemetry.csv: |mav_att_pitch_deg| >= 40` |
| 60.003 | observed | Kinematic handoff (`t_kin_exit`) | `telemetry.csv/events.jsonl: note=="kinematic_exit"` |
| 60.503 (nominal) | scheduled | Test schedules steady takeover (`RAWES_MODE=1` then `RAWES_ALT`) | `test_lua_flight_steady_sitl.py`, `_PASSIVE_SETTLE_S=0.5` |
| 61.060 | observed | Steady capture confirmed | `STATUSTEXT "RAWES steady: captured"` |

Quick summary:
- Horizontal start at `t=0`; no IC tilt yet.
- IC tilt targets are scheduled right after arm (~`t=14`).
- Targets appear on AP side around `t=26.748`, and physical tilt follows shortly (`t=26.848`).
- Handoff is at `t=60.003`; steady capture follows near `t=61.060`.

Rotor state reference for this run:
- `omega_rotor` is already ~`38.10 rad/s` from the first telemetry sample and at
  `kinematic_exit`; there is no distinct spin-start transition in this run.

Tilt scheduling ownership note:
- The kinematic hold controller setup in the stack harness (`tests/sitl/stack_infra.py`)
  is parameter/anchor plumbing only; it is not the runtime source of tilt commands in
  this IC-start path.
- Runtime tilt scheduling during kinematic hold comes from `rawes.lua` in `MODE_PASSIVE`
  once the IC seed is complete (`RAWES_THR` + `RAWES_RIC` + `RAWES_PIC`) and
  `guided_ok` is true.

Tilt command-to-response flow (IC-start path):
1. Fixture schedules IC tilt targets (`RAWES_RIC/PIC`) right after arm (`t_start~14s`).
2. Lua `MODE_PASSIVE` emits GUIDED angle targets once IC seed is complete and `guided_ok` is true.
3. During kinematic hold with `kinematic_aero_mode="nul"`, the physics side realizes tilt by
   integrating body rates (`omega_body = gain * tilt`) until measured attitude matches target.

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
- Shared fixture path: `tests/sitl/flight/conftest.py` (`_ic_trapezoid_stack`).
- Primary diagnosis workflow: `design/sitl_testing.md` and `analysis/diagnose_sitl.py`.

## Validation checklist (after timeline-related changes)

1. Run one IC-start SITL flight test.
2. Confirm telemetry has a single `kinematic_exit` marker.
3. Confirm analysis scripts compute finite `t_rel` values post-release.
4. Confirm first-divergence output references `t_rel` (not only absolute time).

Suggested command:
- `bash test.sh stack -n 1 -k test_lua_flight_steady_sitl`
