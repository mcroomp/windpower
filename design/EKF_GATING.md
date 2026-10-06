# EKF GPS and moving-baseline yaw gating (RAWES SITL)

This document owns the **current RAWES stack configuration**, the project-specific
gate interpretation for EKF bring-up during the kinematic hold, and the
DataFlash fields worth checking when `diagnose_sitl.py` is not enough.

## Scope

Use this document when you need the current repo-truth for:

- which EKF/GPS parameters the RAWES SITL stack sets;
- which gates `analysis/diagnose_sitl.py` CHECK 1 evaluates before
  `kinematic_exit`;
- how to interpret an IC-start run that fails to leave `const_pos_mode`
  during the startup hold.

Do **not** use this file as a historical failure log; run
`uv run python analysis/diagnose_sitl.py <test_name>` against the current run.

## Current RAWES SITL EKF configuration

Verified from `tests/sitl/rawes_sitl_defaults.parm` and
`tests/sitl/rawes_common_defaults.parm`:

- `EK3_SRC1_YAW` selects moving-baseline GPS yaw.
- `GPS1_TYPE` / `GPS2_TYPE` configure the base / rover pair used for that yaw path.
- `GPS*_DELAY_MS` and `SIM_GPS*_LAG_MS` are pinned to matching values so the
  injected lag and EKF-assumed lag stay aligned.
- `COMPASS_USE=0`, so compass yaw is intentionally out of the path.
- `EK3_GPS_CHECK=0`, so the usual HDOP / speed-accuracy / min-sat startup
  gates are intentionally relaxed in SITL.

For exact current numbers, read the parm files directly rather than copying them
from prose.

## Current CHECK 1 contract in `diagnose_sitl.py`

`analysis/diagnose_sitl.py` is the authoritative first-pass diagnostic for
stack EKF bring-up. Its CHECK 1 treats the EKF as capture-ready only when all
required status bits are set and warning bits are clear.

### Capture-ready mask

Current required flags (`_CAPTURE_READY_MASK`):

- attitude
- horiz_vel
- vert_vel
- horiz_pos_rel
- horiz_pos_abs
- vert_pos

Current warning flags that still fail CHECK 1:

- `const_pos_mode`
- `uninitialized`
- `gps_glitching`

### Current gate breakdown

When the EKF is stuck in `const_pos_mode`, `diagnose_sitl.py` enumerates the
same gate chain it expects the stack to clear:

1. PosXY source is GPS
2. `validOrigin`
3. `tiltAlignComplete`
4. `yawAlignComplete`
5. `delAngBiasLearned`
6. `gpsGoodToAlign`
7. `gpsDataToFuse`

For the current RAWES stack:

- gate 4 is the moving-baseline yaw path, because `EK3_SRC1_YAW=2`;
- gate 6 is intentionally relaxed by `EK3_GPS_CHECK=0`, but it still must go
  true in the nav status path the script reads;
- gate 7 still matters even when the quality checks are relaxed, because the
  EKF still needs fresh GPS data at the fusion horizon.

## Current evidence sources

CHECK 1 and follow-up manual inspection rely on these current artifacts:

| Artifact | What it contributes |
|---|---|
| `simulation\logs\<test_name>\telemetry.csv` | timeline anchor (`t_sim`, `note == "kinematic_exit"`), EKF-enriched async columns |
| `simulation\logs\<test_name>\linkhub\` | LinkHub journal for `GPS_RAW_INT`, `EKF_STATUS_REPORT`, `STATUSTEXT`, NVF/param traces |
| `simulation\logs\<test_name>\arducopter.log` | DataFlash-backed EKF/GPS state used by `diagnose_sitl.py` |

There is no stack-exported `mavlink.jsonl` in the current architecture.

## Current workflow

Start here:

```powershell
uv run python analysis/diagnose_sitl.py <test_name>
```

If you need manual confirmation after CHECK 1:

```powershell
linkhub query simulation\logs\<test_name>\linkhub statustext
linkhub query simulation\logs\<test_name>\linkhub show --json
```

Questions to answer before post-release flight analysis:

1. Did the EKF leave `const_pos_mode` before `kinematic_exit`?
2. Did it establish origin and full GPS aiding with the current dual-GPS setup?
3. If not, which of the seven gates above is still false?

Only after that should you move on to controller or hand-off diagnosis.

## `const_pos_mode`

`const_pos_mode` is the nav-status signal that the EKF is healthy but has no
active position/velocity aiding source and is holding a fixed position. If it
is still set at `kinematic_exit`, post-release flight conclusions are not
trustworthy; diagnose that first.

## DataFlash fields for manual inspection

If you must go below `diagnose_sitl.py`:

| Field | Why it matters |
|---|---|
| `XKF4.SS` | solution-status bitmask: `const_pos_mode`, `using_gps`, `gps_quality_good` |
| `XKF4.GPS` | GPS-check failure bitmask when startup gating is the blocker |
| `XKF4.TS` | timeout bitmask; separates a mid-flight fallback from a startup failure |
| `XKFS` | sensor-selection helpers such as `GGA` / `WFG` |
| `XKF3` | innovations, for yaw or sensor-consistency problems |
| `XKF1` | internal-state convergence, e.g. bias settling and origin height |
| `MSG` / `STATUSTEXT` | human-readable transitions such as "is using GPS" |

Flow: find `kinematic_exit`; check whether `XKF4.SS` still has `const_pos_mode`
there; if so inspect `XKF4.GPS`, `XKFS`/`MSG`, and `XKF4.TS`.

## Relationship to other docs

- [sitl_testing.md](sitl_testing.md) owns the run/diagnose workflow and the
  shared time anchors (`t_sim`, `kinematic_exit`, `t_rel`; see
  [Flight timeline anchors](sitl_testing.md#flight-timeline-anchors)).
