# EKF GPS and Moving-Baseline Yaw Gating

This document owns RAWES SITL GPS/yaw configuration, capture-readiness checks,
and interpretation of EKF evidence. Use the
[central documentation map](../README.md#documentation-map) for area ownership.

## Configuration

The current settings are owned by
[rawes_sitl_defaults.parm](../tests/sitl/rawes_sitl_defaults.parm) and
[rawes_common_defaults.parm](../tests/sitl/rawes_common_defaults.parm):

- `EK3_SRC1_YAW` selects moving-baseline GPS yaw.
- `GPS1_TYPE` / `GPS2_TYPE` configure the base/rover pair.
- `GPS*_DELAY_MS` and `SIM_GPS*_LAG_MS` match the injected and assumed GPS lag.
- Compass yaw is disabled in this SITL configuration.
- `EK3_GPS_CHECK=0` relaxes startup quality checks; fresh GPS data at the
  fusion horizon and successful yaw alignment are still required.

Read numeric defaults and explanations from the parm files.

## Capture readiness

For GPS-aided flight, check `EKF_STATUS_REPORT` for attitude, horizontal and
vertical velocity, relative and absolute horizontal position, and vertical
position validity. Warning flags such as `const_pos_mode`, `uninitialized`,
and `gps_glitching` require investigation before post-release conclusions.

Lua steady capture separately requires usable EKF position, a resolved anchor,
and enough anchor-relative distance; exact gates are in
[flight_stack.md](flight_stack.md#42-steady-capture-before-usable-position-and-anchor).
A nominal startup duration does not prove readiness.

### Leaving `const_pos_mode`: the gate chain

EKF3 starts GPS aiding only when all of these hold. If it is stuck in
`const_pos_mode`, find the first one that is false:

1. Horizontal position source is GPS (`EK3_SRC1_POSXY`).
2. EKF origin is set (`validOrigin`).
3. Tilt alignment complete.
4. Yaw alignment complete — here via moving-baseline GPS yaw (`EK3_SRC1_YAW=2`).
5. Delta-angle bias learned (not logged in DataFlash).
6. GPS good to align — relaxed by `EK3_GPS_CHECK=0`, but must still become true.
7. GPS data available at the fusion horizon (needs fresh, correctly delayed data).

`analysis\diagnose_sitl.py <test_name>` walks this chain from the recovered
`dataflash.BIN` and compares the release state with the initial condition. It
does not read the LinkHub journal; when `dataflash.BIN` is missing its EKF
verdict is incomplete.

## Evidence and timeline

| Artifact | Purpose |
|---|---|
| `simulation\logs\<test_name>\telemetry.csv` | Physics/estimator comparison and the `kinematic_exit` boundary |
| `simulation\logs\<test_name>\linkhub\` | Canonical MAVLink and diagnostic journal |
| `simulation\logs\<test_name>\dataflash.BIN` | Recovered binary DataFlash EKF/GPS state, when available |
| `simulation\logs\<test_name>\arducopter.log` | Text process/crash output |

Start with the canonical journal:

```powershell
linkhub query simulation\logs\<test_name>\linkhub types
linkhub query simulation\logs\<test_name>\linkhub statustext
linkhub query simulation\logs\<test_name>\linkhub show --type EKF_STATUS_REPORT --dir rx --limit 50 --json
```

Use the records' simulation-clock metadata and enriched telemetry to align
evidence with `kinematic_exit`. The shared timeline is owned by
[sitl_testing.md](sitl_testing.md#flight-timeline-anchors).

Before analyzing flight control:

1. Check the artifacts belong to this run and are complete.
2. Confirm origin, tilt/yaw alignment, and GPS position/velocity aiding before
   release.
3. If aiding is missing, walk the gate chain above.
4. Compare the release state with the configured initial condition.

## `const_pos_mode`

This nav-status flag means the EKF is holding a fixed position without active
position/velocity aiding. If it is set at release, diagnose aiding before
interpreting post-release controller behavior.

## DataFlash fields

| Field | Purpose |
|---|---|
| `XKF4.SS` | Solution status, including aiding and `const_pos_mode` |
| `XKF4.GPS` | GPS quality-check failures |
| `XKF4.TS` | Sensor/fusion timeouts |
| `XKFS` | Sensor selection |
| `XKF3` | Innovations and consistency |
| `XKF1` | EKF state convergence |
| `MSG` / `STATUSTEXT` | Alignment and aiding transitions |

Inspect these around the release boundary (`kinematic_exit`). A missing field
or sample is missing evidence, not a pass.
