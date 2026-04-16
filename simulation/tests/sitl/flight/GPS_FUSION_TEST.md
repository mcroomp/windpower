# GPS Fusion Diagnostic Test — `test_gps_fusion_armed`

Source: `simulation/tests/sitl/flight/test_gps_fusion_layers.py`

---

## Purpose

Validates the full EKF3 GPS fusion chain in SITL without a mediator or any
real physics model.  A minimal `_Physics` kinematic engine drives the sensor
worker directly so the test can control exactly what ArduPilot sees.

The test gates on five sequential milestones:

```
[1] EKF tilt aligned        (pre-arm, ~3.5 s)
[2] GPS origin set          (pre-arm, ~17.7 s after GPS detected)
[3] Armed in ACRO           (~22.7 s sim time)
[4] Yaw aligned             (pre-arm compass, already done at step [3])
[5] EKF3 is using GPS       (~54.5 s into step [5], ~83 s total)
```

Total sim time: **83 s** (PASS).  At SITL speedup=1 this equals ~83 s wall time.

---

## Architecture

```
test thread (main)
  |
  |--- GCS thread         -- MAVLink: arm, mode set, RC override keepalive
  |
  |--- sensor_worker      -- 100 Hz JSON physics packets to SITL UDP port 9002
         |
         _Physics engine  -- thread-safe kinematic: pos, vel, accel, rpy
                             step(dt): ramps vel toward target_vel at 0.3 m/s^2
                             snapshot(): returns (pos, vel, accel_world, accel_body, rpy_rad)
```

No mediator, no RAWES aero.  ArduPilot SITL receives sensor data exactly as
the production mediator would format it (`sitl_interface.SITLInterface`).

---

## Boot Parameters (rawes_sitl_defaults.parm, relevant subset)

| Parameter        | Value | Reason |
|-----------------|-------|--------|
| EK3_GPS_CHECK   | 0     | Disable all GPS pre-arm checks — removes 10 s hardcoded delay |
| EK3_POS_I_GATE  | 50    | Widen position innovation gate |
| EK3_VEL_I_GATE  | 50    | Widen velocity innovation gate |
| EK3_SRC1_YAW    | 1     | Compass yaw (for this test; production uses 8 = GPS velocity yaw) |
| COMPASS_USE     | 1     | Compass enabled for this test |
| COMPASS_ENABLE  | 1     | |
| H_RSC_MODE      | 1     | Passthrough — CH8 PWM drives throttle directly, no spool-up wait |

---

## EKF Fusion Chain — Observed Timings

All times are **simulation seconds** (from `time_boot_ms`), relative to SITL start (GCS connect = t=0 s).  At SITL speedup=1 these equal wall-clock seconds.

### Pre-arm phase

| t (sim s) | Event | EKF flags | Notes |
|-----------|-------|-----------|-------|
| 0.0   | GCS connected, sensor worker started | — | Stationary: vel=[0,0,0] |
| 1.0   | EKF flags = 0x0400 | pred_horiz_ok only | Filter initialising |
| 2.0   | EKF3 IMU0 initialised; AHRS: EKF3 active | 0x0400 | |
| 3.0   | EKF flags briefly 0x0000 | — | Attitude filter reset |
| 3.5   | **EKF3 IMU0 tilt alignment complete** | — | IMU gravity direction locked |
| 3.9   | **EKF3 IMU0 MAG0 initial yaw alignment complete** | 0x00a7 | Compass yaw locked |
| 3.9   | EKF flags = 0x00a7 | att + horiz_vel + vert_vel + vert_pos + CONST_POS_MODE | |
| 7.5   | GPS 1: detected u-blox | — | SITL synthetic GPS detected |
| 8.4   | GPS fix=6 (RTK fixed), 10 sats, HDOP=1.2 | — | |
| 17.7  | **EKF3 IMU0 origin set** | 0x00a7 | ~10 s after GPS detect (`gpsGoodToAlign`) |
| 22.7  | Armed in ACRO (CH8=1000 → interlock LOW) | — | force=True |

### Post-arm phase

| t (sim s) | Event | pos_h variance | Notes |
|-------|-------|----------------|-------|
| 0.0   | Armed; vel target set to [1.5, 0, -1.5] m/s | 0.00 | |
| 1.0   | Velocity ramp complete (0.3 m/s^2); GPS spd=1.50 m/s | 0.23 | Innovation spike |
| 2–9   | pos_h decreasing: 0.23 → 0.02 | — | EKF absorbing velocity innovation |
| 10–20 | pos_h oscillating: 0.02 → 0.15 | — | Bias learning phase |
| 20–54 | pos_h slowly decreasing: 0.15 → 0.07 | — | delAngBiasLearned converging |
| 54.5  | **EKF3 IMU0 is using GPS** | — | CONST_POS_MODE cleared, AID_ABSOLUTE |
| 54.5  | Test PASS | — | Total: 83 s |

---

## Physics vs EKF Divergence

### 1. Velocity — agreement

| Source | Value at cruise | Comment |
|--------|-----------------|---------|
| Physics (sent to SITL) | vel_x=1.50 m/s | `sens_vel_n` in telemetry.csv |
| GPS_RAW_INT spd | 1.50 m/s | Matches physics |
| EKF velocity_variance | 0.00 | EKF confident; horiz_vel bit = 1 |

GPS velocity fusion is working immediately after the velocity ramp.
`velocity_variance = 0.00` does NOT mean velocity is zero — it is the EKF's
internal variance estimate.  The horiz_vel flag confirms valid velocity.

### 2. Position — divergence until t=54.5 s

| Source | State during step [5] |
|--------|----------------------|
| Physics position | [1.5t, 0, -1.5t] NED — moving at 1.5 m/s |
| EKF position | Constant-position (CONST_POS_MODE, AID_NONE) until t=54.5 s |
| pos_h variance | Grows from 0 → 0.23, oscillates, then slowly falls to 0.07 |

The EKF holds a stationary position estimate throughout step [5] because
`readyToUseGPS()` is blocked by `delAngBiasLearned`.  At t=54.5 s the bias
converges, CONST_POS_MODE clears, and the EKF jumps to GPS position.

**Why does pos_h spike to 0.23 at t=1 s?**
The GPS position innovation is large: physics has moved ~1.5 m from the EKF's
stationary position estimate in the first second.  The EKF rejects the
measurement (innovation > gate) until innovations normalise.

### 3. Attitude — small divergence during acceleration

| Source | Value during velocity ramp | Value at cruise |
|--------|---------------------------|-----------------|
| Physics attitude | roll=0°, pitch=0°, yaw=0° (fixed) | 0°, 0°, 0° |
| EKF ATTITUDE | roll=0.0°, pitch ~0.2°–0.8°, yaw ~10° | pitch returns to 0° |

**Pitch offset during ramp:** Physics accelerates at 0.3 m/s^2 North (NED X).
This appears in accel_body as a +X component alongside the gravity -Z component.
The EKF's tilt estimator sees the resultant gravity vector tilted forward by
arctan(0.3 / 9.81) ≈ 1.75° and slowly adjusts its pitch estimate.  This is
physically correct behaviour — a level vehicle accelerating forward has the
same IMU signature as a 1.75° nose-up vehicle at rest.

**Yaw offset (~10°):** The SITL synthetic compass at London (51.5°N, -0.13°W)
reports magnetic north ~10° east of grid north due to magnetic declination +1°
and inclination effects.  The physics engine sends yaw=0° (grid North) but the
EKF compass calibrates to ~10.4° magnetic north.  This gap is small enough
(< 30°) that GPS velocity yaw fusion works without issue.

**Convergence:** After the velocity ramp completes and acceleration drops to
zero, the EKF pitch estimate returns to 0° (matching physics).  By t=30s the
attitude is roll=0.4°, pitch=0.4°, converging toward 0° by t=54s.

### 4. delAngBiasLearned — the main bottleneck

Flags remain 0x00a7 (CONST_POS_MODE present) for 54 s post-arm before GPS
fuses.  The only remaining blocker after origin + yaw is `delAngBiasLearned`.

With **constant gyro = [0,0,0]** and slowly changing accel during the ramp,
gyro bias convergence is slow.  ArduPilot EKF3 source indicates ~35 s of
varied motion is typical.  The slowly decaying pitch excitation during the
ramp provides the observability needed.

**Fix for production:** The real RAWES rotor has continuous gyro inputs from
orbital motion (~0.2–0.3 rad/s on all axes), which drives delAngBiasLearned
to converge within a few seconds.  This test is a worst-case: zero gyro input.

---

## Telemetry CSV

Written to `simulation/logs/gps_fusion_test_gps_fusion_armed/telemetry.csv`.
756 rows at 10 Hz (~75 s coverage).

Key columns:
- `pos_x/y/z` — physics NED position (m)
- `vel_x/y/z` — physics NED velocity (m/s)
- `accel_x/y/z` — kinematic acceleration in world NED (m/s^2)
- `sens_vel_n/e/d` — GPS velocity sent to SITL (= vel_x/y/z)
- `sens_accel_x/y/z` — body-frame specific force sent to SITL IMU
- `rpy_roll/pitch/yaw` — physics attitude in degrees (fixed at 0° for this test)

The `accel_z` column shows the level-flight specific force: at rest = 0 (world
frame kinematic accel), `sens_accel_z` = -9.81 (body-frame Z, pointing up).

---

## Key Findings

1. **EK3_GPS_CHECK=0 + widened gates are required** — without them the 10 s
   GPS quality wait blocks origin set and fusion is unreliable.

2. **Compass yaw alignment fires pre-arm** at t=3.9 s — much earlier than
   expected.  Step [4] no longer needs to wait post-arm.

3. **GPS velocity fuses immediately** after arm and velocity ramp.
   pos_h variance converges from 0.23 → 0.07 within ~50 s.

4. **delAngBiasLearned is the sole blocker** — takes ~54 s with zero gyro
   input.  With real orbital gyro inputs this will be << 10 s.

5. **Physics/EKF position diverge for 54 s** — the EKF holds a stationary
   position (CONST_POS_MODE) while physics moves ~80 m North + 80 m up before
   GPS fusion starts.  This is expected and harmless for the RAWES stack tests
   because kinematic startup runs for ~60 s before ArduPilot takes control.

6. **SITLInterface 5 ms receive timeout** (vs old raw-socket 100 ms) is
   critical — it allows the sensor worker to hit 100 Hz reliably and gives
   the EKF enough data rate to converge quickly.

---

## Running the Test

```bash
# Single run
bash simulation/dev.sh test-stack -v -k test_gps_fusion_armed

# Also runs the layer-by-layer parametrized version
bash simulation/dev.sh test-stack -v -k test_gps_fusion_layers
bash simulation/dev.sh test-stack -v -k "test_gps_fusion_layers[L0"
```

Logs: `simulation/logs/gps_fusion_test_gps_fusion_armed/`
- `gcs.log` — full EKF + GPS + ATTITUDE timeline
- `telemetry.csv` — physics trajectory (756 rows, 10 Hz)
- `dataflash.BIN` — ArduPilot DataFlash binary (XKF1/XKF3 for deep analysis)
- `arducopter.log` — SITL stdout
