# ArduPilot Helicopter EKF3 GPS Position Fusion — Source Analysis

Analysed from `/ardupilot/build/sitl/bin/arducopter-heli` (ArduCopter V4.8.0-dev).
Source files: `libraries/AP_NavEKF3/AP_NavEKF3_Control.cpp`, `AP_NavEKF3_VehicleStatus.cpp`,
`AP_NavEKF3_Measurements.cpp`, `AP_NavEKF3_MagFusion.cpp`, `AP_NavEKF3_GyroBias.cpp`.

---

## The GPS Position Fusion Gate

EKF3 transitions from `AID_NONE` (= CONST_POS_MODE) to `AID_ABSOLUTE` (= GPS position fusing)
only when `readyToUseGPS()` returns true (Control.cpp:594):

```cpp
bool NavEKF3_core::readyToUseGPS(void) const {
    if (frontend->sources.getPosXYSource(core_index) != AP_NavEKF_Source::SourceXY::GPS)
        return false;
    return validOrigin
        && tiltAlignComplete
        && yawAlignComplete
        && (delAngBiasLearned || assume_zero_sideslip())
        && gpsGoodToAlign
        && gpsDataToFuse;
}
```

All six conditions must be simultaneously true.

---

## Condition 1 — `getPosXYSource == GPS`

Controlled by `EK3_SRC1_POSXY`. Default value: **3 (GPS)**. Already correct — no change needed.

---

## Condition 2 — `gpsGoodToAlign` (10-second mandatory delay)

`calcGpsGoodToAlign()` (VehicleStatus.cpp) runs each EKF loop.
On its very first call, it sets `lastGpsVelFail_ms = now` regardless of check results:

```cpp
if (lastGpsVelFail_ms == 0) {
    lastGpsVelFail_ms = imuSampleTime_ms;  // start the clock
}
// ... individual checks (all masked if EK3_GPS_CHECK=0) ...
if (!gpsGoodToAlign && imuSampleTime_ms - lastGpsVelFail_ms > 10000) {
    gpsGoodToAlign = true;   // ← hardcoded 10 s wait
}
```

**`EK3_GPS_CHECK=0` masks all individual quality checks** (HDOP, sats, speed accuracy,
drift, yaw error, etc.) so they all pass immediately. But the **10-second clock still runs**
from the first GPS check. There is no parameter to shorten this delay.

`validOrigin` and `gpsDataToFuse` are both gated on `gpsGoodToAlign`, so GPS data only
enters the fusion buffer 10 s after SITL starts sending GPS.

**Practical consequence:** GPS position fusion cannot happen until at least **11–12 s** after
SITL launch, even with `EK3_GPS_CHECK=0`. Stack test timeouts must account for this.

---

## Condition 3 — `tiltAlignComplete`

Set when attitude error variance drops below `sq(radians(5.0))` (Control.cpp ~520):

```cpp
if (tiltErrorVariance < sq(radians(5.0))) {
    tiltAlignComplete = true;
    GCS_SEND_TEXT(... "EKF3 IMU%u tilt alignment complete" ...);
}
```

With a flat, non-moving hub sending `accel_body=[0,0,-9.81]`, this completes within **~3 s**.

---

## Condition 4 — `yawAlignComplete` (THE critical gate for our setup)

`yawAlignComplete` starts false and is set only when a yaw measurement successfully fuses.
The source is determined by `EK3_SRC1_YAW`:

| EK3_SRC1_YAW | SourceYaw value | What it uses |
|---|---|---|
| 0 | NONE | No yaw — position fusion never starts |
| 1 | COMPASS | SITL simulated magnetometer |
| 2 | GPS | GPS velocity heading — **default for heli** |
| 3 | GPS_COMPASS_FALLBACK | GPS velocity with compass fallback |
| 6 | EXTNAV | External navigation |
| 8 | GSF | Gaussian Sum Filter (from GPS velocity) |

**`EK3_SRC1_YAW=2` (default) FAILS with near-zero velocity.**
Both GPS velocity heading and GSF require a meaningful velocity vector to compute heading.
With velocity near zero, the computed heading is noise-dominated and never passes the
innovation gate. `yawAlignComplete` stays false and GPS position never fuses.

**Fix: `EK3_SRC1_YAW=1` (COMPASS).**
SITL simulates the magnetometer based on the synthetic earth field at the launch location.
With compass enabled (`COMPASS_USE=1`, `COMPASS_ENABLE=1`), yaw aligns within ~1–2 s
after `tiltAlignComplete`.

`use_compass()` returns true only when:
```cpp
(yaw_source_last == COMPASS || GPS_COMPASS_FALLBACK)
    && compass.use_for_yaw(magSelectIndex)
    && !allMagSensorsFailed
```
`yaw_source_last` is updated every loop from `EK3_SRC1_YAW`, so changing the parameter
at runtime takes effect immediately.

**Important:** during `yawAlignComplete` sequence, if `wasLearningCompass_ms > 0` (compass
offset learning is active), the yaw source is temporarily forced to NONE, blocking alignment.
Ensure `COMPASS_LEARN=0` (disabled) or that inflight learning does not start before yaw aligns.

---

## Condition 5 — `delAngBiasLearned || assume_zero_sideslip()`

### `assume_zero_sideslip()`

```cpp
bool NavEKF3_core::assume_zero_sideslip(void) const {
    return dal.get_fly_forward() && dal.get_vehicle_class() != AP_DAL::VehicleClass::GROUND;
}
```

ArduCopter/ArduHeli **never calls `set_fly_forward(true)`** (only ArduPlane does).
Therefore `assume_zero_sideslip() = false` for helicopter, and `delAngBiasLearned` is
**required** (not bypassed).

### `delAngBiasLearned`

Set when gyro bias state covariance has converged (Control.cpp:758–770):

```cpp
const ftype delAngBiasVarMax = sq(radians(0.15 * dtEkfAvg));
delAngBiasLearned = (P[10][10] <= delAngBiasVarMax) && (P[11][11] <= ...) && (P[12][12] <= ...);
```

Initial covariance (Control.cpp:174): `P[10..12] = sq(radians(2.5 * dtEkfAvg))`
(`InitialGyroBiasUncertainty()` returns 2.5; states are only activated after
`tiltAlignComplete`, so bias states start at t≈4 s).

`dtEkfAvg = EKF_TARGET_DT = 0.012 s` (EKF runs at ~83 Hz, defined in AP_NavEKF3_core.h).

Ratio initial/threshold = (2.5/0.15)² ≈ **278×** — must reduce by this factor.

**checkGyroCalStatus()** (Control.cpp:755): with `use_compass()=true` (our case),
**all three** variances P[10], P[11], P[12] must be below threshold.

**Convergence timing (measured in SITL):**
- With zero motion (gyro=[0,0,0], attitude flat): P does not converge — bias fully
  unobservable, `delAngBiasLearned` never becomes true.
- With yaw-only rotation (gyro=[0,0,5°/s], attitude flat): P[12] converges via
  compass heading updates. P[10] and P[11] converge **passively** (~30–35 s post
  tilt-align) through accumulated cross-covariance from EKF prediction steps +
  accelerometer tilt corrections. `delAngBiasLearned` becomes true ≈ **37 s**
  from SITL start (confirmed by test_stationary_gps).
- With multi-axis oscillation (roll/pitch + yaw): biases converge faster but the
  tilted gravity components cause horizontal position dead-reckoning drift during
  AID_NONE. When GPS fusion starts the position innovation is too large → EKF
  unhealthy → fusion rejected. **Do not add real roll/pitch tilt to the stub**.

**Practical consequence:** Stack tests must allow ≥ 40 s before expecting GPS
position fusion. The binding gate is usually `delAngBiasLearned`, not
`gpsGoodToAlign` (which clears at t≈19 s).

---

## Condition 6 — `gpsDataToFuse`

Set by (PosVelFusion.cpp:537):
```cpp
gpsDataToFuse = storedGPS.recall(gpsDataDelayed, imuDataDelayed.time_ms) && !waitingForGpsChecks;
```

GPS data is only pushed to the buffer when `validOrigin && !waitingForGpsChecks`
(Measurements.cpp). Both require `gpsGoodToAlign` to be true first. See Condition 2.

---

## Summary: Required Parameters for GPS Position Fusion

| Parameter | Required value | Why |
|---|---|---|
| `EK3_SRC1_POSXY` | 3 (GPS) | GPS as horizontal position source (default ✓) |
| `EK3_SRC1_VELXY` | 3 (GPS) | GPS as horizontal velocity source (default ✓) |
| `EK3_SRC1_YAW`   | **1 (COMPASS)** | Compass yaw — default=2 (GPS velocity) never works at low speed |
| `EK3_GPS_CHECK`  | **0** | Mask all GPS quality checks (SITL GPS has no real quality fields) |
| `COMPASS_USE`    | **1** | Enable compass for EKF yaw fusion |
| `COMPASS_ENABLE` | **1** | Enable compass sensor |
| `EK3_MAG_CAL`    | 0 (Never) or 3 (Always) | 0=use raw compass immediately; 3=fine but needs movement |

Parameters that **do NOT exist** in this build (4.8.0-dev heli):
- `EK3_GPS_CTRL` — not compiled; GPS position+velocity fusion is always on when `EK3_SRC1_POSXY=3`

---

## Timing Requirements (stack test design)

### Stationary GPS test (yaw-only stub, EK3_SRC1_YAW=1)

| Event | Time from SITL start |
|---|---|
| EKF3 tilt alignment | ~3–4 s |
| Compass yaw alignment (EK3_SRC1_YAW=1) | ~4–5 s |
| GPS detected (SITL JSON backend) | ~8–9 s |
| `gpsGoodToAlign=true` (10 s delay from GPS detect) | ~18–19 s |
| `validOrigin=true` + GPS data in buffer | ~19 s |
| `delAngBiasLearned=true` (yaw-only stub motion) | **~37–42 s** |
| GPS position fusion starts | **~41 s** |

### Real RAWES mediator (constant-velocity kinematic, EK3_SRC1_YAW=1)

| Event | Time from SITL start |
|---|---|
| EKF3 tilt alignment | ~4–5 s |
| Compass yaw alignment (immediate after tilt) | ~5–6 s |
| GPS detected | ~9 s |
| `gpsGoodToAlign=true` | ~19–20 s |
| `delAngBiasLearned=true` | **~29–33 s** (faster than stub) |
| GPS position fusion ("EKF3 is using GPS") | **~31–33 s** |

With EK3_SRC1_YAW=1 (compass) the real mediator converges delAngBiasLearned
**faster** than the yaw-only stub (~33 s vs ~37 s), confirmed in stack test run.

**Critical: use EK3_SRC1_YAW=1 (compass), NOT EK3_SRC1_YAW=2 (GPS velocity)**
for the real stack test.  With EK3_SRC1_YAW=2 and a moving hub (vel0≈0.96 m/s
kinematic):
- The hub moves ~24 m during the 45 s kinematic window
- When delAngBiasLearned fires (~43 s), GPS position innovation = 24 m
- The combined position+velocity state update causes EKF health reset
- Flags drop from 0x00a7 to 0x0400 (only predicted_pos); GPS never fuses

With EK3_SRC1_YAW=1 (compass) and constant body orientation (R=_R0 locked):
- Compass heading is constant → P[12] converges in ~1 s after tilt align
- P[10,11] converge passively in ~28 s → delAngBiasLearned at ~33 s
- GPS fuses at ~33 s (hub still in kinematic = stable) → SUCCESS

**Stack tests must set STARTUP_DAMP_S = 45 s** so GPS fusion (~33 s) occurs
during the stable kinematic phase.  The binding gate is `delAngBiasLearned`.

Required param set for GPS position fusion in real stack tests:
```
EK3_SRC1_YAW  = 1   (compass, NOT GPS velocity)
EK3_MAG_CAL   = 0   (Never — use raw compass immediately)
COMPASS_USE   = 1
COMPASS_ENABLE= 1
EK3_GPS_CHECK = 0
```

---

## CONST_POS_MODE Bit Meaning

`EKF_STATUS_REPORT.flags` bit 7 (0x0080) = `const_pos_mode`:

```cpp
status.flags.const_pos_mode = (PV_AidingMode == AID_NONE) && filterHealthy;
```

This is set when the EKF is healthy (tilt + yaw aligned, `filterHealthy=true`) but
has no position/velocity aiding source (`AID_NONE`). It clears when `readyToUseGPS()`
returns true and the mode switches to `AID_ABSOLUTE`.

Seeing `0x00a7` = bits {0,1,2,5,7} means:
- 0x0001 att ✓, 0x0002 horiz_vel ✓, 0x0004 vert_vel ✓, 0x0020 vert_pos_abs ✓
- 0x0080 CONST_POS_MODE — stuck in AID_NONE, GPS position not yet fusing

---

## Heli-Specific Behaviour Differences from Copter

The binary is `arducopter-heli` — the helicopter variant of ArduCopter. Key differences
affecting EKF3:

1. **`fly_forward=false`** — copter/heli never sets `fly_forward`, so `assume_zero_sideslip()=false`.
   This means `delAngBiasLearned` is always required (cannot be bypassed).

2. **Default `EK3_SRC1_YAW=2`** (GPS velocity heading) — works for fixed-wing (always moving
   forward) but fails for hover-capable vehicles at low/zero speed.

3. **`EK3_MAG_CAL=3` (ALWAYS)** — more aggressive than fixed-wing default. The EKF tries to
   learn magnetometer biases in-flight always. Does not block yaw alignment itself, but
   `wasLearningCompass_ms` check can temporarily suppress compass yaw if `COMPASS_LEARN=2`
   (inflight learning) is active simultaneously.
