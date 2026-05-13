# EKF3 `const_pos_mode` — Causes and Log Diagnostics

## What `const_pos_mode` means

Defined in `AP_NavEKF3/AP_NavEKF3_Control.cpp:783`:
```cpp
status.flags.const_pos_mode = (PV_AidingMode == AID_NONE) && filterHealthy;
```
The EKF has no position/velocity aiding source and is holding a fixed position estimate. It requires both conditions: the filter is in `AID_NONE` **and** it is healthy (otherwise it reports nothing meaningful).

`filterHealthy` is defined at line 770:
```cpp
healthy() && tiltAlignComplete && (yawAlignComplete || (!use_compass() && PV_AidingMode != AID_ABSOLUTE))
```

---

## Reason 1: Never left AID_NONE at startup

The EKF initializes in `AID_NONE` and only leaves when a `readyToUse*()` function returns true (`AP_NavEKF3_Control.cpp:284-298`). If none of them are satisfied, it stays indefinitely.

### `readyToUseGPS()` fails (`AP_NavEKF3_Control.cpp:585-592`):
| Condition | Cause |
|---|---|
| `getPosXYSource() != GPS` | `EK3_SRC1_POSXY` not set to GPS |
| `!validOrigin` | EKF origin not yet established (needs `gpsGoodToAlign` first) |
| `!tiltAlignComplete` | IMU tilt not yet converged |
| `!yawAlignComplete` | Heading not yet aligned (compass, GPS-yaw, or GSF) |
| `!delAngBiasLearned && !assume_zero_sideslip()` | Gyro bias not yet converged (non-fixed-wing) |
| `!gpsDataToFuse` | No GPS data arriving at all |
| `!gpsGoodToAlign` | GPS quality checks failing (see below) |

### `gpsGoodToAlign` fails (`AP_NavEKF3_VehicleStatus.cpp:221-232`):

Requires **10 consecutive seconds** with all enabled checks passing. Once passing, reverts to false if **5 seconds** pass without a pass:

| Check | Fail threshold | Controlled by |
|---|---|---|
| Horizontal position drift | > 3 m (on-ground, filtered) | `MASK_GPS_POS_DRIFT` |
| Vertical GPS speed | > 0.3 m/s filtered (on-ground) | `MASK_GPS_VERT_SPD` |
| Horizontal GPS speed | > 0.3 m/s filtered (on-ground) | `MASK_GPS_HORIZ_SPD` |
| Horizontal accuracy (hAcc) | > 5 m | `MASK_GPS_POS_ERR` |
| Vertical accuracy (vAcc) | > 7.5 m | `MASK_GPS_POS_ERR` |
| Speed accuracy (sAcc) | > 1.0 m/s | `MASK_GPS_SPD_ERR` |
| HDOP | > 2.5 | `MASK_GPS_HDOP` |
| Satellite count | < 6 | `MASK_GPS_NSATS` |
| Magnetometer yaw innovation | ratio > 1.0 | `MASK_GPS_YAW_ERR` |

All thresholds are scaled by `EK3_CHECK_SCALE` (default 100 = x1.0, with x1.3 hysteresis once already passing).

### `readyToUseExtNav()` fails (`AP_NavEKF3_Control.cpp:609-619`):
- `getPosXYSource() != EXTNAV`, or `!tiltAlignComplete`, or `!extNavDataToFuse`

### `readyToUseOptFlow()` fails (`AP_NavEKF3_Control.cpp:542-555`):
- `_flowUse != FLOW_USE_NAV`, or XY source not OPTFLOW, or flow data older than 200 ms, or `!tiltAlignComplete`, or `!delAngBiasLearned`

### `readyToUseBodyOdm()` fails (`AP_NavEKF3_Control.cpp:559-582`):
- XY source not EXTNAV/WHEEL_ENCODER, or no fresh odometry/wheel data, or `!tiltAlignComplete`, or `!delAngBiasLearned`

---

## Reason 2: Fell back from AID_ABSOLUTE (critical attitude aiding loss)

`AP_NavEKF3_Control.cpp:388-395` — triggered when `attAidLossCritical` is true:

**All** of the following must have been absent longer than `EK3_TILT_DMAX` (default **15 s**):
- GPS position
- GPS velocity
- Optical flow
- Airspeed
- Range beacon
- Body odometry

This is a total loss of any sensor that can constrain attitude drift. The EKF drops to `AID_NONE` immediately and sets `posTimeout = velTimeout = true`.

---

## Reason 3: Fell back from AID_RELATIVE (flow/odometry timeout)

`AP_NavEKF3_Control.cpp:310-312` — if in relative aiding mode (optical flow / body odometry) and both sensors time out:
- Last successful optical flow fusion > **5 s** ago, **and**
- Last successful body odometry fusion > **5 s** ago

---

## Reason 4: Forced reset — yaw source NONE while disarmed on ground

`AP_NavEKF3_Control.cpp:248-281` — if `EK3_SRC1_YAW = 0` (no yaw source), motors disarmed, and currently on ground, the mode is forced to `AID_NONE` and yaw/velocity/position states are reset. This prevents navigation with an untrustworthy heading.

---

## Summary of common real-world causes

In rough order of frequency:
1. **GPS quality checks not yet passed** — poor sAcc/hAcc/HDOP/sats, especially immediately after power-on
2. **Yaw not aligned** — compass bad or GSF not converged, blocking `yawAlignComplete`
3. **Gyro bias not learned** — `delAngBiasLearned` takes ~30 s after boot on a cold IMU
4. **No GPS data at all** — antenna/wiring fault or GPS module not responding
5. **EK3_SRC1_POSXY misconfigured** — position source not set to GPS (or whichever sensor is present)
6. **In-flight GPS/aiding loss** — all aiding sources disappear for >15 s, forced back from AID_ABSOLUTE

---

## Diagnosing with Log Messages

---

### First stop: GCS text messages (`MSG` in DataFlash)

These are the most direct indicators of mode transitions:

| Message | Meaning |
|---|---|
| `"EKF3 IMU%u stopped aiding"` | Dropped into AID_NONE — look at the timestamp to find the trigger |
| `"EKF starting GPS checks"` | Filter just began its 10-second GPS check window |
| `"EKF3 IMU%u is using GPS"` | Successfully left AID_NONE via GPS |
| `"EKF3 IMU%u started relative aiding"` | Left AID_NONE via optical flow/body odometry |
| `"GPS drift %.1fm (needs %.1f)"` | GPS quality check failing — drift too large |
| `"GPS horiz error %.1fm"` | hAcc failing |
| `"GPS vert error %.1fm"` | vAcc failing |
| `"GPS speed error %.1f"` | sAcc failing |
| `"GPS HDOP %.1f (needs 2.5)"` | HDOP failing |
| `"GPS numsats %u (needs 6)"` | Satellite count failing |
| `"Mag yaw error x=%.1f y=%.1f"` | Compass yaw innovations too large — bad heading alignment |
| `"GPS vertical speed %.2fm/s"` | GPS vertical speed check failing (on ground) |

---

### `XKF4` — the most important log message

Logged every EKF cycle. Two fields answer almost everything:

**`SS` (solution status bitmask)** — `nav_filter_status.value` (`AP_Nav_Common.h:55`):
| Bit | Flag | What it tells you |
|---|---|---|
| 7 | `const_pos_mode` | EKF is in AID_NONE and healthy |
| 4 | `horiz_pos_abs` | GPS navigation working |
| 3 | `horiz_pos_rel` | Relative nav (flow/odometry) working |
| 15 | `gps_quality_good` | `gpsGoodToAlign` — GPS quality checks passed |
| 13 | `using_gps` | GPS was fused in the last 4s |
| 14 | `gps_glitching` | GPS accepted but position estimate is glitching |

If bit 7 is set but bit 15 is clear → GPS quality checks are the blocker.
If bit 7 is set and bit 15 is set → GPS is good quality but something else blocks `readyToUseGPS()` (tilt/yaw alignment, no data, wrong source config).

**`GPS` (GPS check status bitmask)** — `gpsCheckStatus.value` (`AP_NavEKF3_core.h:1521-1535`):
| Bit | Flag | Check failing |
|---|---|---|
| 0 | `bad_sAcc` | Speed accuracy > 1.0 m/s |
| 1 | `bad_hAcc` | Horizontal accuracy > 5 m |
| 2 | `bad_vAcc` | Vertical accuracy > 7.5 m |
| 3 | `bad_yaw` | Compass/yaw innovation ratio > 1.0 |
| 4 | `bad_sats` | Fewer than 6 satellites |
| 5 | `bad_VZ` | Vertical speed check (internal) |
| 6 | `bad_horiz_drift` | Horizontal position drift > 3 m |
| 7 | `bad_hdop` | HDOP > 2.5 |
| 8 | `bad_vert_vel` | GPS vertical velocity > 0.3 m/s |
| 10 | `bad_horiz_vel` | GPS horizontal velocity > 0.3 m/s |

Any non-zero value here keeps `gpsGoodToAlign` false.

**`TS` (timeout bitmask)** — if set during `const_pos_mode`, it was a fallback from AID_ABSOLUTE:
| Bit | Timeout |
|---|---|
| 0 | `posTimeout` |
| 1 | `velTimeout` |
| 2 | `hgtTimeout` |
| 3 | `magTimeout` |
| 4 | `tasTimeout` |
| 5 | `dragTimeout` |

If bits 0+1 both set at the same time `const_pos_mode` appears, the EKF was in flight and lost all aiding sources.

---

### `XKFS` — sensor selection

| Field | Meaning |
|---|---|
| `GGA` | `gpsGoodToAlign` — 0 means GPS checks still failing |
| `WFG` | `waitingForGpsChecks` — 1 means actively waiting for the 10s window |
| `MF` | Magnetometer fusion mode (0=none, useful to confirm compass is active) |
| `SS` | Source set index (`EK3_SRC1_*` etc.) |

If `WFG=1` for a long time → GPS signal quality is intermittent, checks keep resetting.

---

### `XKF3` — innovations (for yaw/compass diagnosis)

| Field | Meaning |
|---|---|
| `IMX`, `IMY`, `IMZ` | Magnetometer innovations — large values → compass interference → `bad_yaw` |
| `IYW` | Yaw innovation — non-zero while not aiding means heading is drifting |
| `IVN`, `IVE` | Velocity innovations — should collapse to 0 once aiding starts |

---

### `XKF1` — internal state convergence

| Field | Meaning |
|---|---|
| `GX`, `GY`, `GZ` (gyroBias) | If these are still changing rapidly, `delAngBiasLearned` is likely false |
| `originHgt` | If 0, `validOrigin` has not been set yet |

---

### Diagnostic flow for a log file

```
1. Find timestamp where SS bit 7 goes high → start of const_pos_mode

2. Check XKF4.GPS → non-zero bits = which GPS quality check is failing
   └─ bit 3 set (bad_yaw): look at XKF3.IMX/IMY/IYW — compass interference
   └─ bit 0/1/4/7: poor GPS signal, look at GPS.NSats, GPS.HDop

3. Check XKFS.GGA → 0 means gpsGoodToAlign never reached
   Check XKFS.WFG → 1 means stuck waiting for 10s clean window

4. Check XKF4.SS bit 15 (gps_quality_good)
   └─ If 1 but still const_pos_mode: tilt/yaw align not complete
      → Check XKF4.TE (tilt error variance) for tiltAlignComplete proxy
      → Check MSG for "EKF starting GPS checks" vs "EKF3 is using GPS"

5. If const_pos_mode starts mid-flight (XKF4.TS bits 0+1 both set):
   → All aiding timed out beyond tiltDriftTimeMax (EK3_TILT_DMAX, default 15s)
   → Look at GPS dropouts, optflow/odometry gaps in that window
```
