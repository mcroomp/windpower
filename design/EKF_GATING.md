# EKF3 GPS-Aiding & Yaw-Alignment Gating (ArduPilot)

How ArduPilot's EKF3 decides to start using GPS, why it can get **stuck in
`const_pos_mode` (AID_NONE)** through the entire kinematic hold, and the exact
source-code path that gates moving-baseline GPS-yaw alignment. Written from a
full source + dataflash trace of `test_lua_flight_steady_sitl` (2026-06).

All ArduPilot line numbers are against the local checkout at `C:/repos/ardupilot`
and may drift; treat them as anchors, not addresses. Re-grep the symbol names.

---

## 0. TL;DR for this project

- The kite spends 60 s in the kinematic hold so the EKF can fuse GPS **before**
  release. If the EKF is still in `const_pos_mode` at t=60 s, the Lua can never
  capture GPS and the post-release flight is meaningless. `diagnose_sitl.py`
  CHECK 1 gates on exactly this.
- **Verified blocker for `test_lua_flight_steady_sitl`:** `yawAlignComplete` is
  never set because `storedYawAng.recall()` fails every cycle, so the EKF holds
  a *fake* `PREDICTED` yaw (correct value, but the alignment flag stays false)
  and `readyToUseGPS()` never passes. The vehicle's level→tilt attitude profile
  and the moving-baseline GPS yaw are **both fine**; the failure is a GPS-yaw
  **timestamp/lag** mismatch into the EKF fusion-horizon buffer.

---

## 1. Aiding modes and `const_pos_mode`

`NavEKF3_core::setAidingMode()` (`AP_NavEKF3_Control.cpp`) moves the filter
between:

| `PV_AidingMode` | Meaning | `nav_filter_status` effect |
|-----------------|---------|----------------------------|
| `AID_NONE`      | No position/velocity aiding. Filter holds a constant position to stop covariance blow-up. | `const_pos_mode=1`, `horiz_pos_abs=0`, `using_gps=0` |
| `AID_RELATIVE`  | Optical-flow relative aiding | — |
| `AID_ABSOLUTE`  | GPS (or beacon/extnav) absolute aiding | `const_pos_mode=0`, `horiz_pos_abs=1`, `using_gps=1` |

To enter `AID_ABSOLUTE` from `AID_NONE`, `setAidingMode()` calls
**`readyToUseGPS()`**. Until that returns true the filter stays in `AID_NONE`,
which is logged as `const_pos_mode`.

The MAVLink `EKF_STATUS_REPORT.flags` and the dataflash `XKF4.SS`
(`nav_filter_status`) encode this. Key bits (see `AP_Nav_Common.h`):

| Bit | Mask | Name |
|-----|------|------|
| 0 | 0x0001 | attitude (tiltAlignComplete proxy) |
| 1 | 0x0002 | horiz_vel |
| 2 | 0x0004 | vert_vel |
| 3 | 0x0008 | horiz_pos_rel |
| 4 | 0x0010 | horiz_pos_abs (GPS pos fusing) |
| 5 | 0x0020 | vert_pos |
| 7 | 0x0080 | **const_pos_mode** (AID_NONE) |
| 13 | 0x2000 | using_gps |
| 14 | 0x4000 | gps_glitching |
| 15 | 0x8000 | gps_quality_good |
| 16 | 0x10000 | initialized |

---

## 2. `readyToUseGPS()` — the gates to leave `const_pos_mode`

`NavEKF3_core::readyToUseGPS()` (`AP_NavEKF3_Control.cpp`, ~line 516+) returns
true only when **all** of:

1. **PosXY source == GPS** — `frontend->sources.getPosXYSource(core_index) == GPS`
   (param `EK3_SRC1_POSXY=3`).
2. **`validOrigin`** — EKF origin set (dataflash `ORGN` Type==0; STATUSTEXT
   "EKF3 IMUx origin set").
3. **`tiltAlignComplete`** — attitude aligned (tiltErrVar small). Latches early;
   message "EKF3 IMUx tilt alignment complete"; proxy = `XKF4.SS` bit 0.
4. **`yawAlignComplete`** — yaw aligned. **This is the one that fails.** See §4.
5. **`delAngBiasLearned`** — gyro delta-angle bias converged. Not directly
   logged; usually clears in seconds.
6. **`gpsGoodToAlign`** — GPS quality checks pass (`XKF4.GPS`/`GCK` == 0) and
   `gps_quality_good`. Controlled by `EK3_GPS_CHECK` + the gate params.
7. **`gpsDataToFuse`** — a fresh GPS sample is available at the fusion horizon.

`diagnose_sitl.py`'s `_const_pos_exit_gates()` prints one PASS/FAIL line per gate
from dataflash + params, and names the blocking gate(s).

---

## 3. Which GPS instance carries yaw (moving baseline)

This project runs a **moving-baseline dual GPS** for yaw (`EK3_SRC1_YAW=2`):

- `GPS1_TYPE=17` (UBLOX moving-baseline **base**, instance 0).
- `GPS2_TYPE=18` (UBLOX moving-baseline **rover**, instance 1).
- Antennas at `GPS1_POS=(+0.25,0,0)`, `GPS2_POS=(-0.25,0,0)` (0.5 m on body X).
- `COMPASS_USE=0` (GB4008 motor interference; compass disabled).

Key facts about how yaw flows to the EKF:

- The **rover** (instance 1) computes the yaw in
  `AP_GPS_Backend::calculate_moving_base_yaw()` (`GPS_Backend.cpp` ~320). It
  sets `state.gps_yaw`, `state.have_gps_yaw=true`, and
  **`state.gps_yaw_time_ms = AP_HAL::millis()`** (wall/boot millis at processing,
  NOT a GPS time-of-week). It also runs a vertical-separation sanity check that
  projects the antenna offset through the AHRS attitude + gyro — at high tilt
  this still passed in our log (`GPYW OK=1` throughout).
- The EKF reads yaw from `selected_gps = gps.primary_sensor()`
  (`update_gps_selection`, `AP_NavEKF3_Measurements.cpp` ~1171).
- **For a moving-baseline pair the primary is forced to the BASE** (instance 0),
  because the rover position is derived from the base and is "worse"
  (`AP_GPS::update_primary`, `AP_GPS.cpp` ~1146-1158).
- That is fine, because **`AP_GPS::gps_yaw_deg(instance)`** (`AP_GPS.cpp` ~1998)
  XORs base↔rover and returns the **rover's** yaw when queried for the base:
  ```cpp
  if (is_rtk_base(instance) && is_rtk_rover(instance^1)) {
      instance ^= 1;   // return the yaw from the rover
  }
  ```
- The returned `time_ms` is `state.gps_yaw_time_ms` **minus the GPS lag**
  (`get_lag()`), and accuracy is floored to 5° later in `readGpsYawData`.

So GPS yaw is correctly available to the EKF even with the base as primary.

`GPYW` dataflash message (logged by `calculate_moving_base_yaw`): `Id` =
instance, `RHD` = reported heading (deg), `RDist`/`RDown` = antenna separation,
`OK` = yaw accepted. `OK=1` means `have_gps_yaw=true` that cycle.

---

## 4. The yaw-alignment path (where it breaks)

`SelectMagFusion()` → `controlMagYawReset()` (`AP_NavEKF3_MagFusion.cpp`) runs
every update. With `COMPASS_USE=0` and `yaw_source_last = getYawSource() = GPS`
(set in `AP_NavEKF3_core.cpp` ~546 and `_Control.cpp` ~226 — **not** latched to
GSF), control enters the **GPS-yaw block** (~line 287):

```cpp
if (yaw_source_last == GPS || yaw_source_last == GPS_COMPASS_FALLBACK) {
    if (storedYawAng.recall(yawAngDataDelayed, imuDataDelayed.time_ms)) {
        if (tiltAlignComplete && (!yawAlignComplete || yaw_source_reset)) {
            alignYawAngle(yawAngDataDelayed);   // <-- sets yawAlignComplete
            ...
        } else if (tiltAlignComplete && yawAlignComplete) {
            fuseEulerYaw(yawFusionMethod::GPS); // incremental fusion
        }
    } else if (tiltAlignComplete && !yawAlignComplete) {
        // recall FAILED: fuse a FAKE yaw at 7 Hz to keep the filter stable
        fuseEulerYaw(yawFusionMethod::PREDICTED);   // or STATIC on ground-not-moving
    }
    ...
}
```

- **`alignYawAngle()`** (`MagFusion.cpp` ~213) is the moving-baseline alignment.
  It sets `yawAlignComplete=true` and prints **`"EKF3 IMUx yaw aligned"`**
  (NO suffix).
- The buffer is fed by **`readGpsYawData()`** (`AP_NavEKF3_Measurements.cpp`
  ~735), which pushes via **`writeEulerYawAngle()`** (~1037) into `storedYawAng`,
  stamping `yawAngDataNew.time_ms = timeStamp_ms` (= the lagged GPS yaw time).
  It only pushes when `statesInitialised` and on a new GPS-yaw timestamp.
- **`recall(element, sample_time_ms)`** looks up the buffered yaw sample at/just
  before `imuDataDelayed.time_ms` (the EKF's delayed fusion horizon). It returns
  false if no buffered sample lands in that window.

### The GSF fallback (the red herring)

Separately, the **EKF-GSF** velocity-based yaw estimator
(`runYawEstimatorCorrection` + `EKFGSF_resetMainFilterYaw`, `_Control.cpp` ~847,
`MagFusion.cpp` ~1429) prints **`"EKF3 IMUx yaw aligned using GPS"`** (WITH the
"using GPS" suffix). It needs real horizontal speed/maneuvering (~5 m/s) to
converge and only fires on a reset request. It is **not** the moving-baseline
path.

> Distinguishing the two messages is the fastest triage:
> - `"yaw aligned"` → moving-baseline `alignYawAngle()` ran (good).
> - `"yaw aligned using GPS"` → GSF velocity fallback ran (needs motion; usually
>   too late, only after release).

---

## 5. Verified failure for `test_lua_flight_steady_sitl`

Evidence (dataflash + STATUSTEXT, times in sim-seconds ≈ dataflash boot-seconds):

| Signal | Observation |
|--------|-------------|
| Attitude (`ATT`) | level (`pitch≈0`) until ~24 s; nul-aero snaps to `pitch=-63.6°` by ~29 s; holds to release at 60 s. Tilt is **early**, not late. |
| `ATT.Yaw` | 90.0° rock-steady through level, tilt, and hold. |
| `GPYW` | `OK=1`, `RHD≈-90°` throughout (rover yaw valid the whole phase). |
| Origin | set ~21.7 s. |
| `XKF4.SS` @60 s | `0x180a7`: attitude=1, gps_good=1, **const_pos=1**, using_gps=0, horiz_pos_abs=0. |
| Yaw messages | plain `"yaw aligned"` **never** appears; only GSF `"yaw aligned using GPS"` at 81.6 s (post-release). |
| Aiding | `"is using GPS"` (AID_ABSOLUTE) at 78.6 s — well after the 60 s release. |

Deduction (every other branch excluded):
`yaw_source=GPS` ✓, GPS-yaw block entered ✓, `tiltAlignComplete=true` ✓,
`yawAlignComplete=false` ✓, `alignYawAngle()` never ran ✓ (no message) ⇒
**`storedYawAng.recall()` returns false every cycle.** The EKF takes the
`else` branch and fuses a fake `PREDICTED` yaw, which merely holds the init yaw
(90°, correct by coincidence) without ever setting `yawAlignComplete`. Hence
`readyToUseGPS()` fails and the EKF stays in `const_pos_mode` through release.

**Root mechanism:** the moving-baseline yaw is stamped
`gps_yaw_time_ms = AP_HAL::millis()` (minus `get_lag()`) and that timestamp never
lands at the EKF's `imuDataDelayed.time_ms` fusion horizon, so `recall()` never
matches. This is independent of the trajectory shape — going in a curve only
helps the GSF velocity path, not `alignYawAngle()`.

---

## 6. SITL moving-baseline yaw synthesis

`SITL::GPS_UBlox::update_relposned()` (`libraries/SITL/SIM_GPS_UBLOX.cpp` ~38)
builds the RELPOSNED from the antenna geometry, projecting the body→NED rotation
back through the gyro by the configured `delay_ms`. It sets
`relPosHeading` and the `relPosValid | relPosHeadingValid` flags. The ArduPilot
UBLOX driver then calls `calculate_moving_base_yaw()`. Relevant SITL params:
`SIM_GPS1_TYPE`/`SIM_GPS2_TYPE`, `SIM_GPS*_POS_*`, `SIM_GPS*_HDG`,
`SIM_GPS*_DELAY_MS`.

The timestamp/lag chain to inspect when `recall()` fails:
`SIM_GPS*_DELAY_MS` (injection delay) → `gps_yaw_time_ms = millis()` →
`gps_yaw_deg` subtracts `get_lag()` → push to `storedYawAng` →
`recall()` at `imuDataDelayed.time_ms` (governed by the IMU delay buffer and
`EK3_GPS_DELAY`). A mismatch between the injected delay and the EKF's expected
GPS lag is the classic cause of `recall()` never landing in-window.

---

## 7. Diagnosing from logs

- **Run `diagnose_sitl.py <test>` first.** CHECK 1 prints the
  `readyToUseGPS()` gate table and names the blocker; CHECK 2 checks the
  kinematic hand-off vs the IC.
- **Dataflash time base:** `TimeUS` is ArduPilot boot-seconds and ≈ **sim time**
  in lockstep SITL (within ~1 s; GPS/EKF logging starts ~14 s after boot). Do
  NOT re-zero dataflash times to the first message — that shifts everything ~14 s
  early and pulls post-release data inside the 60 s window.
- **Yaw triage:** grep STATUSTEXT for `"yaw aligned"` vs `"yaw aligned using
  GPS"` (see §4). Check `GPYW.OK`. Check `XKF4.SS` const_pos/using_gps bits.
- **GPS yaw accuracy field is `GPA.YAcc`** (not `Yaw`); `readGpsYawData` floors
  it to 5° anyway, so accuracy is not the blocker here.

---

## 8. Source-code index (re-grep symbols; line numbers drift)

| Symbol / message | File |
|------------------|------|
| `setAidingMode`, `readyToUseGPS` | `AP_NavEKF3/AP_NavEKF3_Control.cpp` |
| `runYawEstimatorPrediction/Correction`, GSF reset action | `AP_NavEKF3/AP_NavEKF3_Control.cpp` |
| `controlMagYawReset`, GPS-yaw block, `alignYawAngle`, GSF reset | `AP_NavEKF3/AP_NavEKF3_MagFusion.cpp` |
| `readGpsYawData`, `writeEulerYawAngle`, `update_gps_selection` | `AP_NavEKF3/AP_NavEKF3_Measurements.cpp` |
| `yaw_source_last = getYawSource()` | `AP_NavEKF3/AP_NavEKF3_core.cpp` |
| `update_primary` (MB base forced primary), `gps_yaw_deg` (base↔rover XOR) | `AP_GPS/AP_GPS.cpp` |
| `calculate_moving_base_yaw` (sets `gps_yaw_time_ms`, GPYW log) | `AP_GPS/GPS_Backend.cpp` |
| `update_relposned` (SITL RELPOSNED synth) | `SITL/SIM_GPS_UBLOX.cpp` |
| `nav_filter_status` bit defs | `AP_NavEKF/AP_Nav_Common.h` |

Related project docs: [ekf_const_pos_mode.md](ekf_const_pos_mode.md),
[flight_stack.md](flight_stack.md) (EKF3 config, Appendix D GPS timing).
Diagnostic tool: `analysis/diagnose_sitl.py`.
