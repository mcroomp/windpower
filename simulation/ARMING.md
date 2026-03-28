# ArduPilot Helicopter Arming — Knowledge Reference

> **Update this file whenever you discover new arming behavior.**
> See also: `CLAUDE.md` → "Running Tests" for the full test workflow.

---

## How ArduCopter Helicopter Arming Actually Works

ArduCopter Helicopter has a two-stage arm that confuses people who are used to multirotor:

1. **Pre-arm checks** — `AP_Arming_Copter::arm()` runs checks controlled by `ARMING_CHECK`.
   - `force=True` (param2=21196 in `MAV_CMD_COMPONENT_ARM_DISARM`) bypasses these.
   - `ARMING_CHECK=0` disables all checks (including in the non-force path).

2. **Motor armed state** — `AP_MotorsHeli::output()` runs on every loop and resets
   `_flags.armed = false` if `is_armed_and_runup_complete()` returns false.
   **This is NOT bypassed by force arm.** COMMAND_ACK will say ACCEPTED but
   HEARTBEAT armed bit will be false until the RSC has completed its runup.

The HEARTBEAT armed flag = `AP_Notify::flags.armed` = `motors->armed()` = `_flags.armed`.
So you cannot be "armed" in the HEARTBEAT sense until the RSC is happy.

---

## RSC Modes (H_RSC_MODE parameter)

| Mode | Name | Runup behaviour | Notes |
|------|------|-----------------|-------|
| 0 | Disabled | N/A | **INVALID** — causes "PreArm: Motors: H_RSC_MODE invalid" and immediate auto-disarm |
| 1 | CH8 Passthrough | Immediate — RSC = CH8 | No ramp; `runup_complete` fires instantly when CH8 is high. **Use this for SITL.** |
| 2 | Setpoint | Ramp to H_RSC_SETPOINT | Requires H_RUNUP_TIME (minimum > 0); motor interlock must be ON after arm |
| 3 | Throttle Curve | Ramp via throttle curve | Similar runup wait as mode 2 |
| 4 | External Governor | External feedback | Requires RPM telemetry from ESC |

**SITL default is 0 (invalid).** Always set H_RSC_MODE explicitly.

---

## Motor Interlock (CH8)

CH8 PWM controls the "motor interlock" — a safety gate that allows/prevents motor operation:

| CH8 value | Interlock state | RSC behaviour |
|-----------|----------------|---------------|
| 1000 | LOW (disabled) | RSC output = 0 / no runup |
| 2000 | HIGH (enabled) | RSC can run |

**STATUSTEXT messages:**
- `"RC8: MotorInterlock HIGH"` — CH8 crossed HIGH threshold
- `"RC8: MotorInterlock LOW"` — CH8 crossed LOW threshold
- `"PreArm: Motor Interlock Enabled"` — CH8 is HIGH at arm time (pre-arm check)

**Pre-arm check for Motor Interlock Enabled:** fires when CH8 is HIGH and the check is
enabled. With `ARMING_CHECK=0` or `force=True`, this check is bypassed.

---

## Working SITL Arm Sequence

**Confirmed working** (tested `test_arm_minimal.py`, ArduPilot 4.7+):

```python
# Parameters that must be set before arming:
params = {
    "ARMING_SKIPCHK": 0xFFFF,  # skip ALL pre-arm checks (4.7+ renamed from ARMING_CHECK)
    "H_RSC_MODE":     1,        # CH8 passthrough — instant runup_complete
    "FS_THR_ENABLE":  0,        # no RC throttle failsafe
    "FS_GCS_ENABLE":  0,        # no GCS heartbeat failsafe
    "COMPASS_USE":    0,        # velocity-derived yaw only
}

# Sequence:
# 1. Set params above
# 2. Wait for ATTITUDE messages (EKF attitude aligned — gcs.wait_ekf_attitude())
# 3. Send CH8=2000 (motor interlock ON, RSC at setpoint for mode 1)
# 4. Send force arm (param2=21196)
# 5. HEARTBEAT shows armed=True immediately (mode 1 has instant runup_complete)
```

> **Note:** In ArduPilot 4.7+ the parameter was renamed from `ARMING_CHECK` to `ARMING_SKIPCHK`.
> Setting `ARMING_CHECK` silently fails — no ACK, no error. Always use `ARMING_SKIPCHK`.

---

## EKF Initialization Sequence (SITL JSON backend)

The EKF goes through these phases during SITL startup:

| Time | EKF_STATUS flags | Event |
|------|-----------------|-------|
| t=0 | 0x0400 (CONST_POS_MODE) | EKF initialising in constant-position mode |
| t+2s | 0x0400 | "EKF3 IMU0 switching to compass 1" — EKF resets for compass init |
| t+3s | 0x0000 (all zero) | Brief re-init flash — DO NOT ARM HERE |
| t+3s | 0x0000 | "EKF3 IMU0 tilt alignment complete" |
| t+3s | 0x0000 | "EKF3 IMU0 MAG1 initial yaw alignment complete" |
| t+3s | 0x00a7 | EKF has: attitude + velocity (horiz+vert) + vert position |
| t+17s* | 0x00a7 | "EKF3 IMU0 origin set" — GPS origin fixed (fires AFTER arm in some cases) |

`*` "origin set" fires ~14 s after first arm command was sent (GPS detection trigger).
**Do not wait for "origin set" before arming** — it may not fire until after arm.

`0x00a7` decodes as:
- `0x0001` EKF_ATTITUDE ✓
- `0x0002` EKF_VELOCITY_HORIZ ✓
- `0x0004` EKF_VELOCITY_VERT ✓
- `0x0020` EKF_POS_VERT_ABS ✓
- `0x0080` EKF_PRED_POS_HORIZ_REL ✓
- (no EKF_POS_HORIZ_REL/ABS — normal with COMPASS_USE=0 + no GPS lock)

**Safe to arm** when EKF_STATUS ≥ 0x00a7 (or any non-zero, non-0x0400).

---

## COMPASS_USE=0 Effects

With `COMPASS_USE=0`:
- EKF derives yaw from velocity heading (needs non-zero velocity)
- During startup freeze (hub stationary), vel_ned ≈ zero → EKF can't derive yaw
- EKF_STATUS stays at 0x00a7 (no horizontal position), never reaches full 0x00ff
- `LOCAL_POSITION_NED` does NOT appear during freeze (needs horizontal position lock)
- This is normal and expected — force arm with `ARMING_CHECK=0` bypasses the EKF position check

**Important:** Sending real hub velocity (not zeros) in the sensor packet allows the EKF to
derive yaw via velocity heading once the hub starts moving. This improves long-term EKF health
but is not required for initial arming.

---

## Common Failure Modes

| Symptom | Cause | Fix |
|---------|-------|-----|
| "PreArm: Motors: H_RSC_MODE invalid" | H_RSC_MODE=0 (SITL default) | Set H_RSC_MODE=1 |
| COMMAND_ACK ACCEPTED but HEARTBEAT never armed | RSC not at runup_complete | Use H_RSC_MODE=1 + CH8=2000 |
| "PreArm: Motor Interlock Enabled" | CH8=2000 at arm time with checks enabled | Set ARMING_CHECK=0 |
| EKF_STATUS stuck at 0x0400 | EKF in constant-position mode, GPS not locked | Normal during freeze; arm with ARMING_CHECK=0 |
| EKF_STATUS flashes 0x0000 | EKF reinitialising after compass switch | Wait for 0x0000 to pass before arming |
| Vehicle arms but no servo outputs on CH1-3 | Not in a flying mode | Set ACRO mode after arm |

---

## Parameter Reference

| Parameter | Recommended value | Reason |
|-----------|------------------|--------|
| ARMING_SKIPCHK | 0xFFFF | Skip all pre-arm checks for SITL (4.7+ name; old name ARMING_CHECK silently fails) |
| H_RSC_MODE | 1 | CH8 passthrough — instant runup_complete |
| COMPASS_USE | 0 | Velocity-derived yaw only (no hardware compass in SITL) |
| FS_THR_ENABLE | 0 | No RC throttle failsafe (SITL has no real RC) |
| FS_GCS_ENABLE | 0 | No GCS heartbeat failsafe |
| INITIAL_MODE | 1 | Boot into ACRO (takes effect on next boot, so also set mode explicitly) |
