# Running `set_target_angle_and_climbrate` in SITL

Notes for testing direct earth-frame attitude commands from Lua, bypassing the position controller. Useful for the tethered-heli case where the equilibrium attitude is steep and `AC_PosControl`'s small-angle assumptions degrade.

## The API

C++ entry point: [`Copter::set_target_angle_and_climbrate`](../../ardupilot/ArduCopter/Copter.cpp#L380) in `ArduCopter/Copter.cpp:380`.

Lua binding:

```lua
vehicle:set_target_angle_and_climbrate(
    roll_deg,        -- -180..180   earth-frame roll target
    pitch_deg,       -- -90..90     earth-frame pitch target
    yaw_deg,         -- -360..360   earth-frame yaw target (used if use_yaw_rate=false)
    climb_rate_ms,   -- m/s         earth-frame Z-up rate
    use_yaw_rate,    -- bool        true => use yaw_rate_degs, false => use yaw_deg
    yaw_rate_degs    -- deg/s       body-frame yaw rate
) -- returns true on success, false if not in Guided mode
```

Binding declaration: `libraries/AP_Scripting/generator/description/bindings.desc:332`.

### What it actually does

1. Vehicle must be in **GUIDED** (or AUTO with guided-override active). Otherwise returns false.
2. `roll_deg, pitch_deg, yaw_deg` are converted into a quaternion via `Quaternion::from_euler`.
3. The quaternion is fed to `ModeGuided::set_angle(q, ang_vel=0, climb_rate_cms, use_thrust=false)`.
4. The Guided angle path passes the quaternion to `AC_AttitudeControl::input_quaternion(...)` each tick (400 Hz on Copter).
5. Altitude is closed-loop via `climb_rate_ms` feeding the Z position controller. No horizontal position/velocity loop is active.

Key consequence: the attitude target is full-quaternion earth-frame. No `ANGLE_MAX` clip from `AC_PosControl`. **However**, `get_althold_lean_angle_max_cd()` (collective-margin-based lean limit) in the attitude controller can still clip — worth checking with heavy tilt.

## SITL setup (heli)

### Build

From `ardupilot/` repo root:

```bash
./waf configure --board sitl
./waf heli
```

### Launch

```bash
sim_vehicle.py -v ArduCopter -f heli --console --map
```

The heli frame implies `FRAME_CLASS=6` (heli) and the right default `AHRS_ORIENTATION`, swashplate setup, etc.

### Required parameters

Set these once and save (they survive across SITL restarts in `eeprom.bin` if you don't blow it away):

```
SCR_ENABLE       1           # enable Lua scripting (REBOOT after setting)
SCR_HEAP_SIZE    100000      # 100 KB heap; raise if scripts grow
SCR_VM_I_COUNT   200000      # instruction count between yields (default usually fine)
```

After enabling scripting, refresh params and reboot:

```
param set SCR_ENABLE 1
param set SCR_HEAP_SIZE 100000
reboot
```

For the steep-tilt case also bump:

```
ANGLE_MAX        6000        # 60° — adjust to expected operating tilt
ATC_ANG_LIM_TC   1.0         # softer thrust-based lean limiting; tune as needed
PILOT_SPEED_UP   500
PILOT_SPEED_DN   500
```

### Script location

SITL looks for scripts in `<sim_vehicle_dir>/scripts/`. With `sim_vehicle.py`, the working directory is typically `ardupilot/Tools/autotest/` unless you pass `--out-dir`. The simplest workflow:

```bash
mkdir -p scripts
cp my_script.lua scripts/
```

then start SITL from that directory, or use:

```
param set SCR_DIR_DISABLE 0
```

and check the boot console for the actual path scanned.

## Minimal test script

Save as `scripts/test_set_angle.lua`. Holds 30° pitch-forward and climbs at 0 m/s, with free yaw rate of 10 deg/s:

```lua
-- test_set_angle.lua
-- Commands a fixed earth-frame attitude in Guided mode.
-- Switch the vehicle to GUIDED before this will take effect.

local TARGET_ROLL_DEG  = 0
local TARGET_PITCH_DEG = 30
local CLIMB_RATE_MS    = 0
local USE_YAW_RATE     = true
local YAW_RATE_DEGS    = 10

local UPDATE_PERIOD_MS = 50   -- 20 Hz

function update()
    local ok = vehicle:set_target_angle_and_climbrate(
        TARGET_ROLL_DEG,
        TARGET_PITCH_DEG,
        0,                      -- yaw_deg ignored when use_yaw_rate=true
        CLIMB_RATE_MS,
        USE_YAW_RATE,
        YAW_RATE_DEGS)

    if not ok then
        gcs:send_text(6, "set_target_angle failed (not in GUIDED?)")
    end

    return update, UPDATE_PERIOD_MS
end

gcs:send_text(6, "test_set_angle.lua loaded")
return update()
```

## Test sequence in SITL

In the MAVProxy console (or via MAVLink command):

```
mode GUIDED
arm throttle
rc 3 1500            # collective mid-stick so attitude target has thrust
takeoff 5            # get airborne (optional but helps observe behavior)
```

Then enable the script by either dropping it into `scripts/` before boot, or:

```
script /full/path/to/test_set_angle.lua
```

(some versions support live `scripting/restart` MAVLink command — use `SCRIPTING_CMD` over MAVLink to restart without rebooting).

To see what the attitude controller actually receives:

```
graph ATTITUDE.roll ATTITUDE.pitch ATTITUDE.yaw
graph NAV_CONTROLLER_OUTPUT.nav_roll NAV_CONTROLLER_OUTPUT.nav_pitch
```

## Closing the outer loop in Lua (radial-velocity sketch)

For the tether use-case — moving at steady speed along the tether axis while letting the attitude controller handle the steep tilt:

```lua
-- tether_radial.lua (sketch)
-- Drag heli along radial direction at fixed earth-frame speed.

local ANCHOR_LAT = -35.36326   -- example: SITL home
local ANCHOR_LNG = 149.16523
local ANCHOR_ALT_M = 0          -- AMSL

local RADIAL_SPEED_MS = 0.5     -- positive = outbound, negative = inbound
local CLIMB_RATE_MS   = 0
local UPDATE_HZ       = 20

-- Earth-frame attitude that produces accel a in direction r̂ at steep tilt:
-- pitch_eq, roll_eq are computed from estimated tether-equilibrium geometry.
-- For first cut, set them to constants and add a small velocity-correction term.

local function compute_attitude_for_velocity(v_des_NE, current_vel_NE, yaw_rad)
    -- super basic: feedforward equilibrium + P term on velocity error,
    -- both in earth frame, then rotate into body roll/pitch.
    local kp = 5.0  -- deg per m/s error
    local vx_err = v_des_NE.x - current_vel_NE.x
    local vy_err = v_des_NE.y - current_vel_NE.y

    -- desired earth-frame tilt vector (deg): north tilt and east tilt
    local tilt_n = kp * vx_err
    local tilt_e = kp * vy_err

    -- rotate earth tilt into body roll/pitch via current yaw
    local c = math.cos(yaw_rad)
    local s = math.sin(yaw_rad)
    local pitch =  tilt_n * c + tilt_e * s
    local roll  = -tilt_n * s + tilt_e * c
    return roll, pitch
end

function update()
    local pos = ahrs:get_position()
    local vel = ahrs:get_velocity_NED()
    if pos == nil or vel == nil then
        return update, 1000 / UPDATE_HZ
    end

    local anchor = Location()
    anchor:lat(ANCHOR_LAT * 1e7)
    anchor:lng(ANCHOR_LNG * 1e7)
    anchor:alt(ANCHOR_ALT_M * 100)

    -- horizontal offset from anchor to heli, in NE metres
    local offset = anchor:get_distance_NE(pos)
    local r = math.sqrt(offset:x()^2 + offset:y()^2)
    if r < 0.1 then
        return update, 1000 / UPDATE_HZ
    end
    local rhat_n = offset:x() / r
    local rhat_e = offset:y() / r

    local v_des = { x = RADIAL_SPEED_MS * rhat_n,
                    y = RADIAL_SPEED_MS * rhat_e }
    local v_cur = { x = vel:x(), y = vel:y() }

    local yaw_rad = ahrs:get_yaw()
    local roll_deg, pitch_deg = compute_attitude_for_velocity(v_des, v_cur, yaw_rad)

    vehicle:set_target_angle_and_climbrate(
        roll_deg, pitch_deg, 0, CLIMB_RATE_MS, true, 0)

    return update, 1000 / UPDATE_HZ
end

return update()
```

This is a sketch — the real version needs:
- A proper equilibrium-attitude feedforward derived from tether geometry (heli pos minus anchor, plus tether vertical drop).
- I-term on velocity error for steady-state.
- Tilt magnitude limiting so a transient error can't command 90°.
- Safety: if `r > tether_length`, force inbound; if `r < r_min`, force zero or outbound.

## Things to verify in SITL before flying

1. **Script actually runs:** look for the `gcs:send_text` boot message in MAVProxy.
2. **Mode is GUIDED:** `vehicle:set_target_angle_and_climbrate` silently returns false otherwise.
3. **Attitude is reached:** graph `ATTITUDE.pitch` vs commanded pitch. Mismatch indicates `ATC_ANG_LIM_TC` or `get_althold_lean_angle_max_cd` is clipping.
4. **Climb-rate path works:** command `CLIMB_RATE_MS=0.5`, confirm vertical velocity tracks.
5. **Yaw-rate path works:** with `use_yaw_rate=true`, the heli should rotate at the commanded rate while holding roll/pitch in earth frame — this is the heading-decoupled behavior we want.
6. **Steep tilt:** push to 45°, 60°. Watch for attitude controller saturation messages, EKF tilt warnings, collective rail.

## Caveats specific to heli at steep tilt

- `get_althold_lean_angle_max_cd()` derives the max commanded lean from available collective margin above hover thrust. On heli this can be tight; tune `ATC_ANG_LIM_TC` upward to relax it.
- `motors->using_leaky_integrator()` (heli-specific): the swashplate I-term leaks. If your equilibrium needs sustained large cyclic, ensure leak rate doesn't fight you. Check `ATC_RAT_*_FLTE` and integrator behavior.
- `ANGLE_MAX` is not consulted in the angle-input Guided path (PSC is bypassed), but the attitude controller's own thrust-vector limit still applies.
- EKF tilt confidence drops past ~60°. Watch for `EKF3 IMU0 yaw aligned` and lane-switching events.

## References

- API: `ArduCopter/Copter.cpp:380` and `ArduCopter/Copter.h:684`
- Binding: `libraries/AP_Scripting/generator/description/bindings.desc:332`
- Docs stub: `libraries/AP_Scripting/docs/docs.lua:2674`
- Example: `libraries/AP_Scripting/examples/set-angle.lua`
- Real-world applet using it: `libraries/AP_Scripting/applets/copter-deadreckon-home.lua:346,386`
- Heli acro flow (context): `ArduCopter/mode_acro_heli.cpp`
