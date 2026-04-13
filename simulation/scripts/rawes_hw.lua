--[[
rawes.lua — Unified RAWES flight + yaw-trim controller
Works in both ArduPilot SITL (mcroomp fork) and on the Pixhawk 6C.

Mode is selected at runtime via SCR_USER6:
  0  none   — script loaded but both subsystems disabled
  1  flight — cyclic orbit-tracking only (Ch1, Ch2)       50 Hz
  2  yaw    — counter-torque yaw trim only (Ch9/SERVO9)   100 Hz
  3  both   — flight + yaw trim active simultaneously

Parameters (SCR_USER1..6):
  SCR_USER1   RAWES_KP_CYC    Cyclic P gain            [rad/s / rad]  default 1.0
  SCR_USER2   RAWES_BZ_SLEW   body_z slerp rate limit  [rad/s]        default 0.40
  SCR_USER3   RAWES_ANCHOR_N  Anchor North from EKF origin [m]         default 0.0
  SCR_USER4   RAWES_ANCHOR_E  Anchor East  from EKF origin [m]         default 0.0
  SCR_USER5   RAWES_ANCHOR_D  Anchor Down  from EKF origin [m]         default 0.0
  SCR_USER6   RAWES_MODE      Mode selector (0/1/2/3 -- see above)     default 0

Cyclic output rate limiter is hardcoded at 30 PWM/step (0.67 s full-stick traverse).

Yaw-trim subsystem constants (compile-time):
  GEAR_RATIO   80:44 spur gear between GB4008 pinion and rotor hub
  KV_RAD       66 KV converted to rad/s/V
  R_MOTOR      7.5 ohm phase resistance
  K_BEARING    0.005 N·m·s/rad bearing drag
  KP_YAW       0.001 rad/s -> throttle proportional yaw correction

RPM source:
  SITL: mediator sends motor_rpm via JSON "rpm.rpm_1" field; ArduPilot maps
        this to RPM:get_rpm(0).
  Hardware: DSHOT bidirectional ESC telemetry (RPM1_TYPE=5, AM32 firmware).
  The script tries both 'rpm' and 'RPM' library names for firmware compatibility.

Battery voltage:
  Read from onboard sensor (battery:voltage(0)).  Falls back to V_BAT_NOM if
  the sensor is not ready or reads below V_BAT_MIN (early boot / bench testing).

Required ArduPilot parameters:
  SCR_ENABLE      1   -- reboot required after first set
  RPM1_TYPE       5   -- DSHOT bidirectional (hardware) or 10 (SITL JSON)
  RPM1_MIN        0
  SERVO9_FUNCTION 94  -- Script 1: Lua writes motor command to Ch9 (AUX1)

Deployment:
  Copy rawes.lua to APM/scripts/ on the Pixhawk SD card.
  Set SCR_USER6 to activate the desired mode.
--]]

-- ── Shared constants ─────────────────────────────────────────────────────────

local BASE_PERIOD_MS    = 10        -- 100 Hz base tick (yaw trim native rate)
local FLIGHT_PERIOD_MS  = 20        -- 50 Hz  flight subsystem
local ACRO_MODE_NUM     = 1         -- ACRO mode number (ArduCopter ACRO = 1)

-- ── Flight subsystem constants ───────────────────────────────────────────────

local MIN_TETHER_M      = 0.5       -- minimum tether length before orbit tracking activates
local ACRO_RP_RATE_DEG  = 360.0     -- must match ACRO_RP_RATE ArduPilot parameter
local PLANNER_TIMEOUT   = 2000      -- ms: revert to natural orbit after this

-- ── VZ collective controller ─────────────────────────────────────────────────
-- Pixhawk-side descent-rate controller: mirrors AcroController.step_vz() in
-- controller.py.  Runs at 50 Hz inside run_flight().  Computes collective from
-- NED vz feedback and writes Ch3 RC override.
--
-- Ground station encodes vz_setpoint on Ch7 RC override:
--   PWM = 1500 + vz_setpoint_ms * CH7_VZ_SCALE
--   1500 = hold altitude (vz=0), 1600 = descend 0.5 m/s, 1400 = climb 0.5 m/s
-- Ch7 not set or = 0  ->  default vz_setpoint = 0 (hold altitude).

local KP_VZ          = 0.05    -- collective gain [rad/(m/s)]
local COL_CRUISE_RAD = 0.079   -- altitude-neutral feedforward [rad] at xi~80 deg
local COL_MIN_RAD    = -0.28   -- hardware collective floor  [rad]
local COL_MAX_RAD    =  0.10   -- hardware collective ceiling [rad]
-- DS113MG at 6 V: 545 deg/s over 100 deg travel.
-- max_col_rate = 2 * 545/100 * swashplate_pitch_gain(0.10) = 1.09 rad/s
-- At 50 Hz (0.020 s/step): max_col_step = 1.09 * 0.020 = 0.022 rad/step
local COL_SLEW_MAX   = 0.022   -- rad per 50 Hz step

local CH7_VZ_SCALE   = 200.0   -- PWM units per (m/s): 1600 PWM = 0.5 m/s descent

-- ── Yaw-trim subsystem constants ─────────────────────────────────────────────

local GEAR_RATIO    = 80.0 / 44.0
local KV_RAD        = 66.0 * math.pi / 30.0
local R_MOTOR       = 7.5
local K_BEARING     = 0.005
local KP_YAW        = 0.001
local MIN_RPM       = 50            -- RPM below this = no signal from ESC telemetry
local STARTUP_PWM   = 1050          -- ~5% throttle: arms DShot ESC and primes telemetry
local V_BAT_NOM     = 15.2          -- nominal 4S LiPo voltage
local V_BAT_MIN     = 10.0          -- below this = battery sensor not ready; use V_BAT_NOM
local YAW_SRV_FUNC  = 94            -- ArduPilot servo function for Script 1 output (SERVO9)

-- RPM library binding.  SITL (mcroomp fork) exposes 'RPM' (uppercase).
-- Some hardware firmware builds use 'rpm' (lowercase) instead.
-- Try RPM first; fall back to rpm if RPM is nil.
local _rpm_lib = RPM or rpm

-- ── Shared state ─────────────────────────────────────────────────────────────

local _diag             = 0         -- global diagnostic counter (every tick)
local _last_flight_ms   = 0         -- millis() when flight subsystem last ran

-- Cache RC channel objects at module load (rc:get_channel() is the correct API)
local _rc_ch1 = rc:get_channel(1)   -- roll rate override (flight)
local _rc_ch2 = rc:get_channel(2)   -- pitch rate override (flight)
local _rc_ch3 = rc:get_channel(3)   -- collective override (vz controller)
local _rc_ch7 = rc:get_channel(7)   -- vz_setpoint input from ground station

-- ── Flight subsystem state ───────────────────────────────────────────────────

local _last_col_rad = COL_CRUISE_RAD  -- collective slew state [rad]
local _captured     = false           -- true once equilibrium has been snapped
local _bz_eq0       = nil             -- body_z_ned at equilibrium capture
local _tdir0        = nil             -- tether direction at equilibrium capture
local _bz_orbit     = nil             -- instantaneous orbit-tracked body_z setpoint
local _bz_slerp     = nil             -- rate-limited active setpoint (what cyclic tracks)
local _bz_target    = nil             -- planner override (nil = natural orbit)
local _plan_ms      = 0               -- millis() when _bz_target was last commanded
local _prev_ch1     = 1500            -- last sent Ch1 PWM (for output rate limiting)
local _prev_ch2     = 1500            -- last sent Ch2 PWM (for output rate limiting)

-- ── Helpers ──────────────────────────────────────────────────────────────────

-- Deep copy of a Vector3f
local function v3_copy(v)
    local r = Vector3f()
    r:x(v:x()); r:y(v:y()); r:z(v:z())
    return r
end

-- Return a new normalized copy of v (Vector3f:normalize() is in-place only)
local function v3_normalize(v)
    local r = v3_copy(v)
    r:normalize()
    return r
end

-- Unit body-Z vector [0, 0, 1] (Vector3f() constructor ignores args in this build)
local function v3_body_z()
    local v = Vector3f()
    v:z(1.0)
    return v
end

-- Rodrigues rotation: rotate vector v around unit vector axis_n by angle radians
-- v' = v·cos(th) + (axis×v)·sin(th) + axis·(axis·v)·(1-cos(th))
-- Uses only component arithmetic to avoid Vector3f operator overloading issues.
local function rodrigues(v, axis_n, angle)
    local ca  = math.cos(angle)
    local sa  = math.sin(angle)
    local vx, vy, vz = v:x(),      v:y(),      v:z()
    local nx, ny, nz = axis_n:x(), axis_n:y(), axis_n:z()
    -- axis x v
    local acx = ny*vz - nz*vy
    local acy = nz*vx - nx*vz
    local acz = nx*vy - ny*vx
    -- axis . v
    local ad  = nx*vx + ny*vy + nz*vz
    local k   = ad * (1.0 - ca)
    local r   = Vector3f()
    r:x(vx*ca + acx*sa + nx*k)
    r:y(vy*ca + acy*sa + ny*k)
    r:z(vz*ca + acz*sa + nz*k)
    return r
end

-- Read SCR_USER param with fallback default
local function p(name, default)
    local v = param:get(name)
    if v == nil then return default end
    return v
end

-- Build anchor NED vector from SCR_USER3..5
local function anchor_ned()
    local a = Vector3f()
    a:x(p("SCR_USER3", 0.0))   -- North
    a:y(p("SCR_USER4", 0.0))   -- East
    a:z(p("SCR_USER5", 0.0))   -- Down
    return a
end

-- ── Yaw-trim subsystem ───────────────────────────────────────────────────────

local function compute_trim(motor_rpm, v_bat)
    local omega_m    = motor_rpm * math.pi / 30.0
    local omega_a    = omega_m / GEAR_RATIO
    local omega_0    = KV_RAD * v_bat
    local tau_stall  = v_bat / (KV_RAD * R_MOTOR)
    local tau_needed = K_BEARING * omega_a / GEAR_RATIO
    return math.max(0.0, math.min(1.0, tau_needed / tau_stall + omega_m / omega_0))
end

local function run_yaw_trim()
    if not _rpm_lib then
        if _diag % 500 == 1 then
            gcs:send_text(3, "RAWES yaw: rpm library not available on this firmware")
        end
        return
    end
    local motor_rpm = _rpm_lib:get_rpm(0)

    -- Battery voltage from onboard sensor; fall back to nominal if not ready
    -- or if the battery binding is not available in this firmware build.
    local v_bat = V_BAT_NOM
    pcall(function()
        local raw = battery:voltage(0)
        if raw and raw >= V_BAT_MIN then v_bat = raw end
    end)

    if _diag % 500 == 1 then
        gcs:send_text(6, string.format(
            "RAWES yaw: rpm=%.0f  v_bat=%.1f V  diag=%d",
            motor_rpm or -1, v_bat, _diag))
    end

    -- Cold-start: output a small startup throttle so the DShot ESC arms and
    -- begins returning telemetry.  Once RPM exceeds MIN_RPM the trim loop takes
    -- over.  No output if disarmed (safe bench default).
    if not motor_rpm or motor_rpm < MIN_RPM then
        if arming:is_armed() then
            SRV_Channels:set_output_pwm(YAW_SRV_FUNC, STARTUP_PWM)
        end
        return
    end

    local trim = compute_trim(motor_rpm, v_bat)

    local yaw_corr = 0.0
    local gyro = ahrs:get_gyro()
    if gyro then yaw_corr = -KP_YAW * gyro:z() end

    local throttle = math.max(0.0, math.min(1.0, trim + yaw_corr))
    local pwm_us   = math.floor(1000.0 + throttle * 1000.0 + 0.5)

    if _diag % 500 == 1 then
        gcs:send_text(6, string.format(
            "RAWES ACTIVE: trim=%.3f  thr=%.3f  pwm=%d",
            trim, throttle, pwm_us))
    end

    local ok, err = pcall(function()
        SRV_Channels:set_output_pwm(YAW_SRV_FUNC, pwm_us)
    end)
    if not ok and _diag % 500 == 1 then
        gcs:send_text(3, "RAWES: servo err: " .. tostring(err))
    end
end

-- ── Flight subsystem ─────────────────────────────────────────────────────────

local function run_flight()
    -- Only run in ACRO_Heli mode
    if vehicle:get_mode() ~= ACRO_MODE_NUM then return end

    if not ahrs:healthy() then return end

    -- Read position (NED, relative to EKF origin)
    local pos_ned = ahrs:get_relative_position_NED_origin()
    if not pos_ned then return end

    -- Tether vector: use component arithmetic
    local anch = anchor_ned()
    local diff = Vector3f()
    diff:x(pos_ned:x() - anch:x())
    diff:y(pos_ned:y() - anch:y())
    diff:z(pos_ned:z() - anch:z())
    local tlen = diff:length()

    -- Equilibrium capture
    if not _captured then
        if tlen >= MIN_TETHER_M then
            _bz_eq0     = ahrs:body_to_earth(v3_body_z())
            _tdir0      = v3_normalize(diff)
            _bz_orbit   = v3_copy(_bz_eq0)
            _bz_slerp   = v3_copy(_bz_eq0)
            _captured   = true
            gcs:send_text(6, string.format(
                "RAWES flight: captured  tlen=%.1f m  bz=(%.2f %.2f %.2f)",
                tlen, _bz_eq0:x(), _bz_eq0:y(), _bz_eq0:z()))
        end
        return
    end

    -- Orbit tracking (all vectors in NED)
    -- Rotate _bz_eq0 by the same rotation that the tether has made since capture.
    if tlen >= MIN_TETHER_M then
        local bzt   = v3_normalize(diff)
        local axis  = _tdir0:cross(bzt)
        local sinth = axis:length()
        if sinth > 1e-6 then
            local costh = _tdir0:dot(bzt)
            _bz_orbit = rodrigues(_bz_eq0, v3_normalize(axis), math.atan(sinth, costh))
        end
    end

    -- Planner timeout
    if _bz_target and (millis() - _plan_ms) > PLANNER_TIMEOUT then
        _bz_target = nil
        gcs:send_text(6, "RAWES flight: planner timeout -- reverting to orbit")
    end

    -- Rate-limited slerp toward goal
    local bz_slew = p("SCR_USER2", 0.40)
    local dt      = FLIGHT_PERIOD_MS * 0.001
    local goal    = _bz_target or _bz_orbit

    local dot    = math.max(-1.0, math.min(1.0, _bz_slerp:dot(goal)))
    local remain = math.acos(dot)
    if remain > 1e-4 then
        local ax = _bz_slerp:cross(goal)
        if ax:length() > 1e-6 then
            local step = math.min(bz_slew * dt, remain)
            _bz_slerp = rodrigues(_bz_slerp, v3_normalize(ax), step)
        end
    end

    -- Cyclic P loop
    -- error = cross(body_z_now, _bz_slerp) in NED frame, transformed to body frame
    local kp      = p("SCR_USER1", 1.0)
    local bz_now  = ahrs:body_to_earth(v3_body_z())
    local err_ned = bz_now:cross(_bz_slerp)

    local err_body = ahrs:earth_to_body(err_ned)
    local err_bx   = err_body:x()    -- body X (roll)
    local err_by   = err_body:y()    -- body Y (pitch)

    local roll_rads  = kp * err_bx
    local pitch_rads = kp * err_by

    -- Map rate to RC PWM; full stick (+/-500 us) = +/-ACRO_RP_RATE_DEG deg/s
    local scale = 500.0 / (ACRO_RP_RATE_DEG * math.pi / 180.0)

    local ch1 = math.floor(1500.0 + scale * roll_rads  + 0.5)
    local ch2 = math.floor(1500.0 + scale * pitch_rads + 0.5)
    ch1 = math.max(1000, math.min(2000, ch1))
    ch2 = math.max(1000, math.min(2000, ch2))

    -- Output rate limiter (30 PWM/step = ~0.67 s full-stick traverse)
    local d1 = ch1 - _prev_ch1
    local d2 = ch2 - _prev_ch2
    if d1 >  30 then d1 =  30 end
    if d1 < -30 then d1 = -30 end
    if d2 >  30 then d2 =  30 end
    if d2 < -30 then d2 = -30 end
    ch1 = _prev_ch1 + d1
    ch2 = _prev_ch2 + d2
    _prev_ch1 = ch1
    _prev_ch2 = ch2

    if _rc_ch1 then _rc_ch1:set_override(ch1) end
    if _rc_ch2 then _rc_ch2:set_override(ch2) end

    -- ── VZ collective controller ──────────────────────────────────────────
    -- Read vz_setpoint from Ch7 (ground station encodes: PWM = 1500 + vz * 200).
    -- Default to 0 (hold altitude) if Ch7 is not set or returns nil.
    local vz_sp = 0.0
    if _rc_ch7 then
        local ch7_pwm = _rc_ch7:get_pwm()
        if ch7_pwm and ch7_pwm > 0 then
            vz_sp = (ch7_pwm - 1500) / CH7_VZ_SCALE
        end
    end

    -- Read NED vertical velocity from EKF (z positive = downward in NED).
    local vz_actual = 0.0
    local vel_ned = ahrs:get_velocity_NED()
    if vel_ned then vz_actual = vel_ned:z() end

    -- P controller: positive error = descending too fast = increase collective.
    local vz_error = vz_actual - vz_sp
    local col_cmd  = COL_CRUISE_RAD + KP_VZ * vz_error
    if col_cmd < COL_MIN_RAD then col_cmd = COL_MIN_RAD end
    if col_cmd > COL_MAX_RAD then col_cmd = COL_MAX_RAD end

    -- Slew-limit collective change per step.
    local col_delta = col_cmd - _last_col_rad
    if col_delta >  COL_SLEW_MAX then col_delta =  COL_SLEW_MAX end
    if col_delta < -COL_SLEW_MAX then col_delta = -COL_SLEW_MAX end
    _last_col_rad = _last_col_rad + col_delta

    -- Normalise collective_rad -> Ch3 PWM (mirrors planner thrust encoding).
    -- thrust = (col - col_min) / (col_max - col_min)  ->  PWM = 1000 + thrust * 1000
    local col_thrust = (_last_col_rad - COL_MIN_RAD) / (COL_MAX_RAD - COL_MIN_RAD)
    if col_thrust < 0.0 then col_thrust = 0.0 end
    if col_thrust > 1.0 then col_thrust = 1.0 end
    local ch3 = math.floor(1000.0 + col_thrust * 1000.0 + 0.5)

    if _rc_ch3 then _rc_ch3:set_override(ch3) end

    if _diag % 250 == 1 then     -- every ~5 s at 50 Hz
        local err_mag = math.sqrt(err_bx * err_bx + err_by * err_by)
        gcs:send_text(6, string.format(
            "RAWES: ch1=%d ch2=%d ch3=%d |err|=%.3f rad  vz=%.2f/%.2f col=%.3f rad",
            ch1, ch2, ch3, err_mag, vz_actual, vz_sp, _last_col_rad))
    end
end

-- ── Main update ──────────────────────────────────────────────────────────────

local function update()
    _diag = _diag + 1

    local mode = math.floor(p("SCR_USER6", 0) + 0.5)

    -- mode 0: both subsystems disabled
    if mode == 0 then return update, BASE_PERIOD_MS end

    -- Yaw-trim subsystem: runs every tick (100 Hz) when mode 2 or 3
    if mode == 2 or mode == 3 then
        run_yaw_trim()
    end

    -- Flight subsystem: runs at 50 Hz sub-step when mode 1 or 3
    if mode == 1 or mode == 3 then
        local now = millis()
        if now - _last_flight_ms >= FLIGHT_PERIOD_MS then
            _last_flight_ms = now
            run_flight()
        end
    end

    return update, BASE_PERIOD_MS
end

-- ── Entry point ──────────────────────────────────────────────────────────────

local _mode_init  = math.floor(p("SCR_USER6", 0) + 0.5)
local _mode_names = {[0]="none", [1]="flight", [2]="yaw", [3]="both"}
local _mode_str   = _mode_names[_mode_init] or "unknown"

gcs:send_text(6, string.format(
    "RAWES: loaded  mode=%d (%s)  kp=%.2f  slew=%.2f  anchor=(%.1f %.1f %.1f)",
    _mode_init, _mode_str,
    p("SCR_USER1", 1.0), p("SCR_USER2", 0.40),
    p("SCR_USER3", 0.0), p("SCR_USER4", 0.0), p("SCR_USER5", 0.0)))

return update, BASE_PERIOD_MS
