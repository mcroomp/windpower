--[[
rawes.lua -- Unified RAWES flight controller
Works in both ArduPilot SITL (mcroomp fork) and on the Pixhawk 6C.

Mode is selected at runtime via SCR_USER6 (plain integer 0,1,2,5):
  0  none        -- script passive: no RC overrides; logs every 5 s + any NV message
  1  steady      -- cyclic altitude hold (Ch1, Ch2) + VZ collective PI             50 Hz
  2  yaw         -- manual Lua yaw PID only; holds yaw rate = 0 via SERVO4         100 Hz
  4  landing     -- (reserved, not yet implemented)
  5  pumping     -- De Schutter pumping cycle                                       50 Hz

Cyclic control (steady + pumping): AltitudeHoldController.
  Compute body_z_eq = bz_altitude_hold(pos, el_rad, tension_n):
    points the disk at (el_rad, current_azimuth) with a gravity-compensation tilt
    so thrust counteracts the elevation-lowering component of gravity.
  Elevation angle el_rad is rate-limited toward asin(target_alt / tlen).
  Before GPS fuses: gyro feedthrough (zero corrective cyclic) to preserve orbital rate.

Ground planner signals via NAMED_VALUE_FLOAT:
  RAWES_SUB (mode=5): pumping substate 0=hold 1=reel_out 2=transition 3=reel_in 4=transition_back
  RAWES_ALT: target altitude [m] above anchor; Lua rate-limits elevation toward it
  RAWES_TEN: tether tension estimate [N]; used for gravity compensation
  RAWES_ARM: arm vehicle with timed disarm countdown (value = ms until disarm)

Parameters:
  SCR_USER1   RAWES_KP_CYC    Cyclic P gain           [rad/s / rad]  default 1.0
  SCR_USER2   RAWES_BZ_SLEW   elevation slew rate     [rad/s]        default 0.40
  SCR_USER3   RAWES_ANCHOR_N  Anchor North from EKF origin [m]        default 0.0
  SCR_USER4   RAWES_ANCHOR_E  Anchor East  from EKF origin [m]        default 0.0
  SCR_USER5   RAWES_ANCHOR_D  Anchor Down  from EKF origin [m]        default 0.0
  SCR_USER6   RAWES_MODE      Mode selector (0,1,5)                   default 0
--]]

-- ── Constants ─────────────────────────────────────────────────────────────────

local BASE_PERIOD_MS    = 10        -- 100 Hz base tick
local FLIGHT_PERIOD_MS  = 20        -- 50 Hz flight subsystem
local ACRO_MODE_NUM     = 1         -- ArduCopter ACRO = 1

local _NVF_MSG_ID = 251
-- mavlink:init(queue_size, num_msgs).  queue_size = max messages buffered
-- between Lua ticks; with queue=1 (the old default) back-to-back NVFs from
-- the ground get dropped because only the first survives until update()
-- drains it.  20 is plenty for our ~5 named-floats / tick max rate.
mavlink.init(20, 10)
mavlink.register_rx_msgid(_NVF_MSG_ID)

local _nv_floats = {}

-- ── Mode numbers ──────────────────────────────────────────────────────────────

local MODE_NONE    = 0
local MODE_STEADY  = 1
local MODE_YAW     = 2   -- manual Lua yaw PID; holds yaw rate = 0 via SERVO4 direct
local MODE_PASSIVE = 3   -- armed but no commands: keep ch8 high, do nothing else.
                         -- Used by stack tests during kinematic so the rate-PID
                         -- has no setpoint to wind up against before release.
local MODE_LANDING = 4   -- reserved; not implemented here
local MODE_PUMPING = 5

-- ── Pumping substates (mode=5; delivered via RAWES_SUB) ──────────────────────

local PUMP_HOLD            = 0
local PUMP_REEL_OUT        = 1
local PUMP_TRANSITION      = 2
local PUMP_REEL_IN         = 3
local PUMP_TRANSITION_BACK = 4

-- ── Physical constants ────────────────────────────────────────────────────────

local MASS_KG  = 5.0
local G_ACCEL  = 9.81

-- ── RC / attitude ─────────────────────────────────────────────────────────────

local ACRO_RP_RATE_DEG = 360.0
local MIN_TETHER_M     = 0.5        -- minimum tether length before GPS init activates

-- ── Collective limits and cruise values ───────────────────────────────────────

local COL_MIN_RAD           = -0.28
local COL_MAX_RAD           =  0.10
local COL_SLEW_MAX          =  0.022   -- rad per 50 Hz step
local COL_CRUISE_FLIGHT_RAD = -0.18    -- VZ integrator initial value (xi~8 deg, altitude hold)

-- ── Pumping collective constants ──────────────────────────────────────────────

local COL_REEL_OUT      = -0.20   -- TensionPI warm-start collective [rad]
local T_PUMP_TRANSITION =  3.7    -- ramp window used for logging/diagnostics [s]

-- TensionPI for pumping collective.
-- Setpoint and measured tension come from ground via RAWES_TSP / RAWES_TEN
-- at ~10 Hz (UnifiedGroundController).  Mirrors Python TensionApController defaults.
local KP_TEN      = 2e-4    -- rad/N        (10 Hz-tuned proportional gain)
local KI_TEN      = 1e-3    -- rad/(N·s)    (10 Hz-tuned integral gain)
local KD_TEN      = 0.0     -- rad·s/N      (derivative gain; try ~2e-5 to damp tension oscillations)
local COL_MAX_TEN = 0.10    -- TensionPI collective ceiling [rad]
local DT_CMD      = 0.1     -- command period [s] used for integrator/derivative step

-- ── VZ collective controller (steady mode) ───────────────────────────────────

local KP_VZ = 0.05
local KI_VZ = 0.005

-- ── Shared state ─────────────────────────────────────────────────────────────

local _diag           = 0
local _last_flight_ms = 0
local _none_status_ms = 0

-- RAWES_ARM state machine
local _armon_deadline_ms = nil
local _armon_state       = nil
local _armon_armed_sent  = false
local _armon_secs        = 0

-- Mode / substate tracking
local _prev_mode  = -1
local _prev_sub   = 0
local _mode_ms    = 0
local _submode_ms = 0

-- Cached RC channel objects
local _rc_ch1 = rc:get_channel(1)
local _rc_ch2 = rc:get_channel(2)
local _rc_ch3 = rc:get_channel(3)
local _rc_ch4 = rc:get_channel(4)
local _rc_ch8 = rc:get_channel(8)

-- Cyclic output rate-limiting state
local _prev_ch1 = 1500
local _prev_ch2 = 1500

-- Collective state
local _last_col_rad = COL_CRUISE_FLIGHT_RAD
local _col_i        = COL_CRUISE_FLIGHT_RAD

-- TensionPID state for pumping mode.
-- Setpoint arrives from ground via RAWES_TSP; measured via RAWES_TEN.
-- Integrator warm-starts so initial output = COL_REEL_OUT at zero error.
local _col_i_ten       = COL_REEL_OUT / math.max(KI_TEN, 1e-12)
local _col_d_prev_err  = 0.0         -- previous error for derivative term
local _col_held        = COL_REEL_OUT   -- collective held between 10 Hz commands
local _ten_setpoint = 300.0          -- tension setpoint [N]; updated from RAWES_TSP
local _ten_sp_fresh = false          -- true when a new RAWES_TEN_SP has arrived
local _vib_corr_last   = 0.0         -- last vibration damper correction [rad]

-- Accelerometer-based tether spring-mode vibration damper.
-- Passes the 1.5-10 Hz resonance band (above altitude controller, below Nyquist),
-- estimates oscillatory hub velocity, opposes it via collective.
-- Works at 400 Hz with on-board IMU only -- no ground comms dependency.
local K_VIB        = 0.008   -- rad / (m/s) velocity feedback gain
local VIB_HP_TAU   = 1.0 / (2.0 * math.pi * 1.5)  -- 1/(2*pi*1.5 Hz)
local VIB_VEL_TAU  = 0.5     -- leaky integrator time constant [s]
local VIB_COL_MAX  = 0.04    -- max collective correction magnitude [rad]
local _vib_acc_hp   = 0.0    -- HP filter state
local _vib_acc_prev = 0.0    -- previous raw accel for HP derivative
local _vib_vel_est  = 0.0    -- estimated oscillatory velocity [m/s]

local function vib_damper_step(accel_z, dt_s)
    -- First-order high-pass: y[n] = alpha*(y[n-1] + x[n] - x[n-1])
    local alpha = VIB_HP_TAU / (VIB_HP_TAU + dt_s)
    _vib_acc_hp   = alpha * (_vib_acc_hp + accel_z - _vib_acc_prev)
    _vib_acc_prev = accel_z
    -- Leaky integrator: v[n] = exp(-dt/tau)*v[n-1] + dt*a_hp[n]
    local leak = math.exp(-dt_s / VIB_VEL_TAU)
    _vib_vel_est  = leak * _vib_vel_est + dt_s * _vib_acc_hp
    local corr = -K_VIB * _vib_vel_est
    if corr >  VIB_COL_MAX then corr =  VIB_COL_MAX end
    if corr < -VIB_COL_MAX then corr = -VIB_COL_MAX end
    return corr
end

-- Altitude hold state
local _el_initialized = false   -- true once first GPS fix with tlen >= MIN_TETHER_M
local _el_rad         = 0.0     -- current rate-limited elevation angle [rad]
local _target_alt     = 0.0     -- target altitude [m]; updated from RAWES_ALT
local _tension_n      = 200.0   -- tether tension estimate [N]; updated from RAWES_TEN

-- Trim cyclic feedforward [rad] -- ground sends via RAWES_TLN / RAWES_TLT
-- (NAMED_VALUE_FLOAT, max 10-char name).  These cancel the wind-driven
-- baseline hub moment at the IC operating point so the rate PID doesn't
-- have to fight it after kinematic_exit.  Computed by aero.solve_trim_cyclic
-- in the test fixture.  Default 0 = no feedforward.
--
-- The trim values are body-frame at the moment the ground sent them.
-- _yaw_since_trim integrates gyro:z() since the most recent trim update so
-- the body-frame command can be rotated to keep the world-frame disk-tilt
-- direction constant as the body yaws -- making the cyclic effectively
-- "aligned with world-down" regardless of which way the hub is pointing.
local _trim_lon       = 0.0
local _trim_lat       = 0.0
local _yaw_since_trim = 0.0     -- gyro:z() integrated since last RAWES_TLN/TLT [rad]

-- IC collective [rad] — ground sends via RAWES_COL.  Used by MODE_PASSIVE
-- to pin ch3 at the IC value so omega_spin doesn't droop while the body
-- is kinematically locked.  Defaults to the cruise-flight value.
local _ic_col         = COL_CRUISE_FLIGHT_RAD

-- ── Helpers ───────────────────────────────────────────────────────────────────

local function v3_copy(v)
    local r = Vector3f()
    r:x(v:x()); r:y(v:y()); r:z(v:z())
    return r
end

local function v3_body_z()
    local v = Vector3f(); v:z(1.0); return v
end

-- body_z in NED: R[:,2] = rotor axis in world frame
local function disk_normal_ned()
    return ahrs:body_to_earth(v3_body_z())
end

-- Lateral-velocity damping gain [N*s/m] for damp_bz_eq_lateral.  Same value
-- as the Python TensionApController(kd_lat=...) used in test_create_ic
-- warmup.  Damps the tether pendulum mode so steady-flight oscillation
-- doesn't grow over the minute timescale.
local KD_LAT = 50.0

-- Compute body_z_eq for altitude-holding flight.
-- Points the disk at (el_rad, current_azimuth) and adds a gravity-compensation
-- tilt so thrust has an upward-elevation component equal to mass*g*cos(el).
-- Mirrors Python compute_bz_altitude_hold exactly.
-- pos: Vector3f NED relative to anchor; el_rad: target elevation [rad]
local function bz_altitude_hold(pos, el_rad, tension_n)
    local az     = math.atan(pos:y(), pos:x())
    local cos_el = math.cos(el_rad)
    local sin_el = math.sin(el_rad)
    local cos_az = math.cos(az)
    local sin_az = math.sin(az)
    -- FRD: body_z points DOWN through the disk = hub→anchor in tethered hover.
    -- tdir = hub→anchor direction at (el, az); e_dn = elevation-downward tangent.
    local tx, ty, tz = -cos_el * cos_az, -cos_el * sin_az,  sin_el
    local ex, ey, ez =  sin_el * cos_az,  sin_el * sin_az,  cos_el
    -- Gravity compensation: k = mass*g*cos(el) / tension
    local k = MASS_KG * G_ACCEL * cos_el / math.max(tension_n, 1.0)
    local rx, ry, rz = tx + k * ex, ty + k * ey, tz + k * ez
    local rn = math.sqrt(rx*rx + ry*ry + rz*rz)
    if rn < 1e-6 then rn = 1.0 end
    local r = Vector3f()
    r:x(rx / rn); r:y(ry / rn); r:z(rz / rn)
    return r
end

-- Add lateral-velocity damping to a body_z target.  Mirrors Python
-- controller.damp_bz_eq_lateral.  Tilts bz toward the hub's lateral
-- velocity (component perpendicular to the tether) by kd_lat*v_lat/T,
-- so thrust = -T*bz gains a -kd_lat*v_lat component (viscous damping).
-- Anchor at origin (NED).  bz_eq, pos, vel : Vector3f.  Returns Vector3f.
local function damp_bz_eq_lateral(bz_eq, pos, vel, tension_n, kd_lat)
    local tlen = pos:length()
    if tlen < 0.1 then return bz_eq end
    local tx, ty, tz = pos:x() / tlen, pos:y() / tlen, pos:z() / tlen
    local v_along    = vel:x() * tx + vel:y() * ty + vel:z() * tz
    local vlx        = vel:x() - v_along * tx
    local vly        = vel:y() - v_along * ty
    local vlz        = vel:z() - v_along * tz
    local k          = kd_lat / math.max(tension_n, 1.0)
    local nx, ny, nz = bz_eq:x() + k * vlx, bz_eq:y() + k * vly, bz_eq:z() + k * vlz
    local n          = math.sqrt(nx * nx + ny * ny + nz * nz)
    if n < 1e-6 then return bz_eq end
    local r = Vector3f()
    r:x(nx / n); r:y(ny / n); r:z(nz / n)
    return r
end

local function p(name, default)
    local v = param:get(name)
    if v == nil then return default end
    return v
end

local function anchor_ned()
    local a = Vector3f()
    a:x(p("SCR_USER3", 0.0))
    a:y(p("SCR_USER4", 0.0))
    a:z(p("SCR_USER5", 0.0))
    return a
end

local function rate_to_pwm(rate_rads, acro_rp_rate_deg)
    acro_rp_rate_deg = acro_rp_rate_deg or ACRO_RP_RATE_DEG
    local scale = 500.0 / (acro_rp_rate_deg * math.pi / 180.0)
    local ch = math.floor(1500.0 + scale * rate_rads + 0.5)
    if ch < 1000 then ch = 1000 end
    if ch > 2000 then ch = 2000 end
    return ch
end

local function output_rate_limit(desired, prev, max_delta)
    if max_delta == 0 then return desired end
    local d = desired - prev
    if d >  max_delta then d =  max_delta end
    if d < -max_delta then d = -max_delta end
    return prev + d
end

-- Body-frame cyclic error: err_ned = bz_now x bz_target; projected to body frame.
local function cyclic_error_body(bz_now, bz_target_arg)
    local err_ned  = bz_now:cross(bz_target_arg)
    local err_body = ahrs:earth_to_body(err_ned)
    return {err_body:x(), err_body:y()}
end

-- Return the trim cyclic in the CURRENT body frame, rotated by
-- -_yaw_since_trim so the world-frame disk-tilt direction stays constant
-- as the body yaws.  The trim values arrive in the body frame at the
-- moment they were sent; _yaw_since_trim integrates gyro:z() since then.
local function effective_trim_body()
    local c = math.cos(-_yaw_since_trim)
    local s = math.sin(-_yaw_since_trim)
    return _trim_lon * c - _trim_lat * s,
           _trim_lon * s + _trim_lat * c
end

-- Compute the RC1/RC2/RC3 PWM overrides for holding the IC operating
-- point: cyclic = trim (rotated to current body yaw) expressed as the
-- ATC rate-PID setpoint bias that produces the desired cyclic angle at
-- steady state; collective = _ic_col mapped through the standard
-- collective_rad -> PWM formula.  Shared by MODE_PASSIVE and MODE_YAW.
local function set_trim_ic_rc_overrides()
    local _g_rll = math.max(p("ATC_RAT_RLL_P", 0.18) + p("ATC_RAT_RLL_FF", 0.0), 0.01)
    local _g_pit = math.max(p("ATC_RAT_PIT_P", 0.18) + p("ATC_RAT_PIT_FF", 0.0), 0.01)
    local tlon_b, tlat_b = effective_trim_body()
    -- HeliCyclicController sign mapping (controller.py): tilt_lat == roll_cyclic,
    -- tilt_lon == -pitch_cyclic.  Match that here.
    local _ch1 = rate_to_pwm( tlat_b / _g_rll)
    local _ch2 = rate_to_pwm(-tlon_b / _g_pit)
    if _rc_ch1 then _rc_ch1:set_override(_ch1) end
    if _rc_ch2 then _rc_ch2:set_override(_ch2) end

    local _col_thrust = (_ic_col - COL_MIN_RAD) / (COL_MAX_RAD - COL_MIN_RAD)
    if _col_thrust < 0.0 then _col_thrust = 0.0 end
    if _col_thrust > 1.0 then _col_thrust = 1.0 end
    local _ch3 = math.floor(1000.0 + _col_thrust * 1000.0 + 0.5)
    if _rc_ch3 then _rc_ch3:set_override(_ch3) end
end

-- ── MODE_YAW: manual Lua yaw PID ─────────────────────────────────────────────
-- Reads ATC_RAT_YAW_P/I/D/IMAX and H_YAW_TRIM each tick so pidtune takes effect
-- immediately.  Drives SERVO4 directly via set_output_pwm_chan_timeout, bypassing
-- ArduPilot's internal DDFP yaw mixer.
-- Sign convention: err = -gyro_z.  gyro_z > 0 = CW spin (NED right-hand z-down).
-- Positive error (CCW spin) → higher output → more GB4008 throttle → CW torque.

local _yaw_i      = 0.0
local _yaw_prev_e = 0.0

local SERVO4_CHAN    = 3     -- 0-indexed physical channel (servo 4 = index 3)

local function run_yaw_pid(dt)
    local gyro = ahrs:get_gyro()
    if not gyro then return end

    local yaw_rate = gyro:z()
    local err = -yaw_rate   -- setpoint = 0

    local kp        = p("ATC_RAT_YAW_P",    0.1)
    local ki        = p("ATC_RAT_YAW_I",    0.0)
    local kd        = p("ATC_RAT_YAW_D",    0.0)
    local imax      = p("ATC_RAT_YAW_IMAX", 0.7)
    local trim      = p("H_YAW_TRIM",       0.0)
    -- SERVO4_MIN/MAX are read each tick so setparam SERVO4_MAX <x> takes effect
    -- immediately -- useful as a tuning safety cap (drop SERVO4_MAX to limit motor
    -- authority during gain trials, raise it as confidence grows).
    local servo_min = p("SERVO4_MIN", 800)
    local servo_max = p("SERVO4_MAX", 2000)

    local p_out = kp * err
    local d_out = kd * (err - _yaw_prev_e) / dt
    _yaw_prev_e = err

    -- One-directional actuator (motor throttle >= 0): integrate when err > 0 (we
    -- have authority to counter the drift) and bleed off symmetrically when err < 0
    -- so a stuck I-term cannot persist across disturbances or runs.  Clamped to
    -- [0, imax].  Same |ki*err*dt| step in both directions -- the asymmetry is only
    -- in which sign of err winds up vs. bleeds.
    if err > 0 then
        _yaw_i = _yaw_i + ki * err * dt
        if _yaw_i > imax then _yaw_i = imax end
    else
        _yaw_i = _yaw_i - ki * (-err) * dt
        if _yaw_i < 0.0 then _yaw_i = 0.0 end
    end

    local output_unclamp = p_out + _yaw_i + d_out + trim
    local output = output_unclamp
    if output > 1.0 then output = 1.0 end
    if output < 0.0 then output = 0.0 end

    -- Stream internal PID state as NAMED_VALUE_FLOAT for offline tuning analysis.
    -- Higher rate ceiling than STATUSTEXT and structured (no string parsing).
    -- YAW_I       : integrator state (clamped to [0, imax])
    -- YAW_OUT     : pre-clamp output -- shows when PID wants to exceed [0,1]
    -- Clamped output is derivable from the SERVO_OUTPUT_RAW PWM stream.
    gcs:send_named_float("YAW_I",   _yaw_i)
    gcs:send_named_float("YAW_OUT", output_unclamp)

    local pwm = math.floor(servo_min + output * (servo_max - servo_min) + 0.5)
    -- Defensive clamp on top of the [0,1] clamp upstream -- guards against
    -- servo_max < servo_min (misconfig) and rounding drift past the bounds.
    if pwm > servo_max then pwm = servo_max end
    if pwm < servo_min then pwm = servo_min end
    SRV_Channels:set_output_pwm_chan_timeout(SERVO4_CHAN, pwm, 200)

    -- Hold cyclic at IC trim (rotated by -_yaw_since_trim so the world-
    -- frame disk-tilt direction stays constant regardless of body yaw) and
    -- collective at _ic_col.  When the trim NVFs are unset they default to
    -- 0 (neutral cyclic) / COL_CRUISE_FLIGHT_RAD, matching the prior
    -- neutral/cruise behaviour.
    set_trim_ic_rc_overrides()

    if _diag % 100 == 1 then
        gcs:send_text(6, string.format(
            "RAWES yaw: rate=%+.1fdeg/s  P=%+.3f  I=%+.3f  D=%+.3f  out=%.3f  pwm=%d",
            math.deg(yaw_rate), p_out, _yaw_i, d_out, output, pwm))
    end
end

-- ── Mode-entry reset ─────────────────────────────────────────────────────────

local function _on_mode_enter(mode)
    _nv_floats      = {}   -- clear NV inbox so stale substates cannot bleed through
    _none_status_ms = 0
    if mode == MODE_YAW then
        _yaw_i      = 0.0
        _yaw_prev_e = 0.0
    end
end

-- ── Flight subsystem ─────────────────────────────────────────────────────────

local function run_flight()
    if vehicle:get_mode() ~= ACRO_MODE_NUM then return end

    local mode_now    = _prev_mode
    local substate    = _prev_sub
    local dt          = FLIGHT_PERIOD_MS * 0.001
    local _is_pumping = mode_now == MODE_PUMPING

    -- ── Before GPS initialization: stabilize only ─────────────────────────
    -- Goal: keep the hub alive while EKF fuses (~30 s in kinematic, ~80 s total).
    -- Gyro feedthrough → desired_rate = measured_rate → ACRO rate_error = 0 →
    -- no corrective torque → natural orbital rate preserved.
    if not _el_initialized then
        local ct = (COL_CRUISE_FLIGHT_RAD - COL_MIN_RAD) / (COL_MAX_RAD - COL_MIN_RAD)
        if _rc_ch3 then _rc_ch3:set_override(math.floor(1000.0 + ct * 1000.0 + 0.5)) end

        if not ahrs:healthy() then
            if _rc_ch1 then _rc_ch1:set_override(1500) end
            if _rc_ch2 then _rc_ch2:set_override(1500) end
            return
        end

        local gyro = ahrs:get_gyro()
        if gyro then
            if _rc_ch1 then _rc_ch1:set_override(rate_to_pwm(gyro:x())) end
            if _rc_ch2 then _rc_ch2:set_override(rate_to_pwm(gyro:y())) end
        else
            if _rc_ch1 then _rc_ch1:set_override(1500) end
            if _rc_ch2 then _rc_ch2:set_override(1500) end
        end

        -- Check for GPS position; initialize altitude hold on first valid fix
        local pos_ned = ahrs:get_relative_position_NED_origin()
        if pos_ned then
            local anch = anchor_ned()
            local rx = pos_ned:x() - anch:x()
            local ry = pos_ned:y() - anch:y()
            local rz = pos_ned:z() - anch:z()
            local tlen = math.sqrt(rx*rx + ry*ry + rz*rz)
            if tlen >= MIN_TETHER_M then
                _el_rad       = math.asin(math.max(-1.0, math.min(1.0, -rz / math.max(tlen, 0.1))))
                _target_alt   = -rz
                _last_col_rad = COL_CRUISE_FLIGHT_RAD
                _col_i        = COL_CRUISE_FLIGHT_RAD
                _el_initialized = true
                local label = _is_pumping and "pump" or "steady"
                gcs:send_text(6, string.format(
                    "RAWES %s: captured  el=%.1f deg  alt=%.1f m  tlen=%.1f m",
                    label, math.deg(_el_rad), _target_alt, tlen))
            end
        end
        return
    end

    -- ── GPS initialized: altitude hold ────────────────────────────────────

    local pos_ned = ahrs:get_relative_position_NED_origin()
    if not pos_ned then return end

    local anch = anchor_ned()
    local rel  = Vector3f()
    rel:x(pos_ned:x() - anch:x())
    rel:y(pos_ned:y() - anch:y())
    rel:z(pos_ned:z() - anch:z())
    local tlen = rel:length()

    -- Rate-limit elevation toward target altitude
    local bz_slew   = p("SCR_USER2", 0.40)
    local target_el = math.asin(math.max(-1.0, math.min(1.0, _target_alt / math.max(tlen, 0.1))))
    local max_step  = bz_slew * dt
    local el_step   = target_el - _el_rad
    if el_step >  max_step then el_step =  max_step end
    if el_step < -max_step then el_step = -max_step end
    _el_rad = _el_rad + el_step

    -- Cyclic P loop — use setpoint tension (not measured) to keep gravity
    -- compensation stable when tether goes slack (_tension_n → 0).
    local ten_bz  = _is_pumping and _ten_setpoint or _tension_n
    local bz_goal = bz_altitude_hold(rel, _el_rad, ten_bz)
    -- Add lateral-velocity damping to suppress the tether pendulum mode.
    local vel_ned_d = ahrs:get_velocity_NED()
    if vel_ned_d then
        bz_goal = damp_bz_eq_lateral(bz_goal, rel, vel_ned_d, ten_bz, KD_LAT)
    end
    local bz_now  = disk_normal_ned()
    local err     = cyclic_error_body(bz_now, bz_goal)
    local err_bx  = err[1]
    local err_by  = err[2]
    local kp      = p("SCR_USER1", 1.0)

    -- Convert trim cyclic [rad] to ATC rate-setpoint bias [rad/s].
    -- ArduPilot's rate PID maps rate_error -> cyclic angle.  At steady
    -- state with body rate = rate target (zero error), the output is
    -- FF*rate_target.  When rate target ramps from zero, the output is
    -- approximately (P+FF)*rate_target until the I-term takes over.
    -- Use the (P+FF) sum so the steady-state cyclic = trim regardless of
    -- which term dominates.  Sign convention matches HeliCyclicController:
    --   tilt_lat ==  roll_cyclic  -> +bias on err_bx (roll)
    --   tilt_lon == -pitch_cyclic -> -bias on err_by (pitch)
    local _g_rll = math.max(p("ATC_RAT_RLL_P", 0.18) + p("ATC_RAT_RLL_FF", 0.0), 0.01)
    local _g_pit = math.max(p("ATC_RAT_PIT_P", 0.18) + p("ATC_RAT_PIT_FF", 0.0), 0.01)
    local _bias_rll =  _trim_lat / _g_rll
    local _bias_pit = -_trim_lon / _g_pit

    local ch1 = output_rate_limit(rate_to_pwm(kp * err_bx + _bias_rll), _prev_ch1, 100)
    local ch2 = output_rate_limit(rate_to_pwm(kp * err_by + _bias_pit), _prev_ch2, 100)
    _prev_ch1 = ch1
    _prev_ch2 = ch2
    if _rc_ch1 then _rc_ch1:set_override(ch1) end
    if _rc_ch2 then _rc_ch2:set_override(ch2) end

    -- ── Collective ────────────────────────────────────────────────────────
    local col_cmd
    if _is_pumping then
        -- TensionPI: setpoint from RAWES_TSP, measured from RAWES_TEN (both at ~10 Hz).
        -- Step integrator only on fresh command; hold collective between commands.
        if _ten_sp_fresh then
            _ten_sp_fresh = false
            local ten_err = _ten_setpoint - _tension_n
            local d_term  = KD_TEN * (ten_err - _col_d_prev_err) / DT_CMD
            local raw_pre = KP_TEN * ten_err + KI_TEN * _col_i_ten + d_term
            -- Conditional anti-windup (mirrors Python TensionPID)
            if not (raw_pre <= COL_MIN_RAD and ten_err < 0) then
                if not (raw_pre >= COL_MAX_TEN and ten_err > 0) then
                    _col_i_ten = _col_i_ten + ten_err * DT_CMD
                end
            end
            _col_d_prev_err = ten_err
            local out = KP_TEN * ten_err + KI_TEN * _col_i_ten + d_term
            if out > COL_MAX_TEN then out = COL_MAX_TEN end
            if out < COL_MIN_RAD then out = COL_MIN_RAD end
            _col_held = out
        end
        col_cmd = _col_held
        -- Vibration damper: body-Z accel → HP filter → velocity estimate → collective
        local imu_a = ahrs:get_accel()
        if imu_a then
            _vib_corr_last = vib_damper_step(imu_a:z(), dt)
            col_cmd = col_cmd + _vib_corr_last
        end
    else
        -- Steady mode: VZ PI altitude hold (vz setpoint = 0)
        local vz_actual = 0.0
        local vel_ned = ahrs:get_velocity_NED()
        if vel_ned then vz_actual = vel_ned:z() end
        local vz_error = vz_actual
        _col_i = math.max(COL_MIN_RAD, math.min(COL_MAX_RAD, _col_i + KI_VZ * vz_error * dt))
        col_cmd = _col_i + KP_VZ * vz_error
    end

    if col_cmd < COL_MIN_RAD then col_cmd = COL_MIN_RAD end
    if col_cmd > COL_MAX_RAD then col_cmd = COL_MAX_RAD end

    local col_delta = col_cmd - _last_col_rad
    if col_delta >  COL_SLEW_MAX then col_delta =  COL_SLEW_MAX end
    if col_delta < -COL_SLEW_MAX then col_delta = -COL_SLEW_MAX end
    _last_col_rad = _last_col_rad + col_delta

    local col_thrust = (_last_col_rad - COL_MIN_RAD) / (COL_MAX_RAD - COL_MIN_RAD)
    if col_thrust < 0.0 then col_thrust = 0.0 end
    if col_thrust > 1.0 then col_thrust = 1.0 end
    local ch3 = math.floor(1000.0 + col_thrust * 1000.0 + 0.5)
    if _rc_ch3 then _rc_ch3:set_override(ch3) end

    -- Diagnostic log (every ~5 s at 50 Hz)
    if _diag % 250 == 1 then
        local err_mag  = math.sqrt(err_bx * err_bx + err_by * err_by)
        local pump_info = ""
        if _is_pumping and tlen then
            pump_info = string.format("  pump=%d  tlen=%.1f m", substate, tlen)
        end
        gcs:send_text(6, string.format(
            "RAWES: ch1=%d ch2=%d ch3=%d |err|=%.3f  el=%.1f deg  alt=%.1f m%s",
            ch1, ch2, ch3, err_mag, math.deg(_el_rad), _target_alt, pump_info))
    end
end

-- ── RAWES_ARM: timed arm/disarm state machine ────────────────────────────────

local function run_armon(now)
    local armon_ms = _nv_floats["RAWES_ARM"]
    if armon_ms and armon_ms > 0 then
        _nv_floats["RAWES_ARM"] = nil
        _armon_deadline_ms = now + armon_ms
        _armon_secs        = math.floor(armon_ms / 1000)
        _armon_armed_sent  = false
        if _armon_state ~= "armed" then
            _armon_state = "interlock_low"
        end
    end

    if _armon_state == "interlock_low" then
        if _rc_ch3 then _rc_ch3:set_override(1000) end
        if _rc_ch8 then _rc_ch8:set_override(1000) end
        _armon_state = "arming"

    elseif _armon_state == "arming" then
        if _rc_ch3 then _rc_ch3:set_override(1000) end
        if _rc_ch8 then _rc_ch8:set_override(1000) end
        if arming:is_armed() then
            _armon_state = "armed"
        else
            arming:arm()
        end

    elseif _armon_state == "armed" then
        if _rc_ch3 then _rc_ch3:set_override(1000) end
        if _rc_ch8 then _rc_ch8:set_override(2000) end
        if not _armon_armed_sent then
            _armon_armed_sent = true
            gcs:send_text(6, string.format("RAWES arm-on: armed, expires in %ds", _armon_secs))
        end
        if _armon_deadline_ms and now >= _armon_deadline_ms then
            _armon_state       = nil
            _armon_deadline_ms = nil
            _armon_armed_sent  = false
            arming:disarm()
            gcs:send_text(6, "RAWES arm-on: expired, disarmed")
        end
    end
end

-- ── Main update ───────────────────────────────────────────────────────────────

local function update()
    _diag = _diag + 1

    -- Drain MAVLink named-float inbox (all modes including 0)
    local nvf_raw = mavlink.receive_chan()
    while nvf_raw do
        local _, nv_val, nv_name = string.unpack("<Ifc10", nvf_raw, 13)
        nv_name = nv_name:gsub("\0", "")
        _nv_floats[nv_name] = nv_val
        gcs:send_text(6, string.format("RAWES: rcvd %s=%.0f", nv_name, nv_val))
        nvf_raw = mavlink.receive_chan()
    end

    -- Decode mode and substate
    local mode = math.floor(p("SCR_USER6", 0) + 0.5)
    local sub  = math.floor((_nv_floats["RAWES_SUB"] or 0) + 0.5)
    local now  = millis()

    -- Update altitude and tension targets from NV messages (persistent last value)
    if _nv_floats["RAWES_ALT"]    then _target_alt   = _nv_floats["RAWES_ALT"]    end
    if _nv_floats["RAWES_TEN"]    then _tension_n    = _nv_floats["RAWES_TEN"]    end
    if _nv_floats["RAWES_TLN"] then
        _trim_lon       = _nv_floats["RAWES_TLN"]
        _yaw_since_trim = 0.0   -- new reference yaw = current body yaw
        _nv_floats["RAWES_TLN"] = nil
    end
    if _nv_floats["RAWES_TLT"] then
        _trim_lat       = _nv_floats["RAWES_TLT"]
        _yaw_since_trim = 0.0
        _nv_floats["RAWES_TLT"] = nil
    end
    if _nv_floats["RAWES_COL"]    then _ic_col       = _nv_floats["RAWES_COL"]    end
    if _nv_floats["RAWES_TSP"] then
        _ten_setpoint = _nv_floats["RAWES_TSP"]
        _ten_sp_fresh = true
        _nv_floats["RAWES_TSP"] = nil   -- consume so it triggers once per command
    end

    -- Ch4 yaw always neutral (no RC receiver; prevents ACRO yaw wind-up)
    if _rc_ch4 then _rc_ch4:set_override(1500) end

    run_armon(now)

    -- Mode/substate change tracking
    if mode ~= _prev_mode then
        _on_mode_enter(mode)
        _prev_mode  = mode
        _prev_sub   = sub
        _mode_ms    = now
        _submode_ms = now
    elseif sub ~= _prev_sub then
        _prev_sub   = sub
        _submode_ms = now
    end

    if mode == MODE_NONE then
        if now - _none_status_ms >= 5000 then
            _none_status_ms = now
            local armed = arming:is_armed() and "ARMED" or "disarmed"
            gcs:send_text(6, "RAWES: mode 0 (none)  " .. armed)
        end
        return update, BASE_PERIOD_MS
    end

    if arming:is_armed() and _rc_ch8 then
        _rc_ch8:set_override(2000)
    end

    -- Integrate gyro:z() so MODE_PASSIVE / MODE_YAW can rotate the trim
    -- cyclic into the current body frame, keeping the world-frame disk-
    -- tilt direction constant as the body yaws.
    if mode == MODE_PASSIVE or mode == MODE_YAW then
        local _gyro = ahrs:get_gyro()
        if _gyro then
            _yaw_since_trim = _yaw_since_trim + _gyro:z() * (BASE_PERIOD_MS * 0.001)
        end
    else
        -- Other modes do not use trim cyclic; reset so the next entry
        -- to PASSIVE/YAW starts from a clean reference yaw.
        _yaw_since_trim = 0.0
    end

    if mode == MODE_PASSIVE then
        -- Armed-but-quiet: hold the IC operating point so the kinematic
        -- release transitions smoothly.  Cyclic rate setpoints are sized
        -- so ArduPilot's rate PID produces _trim_lon / _trim_lat at
        -- steady state (rotated by -_yaw_since_trim into the current
        -- body frame); ch3 pins collective at _ic_col.
        set_trim_ic_rc_overrides()
        if now - _none_status_ms >= 5000 then
            _none_status_ms = now
            gcs:send_text(6, string.format(
                "RAWES PASS: dyaw=%.2fdeg  tlon=%.4f tlat=%.4f col=%.3f",
                math.deg(_yaw_since_trim), _trim_lon, _trim_lat, _ic_col))
        end
        return update, BASE_PERIOD_MS
    end

    if mode == MODE_STEADY or mode == MODE_PUMPING then
        if now - _last_flight_ms >= FLIGHT_PERIOD_MS then
            _last_flight_ms = now
            run_flight()
        end
    end

    if mode == MODE_YAW then
        -- Hold cyclic at IC trim (rotated to current body frame so the
        -- world-frame disk-tilt direction stays aligned with the
        -- gravity-comp reference regardless of how the hub has yawed)
        -- and collective at _ic_col.  run_yaw_pid drives SERVO4 directly
        -- via SRV_Channels and overrides ch1/ch2/ch3 at the end with
        -- the IC trim values (see run_yaw_pid).
        run_yaw_pid(BASE_PERIOD_MS * 0.001)
    end

    -- MODE_LANDING (4): not yet implemented

    return update, BASE_PERIOD_MS
end

-- ── Entry point ───────────────────────────────────────────────────────────────

local _mode_init  = math.floor(p("SCR_USER6", 0) + 0.5)
local _mode_names = {[0]="none", [1]="steady", [2]="yaw", [4]="landing", [5]="pumping"}
local _mode_str   = _mode_names[_mode_init] or "unknown"

gcs:send_text(6, string.format(
    "RAWES: loaded  mode=%d (%s)  kp=%.2f  slew=%.2f  anchor=(%.1f %.1f %.1f)",
    _mode_init, _mode_str,
    p("SCR_USER1", 1.0), p("SCR_USER2", 0.40),
    p("SCR_USER3", 0.0), p("SCR_USER4", 0.0), p("SCR_USER5", 0.0)))

-- @@UNIT_TEST_HOOK

return update, BASE_PERIOD_MS
