--[[
rawes.lua -- Unified RAWES flight controller
Works in both ArduPilot SITL (mcroomp fork) and on the Pixhawk 6C.

Mode is selected at runtime via SCR_USER6 (plain integer 0,1,2,5):
  0  none        -- script passive: no RC overrides; logs every 5 s + any NV message
    1  steady      -- GUIDED rate hold (set_target_rate_and_throttle) + altitude collective PID  50 Hz
  2  manual      -- yaw compensation (SERVO4 PID) + manually commanded cyclic/collective  100 Hz
                    NVFs: RAWES_TLN (tlon rad), RAWES_TLT (tlat rad), RAWES_COL (col rad)
  4  landing     -- (reserved, not yet implemented)
  5  pumping     -- De Schutter pumping cycle                                       50 Hz

Cyclic + collective control (steady + pumping): single GUIDED rate-only call.
  Compute bz_goal = bz_altitude_hold(pos, el_rad, tension_n):
    points the disk at (el_rad, current_azimuth) with a gravity-compensation tilt
    so thrust counteracts the elevation-lowering component of gravity.
    Convert body_z error to body-frame roll/pitch rates with sqrt-shaped P.
    Compute col_thrust in [0,1] from local altitude PID.
    Call vehicle:set_target_rate_and_throttle(roll_rate, pitch_rate, 0, col_thrust) at 50 Hz.
  RC ch3 is NO LONGER overridden by run_flight; collective flows through the AP
  attitude controller's set_throttle_out path.
  Before GPS fuses: command current body attitude (zero corrective torque).

Ground planner signals via NAMED_VALUE_FLOAT:
  RAWES_SUB (mode=5): pumping substate 0=hold 1=reel_out 2=transition 3=reel_in 4=transition_back
  RAWES_ALT: target altitude [m] above anchor; Lua rate-limits elevation toward it
    RAWES_TEN: target/feed-forward tether tension [N]; used for gravity compensation
  RAWES_ARM: arm vehicle with timed disarm countdown (value = ms until disarm)

Parameters:
  SCR_USER2   RAWES_BZ_SLEW   elevation slew rate     [rad/s]        default 0.40
  SCR_USER3   RAWES_ANCHOR_N  Anchor North from EKF origin [m]        default 0.0
  SCR_USER4   RAWES_ANCHOR_E  Anchor East  from EKF origin [m]        default 0.0
  SCR_USER5   RAWES_ANCHOR_D  Anchor Down  from EKF origin [m]        default 0.0
  SCR_USER6   RAWES_MODE      Mode selector (0,1,5)                   default 0
--]]

-- ── Constants ─────────────────────────────────────────────────────────────────

local BASE_PERIOD_MS    = 10        -- 100 Hz base tick
local FLIGHT_PERIOD_MS  = 20        -- 50 Hz flight subsystem
local GUIDED_MODE_NUM   = 4         -- ArduCopter GUIDED = 4

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
local MODE_MANUAL  = 2   -- yaw compensation + manually commanded cyclic/collective via NVFs
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

-- ── GPS / attitude ──────────────────────────────────────────────────────────────────

local MIN_TETHER_M     = 0.5        -- minimum tether length before GPS init activates

-- ── Collective limits and cruise values ───────────────────────────────────────

local COL_MIN_RAD           = -0.28
local COL_MAX_RAD           =  0.10
local COL_SLEW_MAX          =  0.022   -- rad per 50 Hz step
local COL_CRUISE_FLIGHT_RAD = -0.18    -- VZ integrator initial value (xi~8 deg, altitude hold)

-- ── Altitude collective + body-rate controller constants ─────────────────────

local KP_ALT = 0.010
local KI_ALT = 0.001
local KD_VZ  = 0.040
local RATE_KP_OUTER        = 2.5
local RATE_ACCEL_MAX_RADSS = 4.0

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

-- Collective state
local _last_col_rad = COL_CRUISE_FLIGHT_RAD
local _col_trim     = COL_CRUISE_FLIGHT_RAD
local _alt_i        = 0.0
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
local _tension_n      = 200.0   -- target/feed-forward tether tension [N]; updated from RAWES_TEN

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

-- Convert a desired body_z direction (NED Vector3f) to ZYX Euler roll/pitch (degrees)
-- given the current heading yaw_rad.  At RAWES tether elevation 65 deg the resulting
-- pitch is ~25 deg from level -- well clear of the 90 deg gimbal-lock singularity.
-- Derivation: in the yaw-aligned frame, body_z = [sin_p*cos_r, -sin_r, cos_p*cos_r].
local function bz_ned_to_roll_pitch(bz_ned, yaw_rad)
    local cy = math.cos(yaw_rad)
    local sy = math.sin(yaw_rad)
    local bz_fwd   =  cy * bz_ned:x() + sy * bz_ned:y()   -- sin_p * cos_r
    local bz_right = -sy * bz_ned:x() + cy * bz_ned:y()   -- -sin_r
    local bz_down  = bz_ned:z()                            -- cos_p * cos_r
    local pitch_deg = math.deg(math.atan(bz_fwd, bz_down))
    local roll_deg  = math.deg(math.asin(math.max(-1.0, math.min(1.0, -bz_right))))
    return roll_deg, pitch_deg
end

local function sqrt_rate_from_error(error, kp, accel_max, dt)
    local rate
    if accel_max <= 0.0 then
        rate = error * kp
    elseif kp == 0.0 then
        if error == 0.0 then
            rate = 0.0
        elseif error > 0.0 then
            rate = math.sqrt(2.0 * accel_max * math.abs(error))
        else
            rate = -math.sqrt(2.0 * accel_max * math.abs(error))
        end
    else
        local linear_dist = accel_max / (kp * kp)
        if error > linear_dist then
            rate = math.sqrt(2.0 * accel_max * (error - linear_dist / 2.0))
        elseif error < -linear_dist then
            rate = -math.sqrt(2.0 * accel_max * (-error - linear_dist / 2.0))
        else
            rate = error * kp
        end
    end

    if dt > 0.0 then
        local max_rate = math.abs(error) / dt
        if rate >  max_rate then rate =  max_rate end
        if rate < -max_rate then rate = -max_rate end
    end
    return rate
end

local function compute_rate_cmd_sqrt(bz_now, bz_goal, kp, accel_max, dt)
    local cx = bz_now:y() * bz_goal:z() - bz_now:z() * bz_goal:y()
    local cy = bz_now:z() * bz_goal:x() - bz_now:x() * bz_goal:z()
    local cz = bz_now:x() * bz_goal:y() - bz_now:y() * bz_goal:x()
    local cn = math.sqrt(cx*cx + cy*cy + cz*cz)
    local dot = bz_now:x() * bz_goal:x() + bz_now:y() * bz_goal:y() + bz_now:z() * bz_goal:z()
    if dot >  1.0 then dot =  1.0 end
    if dot < -1.0 then dot = -1.0 end
    local angle = math.atan(cn, dot)

    local wx, wy, wz = 0.0, 0.0, 0.0
    if cn > 1e-12 and angle > 1e-12 then
        local rate_mag = sqrt_rate_from_error(angle, kp, accel_max, dt)
        wx = cx / cn * rate_mag
        wy = cy / cn * rate_mag
        wz = cz / cn * rate_mag
    end

    local rate_world = Vector3f()
    rate_world:x(wx); rate_world:y(wy); rate_world:z(wz)
    return ahrs:earth_to_body(rate_world)
end

-- Compute the RC1/RC2/RC3 PWM overrides for holding the IC operating
-- point.  Used by MODE_PASSIVE and MODE_MANUAL, both of which require
-- H_FLYBAR_MODE=1 (calibrate.py's `run` command sets this on entry and
-- restores on exit).
--
-- ── MODE_MANUAL: yaw compensation + manual cyclic/collective ────────────────
-- SERVO4 (GB4008): driven by yaw PID.  Same sign convention as the old MODE_YAW:
--   err = -gyro_z; positive error (CCW drift) -> more motor throttle -> CW torque.
-- RC1/RC2: set from _man_tlat_rad / _man_tlon_rad (updated via RAWES_TLT / RAWES_TLN).
--   PWM = 1500 + (setpoint_rad / H_CYC_MAX_rad) * 500, clamped to [1000, 2000].
--   Requires H_FLYBAR_MODE=1 so the RC override bypasses the rate PID.
-- RC3: set from _ic_col (updated via RAWES_COL NVF).
-- All three NVFs are persistent (last value holds until a new one arrives).

local _yaw_i      = 0.0
local _yaw_prev_e = 0.0

-- Manual mode cyclic setpoints [rad].  Updated from RAWES_TLN / RAWES_TLT NVFs.
-- tlon > 0 = nose-down disk;  tlat > 0 = roll-right.  Reset to 0 on mode entry.
local _man_tlon_rad = 0.0
local _man_tlat_rad = 0.0

local SERVO4_CHAN    = 3     -- 0-indexed physical channel (servo 4 = index 3)

local function run_manual(dt)
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

    -- Set cyclic from NVF setpoints (tlon/tlat in rad) using H_CYC_MAX as the full-range.
    -- With H_FLYBAR_MODE=1 the RC override bypasses the rate PID and goes straight to
    -- the swash mixer.  H_CYC_MAX is in centidegrees (1000 cd = 10 deg).
    local _cyc_max_rad = math.rad(p("H_CYC_MAX", 1000) / 100.0)
    local _ch1_pwm = math.floor(1500.0 + _man_tlat_rad / _cyc_max_rad * 500.0 + 0.5)
    local _ch2_pwm = math.floor(1500.0 + _man_tlon_rad / _cyc_max_rad * 500.0 + 0.5)
    if _ch1_pwm > 2000 then _ch1_pwm = 2000 end
    if _ch1_pwm < 1000 then _ch1_pwm = 1000 end
    if _ch2_pwm > 2000 then _ch2_pwm = 2000 end
    if _ch2_pwm < 1000 then _ch2_pwm = 1000 end
    if _rc_ch1 then _rc_ch1:set_override(_ch1_pwm) end
    if _rc_ch2 then _rc_ch2:set_override(_ch2_pwm) end
    -- Collective from _ic_col (updated via RAWES_COL NVF).
    local _col_thrust_man = (_ic_col - COL_MIN_RAD) / (COL_MAX_RAD - COL_MIN_RAD)
    if _col_thrust_man < 0.0 then _col_thrust_man = 0.0 end
    if _col_thrust_man > 1.0 then _col_thrust_man = 1.0 end
    if _rc_ch3 then _rc_ch3:set_override(math.floor(1000.0 + _col_thrust_man * 1000.0 + 0.5)) end

    if _diag % 100 == 1 then
        gcs:send_text(6, string.format(
            "RAWES man: yrate=%+.1fdeg/s  s4out=%.3f  s4pwm=%d  col=%.2fdeg  tlon=%.2fdeg  tlat=%.2fdeg",
            math.deg(yaw_rate), output, pwm,
            math.deg(_ic_col), math.deg(_man_tlon_rad), math.deg(_man_tlat_rad)))
    end
end

-- ── Mode-entry reset ─────────────────────────────────────────────────────────

local function _on_mode_enter(mode)
    _nv_floats      = {}   -- clear NV inbox so stale substates cannot bleed through
    _none_status_ms = 0
    if mode == MODE_MANUAL then
        _yaw_i        = 0.0
        _yaw_prev_e   = 0.0
        _man_tlon_rad = 0.0
        _man_tlat_rad = 0.0
    end
end

-- ── Flight subsystem ─────────────────────────────────────────────────────────

local function run_flight()
    if vehicle:get_mode() ~= GUIDED_MODE_NUM then return end

    local mode_now    = _prev_mode
    local substate    = _prev_sub
    local dt          = FLIGHT_PERIOD_MS * 0.001
    local _is_pumping = mode_now == MODE_PUMPING

    -- ── Before GPS initialization: hold current body attitude ─────────────
    -- Command the current AHRS attitude so ArduPilot holds it with zero
    -- corrective torque, preserving the orbital rate from kinematic.
    -- Also primes _attitude_target so there is no step transient at GPS init.
    -- Collective is passed as throttle so ArduPilot's set_throttle_out path
    -- controls it directly -- no ch3 RC override needed.
    if not _el_initialized then
        local ct = (COL_CRUISE_FLIGHT_RAD - COL_MIN_RAD) / (COL_MAX_RAD - COL_MIN_RAD)
        if ct < 0.0 then ct = 0.0 end
        if ct > 1.0 then ct = 1.0 end

        if not ahrs:healthy() then return end

        vehicle:set_target_rate_and_throttle(0.0, 0.0, 0.0, ct)

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
                _col_trim     = COL_CRUISE_FLIGHT_RAD
                _last_col_rad = COL_CRUISE_FLIGHT_RAD
                _alt_i        = 0.0
                _el_initialized = true
                local label = _is_pumping and "pump" or "steady"
                gcs:send_text(6, string.format(
                    "RAWES %s: captured  el=%.1f deg  alt=%.1f m  tlen=%.1f m",
                    label, math.deg(_el_rad), _target_alt, tlen))
            end
        end
        return
    end

    -- ── GPS initialized: altitude hold ────────────────────────────────────+

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

    -- Body-z P/sqrt loop: target tension is feed-forward only.
    local bz_goal = bz_altitude_hold(rel, _el_rad, _tension_n)
    local body_z_body = Vector3f()
    body_z_body:x(0.0); body_z_body:y(0.0); body_z_body:z(1.0)
    local bz_now = ahrs:body_to_earth(body_z_body)
    local rate_cmd = compute_rate_cmd_sqrt(
        bz_now, bz_goal, RATE_KP_OUTER, RATE_ACCEL_MAX_RADSS, dt)

    -- Local altitude PID controls collective; ground no longer commands thrust.
    local alt_m = -rel:z()
    local vz_up = 0.0
    local vel_ned = ahrs:get_velocity_NED()
    if vel_ned then vz_up = -vel_ned:z() end
    local alt_err = _target_alt - alt_m
    _alt_i = _alt_i + KI_ALT * alt_err * dt
    local i_min = COL_MIN_RAD - _col_trim
    local i_max = COL_MAX_RAD - _col_trim
    if _alt_i < i_min then _alt_i = i_min end
    if _alt_i > i_max then _alt_i = i_max end
    local col_cmd = _col_trim + KP_ALT * alt_err - KD_VZ * vz_up + _alt_i

    -- Vibration damper: body-Z accel, HP filter, velocity estimate, collective.
    _vib_corr_last = 0.0
    local imu_a = ahrs:get_accel()
    if imu_a then
        _vib_corr_last = vib_damper_step(imu_a:z(), dt)
        col_cmd = col_cmd + _vib_corr_last
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

    -- Rate-only GUIDED call: body-frame rates + collective throttle.
    -- ArduPilot routes the throttle through set_throttle_out; no ch3 RC override needed.
    vehicle:set_target_rate_and_throttle(
        math.deg(rate_cmd:x()), math.deg(rate_cmd:y()), 0.0, col_thrust)

    -- Diagnostic log (every ~5 s at 50 Hz)
    if _diag % 250 == 1 then
        local pump_info = ""
        if _is_pumping and tlen then
            pump_info = string.format("  pump=%d  tlen=%.1f m", substate, tlen)
        end
        gcs:send_text(6, string.format(
            "RAWES: rr=%.1f pr=%.1f thr=%.2f  el=%.1f deg  alt=%.1f m%s",
            math.deg(rate_cmd:x()), math.deg(rate_cmd:y()), col_thrust, math.deg(_el_rad), _target_alt, pump_info))
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

    -- Update altitude, tension, cyclic and collective targets from NV messages
    if _nv_floats["RAWES_ALT"]    then _target_alt   = _nv_floats["RAWES_ALT"]    end
    if _nv_floats["RAWES_TEN"]    then _tension_n    = _nv_floats["RAWES_TEN"]    end
    if _nv_floats["RAWES_COL"]    then _ic_col       = _nv_floats["RAWES_COL"]    end
    if _nv_floats["RAWES_TLN"]    then _man_tlon_rad = _nv_floats["RAWES_TLN"]    end
    if _nv_floats["RAWES_TLT"]    then _man_tlat_rad = _nv_floats["RAWES_TLT"]    end
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

    if mode == MODE_PASSIVE then
        -- Armed-but-quiet: hold the IC operating point so the kinematic
        -- release transitions smoothly.  Cyclic is neutral; ch3 pins
        -- collective at _ic_col.
        if _rc_ch1 then _rc_ch1:set_override(1500) end
        if _rc_ch2 then _rc_ch2:set_override(1500) end
        local _col_thrust_p = (_ic_col - COL_MIN_RAD) / (COL_MAX_RAD - COL_MIN_RAD)
        if _col_thrust_p < 0.0 then _col_thrust_p = 0.0 end
        if _col_thrust_p > 1.0 then _col_thrust_p = 1.0 end
        if _rc_ch3 then _rc_ch3:set_override(math.floor(1000.0 + _col_thrust_p * 1000.0 + 0.5)) end
        -- Take ownership of SERVO4 and pin the GB4008 at 800 us (ESC armed
        -- but motor off).  Matches the safety-shutdown PWM used by
        -- calibrate.py so a PASSIVE session never spins the rotor.  Same
        -- direct-write path MODE_MANUAL uses; needs SERVO4_FUNCTION=0 for the
        -- override to stick (calibrate.py's `passive` command handles that).
        SRV_Channels:set_output_pwm_chan_timeout(SERVO4_CHAN, 800, 200)
        if now - _none_status_ms >= 5000 then
            _none_status_ms = now
            gcs:send_text(6, string.format(
                "RAWES PASS: col=%.2fdeg s4=800",
                math.deg(_ic_col)))
        end
        return update, BASE_PERIOD_MS
    end

    if mode == MODE_STEADY or mode == MODE_PUMPING then
        if now - _last_flight_ms >= FLIGHT_PERIOD_MS then
            _last_flight_ms = now
            run_flight()
        end
    end

    if mode == MODE_MANUAL then
        -- Manual mode: yaw compensation via SERVO4 PID + NVF-commanded cyclic/collective.
        -- run_manual drives SERVO4 directly and sets RC1/RC2/RC3 via override.
        run_manual(BASE_PERIOD_MS * 0.001)
    end

    -- MODE_LANDING (4): not yet implemented

    return update, BASE_PERIOD_MS
end

-- ── Entry point ───────────────────────────────────────────────────────────────

local _mode_init  = math.floor(p("SCR_USER6", 0) + 0.5)
local _mode_names = {[0]="none", [1]="steady", [2]="manual", [4]="landing", [5]="pumping"}
local _mode_str   = _mode_names[_mode_init] or "unknown"

gcs:send_text(6, string.format(
    "RAWES: loaded  mode=%d (%s)  slew=%.2f  anchor=(%.1f %.1f %.1f)",
    _mode_init, _mode_str,
    p("SCR_USER2", 0.40),
    p("SCR_USER3", 0.0), p("SCR_USER4", 0.0), p("SCR_USER5", 0.0)))

-- @@UNIT_TEST_HOOK

return update, BASE_PERIOD_MS
