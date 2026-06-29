--[[
rawes.lua -- Unified RAWES flight controller
Works in both ArduPilot SITL (mcroomp fork) and on the Pixhawk 6C.

Mode is selected at runtime via SCR_USER6:
    0  none        -- script passive: no RC overrides; logs every 5 s + any NV message
    1  steady      -- primary guided flight path (set_target_rate_and_throttle)
    2  manual      -- legacy bench override: yaw compensation + commanded cyclic/collective
    3  passive     -- kinematic capture helper; keeps the IC attitude stable during release
    4  landing     -- reserved, not yet implemented
    5  pumping     -- De Schutter pumping cycle

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
  RAWES_SUB: substate 0=hold 1=reel_out 2=transition 3=reel_in 4=transition_back
             (set by the ground pumping schedule; the vehicle stays in MODE_STEADY)
  RAWES_ALT: target altitude [m] above anchor; Lua rate-limits elevation toward it
    RAWES_TEN: target/feed-forward tether tension [N]; used for gravity compensation
    RAWES_ARM: optional disarm timer (value = ms until forced disarm)

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
-- Smooth handoff after kinematic release: keep plant physics unchanged, but
-- phase in guidance/corrections over a longer window to avoid a command step.
local POST_RELEASE_BLEND_S = 2.5    -- blend current->steady body_z after capture
local POST_RELEASE_RECOVERY_S = 2.0 -- ramp-in for altitude/vibration corrections

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
local MODE_MANUAL  = 2   -- legacy bench override: yaw compensation + manually commanded cyclic/collective via NVFs
local MODE_PASSIVE = 3   -- kinematic capture helper: hold the IC attitude stable during release.
local MODE_LANDING = 4   -- reserved; not implemented here

-- MODE_PASSIVE IC seeds are provided over NVF (10-char names max):
--   RAWES_COL : IC collective [rad]
--   RAWES_RIC : IC roll       [rad]
--   RAWES_PIC : IC pitch      [rad]
-- They start nil and are committed atomically only after all three arrive.

-- RAWES_SUB carries a generic substate index (delivered via NAMED_VALUE_FLOAT).
-- The ground pumping schedule runs in MODE_STEADY and uses RAWES_SUB only for
-- telemetry/diagnostics (0=hold 1=reel-out 2=transition 3=reel-in 4=transition-
-- back).  RAWES_SUB is also reserved for future landing/takeoff sequencing.

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
-- Rate-stability gate on the collective vz-damping term.  While the attitude
-- loop is still slewing (flight start cyclic establishment, reel transitions),
-- the measured vertical velocity is partly rotational coupling rather than a
-- true altitude rate; reacting to it with collective injects a spurious thrust
-- (tether-tension) spike.  Scale KD_VZ down when body rates are elevated -- the
-- KP/KI altitude terms still hold altitude, only the derivative kick is gated.
local VZ_GATE_RATE_RADS = 1.0          -- |gyro| at which the vz-damping fully fades
local VZ_GATE_MIN       = 0.0          -- floor on the gate factor
local RATE_KP_OUTER        = 2.5
local RATE_ACCEL_MAX_RADSS = 4.0

-- Plane-keeping azimuth low-pass time constant [s].  The body-z azimuth
-- reference is slowly slewed toward the instantaneous position azimuth so fast
-- lateral excursions do not chase their own position (positive feedback that
-- pushes the kite off the downwind plane).  Plane-keeping ESTIMATE from the
-- kite's own position only -- no truth-wind oracle (AGENTS.md invariant).
local AZ_REF_TAU_S = 15.0

-- ── Shared state ─────────────────────────────────────────────────────────────

local _diag           = 0
local _last_flight_ms = 0
local _none_status_ms = 0

-- RAWES_ARM disarm timer
local _armon_deadline_ms = nil
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
local _az_ref         = 0.0     -- plane-keeping azimuth estimate [rad] (low-pass of position azimuth)
local _az_initialized = false   -- true once _az_ref seeded from first GPS fix

-- IC collective [rad] — ground sends via RAWES_COL.  Used by MODE_PASSIVE
-- to pin ch3 at the IC value so omega_spin doesn't droop while the body
-- is kinematically locked.  Defaults to the cruise-flight value.
local _ic_col         = nil
local _ic_roll_deg    = nil
local _ic_pitch_deg   = nil
local _ic_seeded      = false
local _ic_pending_col = nil
local _ic_pending_roll_deg = nil
local _ic_pending_pitch_deg = nil

-- One-shot debug state for capture/first-command handoff diagnostics.
local _dbg_cap_logged = false
local _dbg_cmd_logged = false
local _dbg_cap_bz_x   = 0.0
local _dbg_cap_bz_y   = 0.0
local _dbg_cap_bz_z   = 1.0
local _capture_ms     = nil
local _passive_hold_yaw_rad = nil

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
local function bz_altitude_hold(pos, el_rad, tension_n, az_ref)
    local az     = az_ref or math.atan(pos:y(), pos:x())
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

-- Wrap an angle to (-pi, pi].
local function wrap_pi(a)
    return math.atan(math.sin(a), math.cos(a))
end

local function angle_between_deg(a, b)
    local dot = a:x() * b:x() + a:y() * b:y() + a:z() * b:z()
    if dot > 1.0 then dot = 1.0 end
    if dot < -1.0 then dot = -1.0 end
    return math.deg(math.acos(dot))
end

local function body_z_now_ned()
    local z_b = Vector3f()
    z_b:x(0.0); z_b:y(0.0); z_b:z(1.0)
    return ahrs:body_to_earth(z_b)
end

-- Low-pass the downwind-plane azimuth toward the position azimuth.  Portable
-- plane-keeping estimator mirrored 1:1 in controller.py (update_plane_azimuth).
local function update_plane_azimuth(az_ref, pos, tau_s, dt)
    local az_meas = math.atan(pos:y(), pos:x())
    local alpha   = dt / tau_s
    return wrap_pi(az_ref + alpha * wrap_pi(az_meas - az_ref))
end

local function blend_bz(bz_now, bz_goal, alpha)
    local a = alpha
    if a < 0.0 then a = 0.0 end
    if a > 1.0 then a = 1.0 end
    local x = (1.0 - a) * bz_now:x() + a * bz_goal:x()
    local y = (1.0 - a) * bz_now:y() + a * bz_goal:y()
    local z = (1.0 - a) * bz_now:z() + a * bz_goal:z()
    local n = math.sqrt(x*x + y*y + z*z)
    if n < 1e-6 then
        return bz_now
    end
    local r = Vector3f()
    r:x(x / n); r:y(y / n); r:z(z / n)
    return r
end

local function col_rad_to_thrust(col_rad)
    local span = COL_MAX_RAD - COL_MIN_RAD
    if span <= 1e-9 then
        return 0.5
    end
    local t = (col_rad - COL_MIN_RAD) / span
    if t < 0.0 then t = 0.0 end
    if t > 1.0 then t = 1.0 end
    return t
end

local function ic_col_or_default()
    if _ic_col ~= nil then return _ic_col end
    return COL_CRUISE_FLIGHT_RAD
end

local function thrust_to_col_rad(thrust)
    local t = thrust
    if t < 0.0 then t = 0.0 end
    if t > 1.0 then t = 1.0 end
    return COL_MIN_RAD + t * (COL_MAX_RAD - COL_MIN_RAD)
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

-- Legacy manual-mode RC override path.
--
-- ── MODE_MANUAL: legacy yaw compensation + manual cyclic/collective ─────────
-- SERVO4 (GB4008): driven by yaw PID.  Same sign convention as the old MODE_YAW:
--   err = -gyro_z; positive error (CCW drift) -> more motor throttle -> CW torque.
-- RC1/RC2: set from _man_tlat_rad / _man_tlon_rad (updated via RAWES_TLT / RAWES_TLN).
--   PWM = 1500 + (setpoint_rad / H_CYC_MAX_rad) * 500, clamped to [1000, 2000].
--   Requires H_FLYBAR_MODE=1 and ACRO so the RC override bypasses the rate PID
--   and the RC channels directly control the servos.
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
    -- Collective from IC seed if available, else cruise fallback.
    local _col_thrust_man = col_rad_to_thrust(ic_col_or_default())
    if _rc_ch3 then _rc_ch3:set_override(math.floor(1000.0 + _col_thrust_man * 1000.0 + 0.5)) end

    if _diag % 100 == 1 then
        gcs:send_text(6, string.format(
            "RAWES man: yrate=%+.1fdeg/s  s4out=%.3f  s4pwm=%d  col=%.2fdeg  tlon=%.2fdeg  tlat=%.2fdeg",
            math.deg(yaw_rate), output, pwm,
            math.deg(ic_col_or_default()), math.deg(_man_tlon_rad), math.deg(_man_tlat_rad)))
    end
end

-- ── Mode-entry reset ─────────────────────────────────────────────────────────

local function _on_mode_enter(mode)
    _nv_floats      = {}   -- clear NV inbox so stale substates cannot bleed through
    _none_status_ms = 0
    if mode == MODE_STEADY then
        _dbg_cap_logged = false
        _dbg_cmd_logged = false
        -- Start a fresh post-release recovery window at steady entry.
        _capture_ms = millis()
        -- Clear vibration states so stale kinematic-history residuals do not
        -- inject a collective kick exactly at release.
        _vib_acc_hp = 0.0
        _vib_acc_prev = 0.0
        _vib_vel_est = 0.0
        _vib_corr_last = 0.0
        local _thr_ic = col_rad_to_thrust(ic_col_or_default())
        local _col_back = thrust_to_col_rad(_thr_ic)
        gcs:send_text(6, string.format(
            "RAWES steady: IC col=%.2fdeg -> thr=%.3f -> col=%.2fdeg",
            math.deg(ic_col_or_default()), _thr_ic, math.deg(_col_back)))
    end
    if mode == MODE_MANUAL then
        _yaw_i        = 0.0
        _yaw_prev_e   = 0.0
        _man_tlon_rad = 0.0
        _man_tlat_rad = 0.0
    end
    if mode == MODE_PASSIVE then
        local y = ahrs:get_yaw_rad()
        if y ~= nil then
            _passive_hold_yaw_rad = y
        else
            _passive_hold_yaw_rad = 0.0
        end
    end
end

-- ── Flight subsystem ─────────────────────────────────────────────────────────

local function run_flight()
    -- Accept both GUIDED (4) and GUIDED_NOGPS (20) modes.
    -- GUIDED_NOGPS allows arming without GPS; both support set_target_angle_and_rate_and_throttle.
    local mode = vehicle:get_mode()
    if mode ~= GUIDED_MODE_NUM and mode ~= 20 then return end

    local mode_now    = _prev_mode
    local substate    = _prev_sub
    local dt          = FLIGHT_PERIOD_MS * 0.001
    local now_ms      = millis()

    -- ── Before GPS initialization: hold current body attitude ─────────────
    -- Continuously command current AHRS roll/pitch/yaw through GUIDED angle
    -- control so _attitude_target never times out back to level before capture.
    -- This preserves kinematic continuity with near-zero corrective torque.
    -- Collective is passed as throttle so ArduPilot's set_throttle_out path
    -- controls it directly -- no ch3 RC override needed.
    if not _el_initialized then
        local ct = col_rad_to_thrust(ic_col_or_default())

        if not ahrs:healthy() then return end

        local roll_now = ahrs:get_roll_rad()
        local pitch_now = ahrs:get_pitch_rad()
        local yaw_now = ahrs:get_yaw_rad()
        if roll_now == nil or pitch_now == nil or yaw_now == nil then return end

        vehicle:set_target_angle_and_rate_and_throttle(
            math.deg(roll_now), math.deg(pitch_now), math.deg(yaw_now),
            0.0, 0.0, 0.0, ct)

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
                -- Warm-start the collective feedforward from the actual IC
                -- equilibrium collective (TensionPI-settled, sent by ground via
                -- RAWES_COL) instead of the nominal cruise constant.  This is the
                -- Lua analog of TensionPI.warm_coll_rad in controller.py: it makes
                -- col_cmd == _ic_col at capture (alt_err=vz=0), so there is no
                -- thrust/tension step that the slow KI_ALT integrator must chase.
                -- The I-term legitimately starts at 0 (no integrated error yet).
                local col_ff  = ic_col_or_default()
                if col_ff < COL_MIN_RAD then col_ff = COL_MIN_RAD end
                if col_ff > COL_MAX_RAD then col_ff = COL_MAX_RAD end
                _col_trim     = col_ff
                _last_col_rad = col_ff
                _alt_i        = 0.0
                _az_ref         = math.atan(ry, rx)
                _az_initialized = true
                _el_initialized = true
                _capture_ms     = now_ms

                local rel_cap = Vector3f()
                rel_cap:x(rx); rel_cap:y(ry); rel_cap:z(rz)
                local bz_goal_cap = bz_altitude_hold(rel_cap, _el_rad, _tension_n, _az_ref)
                local bz_now_cap  = body_z_now_ned()
                _dbg_cap_bz_x = bz_now_cap:x()
                _dbg_cap_bz_y = bz_now_cap:y()
                _dbg_cap_bz_z = bz_now_cap:z()
                local d_cap = angle_between_deg(bz_now_cap, bz_goal_cap)
                if not _dbg_cap_logged then
                    _dbg_cap_logged = true
                    gcs:send_text(6, string.format(
                        "RAWES DBG cap: d_bz=%.2fdeg el=%.2f az=%.2f T=%.1f",
                        d_cap, math.deg(_el_rad), math.deg(_az_ref), _tension_n))
                end
                gcs:send_text(6, string.format(
                    "RAWES steady: captured  el=%.1f deg  alt=%.1f m  tlen=%.1f m",
                    math.deg(_el_rad), _target_alt, tlen))
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

    -- Force-balance orientation: the disk only balances commanded tension +
    -- gravity at the ACTUAL position, so the elevation it tracks is the actual
    -- tether elevation, NOT the altitude setpoint.  Altitude is held solely by
    -- the collective PID below.  This decouples orientation (tension-driven,
    -- feedforward) from altitude (collective, feedback) so the two never fight.
    -- The elevation is still rate-limited to break the regenerative
    -- position->tilt->lift->position path (design doc, "Why it should stay
    -- stable", path 1).
    local bz_slew   = p("SCR_USER2", 0.40)
    local target_el = math.asin(math.max(-1.0, math.min(1.0, -rel:z() / math.max(tlen, 0.1))))
    local max_step  = bz_slew * dt
    local el_step   = target_el - _el_rad
    if el_step >  max_step then el_step =  max_step end
    if el_step < -max_step then el_step = -max_step end
    _el_rad = _el_rad + el_step

    -- Plane-keeping: slowly slew the body-z azimuth reference toward the
    -- position azimuth so fast lateral excursions don't chase their own pos.
    if not _az_initialized then
        _az_ref = math.atan(rel:y(), rel:x())
        _az_initialized = true
    else
        _az_ref = update_plane_azimuth(_az_ref, rel, AZ_REF_TAU_S, dt)
    end

    -- Body-z altitude-hold target -> absolute roll/pitch attitude command.
    -- The GUIDED angle path (set_target_angle_and_rate_and_throttle) runs
    -- ArduPilot's native attitude controller (ATC_ANG_*_P + rate PID) against
    -- this bounded target.  This replaces the earlier rate-only cascade, which
    -- fed an outer-P rate into GUIDED's attitude integrator and diverged at the
    -- high-tilt / low-tension reel-in operating point.
    local bz_goal = bz_altitude_hold(rel, _el_rad, _tension_n, _az_ref)
    local recovery_alpha = 1.0
    if _capture_ms ~= nil then
        local t_rel_s = (now_ms - _capture_ms) * 0.001
        local blend = t_rel_s / POST_RELEASE_BLEND_S
        recovery_alpha = t_rel_s / POST_RELEASE_RECOVERY_S
        if blend < 1.0 then
            bz_goal = blend_bz(body_z_now_ned(), bz_goal, blend)
        end
        if recovery_alpha < 0.0 then recovery_alpha = 0.0 end
        if recovery_alpha > 1.0 then recovery_alpha = 1.0 end
    end
    local yaw_now = ahrs:get_yaw_rad()
    local roll_deg, pitch_deg = bz_ned_to_roll_pitch(bz_goal, yaw_now)

    local roll_now_deg = math.deg(ahrs:get_roll_rad())
    local pitch_now_deg = math.deg(ahrs:get_pitch_rad())

    if not _dbg_cmd_logged then
        _dbg_cmd_logged = true
        local bz_now = body_z_now_ned()
        local bz_cap = Vector3f()
        bz_cap:x(_dbg_cap_bz_x); bz_cap:y(_dbg_cap_bz_y); bz_cap:z(_dbg_cap_bz_z)
        local d_now = angle_between_deg(bz_now, bz_goal)
        local d_cap = angle_between_deg(bz_cap, bz_goal)
        gcs:send_text(6, string.format(
            "RAWES DBG cmd1: d_now=%.2fdeg d_cap=%.2fdeg cmd_rp=(%.2f,%.2f) now_rp=(%.2f,%.2f)",
            d_now, d_cap, roll_deg, pitch_deg, roll_now_deg, pitch_now_deg))
    end

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
    -- Rate-stability gate: fade the vz-damping term while body rates are high.
    local vz_gate = 1.0
    local gyro_b = ahrs:get_gyro()
    if gyro_b then
        local rate_mag = math.sqrt(gyro_b:x()*gyro_b:x()
                                 + gyro_b:y()*gyro_b:y()
                                 + gyro_b:z()*gyro_b:z())
        vz_gate = 1.0 - rate_mag / VZ_GATE_RATE_RADS
        if vz_gate < VZ_GATE_MIN then vz_gate = VZ_GATE_MIN end
        if vz_gate > 1.0 then vz_gate = 1.0 end
    end
    local col_pid = _col_trim + KP_ALT * alt_err - KD_VZ * vz_gate * vz_up + _alt_i
    -- Expected release transient: keep collective close to IC initially, then
    -- fade in altitude/vibration corrections over POST_RELEASE_RECOVERY_S.
    local _ic_col_now = ic_col_or_default()
    local col_cmd = _ic_col_now + recovery_alpha * (col_pid - _ic_col_now)

    -- Vibration damper: body-Z accel, HP filter, velocity estimate, collective.
    _vib_corr_last = 0.0
    local imu_a = ahrs:get_accel()
    if imu_a then
        _vib_corr_last = recovery_alpha * vib_damper_step(imu_a:z(), dt)
        col_cmd = col_cmd + _vib_corr_last
    end

    if col_cmd < COL_MIN_RAD then col_cmd = COL_MIN_RAD end
    if col_cmd > COL_MAX_RAD then col_cmd = COL_MAX_RAD end

    local col_delta = col_cmd - _last_col_rad
    if col_delta >  COL_SLEW_MAX then col_delta =  COL_SLEW_MAX end
    if col_delta < -COL_SLEW_MAX then col_delta = -COL_SLEW_MAX end
    _last_col_rad = _last_col_rad + col_delta

    local col_thrust = col_rad_to_thrust(_last_col_rad)

    -- GUIDED angle path: absolute roll/pitch + held yaw, zero feedforward rate,
    -- collective as direct throttle (set_throttle_out; no ch3 RC override).
    vehicle:set_target_angle_and_rate_and_throttle(
        roll_deg, pitch_deg, math.deg(yaw_now), 0.0, 0.0, 0.0, col_thrust)

    -- Diagnostic log (every ~5 s at 50 Hz)
    if _diag % 250 == 1 then
        local sub_info = ""
        if tlen then
            sub_info = string.format("  sub=%d  tlen=%.1f m", substate, tlen)
        end
        local d_roll = roll_deg - roll_now_deg
        local d_pitch = pitch_deg - pitch_now_deg
        gcs:send_text(6, string.format(
            "RAWES: cmd_rp=(%.1f,%.1f) now_rp=(%.1f,%.1f) d_rp=(%.1f,%.1f) thr=%.2f el=%.1f alt=%.1f%s",
            roll_deg, pitch_deg, roll_now_deg, pitch_now_deg,
            d_roll, d_pitch, col_thrust, math.deg(_el_rad), _target_alt, sub_info))
    end
end

-- ── RAWES_ARM: optional disarm timer (arming is handled by GCS) ─────────────

local function run_armon(now)
    local armon_ms = _nv_floats["RAWES_ARM"]
    if armon_ms and armon_ms > 0 then
        _nv_floats["RAWES_ARM"] = nil
        _armon_deadline_ms = now + armon_ms
        _armon_secs        = math.floor(armon_ms / 1000)
        if arming:is_armed() then
            gcs:send_text(6, string.format("RAWES disarm timer set: %ds", _armon_secs))
        else
            gcs:send_text(4, "RAWES disarm timer set while unarmed")
        end
    end

    if _armon_deadline_ms and arming:is_armed() and now >= _armon_deadline_ms then
        _armon_deadline_ms = nil
        arming:disarm()
        gcs:send_text(6, "RAWES disarm timer expired, disarmed")
    end
end

local function run_passive_mode(now)
    -- Armed-but-quiet: hold the IC operating point so the kinematic
    -- release transitions smoothly. Hold zero body-rate demand and
    -- pass IC collective through GUIDED throttle (no H_FLYBAR_MODE
    -- dependency).
    local ic_ready = (_ic_col ~= nil and _ic_roll_deg ~= nil and _ic_pitch_deg ~= nil)
    local col_thrust_p = col_rad_to_thrust(ic_col_or_default())
    local mode_now = vehicle:get_mode()
    if ic_ready and (mode_now == GUIDED_MODE_NUM or mode_now == 20) and ahrs:healthy() then
        vehicle:set_target_angle_and_rate_and_throttle(
            _ic_roll_deg, _ic_pitch_deg, math.deg(_passive_hold_yaw_rad or 0.0),
            0.0, 0.0, 0.0, col_thrust_p)
    end

    -- Take ownership of SERVO4 and pin the GB4008 at 800 us (ESC armed
    -- but motor off). Matches the safety-shutdown PWM used by
    -- calibrate.py so a PASSIVE session never spins the rotor.
    SRV_Channels:set_output_pwm_chan_timeout(SERVO4_CHAN, 800, 200)
    if now - _none_status_ms >= 5000 then
        _none_status_ms = now
        if ic_ready then
            gcs:send_text(6, string.format(
                "RAWES PASS IC: r=%.1f p=%.1f y=%.1f col=%.2fdeg thr=%.3f s4=800",
                _ic_roll_deg, _ic_pitch_deg, math.deg(_passive_hold_yaw_rad or 0.0),
                math.deg(_ic_col), col_thrust_p))
        else
            gcs:send_text(6, "RAWES PASS: waiting for IC seed (COL/RIC/PIC)")
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
    if _nv_floats["RAWES_ALT"] then _target_alt = _nv_floats["RAWES_ALT"] end
    if _nv_floats["RAWES_TEN"] then _tension_n  = _nv_floats["RAWES_TEN"] end

    -- IC seed handling: do not set active IC commands until all three
    -- (collective, roll, pitch) have been observed at least once.
    if _nv_floats["RAWES_COL"] then _ic_pending_col = _nv_floats["RAWES_COL"] end
    if _nv_floats["RAWES_RIC"] then _ic_pending_roll_deg = math.deg(_nv_floats["RAWES_RIC"]) end
    if _nv_floats["RAWES_PIC"] then _ic_pending_pitch_deg = math.deg(_nv_floats["RAWES_PIC"]) end

    if not _ic_seeded then
        if _ic_pending_col ~= nil and _ic_pending_roll_deg ~= nil and _ic_pending_pitch_deg ~= nil then
            _ic_col = _ic_pending_col
            _ic_roll_deg = _ic_pending_roll_deg
            _ic_pitch_deg = _ic_pending_pitch_deg
            _ic_seeded = true
            gcs:send_text(6, string.format(
                "RAWES IC seed set: r=%.1f p=%.1f col=%.2fdeg",
                _ic_roll_deg, _ic_pitch_deg, math.deg(_ic_col)))
        end
    else
        -- After initial atomic seed, accept incremental updates.
        if _nv_floats["RAWES_COL"] then _ic_col = _nv_floats["RAWES_COL"] end
        if _nv_floats["RAWES_RIC"] then _ic_roll_deg = math.deg(_nv_floats["RAWES_RIC"]) end
        if _nv_floats["RAWES_PIC"] then _ic_pitch_deg = math.deg(_nv_floats["RAWES_PIC"]) end
    end
    if _nv_floats["RAWES_TLN"]    then _man_tlon_rad = _nv_floats["RAWES_TLN"]    end
    if _nv_floats["RAWES_TLT"]    then _man_tlat_rad = _nv_floats["RAWES_TLT"]    end
    -- Ch4 yaw always neutral (no RC receiver; prevents yaw integrator wind-up)
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

    if mode == MODE_PASSIVE then
        run_passive_mode(now)
        return update, BASE_PERIOD_MS
    end

    if mode == MODE_STEADY then
        if now - _last_flight_ms >= FLIGHT_PERIOD_MS then
            _last_flight_ms = now
            run_flight()
        end
    end

    if mode == MODE_MANUAL then
        -- Legacy manual mode: yaw compensation via SERVO4 PID + NVF-commanded cyclic/collective.
        -- run_manual drives SERVO4 directly and sets RC1/RC2/RC3 via override.
        if _rc_ch8 then _rc_ch8:set_override(2000) end
        run_manual(BASE_PERIOD_MS * 0.001)
    end

    -- MODE_LANDING (4): not yet implemented

    return update, BASE_PERIOD_MS
end

-- ── Entry point ───────────────────────────────────────────────────────────────

local _mode_init  = math.floor(p("SCR_USER6", 0) + 0.5)
local _mode_names = {[0]="none", [1]="steady", [2]="manual", [4]="landing"}
local _mode_str   = _mode_names[_mode_init] or "unknown"

gcs:send_text(6, string.format(
    "RAWES: loaded  mode=%d (%s)  slew=%.2f  anchor=(%.1f %.1f %.1f)",
    _mode_init, _mode_str,
    p("SCR_USER2", 0.40),
    p("SCR_USER3", 0.0), p("SCR_USER4", 0.0), p("SCR_USER5", 0.0)))

-- @@UNIT_TEST_HOOK

return update, BASE_PERIOD_MS
