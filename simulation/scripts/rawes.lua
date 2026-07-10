--[[
rawes.lua -- Unified RAWES flight controller
Works in both ArduPilot SITL (mcroomp fork) and on the Pixhawk 6C.

Mode is selected at runtime via SCR_USER6:
    0  none        -- script passive: no RC overrides; logs every 5 s + any NV message
    1  steady      -- primary guided flight path (set_target_rate_and_throttle)
    2  reserved    -- unused
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
    RAWES_SLW: elevation/body_z slew rate limit [rad/s]              default 0.40
    RAWES_ANN: anchor North from EKF origin [m]                      default 0.0
    RAWES_ANE: anchor East  from EKF origin [m]                      default 0.0
    RAWES_AND: anchor Down  from EKF origin [m]                      default 0.0
               The anchor is safety-critical: MODE_STEADY does NOT initialise
               altitude hold (and therefore commands no cyclic/collective) until
               all three anchor floats (ANN/ANE/AND) have been received at least
               once.  Until then run_flight only holds the current attitude.

Parameters:
  SCR_USER6   RAWES_MODE      Mode selector (0,1,3,4)                 default 0
  SCR_USER1   YAW_RPM_PER_US  Yaw motor slope [RPM/µs] (0=bench default 0.504)
--]]

-- ── Constants ─────────────────────────────────────────────────────────────────

BASE_PERIOD_MS    = 10        -- 100 Hz base tick
FLIGHT_PERIOD_MS  = 20        -- 50 Hz flight subsystem
GUIDED_MODE_NUM   = 4         -- ArduCopter GUIDED = 4
-- Smooth handoff after kinematic release: keep plant physics unchanged, but
-- phase in guidance/corrections over a longer window to avoid a command step.
POST_RELEASE_BLEND_S = 2.5    -- blend current->steady body_z after capture
POST_RELEASE_RECOVERY_S = 2.0 -- ramp-in for altitude corrections

_NVF_MSG_ID = 251
-- mavlink:init(queue_size, num_msgs).  queue_size = max messages buffered
-- between Lua ticks; with queue=1 (the old default) back-to-back NVFs from
-- the ground get dropped because only the first survives until update()
-- drains it.  20 is plenty for our ~5 named-floats / tick max rate.
mavlink.init(20, 10)
mavlink.register_rx_msgid(_NVF_MSG_ID)

_nv_floats = {}

-- ── Mode numbers ──────────────────────────────────────────────────────────────

MODE_NONE    = 0
MODE_STEADY  = 1
MODE_PASSIVE = 3   -- kinematic capture helper: hold the IC attitude stable during release.
MODE_LANDING = 4   -- reserved; not implemented here

-- MODE_PASSIVE IC seeds are provided over NVF (10-char names max):
--   RAWES_COL : IC collective [rad]
--   RAWES_RIC : IC roll       [rad]
--   RAWES_PIC : IC pitch      [rad]
-- They start nil and are committed atomically only after all three arrive.
--   RAWES_YIC : fixed yaw target [rad] (optional).  When present, PASSIVE holds
--               this absolute yaw instead of capturing the (spinning) AHRS yaw.

-- RAWES_SUB carries a generic substate index (delivered via NAMED_VALUE_FLOAT).
-- The ground pumping schedule runs in MODE_STEADY and uses RAWES_SUB only for
-- telemetry/diagnostics (0=hold 1=reel-out 2=transition 3=reel-in 4=transition-
-- back).  RAWES_SUB is also reserved for future landing/takeoff sequencing.

-- ── Physical constants ────────────────────────────────────────────────────────

MASS_KG  = 5.0
G_ACCEL  = 9.81

-- ── Safety: over-spin auto-disarm ─────────────────────────────────────────────
-- If the body spin rate about ANY axis exceeds SPIN_LIMIT_RPM continuously for
-- SPIN_LIMIT_MS, the vehicle auto-disarms.  Runs in EVERY mode while armed as a
-- last-resort protection against a runaway spin (yaw motor fault, EKF/attitude
-- divergence, etc.).  Any dip below the limit resets the timer.
SPIN_LIMIT_RPM  = 60.0                                   -- per-axis body rate cap [RPM]
SPIN_LIMIT_RADS = SPIN_LIMIT_RPM * 2.0 * math.pi / 60.0  -- = 2*pi ~ 6.283 rad/s (360 deg/s)
SPIN_LIMIT_MS   = 20000                                  -- sustained ms before disarm

-- ── GPS / attitude ──────────────────────────────────────────────────────────────────

MIN_TETHER_M     = 0.5        -- minimum tether length before GPS init activates

-- ── Collective limits and cruise values ───────────────────────────────────────

COL_MIN_RAD           = -0.28
COL_MAX_RAD           =  0.10
COL_SLEW_MAX          =  0.022   -- rad per 50 Hz step
COL_CRUISE_FLIGHT_RAD = -0.18    -- VZ integrator initial value (xi~8 deg, altitude hold)

-- ── Altitude collective + body-rate controller constants ─────────────────────

KP_ALT = 0.010
KI_ALT = 0.001
KD_VZ  = 0.040
-- Rate-stability gate on the collective vz-damping term.  While the attitude
-- loop is still slewing (flight start cyclic establishment, reel transitions),
-- the measured vertical velocity is partly rotational coupling rather than a
-- true altitude rate; reacting to it with collective injects a spurious thrust
-- (tether-tension) spike.  Scale KD_VZ down when body rates are elevated -- the
-- KP/KI altitude terms still hold altitude, only the derivative kick is gated.
VZ_GATE_RATE_RADS = 1.0          -- |gyro| at which the vz-damping fully fades
VZ_GATE_MIN       = 0.0          -- floor on the gate factor
RATE_KP_OUTER        = 2.5
RATE_ACCEL_MAX_RADSS = 4.0

-- Plane-keeping azimuth low-pass time constant [s].  The body-z azimuth
-- reference is slowly slewed toward the instantaneous position azimuth so fast
-- lateral excursions do not chase their own position (positive feedback that
-- pushes the kite off the downwind plane).  Plane-keeping ESTIMATE from the
-- kite's own position only -- no truth-wind oracle (AGENTS.md invariant).
AZ_REF_TAU_S = 15.0

-- ── Shared state ─────────────────────────────────────────────────────────────

_diag           = 0
_last_flight_ms = 0
_none_status_ms = 0
_passive_status_ms = 0

-- RAWES_ARM disarm timer
_armon_deadline_ms = nil
_armon_secs        = 0

-- Over-spin auto-disarm: millis() when the spin first exceeded SPIN_LIMIT_RADS
-- (nil while within limits or disarmed).
_spin_over_since_ms = nil

-- Mode / substate tracking
_prev_mode  = -1
_prev_sub   = 0
_mode_ms    = 0
_submode_ms = 0

-- Cached RC channel objects
_rc_ch1 = rc:get_channel(1)
_rc_ch2 = rc:get_channel(2)
_rc_ch3 = rc:get_channel(3)
_rc_ch4 = rc:get_channel(4)
_rc_ch8 = rc:get_channel(8)

-- Collective state
_last_col_rad = COL_CRUISE_FLIGHT_RAD
_col_trim     = COL_CRUISE_FLIGHT_RAD
_alt_i        = 0.0

-- Altitude hold state
_el_initialized = false   -- true once first GPS fix with tlen >= MIN_TETHER_M
_el_rad         = 0.0     -- current rate-limited elevation angle [rad]
_target_alt     = 0.0     -- target altitude [m]; updated from RAWES_ALT
_tension_n      = 200.0   -- target/feed-forward tether tension [N]; updated from RAWES_TEN
_az_ref         = 0.0     -- plane-keeping azimuth estimate [rad] (low-pass of position azimuth)
_az_initialized = false   -- true once _az_ref seeded from first GPS fix

-- Tuning + anchor, delivered via NAMED_VALUE_FLOAT (formerly SCR_USER2/3/4/5).
-- Mode is the ONLY remaining SCR_USER parameter (SCR_USER6).
_bz_slew         = 0.40    -- RAWES_SLW: elevation/body_z slew rate [rad/s]
_anchor_n        = 0.0     -- RAWES_ANN: anchor North from EKF origin [m]
_anchor_e        = 0.0     -- RAWES_ANE: anchor East  from EKF origin [m]
_anchor_d        = 0.0     -- RAWES_AND: anchor Down  from EKF origin [m]
_got_anchor_n    = false   -- true once RAWES_ANN has been received at least once
_got_anchor_e    = false   -- true once RAWES_ANE has been received at least once
_got_anchor_d    = false   -- true once RAWES_AND has been received at least once
_anchor_received = false   -- true once all three anchor floats have arrived

-- IC collective [rad] — ground sends via RAWES_COL.  Used by MODE_PASSIVE
-- to pin ch3 at the IC value so omega_spin doesn't droop while the body
-- is kinematically locked.  Defaults to the cruise-flight value.
_ic_col         = nil
_ic_roll_deg    = nil
_ic_pitch_deg   = nil
_ic_seeded      = false
_ic_pending_col = nil
_ic_pending_roll_deg = nil
_ic_pending_pitch_deg = nil

-- One-shot debug state for capture/first-command handoff diagnostics.
_dbg_cap_logged = false
_dbg_cmd_logged = false
_dbg_cap_bz_x   = 0.0
_dbg_cap_bz_y   = 0.0
_dbg_cap_bz_z   = 1.0
_capture_ms     = nil
_passive_hold_yaw_rad = nil
_passive_yaw_fixed_rad = nil   -- optional ground-provided fixed yaw target [rad] (RAWES_YIC)
_first_nonzero_rate_logged = false
_guided_cmd_last_log_ms = -2000
-- ── Yaw trim observer ────────────────────────────────────────────────────────────
-- Reads back the actual SERVO9 (Motor4) throttle u that ArduPilot applied,
-- computes the equilibrium trim that would hold psi_dot=0:
--   u_eq = u - psi_dot / YFF_A
-- and low-passes u_eq into H_YAW_TRIM (injected downstream of the AP clamps).
-- This absorbs both the AP yaw P-term DC and the rotor-reaction offset into a
-- single slowly-adapting trim; the P-term handles fast disturbances on top.
--
-- Plant model: psi_dot = YFF_A * (u - u_eq)
--   YFF_A = d(psi_dot)/d(u)  bench-calibrated from PWM->RPM curve (2026-07-07):
--     slope 0.504 RPM/us, SERVO9 span 1000 us  =>  YFF_A ≈ 52.8 rad/s per unit
local YFF_MAX                = 0.7     -- trim clamp upper bound [throttle fraction]
local YFF_TRIM_TAU           = 0.3     -- trim low-pass time constant [s]
-- SCR_USER1: yaw-motor slope override [RPM/µs].  0 → use bench default below.
-- Set to the measured d(shaft_RPM)/d(PWM_µs) for the installed motor + gear.
-- The simtest (test_yaw_regulation_lua.py) derives this from torque_model to
-- validate the closed-loop observer with a physically-honest plant.
local YAW_RPM_PER_US_DEFAULT = 0.504   -- bench-measured default
local YAW_RPM_PER_US         = YAW_RPM_PER_US_DEFAULT
local SERVO9_SPAN_US         = 1000.0  -- SERVO9_MAX - SERVO9_MIN
local YFF_A                  = YAW_RPM_PER_US * SERVO9_SPAN_US * (2.0 * math.pi / 60.0)
local YAW_MOTOR_FUNC   = 36      -- ArduPilot servo function for Motor4 (SERVO9)
local YFF_NVF_HZ       = 2.0     -- telemetry stream rate [Hz]

local _yaw_ff_trim     = 0.0     -- current H_YAW_TRIM value [0, YFF_MAX]
local _yaw_nvf_last_ms = nil     -- millis() at last YFF_* NVF emission
-- ── Helpers ───────────────────────────────────────────────────────────────────
-- Convert a millis() result to seconds (float).  On real ArduPilot millis()
-- returns a uint32_t userdata whose __mul/__div cannot coerce a fractional
-- scalar (multiplying by 0.001 raises "Unable to coerce to uint32_t"); the
-- unit-test mock returns an equivalent uint32_t table.  tofloat() exists on
-- both; only a plain Lua number takes the direct path.
local function ms_to_s(ms)
    if type(ms) == "number" then
        return ms * 0.001
    end
    return ms:tofloat() * 0.001
end
local function v3_copy(v)
    local r = Vector3f()
    r:x(v:x()); r:y(v:y()); r:z(v:z())
    return r
end

local function send_guided_angle_rate_throttle(roll_deg, pitch_deg, yaw_deg, roll_rate, pitch_rate, yaw_rate, throttle, src)
    local rr = roll_rate or 0.0
    local pr = pitch_rate or 0.0
    local yr = yaw_rate or 0.0
    local now_ms = millis()
    if now_ms - _guided_cmd_last_log_ms >= 2000 then
        _guided_cmd_last_log_ms = now_ms
        gcs:send_text(6, string.format(
            "RAWES guided cmd: src=%s rpy=(%.1f,%.1f,%.1f) rr/pr/yr=(%.4f,%.4f,%.4f) thr=%.3f",
            src or "unknown",
            roll_deg, pitch_deg, yaw_deg,
            rr, pr, yr,
            throttle))
    end
    if not _first_nonzero_rate_logged then
        if math.abs(rr) > 1.0e-6 or math.abs(pr) > 1.0e-6 or math.abs(yr) > 1.0e-6 then
            _first_nonzero_rate_logged = true
            gcs:send_text(6, string.format(
                "RAWES first non-zero rate cmd: t=%.2fs rr=%.5f pr=%.5f yr=%.5f src=%s",
                ms_to_s(now_ms),
                rr, pr, yr,
                src or "unknown"))
        end
    end
    vehicle:set_target_angle_and_rate_and_throttle(
        roll_deg, pitch_deg, yaw_deg,
        rr, pr, yr,
        throttle)
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

-- Apply SCR_USER1 slope override now that param:get is available.
-- SCR_USER1 > 0 → use as d(shaft_RPM)/d(PWM_µs) in place of the bench default.
do
    local _s = p("SCR_USER1", 0)
    if _s ~= nil and _s > 0 then
        YAW_RPM_PER_US = _s
        YFF_A          = YAW_RPM_PER_US * SERVO9_SPAN_US * (2.0 * math.pi / 60.0)
    end
end

local function anchor_ned()
    -- Anchor (EKF frame) is delivered via NAMED_VALUE_FLOAT RAWES_ANN/ANE/AND,
    -- latched into _anchor_n/e/d.  MODE_STEADY gates altitude-hold init on
    -- _anchor_received so this is only consulted once all three have arrived.
    local a = Vector3f()
    a:x(_anchor_n)
    a:y(_anchor_e)
    a:z(_anchor_d)
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

-- ── Mode-entry reset ─────────────────────────────────────────────────────────

local function _on_mode_enter(mode)
    _nv_floats      = {}   -- clear NV inbox so stale substates cannot bleed through
    _none_status_ms = 0
    if mode == MODE_STEADY then
        _dbg_cap_logged = false
        _dbg_cmd_logged = false
        -- Start a fresh post-release recovery window at steady entry.
        _capture_ms = millis()
        local _thr_ic = col_rad_to_thrust(ic_col_or_default())
        local _col_back = thrust_to_col_rad(_thr_ic)
        gcs:send_text(6, string.format(
            "RAWES steady: IC col=%.2fdeg -> thr=%.3f -> col=%.2fdeg",
            math.deg(ic_col_or_default()), _thr_ic, math.deg(_col_back)))
    end
    if mode == MODE_PASSIVE then
        _passive_status_ms    = 0
        _passive_hold_yaw_rad = nil
        _yaw_ff_trim          = 0.0
        _yaw_nvf_last_ms      = nil
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

        send_guided_angle_rate_throttle(
            math.deg(roll_now), math.deg(pitch_now), math.deg(yaw_now),
            0.0, 0.0, 0.0, ct,
            "pre_capture_hold")

        -- Check for GPS position; initialize altitude hold on first valid fix.
        -- Gate on _anchor_received: the anchor arrives via NAMED_VALUE_FLOAT
        -- (RAWES_ANN/ANE/AND), so do NOT capture geometry (and start commanding
        -- cyclic/collective) until all three have been received -- otherwise a
        -- zero/stale anchor would put the anchor directly below the hub and
        -- drive a saturated cyclic.  Until then we keep holding current attitude.
        local pos_ned = ahrs:get_relative_position_NED_origin()
        if _anchor_received and pos_ned then
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
    local bz_slew   = _bz_slew
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
        local t_rel_s = ms_to_s(now_ms - _capture_ms)
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
    -- fade in altitude corrections over POST_RELEASE_RECOVERY_S.
    local _ic_col_now = ic_col_or_default()
    local col_cmd = _ic_col_now + recovery_alpha * (col_pid - _ic_col_now)

    if col_cmd < COL_MIN_RAD then col_cmd = COL_MIN_RAD end
    if col_cmd > COL_MAX_RAD then col_cmd = COL_MAX_RAD end

    local col_delta = col_cmd - _last_col_rad
    if col_delta >  COL_SLEW_MAX then col_delta =  COL_SLEW_MAX end
    if col_delta < -COL_SLEW_MAX then col_delta = -COL_SLEW_MAX end
    _last_col_rad = _last_col_rad + col_delta

    local col_thrust = col_rad_to_thrust(_last_col_rad)

    -- GUIDED angle path: absolute roll/pitch + held yaw, zero feedforward rate,
    -- collective as direct throttle (set_throttle_out; no ch3 RC override).
    send_guided_angle_rate_throttle(
        roll_deg, pitch_deg, math.deg(yaw_now), 0.0, 0.0, 0.0, col_thrust,
        "steady_attitude")

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

-- ── Over-spin auto-disarm safety (runs in every mode) ──────────────────────────
-- Disarm if |body rate| about any axis stays above SPIN_LIMIT_RADS for longer
-- than SPIN_LIMIT_MS.  The timer latches on first exceedance and resets on any
-- dip below the limit, so only a SUSTAINED overspin trips it.
local function run_spin_safety(now)
    if not arming:is_armed() then
        _spin_over_since_ms = nil
        return
    end
    local gyro = ahrs:get_gyro()
    if not gyro then return end
    local over = (math.abs(gyro:x()) > SPIN_LIMIT_RADS
               or math.abs(gyro:y()) > SPIN_LIMIT_RADS
               or math.abs(gyro:z()) > SPIN_LIMIT_RADS)
    if not over then
        _spin_over_since_ms = nil
        return
    end
    if _spin_over_since_ms == nil then
        _spin_over_since_ms = now
        gcs:send_text(4, string.format(
            "RAWES SPIN WARN: >%.0f RPM about an axis; auto-disarm in %ds if sustained",
            SPIN_LIMIT_RPM, math.floor(SPIN_LIMIT_MS / 1000)))
    elseif now - _spin_over_since_ms >= SPIN_LIMIT_MS then
        _spin_over_since_ms = nil
        arming:disarm()
        gcs:send_text(3, string.format(
            "RAWES SAFETY: spin >%.0f RPM for %ds -- AUTO-DISARMED",
            SPIN_LIMIT_RPM, math.floor(SPIN_LIMIT_MS / 1000)))
    end
end

-- ── Yaw trim observer: model-based servo-readback ───────────────────────────
-- Pure step function (no sensor/param access): given applied throttle u [0,1]
-- and measured body yaw rate psi_dot [rad/s], advance the trim state and return
-- the new H_YAW_TRIM.  Unit-testable in isolation.
local function yaw_trim_step(dt, u, psi_dot)
    local trim_target = u - psi_dot / YFF_A
    if trim_target < 0.0 then trim_target = 0.0
    elseif trim_target > YFF_MAX then trim_target = YFF_MAX end
    local alpha = dt / (YFF_TRIM_TAU + dt)
    _yaw_ff_trim = _yaw_ff_trim + alpha * (trim_target - _yaw_ff_trim)
    if _yaw_ff_trim < 0.0 then _yaw_ff_trim = 0.0
    elseif _yaw_ff_trim > YFF_MAX then _yaw_ff_trim = YFF_MAX end
    return _yaw_ff_trim
end

local function run_yaw_trim(now)
    if not (_ic_seeded and arming:is_armed()) then return end
    local gyro = ahrs:get_gyro()
    if not gyro then return end
    -- Read the total applied throttle from the SERVO9 output (H_YAW_TRIM +
    -- any AP yaw P-term).  Returns nil until AP has written its first output.
    local pwm = SRV_Channels:get_output_pwm(YAW_MOTOR_FUNC)
    if pwm == nil then return end
    local smin = p("SERVO9_MIN", 1000)
    local smax = p("SERVO9_MAX", 2000)
    local span = smax - smin
    if span <= 0 then return end
    local u = (pwm - smin) / span
    if u < 0.0 then u = 0.0 elseif u > 1.0 then u = 1.0 end
    local psi_dot = gyro:z()
    yaw_trim_step(BASE_PERIOD_MS * 0.001, u, psi_dot)
    param:set("H_YAW_TRIM", _yaw_ff_trim)
    if _yaw_nvf_last_ms == nil or (now - _yaw_nvf_last_ms) >= (1000.0 / YFF_NVF_HZ) then
        _yaw_nvf_last_ms = now
        gcs:send_named_float("YFF_T",  _yaw_ff_trim)
        gcs:send_named_float("YFF_U",  u)
        gcs:send_named_float("YFF_GZ", psi_dot)
    end
end

local function run_passive_mode(now)
    -- Armed-but-quiet: hold the IC operating point so the kinematic
    -- release transitions smoothly. Hold zero body-rate demand and
    -- pass IC collective through GUIDED throttle (no H_FLYBAR_MODE
    -- dependency).
    local ic_ready = (_ic_col ~= nil and _ic_roll_deg ~= nil and _ic_pitch_deg ~= nil)

    local mode_now  = vehicle:get_mode()
    local guided_ok = (mode_now == GUIDED_MODE_NUM or mode_now == 20) and ahrs:healthy()

    if now - _passive_status_ms >= 1000 then
        _passive_status_ms = now
        local armed_s = arming:is_armed() and "ARMED" or "disarmed"
        local ic_s = ic_ready and "ready" or "waiting"
        local g_s = guided_ok and "ok" or "no"
        local y_s = (_passive_hold_yaw_rad ~= nil) and string.format("%.1f", math.deg(_passive_hold_yaw_rad)) or "n/a"
        local col_s = string.format("%.2f", math.deg(ic_col_or_default()))
        gcs:send_text(6, string.format(
            "RAWES passive: %s ic=%s guided=%s yaw_hold=%s col=%sdeg",
            armed_s, ic_s, g_s, y_s, col_s))
    end

    if not ic_ready then
        -- No IC seed yet: instead of sitting idle, actively damp any rotation
        -- to a standstill using the rate-only GUIDED API.  Commanding zero body
        -- rates lets ArduPilot's rate controller null the rotor-reaction /
        -- disturbance spin while we wait for the ground to send the IC seed.
        -- Gated on armed so we never emit control-API traffic during the pre-arm
        -- kinematic phase (motors are off when disarmed anyway).  Uses a default
        -- collective so rotor RPM is preserved.
        if guided_ok and arming:is_armed() then
            local col_thrust_d = col_rad_to_thrust(ic_col_or_default())
            vehicle:set_target_rate_and_throttle(0.0, 0.0, 0.0, col_thrust_d)
        end
        return
    end

    -- Yaw target: a ground-provided fixed yaw (RAWES_YIC) takes precedence and
    -- is held as an absolute setpoint, so PASSIVE does not chase the spinning
    -- AHRS yaw.  Without it, fall back to capturing the AHRS yaw once on entry.
    if _passive_yaw_fixed_rad ~= nil then
        _passive_hold_yaw_rad = _passive_yaw_fixed_rad
    elseif _passive_hold_yaw_rad == nil then
        if not ahrs:healthy() then
            return
        end
        local y = ahrs:get_yaw_rad()
        if y == nil then
            return
        end
        _passive_hold_yaw_rad = y
    end

    local col_thrust_p = col_rad_to_thrust(ic_col_or_default())
    if guided_ok then
        send_guided_angle_rate_throttle(
            _ic_roll_deg, _ic_pitch_deg, math.deg(_passive_hold_yaw_rad or 0.0),
            0.0, 0.0, 0.0, col_thrust_p,
            "passive_hold")
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
    -- Elevation/body_z slew rate + anchor position (formerly SCR_USER2/3/4/5).
    -- The anchor is safety-critical: run_flight does not initialise altitude
    -- hold until all three anchor floats have been received (see the capture
    -- gate below), so a stale/zero anchor never drives a cyclic command.
    if _nv_floats["RAWES_SLW"] then _bz_slew = _nv_floats["RAWES_SLW"] end
    if _nv_floats["RAWES_ANN"] then _anchor_n = _nv_floats["RAWES_ANN"]; _got_anchor_n = true end
    if _nv_floats["RAWES_ANE"] then _anchor_e = _nv_floats["RAWES_ANE"]; _got_anchor_e = true end
    if _nv_floats["RAWES_AND"] then _anchor_d = _nv_floats["RAWES_AND"]; _got_anchor_d = true end
    if (not _anchor_received) and _got_anchor_n and _got_anchor_e and _got_anchor_d then
        _anchor_received = true
        gcs:send_text(6, string.format(
            "RAWES anchor set: N=%.1f E=%.1f D=%.1f", _anchor_n, _anchor_e, _anchor_d))
    end
    -- IC seed handling: do not set active IC commands until all three
    -- (collective, roll, pitch) have been observed at least once.
    if _nv_floats["RAWES_COL"] then _ic_pending_col = _nv_floats["RAWES_COL"] end
    if _nv_floats["RAWES_RIC"] then _ic_pending_roll_deg = math.deg(_nv_floats["RAWES_RIC"]) end
    if _nv_floats["RAWES_PIC"] then _ic_pending_pitch_deg = math.deg(_nv_floats["RAWES_PIC"]) end
    -- Optional fixed yaw target for MODE_PASSIVE.  When present, PASSIVE holds
    -- this absolute yaw instead of capturing (and holding) the spinning AHRS yaw.
    if _nv_floats["RAWES_YIC"] then _passive_yaw_fixed_rad = _nv_floats["RAWES_YIC"] end

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
    -- RAWES_ARM disarm timer must always run, regardless of mode/IC seed.  In
    -- particular, MODE_PASSIVE before full IC seed still needs the safety timer
    -- to expire and disarm if requested by the ground.
    run_armon(now)

    -- Over-spin auto-disarm: last-resort protection in EVERY mode.
    run_spin_safety(now)

    -- Ch4 yaw always neutral (no RC receiver; prevents yaw integrator wind-up).
    -- In PASSIVE before full IC seed, avoid all non-essential control API calls.
    if mode ~= MODE_PASSIVE or _ic_seeded then
        if _rc_ch4 then _rc_ch4:set_override(1500) end
    end

    -- Latch the motor interlock (CH8) high while armed so the heli rotor stays
    -- runup-complete and ArduPilot clears land_complete (otherwise GUIDED angle
    -- commands are silently dropped by angle_control_run's takeoff/relax branch).
    -- Scripting RC overrides expire after RC_OVERRIDE_TIME, so this must be
    -- re-asserted every tick.  Gated on armed only (NOT on IC seed): CH8 high
    -- before arm blocks arming ("Motor Interlock Enabled"), but the rotor must
    -- run up through the whole kinematic phase so land_complete is already
    -- cleared at release.
    if _rc_ch8 and arming:is_armed() then _rc_ch8:set_override(2000) end

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
        run_yaw_trim(now)
        return update, BASE_PERIOD_MS
    end

    if mode == MODE_STEADY then
        run_yaw_trim(now)
        if now - _last_flight_ms >= FLIGHT_PERIOD_MS then
            _last_flight_ms = now
            run_flight()
        end
    end

    -- MODE_LANDING (4): not yet implemented

    return update, BASE_PERIOD_MS
end

-- ── Entry point ───────────────────────────────────────────────────────────────

_mode_init  = math.floor(p("SCR_USER6", 0) + 0.5)
_mode_names = {[0]="none", [1]="steady", [3]="passive", [4]="landing"}
_mode_str   = _mode_names[_mode_init] or "unknown"

gcs:send_text(6, string.format(
    "RAWES: loaded  mode=%d (%s)  (slew+anchor via NAMED_VALUE_FLOAT)",
    _mode_init, _mode_str))

-- @@UNIT_TEST_HOOK

return update, BASE_PERIOD_MS
