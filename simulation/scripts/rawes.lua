--[[
rawes.lua -- Unified RAWES flight controller
Works in both ArduPilot SITL (mcroomp fork) and on the Pixhawk 6C.

Mode is selected at runtime via SCR_USER6 (plain integer 0,1,5):
  0  none        -- script passive: no RC overrides; logs every 5 s + any NV message
  1  steady      -- cyclic altitude hold (Ch1, Ch2) + VZ collective PI             50 Hz
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
mavlink.init(1, 10)
mavlink.register_rx_msgid(_NVF_MSG_ID)

local _nv_floats = {}

-- ── Mode numbers ──────────────────────────────────────────────────────────────

local MODE_NONE    = 0
local MODE_STEADY  = 1
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

local COL_REEL_OUT      = -0.20   -- TensionPI warm-start; also reel-out equilibrium reference
local T_PUMP_TRANSITION =  3.7    -- ramp window used for logging/diagnostics [s]

-- TensionPI for pumping collective: feedback on _tension_n (from RAWES_TEN).
-- Mirrors Python controller.TensionPI defaults exactly.
local KP_TEN       = 5e-4    -- rad/N
local KI_TEN       = 1e-4    -- rad/(N·s)
local COL_MAX_TEN  = 0.0     -- TensionPI collective ceiling (neutral pitch)
local TEN_REEL_OUT = 435.0   -- N tension setpoint during hold/reel-out/transition-back
local TEN_REEL_IN  = 226.0   -- N tension setpoint during transition/reel-in

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

-- TensionPI integrator for pumping mode (warm-start maps to COL_REEL_OUT output)
local _col_i_ten    = COL_REEL_OUT / math.max(KI_TEN, 1e-12)

-- Altitude hold state
local _el_initialized = false   -- true once first GPS fix with tlen >= MIN_TETHER_M
local _el_rad         = 0.0     -- current rate-limited elevation angle [rad]
local _target_alt     = 0.0     -- target altitude [m]; updated from RAWES_ALT
local _tension_n      = 200.0   -- tether tension estimate [N]; updated from RAWES_TEN

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
    -- Tether direction at (el, az)
    local tx, ty, tz = cos_el * cos_az, cos_el * sin_az, -sin_el
    -- Elevation-upward tangent: d(tdir)/d(el)
    local ex, ey, ez = -sin_el * cos_az, -sin_el * sin_az, -cos_el
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

-- ── Mode-entry reset ─────────────────────────────────────────────────────────

local function _on_mode_enter(mode)
    _nv_floats      = {}   -- clear NV inbox so stale substates cannot bleed through
    _none_status_ms = 0
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

    -- Cyclic P loop
    local bz_goal = bz_altitude_hold(rel, _el_rad, _tension_n)
    local bz_now  = disk_normal_ned()
    local err     = cyclic_error_body(bz_now, bz_goal)
    local err_bx  = err[1]
    local err_by  = err[2]
    local kp      = p("SCR_USER1", 1.0)

    local ch1 = output_rate_limit(rate_to_pwm(kp * err_bx), _prev_ch1, 100)
    local ch2 = output_rate_limit(rate_to_pwm(kp * err_by), _prev_ch2, 100)
    _prev_ch1 = ch1
    _prev_ch2 = ch2
    if _rc_ch1 then _rc_ch1:set_override(ch1) end
    if _rc_ch2 then _rc_ch2:set_override(ch2) end

    -- ── Collective ────────────────────────────────────────────────────────
    local col_cmd
    if _is_pumping then
        -- TensionPI: feedback on _tension_n (from RAWES_TEN).
        -- High setpoint during reel-out (pay tether), low during reel-in (haul back).
        local ten_sp
        if substate == PUMP_HOLD or substate == PUMP_REEL_OUT
                or substate == PUMP_TRANSITION_BACK then
            ten_sp = TEN_REEL_OUT
        else   -- PUMP_TRANSITION, PUMP_REEL_IN
            ten_sp = TEN_REEL_IN
        end
        local ten_err = ten_sp - _tension_n
        local raw_pre = KP_TEN * ten_err + KI_TEN * _col_i_ten
        -- Conditional anti-windup (mirrors Python TensionPI)
        if not (raw_pre <= COL_MIN_RAD and ten_err < 0) then
            if not (raw_pre >= COL_MAX_TEN and ten_err > 0) then
                _col_i_ten = _col_i_ten + ten_err * dt
            end
        end
        col_cmd = KP_TEN * ten_err + KI_TEN * _col_i_ten
        if col_cmd > COL_MAX_TEN then col_cmd = COL_MAX_TEN end
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
    if _nv_floats["RAWES_ALT"] then _target_alt = _nv_floats["RAWES_ALT"] end
    if _nv_floats["RAWES_TEN"] then _tension_n  = _nv_floats["RAWES_TEN"] end

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

    if mode == MODE_STEADY or mode == MODE_PUMPING then
        if now - _last_flight_ms >= FLIGHT_PERIOD_MS then
            _last_flight_ms = now
            run_flight()
        end
    end
    -- MODE_LANDING (4): not yet implemented

    return update, BASE_PERIOD_MS
end

-- ── Entry point ───────────────────────────────────────────────────────────────

local _mode_init  = math.floor(p("SCR_USER6", 0) + 0.5)
local _mode_names = {[0]="none", [1]="steady", [4]="landing", [5]="pumping"}
local _mode_str   = _mode_names[_mode_init] or "unknown"

gcs:send_text(6, string.format(
    "RAWES: loaded  mode=%d (%s)  kp=%.2f  slew=%.2f  anchor=(%.1f %.1f %.1f)",
    _mode_init, _mode_str,
    p("SCR_USER1", 1.0), p("SCR_USER2", 0.40),
    p("SCR_USER3", 0.0), p("SCR_USER4", 0.0), p("SCR_USER5", 0.0)))

-- @@UNIT_TEST_HOOK

return update, BASE_PERIOD_MS
