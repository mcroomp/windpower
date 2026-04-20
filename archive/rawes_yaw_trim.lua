--[[
rawes_yaw_trim.lua  --  ARCHIVED: Lua PI yaw-trim controller for rawes.lua.

This code was the MODE_YAW_LUA (SCR_USER6=2) subsystem: a 100 Hz Lua PI
controller that drove the GB4008 counter-torque motor via Ch9 (SERVO9_FUNCTION=94).

Archived because the main approach switched to ArduPilot's built-in ACRO yaw
PID (ATC_RAT_YAW_P/I driving SERVO4 via H_TAIL_TYPE=4 DDFP CCW).  Lua now
only handles arming (RAWES_ARM state machine).

To restore: splice back into rawes.lua in place of -- @@ARCHIVEDYAW and
restore the required ArduPilot params:
  H_TAIL_TYPE=0     (disable DDFP so Lua owns Ch9)
  SERVO9_FUNCTION=94
  SERVO9_MIN=800, SERVO9_MAX=2000, SERVO9_TRIM=800
  SCR_USER6=2       (activate MODE_YAW_LUA)
--]]

-- ── Yaw-trim constants (were module-level locals in rawes.lua) ────────────────

-- local MODE_YAW_LUA    = 2   -- counter-torque yaw trim only

-- local BASE_THROTTLE_PCT = 5.0          -- base throttle [%]; I term compensates the rest
-- local KP_YAW        = 3.0              -- P gain [% / (rad/s)]
-- local KI_YAW        = 2.0              -- I gain [% / (rad/s * s)]
-- local YAW_I_MAX     = 80.0             -- anti-windup clamp [%]
-- local YAW_SRV_FUNC  = 94               -- Script 1 output (SERVO9)
-- local YAW_STABLE_RAD_S     = math.rad(5.0)
-- local YAW_STABLE_TIMEOUT_MS = 30000
-- local YAW_DEAD_ZONE_RAD_S  = math.rad(2.0)

-- State variables (were module-level locals):
-- local _yaw_not_stable_ms  = nil
-- local _yaw_stopped        = false
-- local _yaw_i              = 0.0
-- local _yaw_last_throttle  = 0.0
-- local _yaw_in_dead_zone   = true
-- local _yaw_status_ms      = 0

-- These lines were in _on_mode_enter():
--   _yaw_i              = 0.0
--   _yaw_last_throttle  = 0.0
--   _yaw_in_dead_zone   = true
--   _yaw_status_ms      = 0
--   _yaw_not_stable_ms  = nil

-- ── _set_throttle_pct ────────────────────────────────────────────────────────

--[[
local function _set_throttle_pct(pct)
    local pwm = math.floor(800.0 + pct * 12.0 + 0.5)
    SRV_Channels:set_output_pwm(YAW_SRV_FUNC, pwm)
end
--]]

-- ── run_yaw_trim ─────────────────────────────────────────────────────────────

--[[
local function run_yaw_trim()
    local now_ms = millis()

    if not arming:is_armed() then
        _yaw_last_throttle = 0.0; _yaw_in_dead_zone = true
        _set_throttle_pct(0); return
    end

    if _yaw_stopped then
        _yaw_last_throttle = 0.0
        _set_throttle_pct(0); return
    end

    local gyro_z = 0.0
    local gyro = ahrs:get_gyro()
    if gyro then gyro_z = gyro:z() end

    if math.abs(gyro_z) > YAW_STABLE_RAD_S then
        if _yaw_not_stable_ms == nil then _yaw_not_stable_ms = now_ms end
        if now_ms - _yaw_not_stable_ms >= YAW_STABLE_TIMEOUT_MS then
            _yaw_i = 0.0; _yaw_stopped = true; _yaw_last_throttle = 0.0
            _set_throttle_pct(0)
            gcs:send_text(0, "RAWES yaw: unstable 30s - motor off")
            arming:disarm(); return
        end
    else
        _yaw_not_stable_ms = nil
    end

    if gyro_z < -math.rad(30.0) then
        _yaw_i = 0.0; _yaw_last_throttle = 0.0
        _set_throttle_pct(0); return
    end

    local in_dead_zone = _yaw_last_throttle == 0.0 and math.abs(gyro_z) < YAW_DEAD_ZONE_RAD_S
    if in_dead_zone ~= _yaw_in_dead_zone then
        _yaw_in_dead_zone = in_dead_zone
        if in_dead_zone then
            gcs:send_text(6, string.format("RAWES yaw: dead zone  gyro=%.1f deg/s - motor off",
                math.deg(gyro_z)))
        else
            gcs:send_text(6, string.format("RAWES yaw: leaving dead zone  gyro=%.1f deg/s - PI starting",
                math.deg(gyro_z)))
        end
    end
    if in_dead_zone then
        _set_throttle_pct(0); return
    end

    _yaw_i = math.max(-YAW_I_MAX, math.min(YAW_I_MAX,
             _yaw_i + KI_YAW * gyro_z * 0.01))
    local throttle_pct = math.max(0.0, math.min(100.0,
        BASE_THROTTLE_PCT + KP_YAW * gyro_z + _yaw_i))
    _yaw_last_throttle = throttle_pct
    _set_throttle_pct(throttle_pct)

    if now_ms - _yaw_status_ms >= 5000 then
        _yaw_status_ms = now_ms
        gcs:send_text(6, string.format(
            "RAWES yaw: gyro=%.1f deg/s  thr=%.1f%%  I=%.1f%%",
            math.deg(gyro_z), throttle_pct, _yaw_i))
    end
end
--]]

-- ── update() invocation that was removed ─────────────────────────────────────

-- These lines were in update(), after run_armon():
--
--   -- Yaw-trim subsystem: runs every tick (100 Hz).
--   -- MODE_YAW_LUA (2) always runs yaw regardless of RAWES_YAW.
--   -- For all other modes: RAWES_YAW=1 NV float enables yaw trim alongside flight.
--   if mode == MODE_YAW_LUA or (_nv_floats["RAWES_YAW"] or 0) >= 1 then
--       run_yaw_trim()
--   end

-- ── Required ArduPilot parameters for MODE_YAW_LUA ───────────────────────────

--[[
H_TAIL_TYPE     = 0       -- disable DDFP on Ch4; Lua owns Ch9 entirely
SERVO9_FUNCTION = 94      -- Script 1: Lua writes motor command to output 9 (AUX OUT 1)
SERVO9_MIN      = 800     -- PWM off
SERVO9_MAX      = 2000    -- PWM full
SERVO9_TRIM     = 800     -- trim = off
SCR_USER6       = 2       -- RAWES_MODE = 2 (yaw_lua)
ATC_RAT_YAW_P   = 0.02   -- small P so ArduPilot doesn't fight Lua
--]]
