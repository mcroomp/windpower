--[[
rawes_flight.lua — RAWES orbit-tracking cyclic flight controller

Runs in ACRO mode (mode 1).  Injects RC overrides every 20 ms (50 Hz):
  Ch1  roll rate  — orbit tracking cyclic
  Ch2  pitch rate — orbit tracking cyclic
  Ch3  collective — forwarded from ground PI (via RC_CHANNELS_OVERRIDE Ch3)
                    or set_override() if ground sends SET_ATTITUDE_TARGET.thrust

rawes_yaw_trim.lua runs concurrently and owns Ch4 (counter-torque motor).

Parameters (set before flight — no firmware change needed):
  SCR_USER1   RAWES_KP_CYC       Cyclic P gain            [rad/s / rad]  default 1.0
  SCR_USER2   RAWES_BZ_SLEW      body_z slerp rate limit  [rad/s]        default 0.40
  SCR_USER3   RAWES_ANCHOR_N     Anchor North from EKF origin [m]         default 0.0
  SCR_USER4   RAWES_ANCHOR_E     Anchor East  from EKF origin [m]         default 0.0
  SCR_USER5   RAWES_ANCHOR_D     Anchor Down  from EKF origin [m]         default 0.0
  SCR_USER6   RAWES_MAX_CYC_DELTA Max cyclic PWM change per 20 ms step   default 30
                                  Limits swashplate slew rate regardless of error source.
                                  30 PWM/step = 1500 PWM/s (~0.67 s full-stick traverse).
                                  0 = disabled (no rate limiting).

ACRO_RP_RATE (ArduPilot parameter) sets the full-stick rate in deg/s.
ACRO_RP_RATE_DEG below must match the ArduPilot parameter value.

Deployment:
  Copy to APM/scripts/ on the Pixhawk SD card.
  Requires SCR_ENABLE = 1 in ArduPilot parameters.
--]]

local PERIOD_MS         = 20       -- 50 Hz
local ACRO_MODE_NUM     = 1        -- ACRO mode number (ArduCopter ACRO = 1)
local MIN_TETHER_M      = 0.5      -- minimum tether length before orbit tracking activates
local ACRO_RP_RATE_DEG  = 360.0    -- must match ACRO_RP_RATE ArduPilot parameter
local PLANNER_TIMEOUT   = 2000     -- ms: revert to natural orbit after this

-- ── Internal state ──────────────────────────────────────────────────────────

local _captured     = false     -- true once equilibrium has been snapped
local _bz_eq0       = nil       -- body_z_ned at equilibrium capture
local _tdir0        = nil       -- tether direction at equilibrium capture
local _bz_orbit     = nil       -- instantaneous orbit-tracked body_z setpoint
local _bz_slerp     = nil       -- rate-limited active setpoint (what cyclic tracks)
local _bz_target    = nil       -- planner override (nil = natural orbit)
local _plan_ms      = 0         -- millis() when _bz_target was last commanded
local _diag         = 0         -- diagnostic counter
local _prev_ch1     = 1500      -- last sent Ch1 PWM (for output rate limiting)
local _prev_ch2     = 1500      -- last sent Ch2 PWM (for output rate limiting)

-- Cache RC channel objects at module load (rc:get_channel() is the correct API)
local _rc_ch1 = rc:get_channel(1)   -- roll rate override
local _rc_ch2 = rc:get_channel(2)   -- pitch rate override

-- ── Helpers ─────────────────────────────────────────────────────────────────

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

-- Scalar multiply: returns v * s as a new Vector3f (v * s not overloaded here)
local function v3_scale(v, s)
    local r = Vector3f()
    r:x(v:x() * s)
    r:y(v:y() * s)
    r:z(v:z() * s)
    return r
end

-- Unit body-Z vector [0, 0, 1] (Vector3f() constructor ignores args in this build)
local function v3_body_z()
    local v = Vector3f()
    v:z(1.0)
    return v
end

-- Rodrigues rotation: rotate vector v around unit vector axis_n by angle radians
-- v' = v·cos(θ) + (axis×v)·sin(θ) + axis·(axis·v)·(1−cos(θ))
-- Uses only component arithmetic to avoid Vector3f operator overloading issues.
local function rodrigues(v, axis_n, angle)
    local ca  = math.cos(angle)
    local sa  = math.sin(angle)
    local vx, vy, vz     = v:x(),      v:y(),      v:z()
    local nx, ny, nz     = axis_n:x(), axis_n:y(), axis_n:z()
    -- axis × v
    local acx = ny*vz - nz*vy
    local acy = nz*vx - nx*vz
    local acz = nx*vy - ny*vx
    -- axis · v
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

-- ── Main update ─────────────────────────────────────────────────────────────

local function update()
    _diag = _diag + 1

    -- Only run in ACRO_Heli mode
    if vehicle:get_mode() ~= ACRO_MODE_NUM then
        return update, PERIOD_MS
    end

    -- Read attitude — use available API (no get_rotation_body_to_ned in this build)
    if not ahrs:healthy() then return update, PERIOD_MS end

    -- Read position (NED, relative to EKF origin)
    local pos_ned = ahrs:get_relative_position_NED_origin()
    if not pos_ned then return update, PERIOD_MS end

    -- Tether vector: use component arithmetic (vector - may not be overloaded)
    local anch = anchor_ned()
    local diff = Vector3f()
    diff:x(pos_ned:x() - anch:x())
    diff:y(pos_ned:y() - anch:y())
    diff:z(pos_ned:z() - anch:z())
    local tlen = diff:length()

    -- ── Equilibrium capture ──────────────────────────────────────────────

    if not _captured then
        if tlen >= MIN_TETHER_M then
            _bz_eq0     = ahrs:body_to_earth(v3_body_z())   -- body_z in NED
            _tdir0      = v3_normalize(diff)          -- tether direction in NED at capture
            _bz_orbit   = v3_copy(_bz_eq0)
            _bz_slerp   = v3_copy(_bz_eq0)
            _captured   = true
            gcs:send_text(6, string.format(
                "RAWES flight: captured  tlen=%.1f m  bz=(%.2f %.2f %.2f)",
                tlen, _bz_eq0:x(), _bz_eq0:y(), _bz_eq0:z()))
        end
        return update, PERIOD_MS
    end

    -- ── Orbit tracking (all vectors in NED) ─────────────────────────────
    -- Rotate _bz_eq0 by the same rotation that the tether has made since capture.
    -- This keeps body_z aligned with the natural tether direction as the hub orbits.

    if tlen >= MIN_TETHER_M then
        local bzt   = v3_normalize(diff)
        local axis  = _tdir0:cross(bzt)
        local sinth = axis:length()
        if sinth > 1e-6 then
            local costh = _tdir0:dot(bzt)
            _bz_orbit = rodrigues(_bz_eq0, v3_normalize(axis), math.atan(sinth, costh))
        end
        -- if sinth ≤ 1e-6 the tether hasn't moved: keep _bz_orbit = _bz_eq0 (unchanged)
    end

    -- ── Planner timeout ──────────────────────────────────────────────────

    if _bz_target and (millis() - _plan_ms) > PLANNER_TIMEOUT then
        _bz_target = nil
        gcs:send_text(6, "RAWES flight: planner timeout — reverting to orbit")
    end

    -- ── Rate-limited slerp ───────────────────────────────────────────────
    -- _bz_slerp converges toward _bz_target (reel-in tilt) or _bz_orbit (natural orbit).

    local bz_slew = p("SCR_USER2", 0.40)
    local dt      = PERIOD_MS * 0.001
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

    -- ── Cyclic P loop ────────────────────────────────────────────────────
    -- error = cross(body_z_now, _bz_slerp) in NED frame
    -- transform to body frame via ahrs:earth_to_body()

    local kp      = p("SCR_USER1", 1.0)
    local bz_now  = ahrs:body_to_earth(v3_body_z())
    local err_ned = bz_now:cross(_bz_slerp)

    local err_body = ahrs:earth_to_body(err_ned)
    local err_bx   = err_body:x()    -- body X (roll)
    local err_by   = err_body:y()    -- body Y (pitch)

    local roll_rads  = kp * err_bx
    local pitch_rads = kp * err_by

    -- Map rate to RC PWM.
    -- Full stick (±500 µs from 1500) = ±ACRO_RP_RATE_DEG deg/s
    local scale = 500.0 / (ACRO_RP_RATE_DEG * math.pi / 180.0)

    local ch1 = math.floor(1500.0 + scale * roll_rads  + 0.5)
    local ch2 = math.floor(1500.0 + scale * pitch_rads + 0.5)
    ch1 = math.max(1000, math.min(2000, ch1))
    ch2 = math.max(1000, math.min(2000, ch2))

    -- Output rate limiter: clamp per-step PWM change to RAWES_MAX_CYC_DELTA.
    -- Prevents sudden swashplate movements regardless of error source (attitude
    -- jitter, planner timeout, phase transitions).  0 = disabled.
    local max_delta = p("SCR_USER6", 30)
    if max_delta > 0 then
        local d1 = ch1 - _prev_ch1
        local d2 = ch2 - _prev_ch2
        if d1 >  max_delta then d1 =  max_delta end
        if d1 < -max_delta then d1 = -max_delta end
        if d2 >  max_delta then d2 =  max_delta end
        if d2 < -max_delta then d2 = -max_delta end
        ch1 = _prev_ch1 + d1
        ch2 = _prev_ch2 + d2
    end
    _prev_ch1 = ch1
    _prev_ch2 = ch2

    if _rc_ch1 then _rc_ch1:set_override(ch1) end
    if _rc_ch2 then _rc_ch2:set_override(ch2) end

    -- ── Diagnostics ──────────────────────────────────────────────────────

    if _diag % 250 == 1 then     -- every ~5 s
        local err_mag = math.sqrt(err_bx * err_bx + err_by * err_by)
        gcs:send_text(6, string.format(
            "RAWES: ch1=%d ch2=%d |err|=%.3f rad  slerp_to_goal=%.3f rad",
            ch1, ch2, err_mag, remain))
    end

    return update, PERIOD_MS
end

-- ── Entry point ─────────────────────────────────────────────────────────────

gcs:send_text(6, string.format(
    "RAWES flight: loaded  kp=%.2f  slew=%.2f rad/s  anchor=(%.1f %.1f %.1f) NED",
    p("SCR_USER1", 1.0), p("SCR_USER2", 0.40),
    p("SCR_USER3", 0.0), p("SCR_USER4", 0.0), p("SCR_USER5", 0.0)))

return update, PERIOD_MS
