--[[
rawes.lua -- Unified RAWES flight + yaw-trim controller
Works in both ArduPilot SITL (mcroomp fork) and on the Pixhawk 6C.

Mode is selected at runtime via SCR_USER6 (plain integer 0,1,2,4,5):
  0  none        -- script passive: no RC overrides, no keepalive; logs every 5 s + any NV message
  1  steady      -- cyclic orbit-tracking only (Ch1, Ch2)                  50 Hz
  2  (reserved)
  3  (reserved)
  4  landing     -- cyclic + VZ collective + auto final_drop               50 Hz
  5  pumping     -- De Schutter pumping cycle                              50 Hz

RAWES_ARM: arm vehicle and set a timed disarm countdown.
  Send NAMED_VALUE_FLOAT("RAWES_ARM", ms) to arm and start a countdown of <ms> milliseconds.
  When the countdown expires the vehicle is disarmed and a STATUSTEXT is sent.
  Re-sending refreshes the countdown.  Works in any mode.

Substate is delivered via NAMED_VALUE_FLOAT("RAWES_SUB", N) -- not encoded in SCR_USER6.
  Landing substates  (mode=4): 0=descend, 1=final_drop
  Pumping substates  (mode=5): 0=hold, 1=reel_out, 2=transition, 3=reel_in, 4=transition_back
The _nv_floats table is reset to {} on every mode change so stale values cannot bleed through.

Parameters (SCR_USER1..6):
  SCR_USER1   RAWES_KP_CYC    Cyclic P gain            [rad/s / rad]  default 1.0
  SCR_USER2   RAWES_BZ_SLEW   body_z slerp rate limit  [rad/s]        default 0.40
  SCR_USER3   RAWES_ANCHOR_N  Anchor North from EKF origin [m]         default 0.0
  SCR_USER4   RAWES_ANCHOR_E  Anchor East  from EKF origin [m]         default 0.0
  SCR_USER5   RAWES_ANCHOR_D  Anchor Down  from EKF origin [m]         default 0.0
  SCR_USER6   RAWES_MODE      Mode selector (0,1,2,4,5 -- see above)   default 0

All pumping cycle parameters (phase timing, xi angle, collective limits) are
hardcoded constants below -- no free SCR_USER slots remain.

Cyclic output rate limiter is hardcoded at 30 PWM/step (0.67 s full-stick traverse).

No RC receiver: all channel overrides (Ch1, Ch2, Ch3) are owned entirely by Lua.
  The ground planner communicates via SCR_USER params only (MAVLink params, not RC).
  vz_setpoint is hardcoded per mode (landing: 0.5 m/s; flight: 0.0 m/s hold).

Division of labour (pumping mode):
  Pixhawk/Lua (50 Hz): cyclic orbit tracking + body_z slerp + per-phase collective
  Ground planner:      phase state machine (winch), TensionPI, wind estimation
  Synchronisation:     Lua detects tether paying-out / reeling-in to follow the
                       ground planner's actual winch motion -- no RC channel bridge needed.

Yaw-trim control (PI on gyro_z):
  BASE_THROTTLE_PCT  5 %            -- floor; DShot arms on first frame
  KP_YAW             3.0 %/(rad/s) -- proportional; provides motor torque during spin-up
  KI_YAW             2.0 %/(rad/s*s) -- integral; accumulates ~48.5% back-EMF equilibrium offset
  YAW_I_MAX          80 %           -- anti-windup; well above back-EMF equilibrium ~48.5%

Required ArduPilot parameters:
  SCR_ENABLE        1    -- reboot required after first set
  SERVO9_FUNCTION   94   -- Script 1: Lua writes motor command to output 9 (AUX OUT 1)
  SERVO_BLH_MASK    256  -- DShot enabled on output 9 (bit 8)
  SERVO_BLH_OTYPE   5    -- DShot300 (Heli/Copter frame)
  SERVO_BLH_POLES   22   -- GB4008 24N22P (11 pole-pairs; default 14 is wrong)
  SERVO_DSHOT_ESC   3    -- AM32 ESC type
  SERVO_BLH_BDMASK  0    -- one-way DShot; set to 256 only after AM32 EDT enabled on ESC

Deployment:
  Copy rawes.lua to APM/scripts/ on the Pixhawk SD card.
  Set SCR_USER6 to activate the desired mode.
--]]

-- ── Shared constants ─────────────────────────────────────────────────────────

local BASE_PERIOD_MS    = 10        -- 100 Hz base tick (yaw trim native rate)
local FLIGHT_PERIOD_MS  = 20        -- 50 Hz  flight subsystem
local ACRO_MODE_NUM     = 1         -- ACRO mode number (ArduCopter ACRO = 1)

-- ── MAVLink named-float receive ───────────────────────────────────────────────
-- NAMED_VALUE_FLOAT (msgid 251) is the sole runtime signalling channel from the
-- ground planner to Lua (SCR_USER params are boot-time only).
-- All modes including mode 0 drain this inbox every tick.

local _NVF_MSG_ID = 251   -- NAMED_VALUE_FLOAT

mavlink.init(1, 10)
mavlink.register_rx_msgid(_NVF_MSG_ID)

local _nv_floats = {}     -- received named float values:  name -> value

-- ── Mode numbers (SCR_USER6 = plain integer 0,1,2,4,5) ──────────────────────
-- Ground planner writes SCR_USER6 = mode.
-- Substate is delivered separately via NAMED_VALUE_FLOAT("RAWES_SUB", N).
-- Yaw trim alongside a flight mode is enabled by NAMED_VALUE_FLOAT("RAWES_YAW", 1).
-- Timed arm/disarm is controlled by NAMED_VALUE_FLOAT("RAWES_ARM", ms).
local MODE_NONE       = 0
local MODE_STEADY     = 1   -- cyclic orbit-tracking
-- modes 2,3 reserved
local MODE_LANDING    = 4   -- cyclic + VZ descent + auto final_drop
local MODE_PUMPING    = 5   -- De Schutter pumping cycle

-- ── Landing substates (mode=4; sent via NAMED_VALUE_FLOAT "RAWES_SUB") ────────
local LAND_DESCEND    = 0   -- VZ controller active; ground planner reels in tether
local LAND_FINAL_DROP = 1   -- collective → 0, neutral cyclic, drop to floor

-- ── Pumping substates (mode=5; sent via NAMED_VALUE_FLOAT "RAWES_SUB") ────────
-- Ground planner owns the phase state machine; Lua executes the correct
-- collective and body_z slerp goal for each substate.
--
-- Transition timing mirrors DeschutterPlanner:
--   TRANSITION      starts when winch REVERSES (at T_reel_out), lasts T_TRANSITION.
--   TRANSITION_BACK starts at cycle start for cycles >0, lasts T_TRANSITION.
-- Collective ramps are synchronised with body_z slerp via _submode_ms timer.
local PUMP_HOLD            = 0   -- waiting; reel-out collective, orbit tracking
local PUMP_REEL_OUT        = 1   -- paying out; reel-out collective, orbit tracking
local PUMP_TRANSITION      = 2   -- winch just reversed; col ramps to reel-in, bz slews to xi=80
local PUMP_REEL_IN         = 3   -- reeling in; reel-in collective, body_z at xi=80 deg
local PUMP_TRANSITION_BACK = 4   -- winch just started out; col ramps back, bz slews to orbit

-- ── Flight subsystem constants ───────────────────────────────────────────────

local MIN_TETHER_M      = 0.5       -- minimum tether length before orbit tracking activates
local ACRO_RP_RATE_DEG  = 360.0     -- must match ACRO_RP_RATE ArduPilot parameter
local PLANNER_TIMEOUT   = 2000      -- ms: revert to natural orbit after this

-- ── VZ collective controller ─────────────────────────────────────────────────
-- Pixhawk-side PI VZ controller for all non-pumping modes.  Runs at 50 Hz
-- inside run_flight().  The integrator starts at col_cruise (mode-appropriate
-- value, set at capture) and winds to the actual trim collective regardless of
-- how well col_cruise is calibrated.
--
-- vz_setpoint (hardcoded per mode; no RC receiver in this system):
--   landing (mode 4): VZ_LAND_SP = 0.5 m/s descent
--   cruise  (mode 1): 0.0 m/s altitude hold
-- pumping (mode 5): open-loop per phase; this controller is bypassed.

local KP_VZ               = 0.05   -- collective gain [rad/(m/s)]
local KI_VZ               = 0.005  -- collective integrator gain [rad/(m/s)/s]
                                    -- winds up from COL_CRUISE_LAND_RAD to the actual
                                    -- trim collective, so COL_CRUISE_LAND_RAD need not be exact.
local COL_CRUISE_FLIGHT_RAD = -0.18  -- VZ integrator initial value for cruise (xi~8 deg, 100 m tether)
local COL_CRUISE_LAND_RAD   =  0.079 -- VZ integrator initial value for landing (xi~80 deg)
local COL_MIN_RAD    = -0.28   -- hardware collective floor  [rad]
local COL_MAX_RAD    =  0.10   -- hardware collective ceiling [rad]
-- DS113MG at 6 V: 545 deg/s over 100 deg travel.
-- max_col_rate = 2 * 545/100 * swashplate_pitch_gain(0.10) = 1.09 rad/s
-- At 50 Hz (0.020 s/step): max_col_step = 1.09 * 0.020 = 0.022 rad/step
local COL_SLEW_MAX   = 0.022   -- rad per 50 Hz step

local VZ_LAND_SP     = 0.5     -- landing descent rate [m/s] (positive NED = down)

-- ── Pumping cycle constants ───────────────────────────────────────────────────
-- Mirrors config.py DEFAULTS["trajectory"]["deschutter"].
-- No SCR_USER slots available; all values hardcoded here.
--
-- Collective strategy (open-loop per phase):
--   reel_out: col near equilibrium so TensionPI (ground) controls tension via winch
--   reel_in:  col_min_reel_in_rad = altitude-neutral at xi=80 deg (keeps hub aloft)
--
-- Phase synchronisation: ground planner owns the state machine and sends
--   NAMED_VALUE_FLOAT("RAWES_SUB", N) to signal each phase transition.
--   Lua executes the correct collective and body_z slerp for each substate.

local XI_REEL_IN_DEG  = 80.0   -- reel-in disk tilt from wind direction [deg]
local COL_REEL_OUT    = -0.20  -- collective during hold/reel-out [rad]
local COL_REEL_IN     =  0.079 -- collective during transition/reel-in [rad]
local T_TRANSITION    =  3.7   -- body_z slerp window [s] (from config default)

-- ── Shared state ─────────────────────────────────────────────────────────────

local _diag             = 0         -- global diagnostic counter (every tick)
local _last_flight_ms   = 0         -- millis() when flight subsystem last ran
local _none_status_ms   = 0         -- millis() of last mode-0 periodic status log
-- RAWES_ARM state machine.
-- States: nil=inactive  "interlock_low"=Ch8 set LOW, waiting 1 tick before arm attempt
--         "arming"=retrying arming:arm() each tick until is_armed()
--         "armed"=armed, Ch8 HIGH, holding overrides until deadline
local _armon_deadline_ms = nil      -- millis() deadline for RAWES_ARM disarm; nil = inactive
local _armon_state       = nil      -- current RAWES_ARM state (see above)
local _armon_armed_sent  = false    -- true once "RAWES arm-on: armed" STATUSTEXT has been sent
local _armon_secs        = 0        -- integer seconds from last RAWES_ARM (for confirmation log)

-- Mode tracking (SCR_USER6); substate via NAMED_VALUE_FLOAT "RAWES_SUB"
local _prev_mode        = -1        -- last decoded mode number (-1 = uninitialised)
local _prev_sub         = 0         -- last decoded substate (from _nv_floats["RAWES_SUB"])
local _mode_ms          = 0         -- millis() when current mode was entered
local _submode_ms       = 0         -- millis() when current substate was entered

-- Cache RC channel objects at module load (rc:get_channel() is the correct API)
local _rc_ch1 = rc:get_channel(1)   -- roll rate override (flight)
local _rc_ch2 = rc:get_channel(2)   -- pitch rate override (flight)
local _rc_ch3 = rc:get_channel(3)   -- collective override (vz controller)
local _rc_ch4 = rc:get_channel(4)   -- yaw rate override (always neutral; no RC receiver)
local _rc_ch8 = rc:get_channel(8)   -- motor interlock keepalive (H_RSC_MODE=1 passthrough)

-- ── Flight subsystem state ───────────────────────────────────────────────────

local _last_col_rad     = COL_CRUISE_FLIGHT_RAD  -- collective slew state [rad]
local _col_i            = COL_CRUISE_FLIGHT_RAD  -- VZ integrator state [rad]; reset to col_cruise at capture
local _captured         = false           -- true once equilibrium has been snapped
local _bz_eq0           = nil             -- body_z_ned at equilibrium capture (from DCM)
local _tdir0            = nil             -- tether direction at equilibrium capture
local _bz_orbit         = nil             -- instantaneous orbit-tracked body_z setpoint
local _bz_slerp         = nil             -- rate-limited active setpoint (what cyclic tracks)
local _bz_target        = nil             -- external planner override (nil = natural orbit)
local _plan_ms          = 0               -- millis() when _bz_target was last commanded
local _prev_ch1         = 1500            -- last sent Ch1 PWM (for output rate limiting)
local _prev_ch2         = 1500            -- last sent Ch2 PWM (for output rate limiting)

-- Pumping state (reset on mode entry; substate + timers owned by ground planner)
local _pump_bz_ri       = nil     -- reel-in body_z setpoint (computed internally from tether dir)

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

-- Return body_z in NED: body_to_earth([0,0,1]) = R[:,2] = rotor axis in world frame.
-- Positive convention matches Python controller (R[:,2]).
-- Cross-product error is sign-consistent because both bz_now and _bz_orbit
-- use the same positive convention.
local function disk_normal_ned()
    return ahrs:body_to_earth(v3_body_z())
end

-- Azimuthal orbit tracking: rotate bz_eq0 around NED Z by the horizontal
-- angle from tdir0 to the current tether direction (diff/|diff|).
-- Matches Python orbit_tracked_body_z_eq() exactly:
--   preserves bz_eq0:z() — no altitude feedback → no positive-feedback instability.
-- Safe at 50 Hz without rate limiting (azimuthal rotation only, no Z change).
local function orbit_track_azimuthal(bz_eq0, tdir0, diff)
    local dx, dy = diff:x(), diff:y()
    local dh = math.sqrt(dx*dx + dy*dy)
    if dh < 0.01 then return v3_copy(bz_eq0) end
    local thx = dx / dh
    local thy = dy / dh
    local t0x = tdir0:x()
    local t0y = tdir0:y()
    local n0 = math.sqrt(t0x*t0x + t0y*t0y)
    if n0 < 0.01 then return v3_copy(bz_eq0) end
    t0x = t0x / n0; t0y = t0y / n0
    -- 2-D rotation angle from tdir0 to current direction
    local cos_phi = math.max(-1.0, math.min(1.0, t0x*thx + t0y*thy))
    local sin_phi = t0x*thy - t0y*thx
    -- Rotate only X,Y of bz_eq0; Z is preserved exactly
    local bx, by, bz = bz_eq0:x(), bz_eq0:y(), bz_eq0:z()
    local r = Vector3f()
    r:x(cos_phi*bx - sin_phi*by)
    r:y(sin_phi*bx + cos_phi*by)
    r:z(bz)
    r:normalize()
    return r
end

-- Rodrigues rotation kept for pumping slerp (body_z transitions between phases).
local function rodrigues(v, axis_n, angle)
    local ca  = math.cos(angle)
    local sa  = math.sin(angle)
    local vx, vy, vz = v:x(),      v:y(),      v:z()
    local nx, ny, nz = axis_n:x(), axis_n:y(), axis_n:z()
    local acx = ny*vz - nz*vy
    local acy = nz*vx - nx*vz
    local acz = nx*vy - ny*vx
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

-- ── Mode-entry reset ─────────────────────────────────────────────────────────
-- Called from update() when SCR_USER6 changes value.
-- Resets per-mode state so stale flags from a previous mode don't bleed through.
-- Shared capture state (_captured, _bz_eq0, _tdir0, _bz_orbit, _bz_slerp) is
-- intentionally preserved: the captured orbit orientation is valid across mode
-- switches (hub position hasn't moved).

local function _on_mode_enter(mode)
    _nv_floats          = {}    -- clear named-float inbox on mode change so stale substates cannot bleed through
    _none_status_ms     = 0     -- fire mode-0 log immediately on entry
    if mode == MODE_PUMPING then
        -- Ground planner owns substate; Lua resets only the internally-derived target.
        _pump_bz_ri = nil   -- recomputed from tether direction after first GPS capture
    end
end

-- ── Flight subsystem ─────────────────────────────────────────────────────────

local function run_flight()
    -- Only run in ACRO_Heli mode
    if vehicle:get_mode() ~= ACRO_MODE_NUM then return end

    local mode_now    = _prev_mode          -- decoded from SCR_USER6 in update()
    local substate    = _prev_sub
    local dt          = FLIGHT_PERIOD_MS * 0.001   -- 50 Hz step [s]
    local _is_landing = mode_now == MODE_LANDING
    local _is_pumping = mode_now == MODE_PUMPING
    -- Select altitude-neutral collective for the current mode.
    local col_cruise  = _is_landing and COL_CRUISE_LAND_RAD or COL_CRUISE_FLIGHT_RAD

    -- ── Pre-capture: gyro feedthrough to preserve orbital rate ───────────
    -- Runs BEFORE the ahrs:healthy() guard so Ch3 gets collective support even
    -- during AHRS initialisation.  Ch1/Ch2 use pure gyro feedthrough so ACRO's
    -- desired_rate = measured_rate -> rate_error = 0 -> zero corrective torque.
    -- Hub orbits naturally while GPS fuses (~35 s after kinematic exit).
    -- Without this, ch1=1500 -> desired_rate=0 -> ACRO kills 229 deg/s orbital
    -- rate within seconds -> crash before GPS fuses.
    if not _captured and mode_now ~= 0 then
        local ct = (col_cruise - COL_MIN_RAD) / (COL_MAX_RAD - COL_MIN_RAD)
        if _rc_ch3 then _rc_ch3:set_override(math.floor(1000.0 + ct * 1000.0 + 0.5)) end
        if ahrs:healthy() then
            local gyro_pre = ahrs:get_gyro()
            local gx = gyro_pre and gyro_pre:x() or 0.0
            local gy = gyro_pre and gyro_pre:y() or 0.0
            local sc = 500.0 / (ACRO_RP_RATE_DEG * math.pi / 180.0)
            local c1 = math.max(1000, math.min(2000, math.floor(1500.0 + sc * gx + 0.5)))
            local c2 = math.max(1000, math.min(2000, math.floor(1500.0 + sc * gy + 0.5)))
            if _rc_ch1 then _rc_ch1:set_override(c1) end
            if _rc_ch2 then _rc_ch2:set_override(c2) end
        else
            if _rc_ch1 then _rc_ch1:set_override(1500) end
            if _rc_ch2 then _rc_ch2:set_override(1500) end
        end
    end

    -- AHRS must be healthy for attitude-dependent calculations (body_to_earth,
    -- earth_to_body, gyro).  The pre-capture RC above is the only path that
    -- runs without healthy AHRS.
    if not ahrs:healthy() then return end

    -- ── DCM capture: immediate bz_eq0 from attitude estimate ─────────────
    -- Captures bz_eq0 = disk_normal_ned() as soon as AHRS becomes healthy
    -- (~t=15 s during kinematic, well before GPS fuses).
    -- EKF yaw is initially wrong (GPS velocity yaw hasn't aligned), but because
    -- bz_now and bz_eq0 are computed in the SAME EKF frame, bz_now × bz_eq0 ≈ 0
    -- near equilibrium → cyclic stays near neutral → hub doesn't tumble at
    -- kinematic exit.  GPS tether recapture (below) corrects bz_eq0 once GPS
    -- fuses (~35 s after kinematic exit).
    if not _captured and mode_now ~= 0 then
        _bz_eq0       = disk_normal_ned()
        _bz_orbit     = v3_copy(_bz_eq0)
        _bz_slerp     = v3_copy(_bz_eq0)
        _tdir0        = nil   -- set by GPS tether recapture when GPS fuses
        _last_col_rad = col_cruise  -- start at mode-appropriate cruise; avoids thrust
                                    -- dropout when entering landing/pumping from orbit
        _col_i        = col_cruise  -- reset integrator to mode-appropriate initial value
        _captured     = true
        local label = _is_landing and "land" or (_is_pumping and "pump" or "steady")
        gcs:send_text(6, string.format(
            "RAWES %s: captured  bz=(%.2f %.2f %.2f)  sub=%d",
            label, _bz_eq0:x(), _bz_eq0:y(), _bz_eq0:z(), substate))
    end

    -- ── GPS tether vector + synchronized capture ──────────────────────────
    -- diff/tlen: used for azimuthal orbit tracking and pumping phase detection.
    -- GPS synchronized capture: when GPS first fuses (_tdir0 still nil), capture
    -- bz_eq0 from disk_normal_ned() at that exact moment.
    -- The pre-GPS DCM tracking (below) continuously sets bz_orbit = bz_now, so
    -- at GPS fire time bz_orbit = bz_now.  Resetting bz_eq0 = bz_orbit (= bz_now)
    -- and _tdir0 = normalize(diff) makes orbit tracking start with zero error:
    --   azimuthal(bz_eq0, _tdir0, diff) = bz_eq0 (diff == _tdir0 at fire time).
    -- As the hub orbits: diff rotates -> bz_orbit rotates; bz_now tracks via
    -- gyros at the same rate -> error stays near-zero throughout.
    local pos_ned = ahrs:get_relative_position_NED_origin()
    local diff, tlen
    if pos_ned then
        local anch = anchor_ned()
        diff = Vector3f()
        diff:x(pos_ned:x() - anch:x())
        diff:y(pos_ned:y() - anch:y())
        diff:z(pos_ned:z() - anch:z())
        tlen = diff:length()
        if _tdir0 == nil and tlen >= MIN_TETHER_M then
            _tdir0    = v3_normalize(diff)
            _bz_eq0   = disk_normal_ned()   -- capture from DCM at GPS fire time
            _bz_orbit = v3_copy(_bz_eq0)    -- bz_orbit was already tracking bz_now
            _bz_slerp = v3_copy(_bz_eq0)    -- no step change in slerp
            local label = _is_landing and "land" or (_is_pumping and "pump" or "steady")
            gcs:send_text(6, string.format(
                "RAWES %s: GPS  bz=(%.2f %.2f %.2f)  tlen=%.1f m",
                label, _bz_eq0:x(), _bz_eq0:y(), _bz_eq0:z(), tlen))
        end
    end

    -- ── Pumping: compute reel-in body_z once _tdir0 is known ─────────────
    -- body_z_reel_in: xi = XI_REEL_IN_DEG from the horizontal wind direction.
    -- Wind direction estimated from tether direction (_tdir0).
    -- At xi=80 deg East wind: bz_ri = [0, cos(80), -sin(80)] = [0, 0.17, -0.98]
    if _is_pumping and _pump_bz_ri == nil and _tdir0 ~= nil then
        local wx, wy = _tdir0:x(), _tdir0:y()
        local wh = math.sqrt(wx*wx + wy*wy)
        if wh > 1e-3 then wx = wx/wh; wy = wy/wh
        else wx = 0.0; wy = 1.0 end  -- fallback: assume East wind
        local cos_xi = math.cos(XI_REEL_IN_DEG * math.pi / 180.0)
        local sin_xi = math.sin(XI_REEL_IN_DEG * math.pi / 180.0)
        _pump_bz_ri = Vector3f()
        _pump_bz_ri:x(cos_xi * wx)
        _pump_bz_ri:y(cos_xi * wy)
        _pump_bz_ri:z(-sin_xi)   -- upward: negative NED z
        gcs:send_text(6, string.format(
            "RAWES pump: bz_ri=(%.2f %.2f %.2f)  xi=%.0f deg",
            _pump_bz_ri:x(), _pump_bz_ri:y(), _pump_bz_ri:z(), XI_REEL_IN_DEG))
    end

    -- ── Landing: final_drop (substate commanded by ground planner) ────────
    if _is_landing and substate == LAND_FINAL_DROP then
        -- Zero collective (0 rad), neutral cyclic, drop to floor.
        local col_thrust = (0.0 - COL_MIN_RAD) / (COL_MAX_RAD - COL_MIN_RAD)
        local ch3_fd = math.floor(1000.0 + col_thrust * 1000.0 + 0.5)
        if _rc_ch1 then _rc_ch1:set_override(1500) end
        if _rc_ch2 then _rc_ch2:set_override(1500) end
        if _rc_ch3 then _rc_ch3:set_override(ch3_fd) end
        _prev_ch1 = 1500
        _prev_ch2 = 1500
        return
    end

    -- ── Orbit tracking update (requires GPS tether direction) ─────────────
    -- Azimuthal (horizontal-only) rotation: rotate _bz_eq0 around NED Z by
    -- the horizontal angle from _tdir0 to the current tether direction.
    -- Preserves _bz_eq0:z() exactly — no altitude feedback, no instability.
    -- Matches Python orbit_tracked_body_z_eq(); safe at 50 Hz without slerp.
    -- Landing: skip (orbit stays at _bz_eq0 = straight-descent direction).
    -- Pre-GPS: skip; handled by DCM tracking block below.
    if not _is_landing and _tdir0 ~= nil and diff ~= nil and tlen and tlen >= MIN_TETHER_M then
        _bz_orbit = orbit_track_azimuthal(_bz_eq0, _tdir0, diff)
    end

    -- ── Pre-GPS attitude hold ─────────────────────────────────────────────
    -- Before GPS fuses (_tdir0 == nil): track bz_now so cyclic = pure gyro
    -- feedthrough (err = bz_now x bz_orbit = 0).
    --
    -- Why not hold bz_eq0 fixed (the old design):
    --   bz_eq0 is captured at AHRS healthy time (~t=0 in kinematic).  EKF yaw
    --   at t=0 is wrong (GPS velocity yaw hasn't aligned yet).  Over the next
    --   65 s the EKF yaw converges to the GPS velocity heading, rotating
    --   disk_normal_ned() by ~27 deg.  At kinematic exit bz_now (GPS-aligned)
    --   diverges from bz_eq0 (t=0 snapshot) -> err = bz_now x bz_eq0 != 0 ->
    --   full cyclic saturation -> hub tumbles immediately.
    --
    -- Why tracking bz_now is safe during kinematic:
    --   The kinematic locks hub orientation, so bz_now = physics body_z in EKF
    --   frame regardless of EKF yaw drift.  err = bz_now x bz_now = 0 always
    --   -> cyclic = gyro feedthrough only -> no conflict with kinematic.
    --   At kinematic exit: bz_orbit = bz_now -> err = 0 -> stable entry.
    --
    -- After kinematic exit (if GPS not yet fused):
    --   Hub in free flight with pure gyro feedthrough (zero corrective cyclic).
    --   Hub may drift slightly under natural dynamics but won't actively tumble.
    --   GPS recapture fires when pos_ned becomes available, freshly capturing
    --   bz_eq0 = disk_normal_ned() at that moment -> zero step change in error.
    if _tdir0 == nil then
        local bz_now_pre = disk_normal_ned()
        if bz_now_pre then
            _bz_orbit = bz_now_pre
        end
    end

    -- ── External planner timeout ──────────────────────────────────────────
    -- Only applies to external planner overrides (_bz_target set externally).
    -- Pumping uses _pump_bz_ri as the slerp goal directly; not affected here.
    if _bz_target and (millis() - _plan_ms) > PLANNER_TIMEOUT then
        _bz_target = nil
        gcs:send_text(6, "RAWES steady: planner timeout -- reverting to orbit")
    end

    -- ── Rate-limited slerp toward goal ────────────────────────────────────
    -- Pumping transition/reel_in: slerp toward xi=80 deg orientation.
    -- All other modes: slerp toward external planner override or orbit tracking.
    local bz_slew = p("SCR_USER2", 0.40)
    local goal
    if _is_pumping and (substate == PUMP_TRANSITION or substate == PUMP_REEL_IN) then
        goal = _pump_bz_ri or _bz_orbit  -- fallback if _pump_bz_ri not yet computed
    else
        -- PUMP_REEL_OUT, PUMP_TRANSITION_BACK, PUMP_HOLD: slerp toward orbit
        goal = _bz_target or _bz_orbit
    end

    local dot    = math.max(-1.0, math.min(1.0, _bz_slerp:dot(goal)))
    local remain = math.acos(dot)
    if remain > 1e-4 then
        local ax = _bz_slerp:cross(goal)
        if ax:length() > 1e-6 then
            local step = math.min(bz_slew * dt, remain)
            _bz_slerp = rodrigues(_bz_slerp, v3_normalize(ax), step)
        end
    end

    -- ── Cyclic P loop ─────────────────────────────────────────────────────
    -- Orbit modes (HOLD/REEL_OUT): use _bz_orbit (azimuthal tracker).
    -- Pumping transitions (TRANSITION/REEL_IN/TRANSITION_BACK): use _bz_slerp.
    --   _bz_orbit is azimuthal-only (z fixed at orbit xi~23 deg). Using it
    --   during TRANSITION resists xi=80 rotation → hub stays at xi~23 with
    --   col=0.079 → massive over-thrust → tether slack → crash.
    local kp  = p("SCR_USER1", 1.0)
    local bz_now = disk_normal_ned()
    local err_ned = bz_now:cross(_bz_slerp)

    local err_body = ahrs:earth_to_body(err_ned)
    local err_bx   = err_body:x()    -- body X (roll)
    local err_by   = err_body:y()    -- body Y (pitch)

    -- Pure P control (no gyro feedthrough).
    -- The orbital body_z precession rate is ~0.01 rad/s; the P gain corrects
    -- any lag with a negligible steady-state error.  Gyro feedthrough was
    -- removed because it makes the control sensitive to EKF yaw error: with
    -- EKF yaw ~46 deg off from physical yaw, gyro_x/y in EKF body frame are
    -- rotated versions of physical rates -> feedthrough applies corrective
    -- torque in the wrong body axes -> hub tumbles at kinematic exit.
    local roll_rads  = kp * err_bx
    local pitch_rads = kp * err_by

    -- Map rate to RC PWM; full stick (+/-500 us) = +/-ACRO_RP_RATE_DEG deg/s
    local scale = 500.0 / (ACRO_RP_RATE_DEG * math.pi / 180.0)

    local ch1 = math.floor(1500.0 + scale * roll_rads  + 0.5)
    local ch2 = math.floor(1500.0 + scale * pitch_rads + 0.5)
    ch1 = math.max(1000, math.min(2000, ch1))
    ch2 = math.max(1000, math.min(2000, ch2))

    -- Output rate limiter (100 PWM/step = 1.26 rad/s/step at 50 Hz)
    local d1 = ch1 - _prev_ch1
    local d2 = ch2 - _prev_ch2
    if d1 >  100 then d1 =  100 end
    if d1 < -100 then d1 = -100 end
    if d2 >  100 then d2 =  100 end
    if d2 < -100 then d2 = -100 end
    ch1 = _prev_ch1 + d1
    ch2 = _prev_ch2 + d2
    _prev_ch1 = ch1
    _prev_ch2 = ch2

    if _rc_ch1 then _rc_ch1:set_override(ch1) end
    if _rc_ch2 then _rc_ch2:set_override(ch2) end

    -- ── Collective ────────────────────────────────────────────────────────
    -- Pumping: fixed open-loop collective per phase.
    --   reel_out: COL_REEL_OUT (-0.20 rad) -- rotor near equilibrium; winch controls tension
    --   reel_in:  COL_REEL_IN  (0.079 rad) -- altitude-neutral at xi=80 deg
    -- Landing / flight: VZ descent-rate controller (hardcoded vz_sp per mode).
    local col_cmd
    if _is_pumping then
        local t_in_sub = (millis() - _submode_ms) * 0.001
        local progress = math.min(1.0, t_in_sub / T_TRANSITION)
        if substate == PUMP_HOLD or substate == PUMP_REEL_OUT then
            col_cmd = COL_REEL_OUT
        elseif substate == PUMP_TRANSITION then
            -- Ramp from reel-out to reel-in over T_TRANSITION; body_z slerps toward
            -- bz_ri simultaneously.  At progress=1 both reach reel-in targets.
            col_cmd = COL_REEL_OUT + progress * (COL_REEL_IN - COL_REEL_OUT)
        elseif substate == PUMP_REEL_IN then
            col_cmd = COL_REEL_IN
        elseif substate == PUMP_TRANSITION_BACK then
            -- Ramp from reel-in back to reel-out over T_TRANSITION; body_z slerps
            -- toward _bz_orbit simultaneously.  Without this ramp: at xi=80 deg,
            -- COL_REEL_OUT=-0.20 produces near-zero lift -> hub drops at cycle start.
            col_cmd = COL_REEL_IN + progress * (COL_REEL_OUT - COL_REEL_IN)
        else
            col_cmd = COL_REEL_OUT  -- fallback: unknown substate
        end
    elseif _tdir0 == nil then
        -- Pre-GPS: EKF vz is biased (GPS altitude noise during horizontal kinematic flight).
        -- VZ controller would over-command collective → excess thrust → tether tension runaway.
        -- Hold at cruise collective until GPS recapture fires (_tdir0 becomes non-nil).
        col_cmd = col_cruise
    else
        -- PI VZ controller: integrator starts at col_cruise (mode-appropriate initial
        -- value set at capture) and winds to the actual trim collective.
        --   landing: vz_sp = VZ_LAND_SP (0.5 m/s descent)
        --   cruise:  vz_sp = 0.0 m/s (altitude hold)
        local vz_sp     = _is_landing and VZ_LAND_SP or 0.0
        local vz_actual = 0.0
        local vel_ned   = ahrs:get_velocity_NED()
        if vel_ned then vz_actual = vel_ned:z() end
        local vz_error  = vz_actual - vz_sp
        _col_i = math.max(COL_MIN_RAD, math.min(COL_MAX_RAD, _col_i + KI_VZ * vz_error * dt))
        col_cmd = _col_i + KP_VZ * vz_error
    end

    if col_cmd < COL_MIN_RAD then col_cmd = COL_MIN_RAD end
    if col_cmd > COL_MAX_RAD then col_cmd = COL_MAX_RAD end

    -- Slew-limit collective change per step
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

    -- ── Diagnostic log (every ~5 s at 50 Hz) ─────────────────────────────
    if _diag % 250 == 1 then
        local err_mag  = math.sqrt(err_bx * err_bx + err_by * err_by)
        local pump_info = ""
        if _is_pumping and tlen then
            pump_info = string.format("  pump=%d  tlen=%.1f m", substate, tlen)
        end
        gcs:send_text(6, string.format(
            "RAWES: ch1=%d ch2=%d ch3=%d |err|=%.3f rad  col=%.3f rad%s",
            ch1, ch2, ch3, err_mag, _last_col_rad, pump_info))
    end
end

-- ── RAWES_ARM: timed arm/disarm state machine ────────────────────────────────
-- States: nil → interlock_low → arming → armed → nil (on expiry/disarm)
-- Arming sequence avoids "Motor Interlock Enabled" prearm failure by holding
-- Ch8 LOW for one tick before calling arming:arm(), then retrying each tick
-- until is_armed().  Re-sending refreshes the deadline; while already armed
-- it only refreshes the deadline (no re-arm cycle needed).

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
        -- Step 1: set interlock LOW and wait one tick so ArduPilot sees it before arm().
        if _rc_ch3 then _rc_ch3:set_override(1000) end
        if _rc_ch8 then _rc_ch8:set_override(1000) end  -- LOW: prearm check will pass
        _armon_state = "arming"

    elseif _armon_state == "arming" then
        -- Step 2: keep interlock LOW, retry arm() each tick until is_armed().
        if _rc_ch3 then _rc_ch3:set_override(1000) end
        if _rc_ch8 then _rc_ch8:set_override(1000) end
        if arming:is_armed() then
            _armon_state = "armed"
        else
            arming:arm()
        end

    elseif _armon_state == "armed" then
        -- Step 3: armed — interlock HIGH so motor can spin; hold overrides.
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

-- ── Main update ──────────────────────────────────────────────────────────────

local function update()
    _diag = _diag + 1

    -- ── Drain MAVLink named-float inbox ──────────────────────────────────
    -- Runs first (before mode check) so NV messages are visible in mode 0 too.
    -- Each received message is echoed as STATUSTEXT so senders can verify receipt.
    local nvf_raw = mavlink.receive_chan()
    while nvf_raw do
        local _, nv_val, nv_name = string.unpack("<Ifc10", nvf_raw, 13)
        nv_name = nv_name:gsub("\0", "")
        _nv_floats[nv_name] = nv_val
        gcs:send_text(6, string.format("RAWES: rcvd %s=%.0f", nv_name, nv_val))
        nvf_raw = mavlink.receive_chan()
    end

    -- ── Decode mode + substate ────────────────────────────────────────────
    local mode = math.floor(p("SCR_USER6", 0) + 0.5)
    local sub  = math.floor((_nv_floats["RAWES_SUB"] or 0) + 0.5)
    local now  = millis()

    -- ── CH4 yaw always neutral (no RC receiver; prevents ACRO yaw wind-up) ──
    if _rc_ch4 then _rc_ch4:set_override(1500) end

    -- ── RAWES_ARM state machine (cross-mode) ─────────────────────────────
    run_armon(now)

    -- Track mode and substate changes; reset state on mode entry.
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

    -- mode 0: completely passive -- no RC overrides, no keepalive.
    -- NV inbox was drained above so any received messages are already logged.
    -- Periodic status every 5 s so the GCS always knows the script is alive.
    if mode == MODE_NONE then
        if now - _none_status_ms >= 5000 then
            _none_status_ms = now
            local armed = arming:is_armed() and "ARMED" or "disarmed"
            gcs:send_text(6, "RAWES: mode 0 (none)  " .. armed)
        end
        return update, BASE_PERIOD_MS
    end

    -- Motor interlock keepalive: RC overrides expire after ~1 s; 100 Hz keeps Ch8 HIGH.
    -- run_armon() already sets Ch8=2000 each tick while in "armed" state; this is the
    -- fallback for vehicles armed via other means (non-ARMON paths).
    if arming:is_armed() and _rc_ch8 then
        _rc_ch8:set_override(2000)
    end

    -- Flight subsystem: runs at 50 Hz sub-step
    if mode == MODE_STEADY or mode == MODE_LANDING or mode == MODE_PUMPING then
        if now - _last_flight_ms >= FLIGHT_PERIOD_MS then
            _last_flight_ms = now
            run_flight()
        end
    end

    return update, BASE_PERIOD_MS
end

-- ── Entry point ──────────────────────────────────────────────────────────────

local _mode_init  = math.floor(p("SCR_USER6", 0) + 0.5)
local _sub_init   = 0   -- substate arrives via NAMED_VALUE_FLOAT "RAWES_SUB", not SCR_USER6
local _mode_names = {[0]="none", [1]="steady", [4]="landing", [5]="pumping"}
local _mode_str   = _mode_names[_mode_init] or "unknown"

gcs:send_text(6, string.format(
    "RAWES: loaded  mode=%d (%s)  sub=%d  kp=%.2f  slew=%.2f  anchor=(%.1f %.1f %.1f)",
    _mode_init, _mode_str, _sub_init,
    p("SCR_USER1", 1.0), p("SCR_USER2", 0.40),
    p("SCR_USER3", 0.0), p("SCR_USER4", 0.0), p("SCR_USER5", 0.0)))

-- @@UNIT_TEST_HOOK

return update, BASE_PERIOD_MS
