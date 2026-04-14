--[[
rawes.lua -- Unified RAWES flight + yaw-trim controller
Works in both ArduPilot SITL (mcroomp fork) and on the Pixhawk 6C.

Mode is selected at runtime via SCR_USER6:
  0  none    -- script loaded but both subsystems disabled
  1  flight  -- cyclic orbit-tracking only (Ch1, Ch2)                     50 Hz
  2  yaw     -- counter-torque yaw trim only (Ch9/SERVO9)                100 Hz
  3  both    -- flight + yaw trim active simultaneously
  4  landing -- cyclic orbit-tracking + VZ collective + auto final_drop  50 Hz
  5  pumping -- De Schutter pumping cycle (cyclic + per-phase collective) 50 Hz

Parameters (SCR_USER1..6):
  SCR_USER1   RAWES_KP_CYC    Cyclic P gain            [rad/s / rad]  default 1.0
  SCR_USER2   RAWES_BZ_SLEW   body_z slerp rate limit  [rad/s]        default 0.40
  SCR_USER3   RAWES_ANCHOR_N  Anchor North from EKF origin [m]         default 0.0
  SCR_USER4   RAWES_ANCHOR_E  Anchor East  from EKF origin [m]         default 0.0
  SCR_USER5   RAWES_ANCHOR_D  Anchor Down  from EKF origin [m]         default 0.0
  SCR_USER6   RAWES_MODE      Mode selector (0..5 -- see above)        default 0

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

Yaw-trim subsystem constants (compile-time):
  GEAR_RATIO   80:44 spur gear between GB4008 pinion and rotor hub
  KV_RAD       66 KV converted to rad/s/V
  R_MOTOR      7.5 ohm phase resistance
  K_BEARING    0.005 N*m*s/rad bearing drag
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
local LAND_MIN_TETHER_M = 2.0       -- landing mode: final_drop below this tether length [m]
local ACRO_RP_RATE_DEG  = 360.0     -- must match ACRO_RP_RATE ArduPilot parameter
local PLANNER_TIMEOUT   = 2000      -- ms: revert to natural orbit after this
-- Capture delay: millis() must exceed this value before body_z is snapped.
-- Stack test kinematic = 65 s; both thresholds set to 62 s (3 s before exit)
-- so orbit-tracking cyclic with gyro feedthrough is active AT kinematic exit.
-- _tdir0 is set to nil at capture (GPS may not yet be fused); the Rodrigues
-- orbit update is deferred until the first valid pos_ned from the EKF.
-- On real hardware millis() >> these thresholds so capture is immediate.
local KINEMATIC_SETTLE_MS  = 62000  -- landing: delay capture until 3 s before kinematic exit [ms]
local FLIGHT_SETTLE_MS     = 62000  -- flight/pumping: capture 3 s BEFORE kinematic exit [ms]
                                    -- Same timing as KINEMATIC_SETTLE_MS so orbit-tracking
                                    -- cyclic (with gyro feedthrough) is active at kinematic
                                    -- exit.  Without this, neutral ACRO sticks zeroes the
                                    -- natural orbital rate (~229 deg/s body X) -> crash.
                                    -- _tdir0 is nil at capture (no GPS yet); set on first
                                    -- pos_ned availability.  Rodrigues orbit tracking holds
                                    -- _bz_orbit = _bz_eq0 until GPS fuses.

-- ── VZ collective controller ─────────────────────────────────────────────────
-- Pixhawk-side descent-rate controller: mirrors AcroController.step_vz() in
-- controller.py.  Runs at 50 Hz inside run_flight().  Computes collective from
-- NED vz feedback and writes Ch3 RC override.
--
-- vz_setpoint is hardcoded per mode (no RC receiver in the system):
--   landing (mode 4): 0.5 m/s descent
--   flight / both (modes 1, 3): 0.0 m/s (altitude hold)
-- pumping (mode 5) uses open-loop collective per phase; this controller is bypassed.

local KP_VZ               = 0.05   -- collective gain [rad/(m/s)]
local COL_CRUISE_FLIGHT_RAD = -0.18  -- altitude-neutral at natural orbit (xi~8 deg, 100 m tether)
local COL_CRUISE_LAND_RAD   =  0.079 -- altitude-neutral at reel-in position (xi~80 deg, 20 m tether)
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
-- Phase synchronisation: Lua detects tether motion to follow ground planner winch.
--   hold     -> reel_out : tether length increases by > PUMP_LEN_THRESH m
--   reel_out -> transition: tether length decreases by > PUMP_LEN_THRESH m
--   transition-> reel_in : time-based (T_TRANSITION seconds; body_z slerp window)
--   reel_in  -> reel_out : tether length increases by > PUMP_LEN_THRESH m

local XI_REEL_IN_DEG  = 80.0   -- reel-in disk tilt from wind direction [deg]
local COL_REEL_OUT    = -0.20  -- collective during hold/reel-out [rad]
local COL_REEL_IN     =  0.079 -- collective during transition/reel-in [rad]
local T_TRANSITION    =  3.7   -- body_z slerp window [s] (from config default)
local PUMP_LEN_THRESH =  0.05  -- tether length change threshold to detect motion [m]

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
local _active_mode      = -1        -- last mode seen; -1 = uninitialised (triggers _on_mode_enter)

-- Cache RC channel objects at module load (rc:get_channel() is the correct API)
local _rc_ch1 = rc:get_channel(1)   -- roll rate override (flight)
local _rc_ch2 = rc:get_channel(2)   -- pitch rate override (flight)
local _rc_ch3 = rc:get_channel(3)   -- collective override (vz controller)
local _rc_ch8 = rc:get_channel(8)   -- motor interlock keepalive (H_RSC_MODE=1 passthrough)

-- ── Flight subsystem state ───────────────────────────────────────────────────

local _last_col_rad     = COL_CRUISE_FLIGHT_RAD  -- collective slew state [rad]
local _captured         = false           -- true once equilibrium has been snapped
local _bz_eq0           = nil             -- body_z_ned at equilibrium capture (from DCM)
local _tdir0            = nil             -- tether direction at equilibrium capture
local _bz_orbit         = nil             -- instantaneous orbit-tracked body_z setpoint
local _bz_slerp         = nil             -- rate-limited active setpoint (what cyclic tracks)
local _bz_target        = nil             -- external planner override (nil = natural orbit)
local _plan_ms          = 0               -- millis() when _bz_target was last commanded
local _prev_ch1         = 1500            -- last sent Ch1 PWM (for output rate limiting)
local _prev_ch2         = 1500            -- last sent Ch2 PWM (for output rate limiting)

-- Landing state
local _land_final_drop  = false           -- true after tether <= LAND_MIN_TETHER_M

-- Pumping cycle state (reset on mode entry)
local _pump_phase       = "hold"  -- "hold" | "reel_out" | "transition" | "reel_in"
local _pump_t_phase     = 0.0     -- time in current phase [s]
local _pump_bz_ri       = nil     -- reel-in body_z setpoint (computed after capture)
local _pump_tlen_ref    = nil     -- tether length reference for phase transition detection [m]
local _pump_cycle       = 0       -- pumping cycle counter (for logging)

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

-- Return -body_z in NED (= -disk_normal = disk DOWN direction).
-- sensor.py body Z = disk_normal (UP, roll~180 deg from NED).
-- body_to_earth([0,0,1]) = disk_normal (UP). Negate to get -disk_normal (DOWN).
-- Orbit tracking cross products are sign-invariant: this DOWN direction is
-- fully consistent (bz_now = -dn_current, _bz_orbit = -dn_target, err cancels).
local function disk_normal_ned()
    local bz = ahrs:body_to_earth(v3_body_z())
    local r = Vector3f()
    r:x(-bz:x()); r:y(-bz:y()); r:z(-bz:z())
    return r
end

-- Rodrigues rotation: rotate vector v around unit vector axis_n by angle radians
-- v' = v*cos(th) + (axis x v)*sin(th) + axis*(axis.v)*(1-cos(th))
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

-- ── Mode-entry reset ─────────────────────────────────────────────────────────
-- Called from update() when SCR_USER6 changes value.
-- Resets per-mode state so stale flags from a previous mode don't bleed through.
-- Shared capture state (_captured, _bz_eq0, _tdir0, _bz_orbit, _bz_slerp) is
-- intentionally preserved: the captured orbit orientation is valid across mode
-- switches (hub position hasn't moved).

local function _on_mode_enter(mode)
    if mode == 4 then
        -- Landing: clear final_drop latch so a re-entry works correctly
        _land_final_drop = false

    elseif mode == 5 then
        -- Pumping: reset phase state machine
        _pump_phase     = "hold"
        _pump_t_phase   = 0.0
        _pump_bz_ri     = nil     -- recomputed after capture
        _pump_tlen_ref  = nil
        _pump_cycle     = 0
    end
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

    -- If the RPM driver is not active (returns nil), the sensor is unavailable
    -- (e.g. RPM1_TYPE set post-boot in SITL before the driver has initialised).
    -- Do nothing; the mediator safety fallback will maintain hub stability.
    -- Do NOT send STARTUP_PWM in this case -- it would cause the hub to spin freely.
    if motor_rpm == nil then return end

    -- Cold-start: output a small startup throttle so the DShot ESC arms and
    -- begins returning telemetry.  Once RPM exceeds MIN_RPM the trim loop takes
    -- over.  No output if disarmed (safe bench default).
    if motor_rpm < MIN_RPM then
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

    local mode_now    = math.floor(p("SCR_USER6", 0) + 0.5)
    local _is_landing = mode_now == 4
    local _is_pumping = mode_now == 5
    -- Select altitude-neutral collective for the current mode.
    local col_cruise  = _is_landing and COL_CRUISE_LAND_RAD or COL_CRUISE_FLIGHT_RAD

    -- ── Pre-capture: hold Ch1/Ch2 neutral and Ch3 at cruise collective ────
    -- Runs BEFORE the ahrs:healthy() guard so the hub gets collective support
    -- even during AHRS initialisation (before GPS fusion).  Without this,
    -- Ch1/Ch2/Ch3 stay at ArduPilot default neutral (1500) and ACRO's rate
    -- loop zeroes the natural orbital rate (~229 deg/s body X) -> crash.
    -- This block only matters in the pre-capture window; after _captured=true
    -- the orbit-tracking cyclic below drives Ch1/Ch2.
    if not _captured and mode_now ~= 0 then
        local ct = (col_cruise - COL_MIN_RAD) / (COL_MAX_RAD - COL_MIN_RAD)
        if _rc_ch3 then _rc_ch3:set_override(math.floor(1000.0 + ct * 1000.0 + 0.5)) end
        if _rc_ch1 then _rc_ch1:set_override(1500) end
        if _rc_ch2 then _rc_ch2:set_override(1500) end
    end

    -- AHRS must be healthy for attitude-dependent calculations (body_to_earth,
    -- earth_to_body, gyro).  The pre-capture RC above is the only path that
    -- runs without healthy AHRS.
    if not ahrs:healthy() then return end

    -- ── DCM capture: snap body_z at settle time using attitude only ───────
    -- Captures at FLIGHT_SETTLE_MS (62 s) = 3 s before kinematic exits (65 s),
    -- or at KINEMATIC_SETTLE_MS (62 s) for landing.  Uses body_to_earth (DCM)
    -- -- no GPS needed.  Starting orbit-tracking cyclic BEFORE kinematic exit
    -- ensures gyro feedthrough ramps up correctly over the 100 PWM/step limiter
    -- so the correct cyclic is active the instant the hub enters free flight.
    -- _tdir0 = nil at capture (GPS not yet fused); populated below on first
    -- pos_ned.  Until then, _bz_orbit = _bz_eq0 (no Rodrigues update).
    if not _captured and mode_now ~= 0 then
        local settle_ms = _is_landing and KINEMATIC_SETTLE_MS or FLIGHT_SETTLE_MS
        if millis() >= settle_ms then
            _bz_eq0   = disk_normal_ned()
            _bz_orbit = v3_copy(_bz_eq0)
            _bz_slerp = disk_normal_ned()
            _tdir0    = nil   -- set on first GPS fix below
            _captured = true
            local label
            if _is_landing then label = "land"
            elseif _is_pumping then label = "pump"
            else label = "flight" end
            gcs:send_text(6, string.format(
                "RAWES %s: captured  bz=(%.2f %.2f %.2f)",
                label, _bz_eq0:x(), _bz_eq0:y(), _bz_eq0:z()))
        end
        return
    end

    -- ── GPS tether vector (optional; nil until EKF fuses position) ────────
    -- Used for Rodrigues orbit update and pumping phase detection.
    -- If nil, _bz_orbit stays at _bz_eq0 (safe fallback -- holds initial bz).
    local pos_ned = ahrs:get_relative_position_NED_origin()
    local diff, tlen
    if pos_ned then
        local anch = anchor_ned()
        diff = Vector3f()
        diff:x(pos_ned:x() - anch:x())
        diff:y(pos_ned:y() - anch:y())
        diff:z(pos_ned:z() - anch:z())
        tlen = diff:length()
        -- First GPS position after DCM capture: initialise tether direction.
        if _tdir0 == nil and tlen >= MIN_TETHER_M then
            _tdir0 = v3_normalize(diff)
            gcs:send_text(6, string.format(
                "RAWES flight: GPS  tlen=%.1f m", tlen))
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
        if tlen then _pump_tlen_ref = tlen end
    end

    -- ── Landing: final_drop transition ────────────────────────────────────
    -- Use altitude estimate (anchor.z - hub.z) rather than 3D tlen.
    -- With vel0 pointing horizontally, EKF origin Z = 0, so:
    --   altitude_est = anch:z() - pos_ned:z() = anchor_local_z - hub_local_z
    -- This is independent of EKF horizontal origin offset.
    if _is_landing and pos_ned then
        local anch = anchor_ned()
        local alt_est = anch:z() - pos_ned:z()
        if (not _land_final_drop) and alt_est <= LAND_MIN_TETHER_M then
            _land_final_drop = true
            gcs:send_text(6, string.format(
                "RAWES land: final_drop  alt_est=%.2f m", alt_est))
        end
        if _land_final_drop then
            -- Zero collective (0 rad), neutral cyclic, stop
            local col_thrust = (0.0 - COL_MIN_RAD) / (COL_MAX_RAD - COL_MIN_RAD)
            local ch3_fd = math.floor(1000.0 + col_thrust * 1000.0 + 0.5)
            if _rc_ch1 then _rc_ch1:set_override(1500) end
            if _rc_ch2 then _rc_ch2:set_override(1500) end
            if _rc_ch3 then _rc_ch3:set_override(ch3_fd) end
            _prev_ch1 = 1500
            _prev_ch2 = 1500
            return
        end
    end

    -- ── Pumping: phase state machine (requires GPS tlen) ─────────────────
    -- Phases follow the ground planner's actual winch motion (tether length
    -- change), not a local timer, so Lua stays synchronised regardless of
    -- kinematic startup timing.
    --
    -- hold      -> reel_out  : tether pays out (mediator starts reel_out)
    -- reel_out  -> transition: tether reels in (mediator ends reel_out)
    -- transition-> reel_in   : time-based (T_TRANSITION seconds)
    -- reel_in   -> reel_out  : tether pays out (mediator starts next cycle)
    local dt = FLIGHT_PERIOD_MS * 0.001

    if _is_pumping and tlen then
        _pump_t_phase = _pump_t_phase + dt

        if _pump_phase == "hold" then
            -- Detect cumulative reel-out: tlen must increase PUMP_LEN_THRESH above
            -- the minimum tlen seen since GPS became available.  Track minimum so
            -- the reference follows the hub to equilibrium after kinematic exit.
            if _pump_tlen_ref == nil or tlen < _pump_tlen_ref then
                _pump_tlen_ref = tlen
            end
            if _pump_tlen_ref ~= nil and tlen > _pump_tlen_ref + PUMP_LEN_THRESH then
                _pump_phase    = "reel_out"
                _pump_t_phase  = 0.0
                _pump_cycle    = _pump_cycle + 1
                _pump_tlen_ref = tlen
                gcs:send_text(6, string.format(
                    "RAWES pump: reel_out  cycle=%d  tlen=%.1f m",
                    _pump_cycle, tlen))
            end

        elseif _pump_phase == "reel_out" then
            -- Track peak tether length; detect start of reel-in (tlen drops from peak)
            if _pump_tlen_ref == nil or tlen > _pump_tlen_ref then _pump_tlen_ref = tlen end
            if tlen < _pump_tlen_ref - PUMP_LEN_THRESH then
                _pump_phase    = "transition"
                _pump_t_phase  = 0.0
                _pump_tlen_ref = tlen
                gcs:send_text(6, string.format(
                    "RAWES pump: transition  tlen=%.1f m", tlen))
            end

        elseif _pump_phase == "transition" then
            if _pump_t_phase >= T_TRANSITION then
                _pump_phase   = "reel_in"
                _pump_t_phase = 0.0
                gcs:send_text(6, string.format(
                    "RAWES pump: reel_in  tlen=%.1f m", tlen))
            end

        elseif _pump_phase == "reel_in" then
            -- Track trough tether length; detect start of next reel-out (tlen rises)
            if _pump_tlen_ref == nil or tlen < _pump_tlen_ref then _pump_tlen_ref = tlen end
            if tlen > _pump_tlen_ref + PUMP_LEN_THRESH then
                _pump_phase    = "reel_out"
                _pump_t_phase  = 0.0
                _pump_cycle    = _pump_cycle + 1
                _pump_tlen_ref = tlen
                gcs:send_text(6, string.format(
                    "RAWES pump: reel_out  cycle=%d  tlen=%.1f m",
                    _pump_cycle, tlen))
            end
        end
    end

    -- ── Orbit tracking update (requires GPS tether direction) ─────────────
    -- Rotate _bz_eq0 by the Rodrigues rotation from _tdir0 to current tether
    -- direction.  Only runs once _tdir0 is known (GPS fused after DCM capture).
    -- Landing: skip (orbit stays at _bz_eq0 = straight-descent direction).
    -- Pre-GPS: skip (holds _bz_orbit = _bz_eq0 as safe fallback).
    if not _is_landing and _tdir0 ~= nil and tlen and tlen >= MIN_TETHER_M then
        local bzt   = v3_normalize(diff)
        local axis  = _tdir0:cross(bzt)
        local sinth = axis:length()
        if sinth > 1e-6 then
            local costh = _tdir0:dot(bzt)
            _bz_orbit = rodrigues(_bz_eq0, v3_normalize(axis), math.atan(sinth, costh))
        end
    end

    -- ── External planner timeout ──────────────────────────────────────────
    -- Only applies to external planner overrides (_bz_target set externally).
    -- Pumping uses _pump_bz_ri as the slerp goal directly; not affected here.
    if _bz_target and (millis() - _plan_ms) > PLANNER_TIMEOUT then
        _bz_target = nil
        gcs:send_text(6, "RAWES flight: planner timeout -- reverting to orbit")
    end

    -- ── Rate-limited slerp toward goal ────────────────────────────────────
    -- Pumping reel-in/transition: slerp toward fixed xi=80 deg orientation.
    -- All other modes: slerp toward external planner override or orbit tracking.
    local bz_slew = p("SCR_USER2", 0.40)
    local goal
    if _is_pumping and (_pump_phase == "transition" or _pump_phase == "reel_in") then
        goal = _pump_bz_ri or _bz_orbit  -- fallback if _pump_bz_ri not yet computed
    else
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
    -- Error: bz_now × _bz_orbit (NOT _bz_slerp).
    -- _bz_slerp moves at 0.40 rad/s; orbit rate is ~4 rad/s.  When slerp
    -- lags behind orbit, bz_now × bz_slerp pushes BACKWARD, fighting the
    -- natural precession.  Using _bz_orbit (the advancing orbit target)
    -- gives the correct restoring direction.
    local kp      = p("SCR_USER1", 1.0)
    local bz_now  = disk_normal_ned()
    local err_ned = bz_now:cross(_bz_orbit)

    local err_body = ahrs:earth_to_body(err_ned)
    local err_bx   = err_body:x()    -- body X (roll)
    local err_by   = err_body:y()    -- body Y (pitch)

    -- Gyro feedthrough: cmd = kp*err + gyro so ACRO desired rate = actual + correction.
    -- Without this, ch1=1500+scale*kp*err would set desired rate to kp*err only, and
    -- ACRO's rate loop would zero out the orbital body rate (~229 deg/s body X),
    -- killing the natural orbit.  With feedthrough, ACRO tracks actual body rate + err.
    local gyro   = ahrs:get_gyro()
    local gyro_x = gyro and gyro:x() or 0.0
    local gyro_y = gyro and gyro:y() or 0.0

    local roll_rads  = kp * err_bx + gyro_x
    local pitch_rads = kp * err_by + gyro_y

    -- Map rate to RC PWM; full stick (+/-500 us) = +/-ACRO_RP_RATE_DEG deg/s
    local scale = 500.0 / (ACRO_RP_RATE_DEG * math.pi / 180.0)

    local ch1 = math.floor(1500.0 + scale * roll_rads  + 0.5)
    local ch2 = math.floor(1500.0 + scale * pitch_rads + 0.5)
    ch1 = math.max(1000, math.min(2000, ch1))
    ch2 = math.max(1000, math.min(2000, ch2))

    -- Output rate limiter (100 PWM/step -- fast enough for gyro feedthrough ramp-up)
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
        if _pump_phase == "hold" or _pump_phase == "reel_out" then
            col_cmd = COL_REEL_OUT
        else   -- transition or reel_in
            col_cmd = COL_REEL_IN
        end
    else
        -- VZ controller: hardcoded setpoint per mode (no RC receiver in this system).
        -- Landing: descend at VZ_LAND_SP = 0.5 m/s.  Flight/yaw: hold altitude (0.0).
        local vz_sp = 0.0
        if _is_landing then vz_sp = VZ_LAND_SP end
        local vz_actual = 0.0
        local vel_ned = ahrs:get_velocity_NED()
        if vel_ned then vz_actual = vel_ned:z() end
        local vz_error = vz_actual - vz_sp
        col_cmd = col_cruise + KP_VZ * vz_error
        -- Landing only: never reduce collective below cruise so the winch
        -- controls descent rate; collective only corrects overshoot.
        -- Flight mode: allow collective below col_cruise to correct upward drift
        -- (bidirectional P controller; hard floor is COL_MIN_RAD below).
        if _is_landing then
            col_cmd = math.max(col_cmd, col_cruise)
        end
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
            pump_info = string.format("  pump=%s/c%d  tlen=%.1f m",
                _pump_phase, _pump_cycle, tlen)
        end
        gcs:send_text(6, string.format(
            "RAWES: ch1=%d ch2=%d ch3=%d |err|=%.3f rad  col=%.3f rad%s",
            ch1, ch2, ch3, err_mag, _last_col_rad, pump_info))
    end
end

-- ── Main update ──────────────────────────────────────────────────────────────

local function update()
    _diag = _diag + 1

    -- Motor interlock keepalive: RC overrides expire after ~1 s; 100 Hz keeps Ch8 HIGH.
    -- Tests must NOT send Ch8 -- Lua owns the interlock once scripting is active.
    if arming:is_armed() and _rc_ch8 then
        _rc_ch8:set_override(2000)
    end

    local mode = math.floor(p("SCR_USER6", 0) + 0.5)

    -- Mode-change detection: reset per-mode state on entry.
    -- Fires on the very first tick (_active_mode = -1) and on any subsequent change.
    if mode ~= _active_mode then
        _on_mode_enter(mode)
        _active_mode = mode
    end

    -- mode 0: both subsystems disabled
    if mode == 0 then return update, BASE_PERIOD_MS end

    -- Yaw-trim subsystem: runs every tick (100 Hz) when mode 2 or 3
    if mode == 2 or mode == 3 then
        run_yaw_trim()
    end

    -- Flight subsystem: runs at 50 Hz sub-step for modes 1, 3, 4, 5
    if mode == 1 or mode == 3 or mode == 4 or mode == 5 then
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
local _mode_names = {[0]="none", [1]="flight", [2]="yaw",
                     [3]="both", [4]="landing", [5]="pumping"}
local _mode_str   = _mode_names[_mode_init] or "unknown"

gcs:send_text(6, string.format(
    "RAWES: loaded  mode=%d (%s)  kp=%.2f  slew=%.2f  anchor=(%.1f %.1f %.1f)",
    _mode_init, _mode_str,
    p("SCR_USER1", 1.0), p("SCR_USER2", 0.40),
    p("SCR_USER3", 0.0), p("SCR_USER4", 0.0), p("SCR_USER5", 0.0)))

return update, BASE_PERIOD_MS
