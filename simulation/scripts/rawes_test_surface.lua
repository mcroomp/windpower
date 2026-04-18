--[[
rawes_test_surface.lua  --  Unit-test surface for rawes.lua.

This file is NOT loaded on hardware.  The Python test harness splices its
content in place of the  -- @@UNIT_TEST_HOOK  comment in rawes.lua before
executing the script.  Because the splice happens inside the anonymous
function wrapper the harness uses to load rawes.lua, every local defined
in rawes.lua is in scope here -- constants, helpers, and the private logic
extracted into testable wrappers below.

After load, the global _rawes_fns table gives tests direct access to the
internal functions without polluting the global namespace during normal flight.

Multi-value returns are wrapped in tables so lupa sees a single Lua value.
--]]

_rawes_fns = {
    -- ── Constants ────────────────────────────────────────────────────────

    ACRO_RP_RATE_DEG       = ACRO_RP_RATE_DEG,
    BASE_PERIOD_MS         = BASE_PERIOD_MS,
    FLIGHT_PERIOD_MS       = FLIGHT_PERIOD_MS,
    COL_CRUISE_FLIGHT_RAD  = COL_CRUISE_FLIGHT_RAD,
    COL_CRUISE_LAND_RAD    = COL_CRUISE_LAND_RAD,
    COL_MIN_RAD            = COL_MIN_RAD,
    COL_MAX_RAD            = COL_MAX_RAD,
    COL_SLEW_MAX           = COL_SLEW_MAX,
    MIN_TETHER_M           = MIN_TETHER_M,
    T_TRANSITION           = T_TRANSITION,
    XI_REEL_IN_DEG         = XI_REEL_IN_DEG,
    KP_VZ                  = KP_VZ,
    VZ_LAND_SP             = VZ_LAND_SP,

    -- ── Yaw-trim constants ───────────────────────────────────────────────
    BASE_THROTTLE_PCT      = BASE_THROTTLE_PCT,
    KP_YAW                 = KP_YAW,
    KI_YAW                 = KI_YAW,
    YAW_I_MAX              = YAW_I_MAX,
    YAW_SRV_FUNC           = YAW_SRV_FUNC,
    YAW_STABLE_RAD_S       = YAW_STABLE_RAD_S,
    YAW_STABLE_TIMEOUT_MS  = YAW_STABLE_TIMEOUT_MS,
    YAW_DEAD_ZONE_RAD_S    = YAW_DEAD_ZONE_RAD_S,
    YAW_TEST_THROTTLE_PCT  = YAW_TEST_THROTTLE_PCT,
    YAW_TEST_DURATION_MS   = YAW_TEST_DURATION_MS,

    -- ── Pure math / geometry ─────────────────────────────────────────────

    rodrigues              = rodrigues,
    orbit_track_azimuthal  = orbit_track_azimuthal,

    -- ── Vector3f helpers ────────────────────────────────────────────────

    v3_copy      = v3_copy,
    v3_normalize = v3_normalize,
    v3_body_z    = v3_body_z,

    -- ── Param / anchor ───────────────────────────────────────────────────

    p          = p,
    anchor_ned = anchor_ned,

    -- ── Subsystem entry points ───────────────────────────────────────────

    run_flight   = run_flight,
    run_yaw_trim = run_yaw_trim,

    -- ── Yaw subsystem state accessors (for test_yaw_lua.py) ─────────────
    -- Return the current value of each module-level yaw state variable.
    -- Lua nil is returned as-is; lupa maps it to Python None.

    yaw_i                = function() return _yaw_i end,
    yaw_stopped          = function() return _yaw_stopped end,
    yaw_in_dead_zone     = function() return _yaw_in_dead_zone end,
    yaw_not_stable_ms    = function() return _yaw_not_stable_ms end,
    yaw_limited_start_ms = function() return _yaw_limited_start_ms end,

    -- PWM formula mirror of _set_throttle_pct() for conversion tests.
    pwm_for_pct = function(pct)
        return math.floor(800.0 + pct * 12.0 + 0.5)
    end,

    -- ── Inline logic extracted for unit testing ──────────────────────────
    -- These mirror code that is inlined inside run_flight() so that tests
    -- can call each piece in isolation.

    -- Rate-limited slerp step toward a goal direction.
    -- Mirrors the inline slerp block in run_flight() (lines ~705-713).
    -- Returns a new Vector3f advanced at most slew_rate*dt radians toward goal.
    slerp_step = function(bz_slerp, goal, slew_rate, dt)
        local dot    = math.max(-1.0, math.min(1.0, bz_slerp:dot(goal)))
        local remain = math.acos(dot)
        if remain > 1e-4 then
            local ax = bz_slerp:cross(goal)
            if ax:length() > 1e-6 then
                local step = math.min(slew_rate * dt, remain)
                return rodrigues(bz_slerp, v3_normalize(ax), step)
            end
        end
        return v3_copy(bz_slerp)
    end,

    -- Body-frame cyclic error from the P-gain loop.
    -- Mirrors rawes.lua: err_ned = bz_now x bz_orbit; earth_to_body(err_ned).
    -- Uses the current mock R matrix via ahrs:earth_to_body().
    -- Returns {err_bx, err_by} (1-indexed Lua table for clean Python access).
    cyclic_error_body = function(bz_now, bz_orbit)
        local err_ned  = bz_now:cross(bz_orbit)
        local err_body = ahrs:earth_to_body(err_ned)
        return {err_body:x(), err_body:y()}
    end,

    -- Full cyclic rate command: error → roll/pitch rates → ch1/ch2 PWM.
    -- Mirrors the inline cyclic P loop in run_flight().
    -- Uses the current mock R matrix via ahrs:earth_to_body().
    -- Returns {roll_rads, pitch_rads, ch1_pwm, ch2_pwm} (1-indexed table).
    cyclic_rates = function(bz_now, bz_orbit, kp)
        kp = kp or 1.0
        local err_ned  = bz_now:cross(bz_orbit)
        local err_body = ahrs:earth_to_body(err_ned)
        local err_bx   = err_body:x()
        local err_by   = err_body:y()
        local roll_rads  = kp * err_bx
        local pitch_rads = kp * err_by
        local scale = 500.0 / (ACRO_RP_RATE_DEG * math.pi / 180.0)
        local ch1 = math.floor(1500.0 + scale * roll_rads  + 0.5)
        local ch2 = math.floor(1500.0 + scale * pitch_rads + 0.5)
        ch1 = math.max(1000, math.min(2000, ch1))
        ch2 = math.max(1000, math.min(2000, ch2))
        return {roll_rads, pitch_rads, ch1, ch2}
    end,

    -- Per-step PWM slew limiter.
    -- Mirrors the inline rate-limiter applied to ch1/ch2 in run_flight().
    -- max_delta == 0 disables clamping (passes desired through unchanged).
    output_rate_limit = function(desired, prev, max_delta)
        if max_delta == 0 then return desired end
        local d = desired - prev
        if d >  max_delta then d =  max_delta end
        if d < -max_delta then d = -max_delta end
        return prev + d
    end,

    -- Rate-to-PWM scalar conversion.
    -- Mirrors the inline ch1/ch2 computation in run_flight().
    -- acro_rp_rate_deg defaults to ACRO_RP_RATE_DEG (360 deg/s).
    rate_to_pwm = function(rate_rads, acro_rp_rate_deg)
        acro_rp_rate_deg = acro_rp_rate_deg or ACRO_RP_RATE_DEG
        local scale = 500.0 / (acro_rp_rate_deg * math.pi / 180.0)
        local ch = math.floor(1500.0 + scale * rate_rads + 0.5)
        if ch < 1000 then ch = 1000 end
        if ch > 2000 then ch = 2000 end
        return ch
    end,
}
