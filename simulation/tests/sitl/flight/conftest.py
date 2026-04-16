"""
flight/conftest.py — pytest fixtures for RAWES flight stack integration tests.

Fixtures:
  acro_armed              — full ACRO stack (mediator + arm).
  acro_armed_pumping_lua  — ACRO stack with rawes.lua in pumping mode (SCR_USER6=5).
  acro_armed_landing_lua  — ACRO stack with rawes.lua in landing mode (SCR_USER6=4).
  acro_armed_lua_full     — ACRO stack with rawes.lua in flight mode (SCR_USER6=1),
                            internal_controller=False.
"""
import math
import os

import pytest

from stack_infra import *  # noqa: F401,F403  — re-export everything for test imports
from stack_infra import (
    _acro_stack,
    _BASE_ACRO_PARAMS,
    _RAWES_DEFAULTS_PARM,
    _install_lua_scripts,
    _STARTUP_DAMP_S,
)


# ---------------------------------------------------------------------------
# Fixtures — thin wrappers around _acro_stack
# ---------------------------------------------------------------------------

@pytest.fixture
def acro_armed(tmp_path, request):
    """Full ACRO stack fixture. Yields StackContext armed in ACRO mode."""
    with _acro_stack(tmp_path, test_name=request.node.name) as ctx:
        yield ctx


@pytest.fixture
def acro_armed_pumping_lua(tmp_path, request):
    """
    Pumping-cycle stack fixture with rawes.lua active in pumping mode (SCR_USER6=5).

    Division of labour:
      - Mediator: DeschutterPlanner owns phase state machine, winch, TensionPI.
      - Lua (Pixhawk, 50 Hz): cyclic orbit tracking + per-phase body_z slerp +
        per-phase collective (fixed open-loop: COL_REEL_OUT / COL_REEL_IN).
      - Synchronisation: Lua detects tether paying-out / reeling-in to follow
        the mediator's winch motion without any RC channel bridge.

    The mediator trajectory=deschutter config matches acro_armed_pumping
    (same phase timing, tension targets, winch speeds).

    internal_controller=True (same as acro_armed_pumping): at the orbital
    equilibrium the tether acts mostly HORIZONTALLY and slightly DOWNWARD
    (anchor is 8 deg below the hub, not above it), so tether tension at
    kinematic exit gives a NET DOWNWARD force on the hub (~22 m/s^2), not
    upward support.  Lua's 50 Hz RC overrides cannot recover from this
    kinematic-exit jolt; the internal 400 Hz controller provides the
    stability needed.  This matches CLAUDE.md exception (2): "Pumping cycle
    fixture -- ArduPilot RC overrides at 10 Hz cannot stabilize the hub
    after the kinematic-exit tether jolt."

    What is validated here vs acro_armed_pumping (no Lua):
      - Lua SCR_USER6=5 starts without error.
      - Lua detects winch reel-out from tether length change ->
        "RAWES pump: reel_out" STATUSTEXT appears.
      - Pumping cycle physics (TensionPI + OrbitTracker) remain unchanged.
    Lua's cyclic orbit tracking and collective logic (COL_REEL_OUT / COL_REEL_IN)
    are validated separately by test_lua_flight_steady (cyclic) and unit
    tests (collective formula).
    """
    import config as _mcfg
    _dcfg = _mcfg.DEFAULTS["trajectory"]["deschutter"]
    extra = {
        # kinematic_vel_ramp_s=0: keep vel0 constant throughout the 65 s kinematic
        # so GPS innovations stay small and EKF fuses GPS during kinematic (~37-54 s).
        # rawes_sitl_defaults.parm is tuned for this constant-velocity pattern
        # (EK3_GPS_CHECK=0, widened gates).  Lua captures ~2 s after GPS fuses.
        "kinematic_vel_ramp_s": 0.0,
        "trajectory": {
            "type": "deschutter",
            "hold": {},
            "deschutter": {
                **{k: _dcfg[k] for k in (
                    "t_reel_out", "t_reel_in", "t_transition",
                    "v_reel_out", "v_reel_in", "tension_out", "tension_in",
                )},
                "t_hold_s": 10.0,
            },
        },
    }
    with _acro_stack(tmp_path, extra_config=extra,
                     log_name="acro_armed_pumping_lua", log_prefix="pumping_lua",
                     test_name=request.node.name,

                     # internal_controller=False: SITL stack tests must let ArduPilot
                     # + Lua drive physics (CLAUDE.md critical rule).  Lua RC overrides
                     # at 50 Hz control cyclic; collective is also via Lua Ch3.
                     internal_controller=False) as ctx:
        # Post-arm: configure rawes.lua via SCR_USER params.
        ctx.log.info("Setting SCR_USER params for rawes.lua (pumping mode) ...")
        lua_params = {
            "SCR_ENABLE": 1,
            "SCR_USER1": 1.0,             # RAWES_KP_CYC   [rad/s / rad]
            "SCR_USER2": 0.40,            # RAWES_BZ_SLEW  [rad/s]
            "SCR_USER3": 0.0,             # anchor North   [m]
            "SCR_USER4": 0.0,             # anchor East    [m]
            "SCR_USER5": ctx.home_alt_m,  # anchor Down from EKF HOME [m]
            "SCR_USER6": 5,               # RAWES_MODE = 5 (pumping)
        }
        for pname, pvalue in lua_params.items():
            ok = ctx.gcs.set_param(pname, pvalue, timeout=5.0)
            ctx.log.info("  %-12s = %g  ACK=%s", pname, pvalue, ok)

        # Lua owns Ch1/Ch2/Ch3/Ch8 -- rawes.lua sends Ch8=2000 keepalive when armed.
        # Tests must NOT send RC overrides.

        ctx.wait_drain(timeout=1.0, label="post-param")
        yield ctx


@pytest.fixture
def acro_armed_landing_lua(tmp_path, request):
    """
    Landing stack fixture with rawes.lua active in landing mode (SCR_USER6=4).

    Extends acro_armed_landing:
      - internal_controller=False: ArduPilot + Lua own the physics (CLAUDE.md
        critical rule -- SITL tests must validate the actual control loop).
      - kinematic_vel_ramp_s=20: hub exits kinematic at vel=0, eliminating the
        linear tether jolt that required internal_controller=True in prior attempts.
        Tether extension at exit ~ 0 m, tension ~ 0 N; Lua's 50 Hz is sufficient.
      - rawes.lua installed before SITL starts.
      - SCR_USER1..5 configured post-arm immediately; SCR_USER6=4 set so Lua
        starts in landing mode; KINEMATIC_SETTLE_MS=62000 delays body_z capture
        until EKF has converged.
      What is validated:
          (a) Lua enters landing mode and body_z capture fires on schedule
              ("RAWES land: captured" STATUSTEXT at t~62 s).
          (b) Lua alt_est computation is correct: "RAWES land: final_drop"
              STATUSTEXT fires when alt_est <= LAND_MIN_TETHER_M=2 m.
          (c) Hub descends to floor and tension stays safe (Lua + LandingPlanner).
        Lua's VZ descent and orbit-tracking formulas are covered by unit tests
        and test_lua_flight_steady.

    Hub starts at tether equilibrium with xi=80 deg (10 deg from horizontal).
    Matches test_landing.py: BZ_INIT=[0, cos(80), -sin(80)], pos0=20*BZ_INIT.
      - BEM valid: chi=80 deg < 85 deg limit (chi=90 = horizontal disk fails).
      - body_z=[0,0,-1] (horizontal) is outside SkewedWakeBEM valid range and
        produces degenerate negative thrust (-762 N) causing immediate free-fall.
      - Hub at tether equilibrium: pos0 = tether_rest_length * body_z, so
        tether is nearly slack at kinematic exit (extension ~ 0 m).
      - orb_yaw for body_z=[0, 0.174, -0.985] = +pi/2 (East). Matches
        vel0=[0, 0.96, 0] yaw=+pi/2, so no GPS Glitch at kinematic exit.
      - vel0[2]=0: altitude constant during kinematic => EKF_ORIGIN.z = pos0[2].

    Timing (from mediator start, speedup=1):
      t=0..45 s   kinematic constant-velocity phase (vel=0.96 m/s East)
      t~23 s      GPS fuses (EK3_GPS_CHECK=0 + widened gates)
      t~15 s      arm complete; fixture sets SCR_USER1-6; yields to test
      t=45..65 s  kinematic ramp phase: vel ramps 0.96->0 m/s (vel_ramp_s=20)
      t~51 s      ahrs:healthy() True; Lua enters KINEMATIC_SETTLE_MS wait
      t~62 s      Lua KINEMATIC_SETTLE_MS (62 s) expires; captures body_z
      t~62..65 s  Lua sends RC overrides; kinematic still ramps vel to 0
      t=65 s      kinematic exits; hub at pos0 with vel=0, tension~0
      t~65..102 s Lua VZ controller descends hub; LandingPlanner reels in winch
      t~102 s     Lua triggers final_drop STATUSTEXT (alt_est <= 2 m)
      fixture yields at t~15 s; test observes for 165 s (until t~180 s SITL)
    """
    _xi_rad = math.radians(80.0)
    _tether_m = 20.0
    extra = {
        # kinematic_vel_ramp_s=20: hub velocity ramps from 0.96 m/s (East) to 0
        # over the last 20 s of the kinematic phase (t=45..65 s), so hub arrives at
        # pos0 with vel=0.  This eliminates the linear tether jolt: at kinematic
        # exit the hub is stationary at the tether equilibrium point, tether
        # extension ~ 0, tension ~ 0.  GPS fuses during the constant-velocity phase
        # (t ~ 23 s; EK3_GPS_CHECK=0 + widened gates in rawes_sitl_defaults.parm).
        "kinematic_vel_ramp_s": 20.0,
        # pos0: hub at tether equilibrium for xi=80 deg (matches test_landing.py).
        # tether direction = body_z, so tether is nearly slack at kinematic exit.
        "pos0":              [0.0,
                              math.cos(_xi_rad) * _tether_m,   # ~3.473 m East
                              -math.sin(_xi_rad) * _tether_m], # ~-19.696 m (alt 19.7 m)
        # vel0 points East; EKF establishes yaw=+pi/2 (East) during kinematic.
        # orb_yaw for body_z=[0, cos(80), -sin(80)] is also +pi/2 (East), so no
        # GPS Glitch at kinematic exit.
        "vel0":              [0.0, 0.96, 0.0],
        "body_z":            [0.0, math.cos(_xi_rad), -math.sin(_xi_rad)],
        "omega_spin":        20.0,
        "tether_rest_length": _tether_m,
        "trajectory": {
            "type":    "landing",
            "landing": {
                # tension_target_n: at xi=80 deg hover the equilibrium tether
                # tension is ~190 N (hub weight + thrust vertical imbalance).
                # The default (80 N) was designed for orbital transition where
                # tether tension is low.  At 80 N the PI pays OUT instead of
                # reeling in.  200 N keeps PI in reel-in mode throughout descent.
                "tension_target_n": 200.0,
            },
        },
    }
    with _acro_stack(tmp_path, extra_config=extra,
                     log_name="acro_armed_landing_lua", log_prefix="landing_lua",
                     test_name=request.node.name,

                     # internal_controller=False: SITL stack tests must let
                     # ArduPilot + Lua drive physics (CLAUDE.md critical rule).
                     # kinematic_vel_ramp_s=20 ensures vel=0 at kinematic exit,
                     # eliminating the linear tether jolt.  Lua's 50 Hz is
                     # sufficient for the residual angular perturbations.
                     internal_controller=False) as ctx:
        # Post-arm: configure rawes.lua via SCR_USER params.
        # SCR_USER5 = -pos0[2] = altitude of pos0 above anchor ~ 19.696 m.
        # vel0[2]=0 => altitude constant during kinematic => EKF_ORIGIN.z = pos0[2].
        # anch_EKF.z = SCR_USER5 = -pos0[2] = altitude above EKF origin.
        # Lua: alt_est = anch.z - hub_ned.z (hub_ned.z from LOCAL_POSITION_NED).
        #
        # SCR_USER6=4 set here; rawes.lua DELAYS body_z capture until
        # millis() >= KINEMATIC_SETTLE_MS (62 s) to ensure EKF has converged.
        ctx.log.info("Setting SCR_USER params for rawes.lua (landing mode) ...")
        lua_params = {
            "SCR_ENABLE": 1,                    # persist scripting in EEPROM for future boots
            "SCR_USER1": 1.0,                   # RAWES_KP_CYC   [rad/s / rad]
            "SCR_USER2": 0.40,                  # RAWES_BZ_SLEW  [rad/s]
            "SCR_USER3": 0.0,                   # anchor North   [m]
            "SCR_USER4": 0.0,                   # anchor East    [m]
            "SCR_USER5": -extra["pos0"][2],     # anchor Down in EKF frame = 20.0 m
            "SCR_USER6": 4,                     # RAWES_MODE = 4 (landing)
        }
        for pname, pvalue in lua_params.items():
            ok = ctx.gcs.set_param(pname, pvalue, timeout=5.0)
            ctx.log.info("  %-12s = %g  ACK=%s", pname, pvalue, ok)

        ctx.wait_drain(timeout=1.0, label="post-param")
        yield ctx


@pytest.fixture
def acro_armed_lua_full(tmp_path, request):
    """
    Full-stack ACRO fixture with rawes.lua in flight mode (SCR_USER6=1),
    internal_controller=False.

    Uses the same initial conditions as every other passing stack test:
    pos0/body_z/omega_spin/rest_length from steady_state_starting.json
    (natural orbit equilibrium at xi~8 deg, 100 m tether, altitude~14 m);
    vel0 from config.py defaults (~0.96 m/s orbital velocity).

    This mirrors test_closed_loop_60s.py (the validating simtest) exactly:
    same position, same orbit, but Lua owns cyclic + collective instead of
    the internal Python controller.

    Key design points:
      - internal_controller=False: ArduPilot + Lua own physics (CLAUDE.md rule).
      - kinematic_traj_type=fast_circle (n_fast=1, v_fast=5 m/s, r=5 m):
        Phases 1-3 (hold→accel→const→decel) complete at t=44.4 s.
        Phase 4 (large orbit at 0.96 m/s) runs from t=44.4 s until kinematic exit
        at t=80 s.  Hub exits at the large orbit with omega~0.01 rad/s (tiny).
        Orbital kinematic alone (0.56 deg/s) causes EKFGSF to run for 60-97 s
        without converging, leading to FPE in EKFGSF_yaw::predict() → GPS Glitch
        → hub crashes at kinematic exit.
      - v_fast=5 m/s: Phase 1/2/3 heading rotates at 1 rad/s → EKFGSF converges
        quickly.  But velNEDdiff=5 m/s > 1 m/s during Phase 1/2/3 so GPS cannot
        fuse yet.  GPS fuses in Phase 4 (v=0.96 m/s, velNEDdiff=0.96<1 m/s) at
        t~76 s after ~32 s of slow heading rotation (0.0097 rad/s).
      - SCR_USER6=0 during fast circles and decel (hub near level, wrong equilibrium).
        SCR_USER6=1 set at Phase 4 start (t~45 s): hub is in large-orbit tether
        orientation.  Pre-GPS bypass holds col_cruise while _tdir0=nil.
        GPS tether recapture fires at t~76 s (_tdir0 set, orbit tracking active).
        Hub exits kinematic at t=80 s with 4 s of active orbit tracking.
      - COL_CRUISE_FLIGHT_RAD=-0.18 rad in rawes.lua matches stack_coll_eq.
      - Lua VZ altitude-hold (vz_sp=0): bidirectional P controller around
        COL_CRUISE_FLIGHT_RAD.  Mediator decode is asymmetric (col_min=-0.28,
        col_max=0.10) matching Lua's encoding exactly.

    Timeline (from mediator start, speedup=1):
      t=0..15 s    Phase 0: stationary hold.
      t=15..27.6 s Phase 1: accel circle (v 0→5 m/s, r=5 m, omega up to 57 deg/s).
      t=27.6..33.9 s Phase 2: constant-speed circle (v=5 m/s).
      t=33.9..44.4 s Phase 3: decel circle (v 5→0.96 m/s).
      t~11 s       GPS first fix.
      t~10 s       arm complete; fixture sets SCR_USER1-6 (SCR_USER6=0).
      t~19 s       GPS origin set.
      t=44.4..80 s Phase 4: large orbit (r=99 m, v=0.96 m/s, omega=0.56 deg/s).
      t~45 s       fixture wait expires; SCR_USER6=1 set.
                   Lua pre-GPS bypass: col=col_cruise, _tdir0=nil.
      t~76 s       GPS fuses (yawAlignComplete + delAngBiasLearned in Phase 4).
                   GPS tether recapture: _tdir0 set, orbit tracking active.
      t=80 s       kinematic exits at large-orbit position; omega~0.01 rad/s.
      t~80+        free flight under ArduPilot + Lua; Lua already tracking orbit.

    Fixture yields at Phase 4 start (~t=45 s).  wait_kinematic_done() in the test
    waits for the remaining ~35 s of Phase 4.  STATUSTEXTs from the kinematic
    (GPS fusion at t~76 s, Lua capture) are in ctx.all_statustext.
    """
    extra = {
        # Fast-circle kinematic for reliable EKFGSF convergence + clean kinematic exit.
        #
        # v_fast=5 m/s (57 deg/s heading): fast EKFGSF convergence during Phase 1/2/3.
        # velNEDdiff=5 m/s > 1 m/s threshold → GPS cannot fuse during Phase 1/2/3.
        # Phase 4 (large orbit, v=0.96 m/s, velNEDdiff=0.96 < 1 m/s): GPS fuses ~32 s
        # into Phase 4 (t~76 s) after EKFGSF converges from slow heading rotation.
        #
        # CRITICAL: kinematic must exit during Phase 4 (large orbit, omega~0.01 rad/s).
        # Phase timings with v_fast=5, r=5, n_fast=1, T_lead=10, t_hold=15:
        #   T_accel = 4pi*5/5 = 12.6 s     (Phase 1: 0→5 m/s over 2pi)
        #   T_per   = 2pi*5/5 =  6.3 s     (Phase 2: constant 5 m/s, n_fast=1 circle)
        #   T_decel = 2pi*5/2.98 = 10.5 s  (Phase 3: 5→0.96 m/s, mean=2.98)
        #   t_d     = 15+12.6+6.3+10.5 = 44.4 s  (Phase 4 start)
        #   t_total = 44.4+10 = 54.4 s     (trajectory end, but kinematic keeps running
        #                                    in Phase 4 beyond t_total until duration=80 s)
        # startup_damp_seconds=80 s → kinematic exits at t=80 s (35.6 s into Phase 4).
        # At exit: position = large-orbit point (r=99 m), omega~0.01 rad/s (tiny). ✓
        #
        # Contrast: v_fast=0.96 m/s gives t_d=145.8 s >> startup_damp=80 s, so the
        # kinematic exits during Phase 1 (accel) with omega~0.19 rad/s → hub tumbles.
        "kinematic_traj_type":       "fast_circle",
        "kinematic_orbital_dir":     1,
        "kinematic_fast_speed":      5.0,    # v_fast [m/s]: fast EKFGSF convergence
        "kinematic_circle_radius":   5.0,    # r_circle [m]: omega_fast = 57 deg/s
        "kinematic_fast_circles":    1,      # 1 constant-speed circle after accel
        "kinematic_orbit_lead_s":    10.0,   # T_lead [s] on large orbit before t_total
        "vel0":                      [0.0, 0.96, 0.0],
        # startup_damp=80 s > t_total=54.4 s: kinematic exits at t=80 s in Phase 4.
        "startup_damp_seconds":      80.0,
    }
    with _acro_stack(tmp_path, extra_config=extra,
                     log_name="acro_armed_lua_full", log_prefix="",
                     test_name=request.node.name,

                     # CLAUDE.md critical rule: internal_controller=False for all
                     # full-stack flight tests.  ArduPilot + Lua own the physics.
                     internal_controller=False) as ctx:
        # Post-arm: configure rawes.lua params.  SCR_USER6=0 (Lua inactive) until
        # Phase 3 (large orbit) starts — avoids Lua capturing wrong equilibrium
        # while hub is level during fast circles (Phases 1 and 2).
        ctx.log.info("Setting SCR_USER params for rawes.lua (SCR_USER6=0 until Phase 3) ...")
        lua_params = {
            "SCR_ENABLE": 1,              # persist scripting in EEPROM
            "SCR_USER1": 1.0,             # RAWES_KP_CYC   [rad/s / rad]
            "SCR_USER2": 0.40,            # RAWES_BZ_SLEW  [rad/s]
            "SCR_USER3": 0.0,             # anchor North   [m]
            "SCR_USER4": 0.0,             # anchor East    [m]
            "SCR_USER5": ctx.home_alt_m,  # anchor Down from EKF HOME [m]
            "SCR_USER6": 0,               # inactive during fast circles
        }
        for pname, pvalue in lua_params.items():
            ok = ctx.gcs.set_param(pname, pvalue, timeout=5.0)
            ctx.log.info("  %-12s = %g  ACK=%s", pname, pvalue, ok)

        ctx.wait_drain(timeout=1.0, label="post-param")

        # Wait for fast circles + decel circle to finish before enabling Lua.
        # Phase boundary timing (deterministic at speedup=1):
        #   T_accel  = 4*pi*5/5.0 = 12.6 s  (Phase 1: accel circle, v_fast=5)
        #   T_per    = 2*pi*5/5.0 =  6.3 s  (Phase 2: constant circle, n_fast=1)
        #   T_decel  = 2*pi*5/2.98 = 10.5 s (Phase 3: decel 5→0.96, mean=2.98)
        #   t_d      = 15+12.6+6.3+10.5 = 44.4 s from mediator start (Phase 4 start)
        #   Arm fires  ~ 10 s from mediator start
        #   Wait after arm: 44.4 - 10 = 34.4 s -> use 35 s
        _PHASE3_WAIT_S = 35.0
        ctx.log.info(
            "Waiting %.0f s for fast circle + decel to finish before enabling Lua ...",
            _PHASE3_WAIT_S,
        )
        ctx.wait_drain(timeout=_PHASE3_WAIT_S, check_procs=True, label="circles-wait")
        ok = ctx.gcs.set_param("SCR_USER6", 1, timeout=5.0)
        ctx.log.info("SCR_USER6=1 enabled (Phase 3 started)  ACK=%s", ok)

        ctx.wait_drain(timeout=1.0, label="post-lua-enable")
        yield ctx
