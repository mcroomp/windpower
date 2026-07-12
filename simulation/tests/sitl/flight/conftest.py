"""
flight/conftest.py — pytest fixtures for RAWES flight stack integration tests.

Fixtures:
    guided_nogps_armed              — full GUIDED_NOGPS stack (mediator + arm).
    guided_nogps_armed_pumping_lua  - starts at IC (trapezoid); pumping test owns winch.
    guided_nogps_armed_landing_lua  — GUIDED_NOGPS stack with rawes.lua in landing mode (RAWES_MODE=4).
    guided_nogps_armed_lua_full     — starts at IC (trapezoid); steady/ic tests (MODE_PASSIVE seed).

All IC-start fixtures share the _ic_trapezoid_stack initialization helper.
"""
import contextlib
import json
import math
import os

import numpy as np

import pytest

from stack_infra import *  # noqa: F401,F403  — re-export everything for test imports
from stack_infra import (
    _acro_stack,
    _RAWES_DEFAULTS_PARM,
    _install_lua_scripts,
    _STARTING_STATE,
    _STARTUP_DAMP_S,
)
from ic import load_ic


# ---------------------------------------------------------------------------
# Fixtures — thin wrappers around _acro_stack
# ---------------------------------------------------------------------------

@pytest.fixture
def guided_nogps_armed(tmp_path, request):
    """Full GUIDED_NOGPS stack fixture. Yields StackContext armed in GUIDED_NOGPS mode."""
    with _acro_stack(tmp_path, test_name=request.node.name) as ctx:
        yield ctx


@pytest.fixture
def guided_nogps_armed_pumping_lua(tmp_path, request):
    """
    Pumping-cycle stack fixture — starts at the IC via the shared trapezoid init.

    Thin wrapper over _ic_trapezoid_stack (run_ground_winch=False): the hub
    starts at the IC operating point at rest, Lua is left in MODE_PASSIVE
    (RAWES_MODE=3) with the IC operating point seeded, and GPS is fused before
    yielding.  The pumping test owns the winch loop from the test process
    (mirroring the test_pump_cycle_lua.py simtest) and promotes RAWES_MODE 3 -> 1
    (MODE_STEADY) right after kinematic_exit.

    Division of labour (mirrors test_pump_cycle_lua.py):
      - Test process: inline 2-phase state machine + GovernedWinchController
        commands sent to the mediator WinchController via UDP, plus RAWES_TEN /
        RAWES_ALT / RAWES_SUB NVF to Lua via GCS.
      - Mediator: WinchController (400 Hz) owns tether rest_length physics.
      - Lua (50 Hz): altitude PID collective + rate-only bz_altitude_hold cyclic.
    """
    import socket as _socket

    # Find a free UDP port for the winch command socket.
    with _socket.socket(_socket.AF_INET, _socket.SOCK_DGRAM) as _s:
        _s.bind(("127.0.0.1", 0))
        _winch_port = _s.getsockname()[1]

    with _ic_trapezoid_stack(
        tmp_path,
        test_name=request.node.name,
        winch_cmd_port=_winch_port,
        run_ground_winch=False,
    ) as ctx:
        yield ctx



@pytest.fixture
def guided_nogps_armed_landing_lua(tmp_path, request):
    """
    Landing stack fixture with rawes.lua active in landing mode (RAWES_MODE=4).

    Extends guided_nogps_armed_landing_lua:
      - kinematic_vel_ramp_s=20: hub exits kinematic at vel=0, eliminating the
        linear tether jolt. Tether extension at exit ~ 0 m, tension ~ 0 N.
      - rawes.lua installed before SITL starts.
      - RAWES_MODE=4 set post-arm so Lua starts in landing mode; KINEMATIC_SETTLE_MS=62000 delays body_z capture
        until EKF has converged.
      What is validated:
          (a) Lua enters landing mode and body_z capture fires on schedule
              ("RAWES land: captured" STATUSTEXT at t~62 s).
          (b) Lua alt_est computation is correct: "RAWES land: final_drop"
              STATUSTEXT fires when alt_est <= LAND_MIN_TETHER_M=2 m.
          (c) Hub descends to floor and tension stays safe (Lua + WinchController).
        Lua's VZ descent and steady-guidance formulas are covered by unit tests
        and test_lua_flight_steady_sitl.

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
      t~15 s      arm complete; fixture sets RAWES_MODE and NVFs; yields to test
      t=45..65 s  kinematic ramp phase: vel ramps 0.96->0 m/s (vel_ramp_s=20)
      t~51 s      ahrs:healthy() True; Lua enters KINEMATIC_SETTLE_MS wait
      t~62 s      Lua KINEMATIC_SETTLE_MS (62 s) expires; captures body_z
      t~62..65 s  Lua sends RC overrides; kinematic still ramps vel to 0
      t=65 s      kinematic exits; hub at pos0 with vel=0, tension~0
      t~65..102 s Lua VZ controller descends hub; WinchController reels in tether
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
                     test_name=request.node.name) as ctx:
        # Post-arm: configure rawes.lua.  Mode via RAWES_MODE; slew + anchor via
        # NAMED_VALUE_FLOAT.
        # RAWES_AND = -pos0[2] = altitude of pos0 above anchor ~ 19.696 m.
        # vel0[2]=0 => altitude constant during kinematic => EKF_ORIGIN.z = pos0[2].
        # anch_EKF.z = RAWES_AND = -pos0[2] = altitude above EKF origin.
        # Lua: alt_est = anch.z - hub_ned.z (hub_ned.z from LOCAL_POSITION_NED).
        #
        # RAWES_MODE=4 set here; rawes.lua DELAYS body_z capture until
        # millis() >= KINEMATIC_SETTLE_MS (62 s) to ensure EKF has converged.
        ctx.log.info("Setting RAWES_MODE + slew/anchor NVFs for rawes.lua (landing mode) ...")
        ctx.gcs.set_param("SCR_ENABLE", 1, timeout=5.0)   # persist scripting in EEPROM
        ctx.gcs.set_param("RAWES_MODE", 4, timeout=5.0)   # landing mode
        # Slew + anchor are NAMED_VALUE_FLOAT.  The anchor is at world origin;
        # here anchor North/East = 0 and anchor Down (EKF frame) = -pos0[2].
        # rawes.lua gates altitude-hold capture on all three anchor floats.
        ctx.gcs.send_named_float("RAWES_ANN", 0.0)                      # anchor North [m]
        ctx.gcs.send_named_float("RAWES_ANE", 0.0)                      # anchor East  [m]
        ctx.gcs.send_named_float("RAWES_AND", -float(extra["pos0"][2])) # anchor Down  [m]
        ctx.log.info("  mode=4 (landing); slew+anchor sent via NVF (anchor D=%.2f)",
                     -float(extra["pos0"][2]))

        ctx.wait_drain(timeout=1.0, label="post-param")
        yield ctx


@contextlib.contextmanager
def _ic_trapezoid_stack(tmp_path, *, test_name, winch_cmd_port, run_ground_winch):
    """
    Shared SITL initialization for fixtures that must START AT THE IC.

    Brings the hub to the IC operating point (pos0) at rest using a smooth
    trapezoidal kinematic motion, seeds the IC operating point into rawes.lua
    (MODE_PASSIVE), and waits for GPS fusion before yielding.  Used by every
    flight fixture that starts at the IC (steady, ic-passive, pumping).

    Parameters
    ----------
    winch_cmd_port   : UDP port the mediator's WinchController listens on.
    run_ground_winch : if True, start an in-fixture ground-side tension regulator
                       thread (used by steady/ic where no test-side winch loop
                       exists).  Pumping passes False and drives the winch itself.

    Both modes leave the Lua in MODE_PASSIVE (RAWES_MODE=3); the test promotes to
    its flight mode (MODE_STEADY=1) right after kinematic_exit.

    Uses a smooth trapezoidal kinematic motion: the hub accelerates from rest to
    1 m/s over the first 5 s, cruises, then decelerates back to rest over the
    final 5 s, travelling along the IC yaw heading so it ends EXACTLY at pos0 with
    zero velocity at kinematic exit (t=60s). The motion gives the EKF velocity
    observability during the hold (helping delAngBiasLearned / GPS aiding) while
    leaving no residual position/velocity error at release.

        Key design points:
            - RAWES_MODE=3 (MODE_PASSIVE) set immediately after arm; Lua does not emit
                rate commands, preventing ArduPilot rate PID windup during kinematic hold.
            - Smooth (raised-cosine) accel/decel => continuous acceleration, no jerk
                step at the phase boundaries.
            - Hub ends exactly at pos0 with zero velocity, so GPS aiding engages with
                no accumulated position mismatch to shock the EKF.
            - Fixture waits for GPS fusion before yielding.
            - Test promotes RAWES_MODE from 3 (MODE_PASSIVE) to 1 (MODE_STEADY) after
                kinematic_exit (t=60s) to activate altitude-hold steady guidance.

        Timeline (from mediator start, speedup=1):
            t=0..5 s    accelerate 0 -> 1 m/s along IC heading (raised cosine).
            t=5..55 s   cruise at 1 m/s along IC heading.
            t=55..60 s  decelerate 1 -> 0 m/s, arriving exactly at pos0 at rest.
            t~6 s       GPS first fix; EKF3 origin set.
            t~8 s       arm (after EKF tilt alignment); RAWES_MODE=3 (MODE_PASSIVE)
                                    set; IC attitude commanded during kinematic hold.
            t~34 s      GPS fuses (delAngBiasLearned converges); _tdir0 fires.
            t~60 s      kinematic exits; test promotes RAWES_MODE 3 -> 1 (MODE_STEADY).
            t~60+       free flight under ArduPilot + Lua with steady guidance active.
    """
    # Standard kinematic start for all flight stack tests on this fixture:
    # a LEVEL frame (roll=pitch=0) YAWED to the IC heading.  The nul-aero
    # cyclic can only apply pitch/roll body rates (never yaw), so the IC
    # heading is the one orientation the body cannot reach via cyclic alone and
    # therefore must be seeded here.  Starting from pure identity (nose North)
    # tilts the disk into the X-Z plane (crosswind), leaving it edge-on to the
    # +Y wind -> ~0 axial inflow -> ~6 N thrust -> free-fall at release.  With
    # the heading pre-set, nul-aero pitch-down lands body_z in the tether/wind
    # plane and autorotation produces full thrust (~334 N) so the tether stays
    # taut.  Roll/pitch stay 0 here and are commanded later via cyclic.
    _ic_R0 = load_ic().R0
    _ic_yaw = math.atan2(_ic_R0[1, 0], _ic_R0[0, 0])  # ZYX yaw of IC attitude
    _cy, _sy = math.cos(_ic_yaw), math.sin(_ic_yaw)
    _R0_level_yaw = [
        [_cy, -_sy, 0.0],
        [_sy,  _cy, 0.0],
        [0.0,  0.0, 1.0],
    ]
    extra = {
        # Kinematic start: level frame yawed to the IC heading (see above).
        "R0": _R0_level_yaw,
        # Keep the EKF pre-arm seed level (live attitude), consistent with the
        # level R0 start.  The IC roll/pitch is applied later via the nul-aero
        # cyclic, NOT seeded into the EKF.  Must be False to match level R0:
        # seeding IC pitch into the EKF while physics starts level would create
        # an attitude mismatch.
        "use_ic_pre_arm_attitude": False,
        # Smooth trapezoidal kinematic motion: accelerate from rest to 1 m/s over
        # the first 5 s, cruise, then decelerate back to rest over the final 5 s,
        # travelling along the IC yaw heading so the hub ends EXACTLY at pos0 with
        # zero velocity at kinematic exit.  Raised-cosine ramps give continuous
        # acceleration (no jerk step).  This gives the EKF velocity observability
        # during the hold (helping delAngBiasLearned / GPS aiding) while leaving
        # no residual position/velocity error at release -- unlike a constant-vel
        # drift, which left the hub ~58 m from pos0 and shocked the EKF when GPS
        # aiding finally engaged.
        "kinematic_cruise_speed": 1.0,
        "kinematic_accel_s": 5.0,
        "kinematic_decel_s": 5.0,
        "kinematic_vel_ramp_s": 0.0,
        "startup_damp_seconds": 60.0,
        # Kinematic debug physics: simplified cyclic->rotation response.
        "kinematic_aero_mode": "nul",
        # Rate gain sized so the body can slew to the IC attitude within ~0.5 s.
        # nul aero rotates at omega_body = gain * tilt; cyclic saturates at
        # tilt ~= 0.556 (H_CYC_MAX = 2500 cdeg / 4500).  Worst-case IC tilt is
        # ~63.55 deg = 1.11 rad, so to cover it in 0.5 s at saturated cyclic:
        #   gain = (1.11 / 0.5) / 0.556 ~= 4.0 rad/s per rad.
        # This keeps attitude error below the crash-check envelope (30 deg / 2 s)
        # during the kinematic hold once the IC attitude command is applied.
        "kinematic_nul_rate_gain_rads_per_rad": 4.0,
        # Mediator-side cyclic handoff smoothing after kinematic release:
        # disabled for tilt-response comparison against the IC-angle-only test.
        "post_release_cyclic_blend_s": 0.0,
        # Enable the mediator's winch command socket so we can run a
        # ground-side tension regulator (mirrors test_create_ic warmup).
        "winch_cmd_port":       winch_cmd_port,
    }
    # Arm AFTER EKF GPS-yaw alignment ("EKF3 IMU0 yaw aligned", ~t=11 s with
    # the moving-baseline dual GPS).  MODE_PASSIVE freezes _passive_hold_yaw_rad
    # at the first ready tick (gated only on ahrs:healthy()), and the IC seed is
    # now sent immediately after arm -- so if we arm before the GPS-yaw snaps to
    # the IC heading (+90 deg), the passive hold latches the EKF's pre-alignment
    # yaw (~0 deg) and the disk tilts into the wrong (crosswind) plane, tumbling
    # at release.  Arming at t=14 s leaves a ~3 s margin past the t=11 s
    # alignment while still giving the nul-aero the bulk of the hold window to
    # slew the disk to the IC tilt before kinematic exit (t=60 s).
    _arm_at_sim_s = 14.0

    with _acro_stack(
        tmp_path,
        extra_config=extra,
        test_name=test_name,
        arm_at_sim_s=_arm_at_sim_s,
    ) as ctx:
        ctx.log.info("Setting mode param + slew/anchor NVFs for rawes.lua ...")
        # The anchor is at world origin; the EKF origin is the hub's first
        # GPS fix = launch position pos0.  So anchor-in-EKF-frame = -pos0.
        # RAWES_ANN/ANE must encode the horizontal offset (negated launch x/y)
        # or the Lua sees the anchor directly below the hub and computes
        # elevation = 90 deg, producing a saturated cyclic that kicks the
        # body at kinematic_exit.
        _pos0 = ctx.initial_state["pos"] if ctx.initial_state else [0.0, 0.0, -ctx.home_alt_m]
        # Mode is the only RAWES_* script-generated param set here; slew + anchor are NAMED_VALUE_FLOAT.
        ctx.gcs.set_param("SCR_ENABLE", 1, timeout=5.0)   # persist scripting in EEPROM
        # MODE_PASSIVE (3): vehicle stays armed (motor interlock kept high) but
        # Lua emits no rate commands, so ArduPilot's rate PID has no setpoint to
        # wind up against while the body is kinematically locked.  The test must
        # promote to MODE_STEADY (1) immediately after kinematic_exit.
        ctx.gcs.set_param("RAWES_MODE", 3, timeout=5.0)   # MODE_PASSIVE
        # rawes.lua gates altitude-hold capture on all three anchor floats arriving.
        ctx.gcs.send_named_float("RAWES_ANN", -float(_pos0[0]))      # anchor North (EKF) [m]
        ctx.gcs.send_named_float("RAWES_ANE", -float(_pos0[1]))      # anchor East  (EKF) [m]
        ctx.gcs.send_named_float("RAWES_AND", float(ctx.home_alt_m)) # anchor Down  (EKF) [m]
        ctx.log.info("  mode=3 (PASSIVE); slew+anchor via NVF (anchor EKF=%.2f,%.2f,%.2f)",
                     -float(_pos0[0]), -float(_pos0[1]), float(ctx.home_alt_m))
        ctx.wait_drain(timeout=1.0, label="post-param")

        # Stream IC collective to Lua so MODE_PASSIVE holds the IC collective
        # through GUIDED throttle and omega_spin doesn't droop while the body is kinematically
        # locked.
        _ic = ctx.initial_state
        if _ic is not None:
            # Seed the IC immediately, together with MODE_PASSIVE, right after
            # arm.  MODE_PASSIVE only commands the IC attitude/collective once
            # the full atomic seed (RAWES_RIC + RAWES_PIC + RAWES_THR) has
            # arrived, so sending it now gives the nul-aero the entire kinematic
            # hold window to slew the disk to the IC tilt before release.

            # Seed Lua with the IC thrust [0..1].
            if "eq_thrust" in _ic:
                _ic_thrust = float(_ic["eq_thrust"])
            else:
                raise KeyError(
                    "initial_state missing thrust seed: eq_thrust"
                )
            ctx.gcs.send_named_float("RAWES_THR", float(_ic_thrust))
            ctx.log.info("IC thrust: %.3f", _ic_thrust)

            # Seed passive IC roll/pitch via short NV names (10-char limit):
            #   RAWES_RIC = IC roll [rad], RAWES_PIC = IC pitch [rad]
            _R0 = _ic.get("R0")
            if _R0 is None:
                raise KeyError("initial_state missing R0 for IC passive attitude seed")
            _r20 = float(_R0[2][0])
            _r21 = float(_R0[2][1])
            _r22 = float(_R0[2][2])
            _ic_roll_rad = math.atan2(_r21, _r22)
            _ic_pitch_rad = -math.asin(max(-1.0, min(1.0, _r20)))
            ctx.gcs.send_named_float("RAWES_RIC", _ic_roll_rad)
            ctx.gcs.send_named_float("RAWES_PIC", _ic_pitch_rad)
            ctx.log.info(
                "IC passive attitude: roll=%+.2f deg pitch=%+.2f deg",
                math.degrees(_ic_roll_rad), math.degrees(_ic_pitch_rad),
            )

            # Stream the IC equilibrium tension so the gravity-compensation disk
            # axis the Lua targets in MODE_STEADY matches the IC that generated
            # this starting state.  Without it the Lua flies with its default
            # _tension_n (200 N), which over-tilts the disk relative to the IC's
            # 300 N equilibrium (k = m*g*cos(el)/tension) and injects an
            # orientation step at the MODE_PASSIVE -> MODE_STEADY handover.
            # Tension feeds the orientation force balance only -- it is position-
            # independent, so this is safe to pin regardless of kinematic drift.
            # (RAWES_ALT is deliberately NOT sent: the Lua captures the live
            # altitude at handover so alt_err=0 with a warm-started collective,
            # avoiding a collective step.  Pinning the IC altitude here would
            # re-introduce that step because the hub drifts ~7 m up during the
            # kinematic ramp.)
            _tension_eq = float(_ic["tension_eq_n"])
            ctx.gcs.send_named_float("RAWES_TEN", _tension_eq)
            ctx.log.info("IC equilibrium tension: %.0f N", _tension_eq)
        ctx.wait_drain(timeout=0.5, label="post-col")

        # Wait for GPS fusion before yielding.
        # Lua needs _tdir0 (fires on GPS fusion) to begin steady guidance.
        # With dual GPS the wait is ~44 s (delAngBiasLearned bottleneck).
        ctx.log.info("Waiting for GPS fusion before yielding (up to 60 s) ...")
        _prior = [str(t).lower() for t in ctx.all_statustext]
        _origin_seen: list[bool] = [any("origin set" in t for t in _prior)]
        _gps_seen: list[bool] = [
            any("is using gps" in t for t in _prior)
            or (_origin_seen[0] and ctx.gcs.sim_now() >= 34.0)
        ]

        def _gps_fused(text: str | None) -> bool:
            if not text:
                return False
            if "is using GPS" in text:
                _gps_seen[0] = True
                return True
            # ArduPilot 4.7 often reports "origin set" earlier than the
            # legacy fusion text. Keep waiting until the historical ~34 s
            # fusion epoch to avoid yielding too early and timing out
            # kinematic_exit in the test body.
            if "origin set" in text:
                _origin_seen[0] = True
            if _origin_seen[0] and ctx.gcs.sim_now() >= 34.0:
                _gps_seen[0] = True
                return True
            return False

        if _gps_seen[0]:
            ctx.log.info("GPS fusion already observed before wait; yielding without extra delay")
        else:
            ctx.wait_drain(
                until       = _gps_fused,
                timeout     = 60.0,
                drain_s     = 1.0,
                check_procs = True,
                label       = "gps-fuse",
            )
        if not _gps_seen[0]:
            raise RuntimeError("GPS did not fuse within 60 s — cannot start steady guidance")
        ctx.log.info("GPS fused — Lua steady guidance active; yielding to test")

        if not run_ground_winch:
            # Pumping (and any caller that owns the winch from the test process)
            # skips the in-fixture regulator and drives the WinchController itself.
            yield ctx
            return

        # ── Ground-side tension-regulating winch ─────────────────────────────
        # Mirrors the test_create_ic warmup pattern: the GovernedWinchNode in
        # the mediator holds tension natively.  Without an active hold command
        # the tether spring mode is undamped after kinematic_exit and the
        # kinematic-> free-flight transient blows up within ~700 ms (tension
        # peaks >1000 N, SITL crashes).
        #
        # The mediator hosts a GovernedWinchController; a hold command
        # (cruise_v=0 at the target tension) makes the governor pay out / reel
        # in just enough to keep tension at the set point.  No length math is
        # needed on the test side.
        import socket as _sock
        import json as _json_w
        import threading as _thr
        import time as _time_w

        _winch_stop = _thr.Event()
        _tension_target_n = 300.0
        _winch_addr  = ("127.0.0.1", winch_cmd_port)
        _winch_sock  = _sock.socket(_sock.AF_INET, _sock.SOCK_DGRAM)
        _winch_sock.bind(("127.0.0.1", 0))
        _winch_sock.settimeout(0.05)

        # Seed the mediator with an initial hold command so it knows our
        # address and starts streaming telemetry back.
        _winch_sock.sendto(_json_w.dumps({
            "cruise_v":       0.0,
            "tension_target": _tension_target_n,
        }).encode(), _winch_addr)

        def _winch_regulator():
            while not _winch_stop.is_set():
                # Drain any pending telemetry from the mediator (we don't need
                # it -- the governor closes the tension loop on its own load
                # cell; we just keep the socket from backing up).
                try:
                    while True:
                        _winch_sock.recvfrom(256)
                except (TimeoutError, _sock.timeout, BlockingIOError):
                    pass

                # Re-issue the hold command at 10 Hz.  The GovernedWinch holds
                # tension natively via cruise_v=0 + tension_target.
                try:
                    _winch_sock.sendto(_json_w.dumps({
                        "cruise_v":       0.0,
                        "tension_target": _tension_target_n,
                    }).encode(), _winch_addr)
                except OSError:
                    pass
                _time_w.sleep(0.1)   # 10 Hz

        _winch_thread = _thr.Thread(target=_winch_regulator, daemon=True,
                                     name="winch-regulator")
        _winch_thread.start()
        ctx.log.info("Ground winch tension regulator started (target=%.0f N)",
                     _tension_target_n)

        try:
            yield ctx
        finally:
            _winch_stop.set()
            _winch_thread.join(timeout=2.0)
            try:
                _winch_sock.close()
            except OSError:
                pass


@pytest.fixture
def guided_nogps_armed_lua_full(tmp_path, request):
    """
    Full-stack GUIDED_NOGPS fixture with rawes.lua, internal_controller=False.

    Thin wrapper over _ic_trapezoid_stack: the hub starts at the IC via the
    smooth trapezoidal kinematic motion, Lua is left in MODE_PASSIVE (RAWES_MODE=3)
    with the IC operating point seeded, and an in-fixture ground winch tension
    regulator (target 300 N) runs after kinematic exit.  Used by the steady and
    ic-passive flight stack tests, which promote RAWES_MODE 3 -> 1 (MODE_STEADY)
    after kinematic_exit.
    """
    with _ic_trapezoid_stack(
        tmp_path,
        test_name=request.node.name,
        winch_cmd_port=14570,
        run_ground_winch=True,
    ) as ctx:
        yield ctx

