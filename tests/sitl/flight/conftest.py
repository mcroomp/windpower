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
import math

import numpy as np
import pytest

from tests.sitl.stack_infra import *  # noqa: F401,F403  — re-export everything for test imports
from tests.sitl.stack_infra import (
    _acro_stack,
    _RAWES_DEFAULTS_PARM,
    _install_lua_scripts,
    _STARTING_STATE,
    _STARTUP_DAMP_S,
    HOME_LAT_DEG,
    HOME_LON_DEG,
    HOME_ALT_M,
)
from linkhub_client.messages import NamedValueFloat, NamedValueInt
from simulation.ic import load_ic
from simulation.sensor import _rotation_matrix_to_euler_zyx
from simulation.torque_model import HubParams, equilibrium_throttle


def _send_anchor_location(ctx) -> None:
    """Send the tether anchor's absolute GPS location to rawes.lua via NAMED_VALUE_INT.

    The anchor is fixed at the mediator's world-frame origin (config.py
    anchor_ned=[0, 0, 0]).  Horizontally, the bare-NED JSON physics backend
    maps this directly onto SITL's fixed launch location (stack_utils.
    HOME_LAT_DEG / HOME_LON_DEG, matching the --custom-location SITL launch
    arg) -- GPS lat/lon is an absolute measurement, so rawes.lua's
    Location:get_vector_from_origin_NEU_m() resolves the correct N/E offset
    regardless of where the EKF origin actually lands.

    Altitude is different: ArduPilot's vertical estimate is baro-anchored,
    so whatever altitude the vehicle physically occupies when the EKF's
    local origin is established becomes its "zero" -- a real sensor
    behavior (baro is zeroed at boot), not a simulation artifact.  This
    stack test boots the vehicle already kinematically locked ctx.home_alt_m
    metres above the anchor, so the EKF's local origin altitude ends up that
    far above the anchor's true altitude.  The anchor's sent altitude must
    therefore be offset by -ctx.home_alt_m so it resolves to the correct
    (positive-down) D offset once EKF-local.

    Sent as NAMED_VALUE_INT (not NAMED_VALUE_FLOAT): lat/lon in degrees lose
    ~0.5-1 m of precision when round-tripped through a float32 NVF wire
    field, whereas NAMED_VALUE_INT carries the same int32 degrees*1e7 / cm
    representation ArduPilot's own Location object uses natively.
    """
    anchor_alt_m = HOME_ALT_M - ctx.home_alt_m
    lat_e7 = round(HOME_LAT_DEG * 1e7)
    lon_e7 = round(HOME_LON_DEG * 1e7)
    alt_cm = round(anchor_alt_m * 100)
    ctx.gcs.send_message(NamedValueInt("RAWES_LAT", lat_e7))
    ctx.gcs.send_message(NamedValueInt("RAWES_LON", lon_e7))
    ctx.gcs.send_message(NamedValueInt("RAWES_AAL", alt_cm))
    ctx.log.info(
        "  anchor location sent via NVI (lat=%.7f lon=%.7f alt=%.1fm, "
        "home_alt_m=%.2f)",
        HOME_LAT_DEG, HOME_LON_DEG, anchor_alt_m, ctx.home_alt_m,
    )


def _wait_and_configure_mode_anchor(
    ctx,
    *,
    mode: int,
    mode_label: str,
) -> None:
    """Send the anchor location once, then set RAWES_MODE."""
    _send_anchor_location(ctx)
    ctx.gcs.set_param("RAWES_MODE", int(mode), timeout=5.0)
    ctx.log.info("  mode=%s set", mode_label)


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
        message_rates={
            "ATTITUDE": 10.0,
            "EKF_STATUS_REPORT": 10.0,
            "LOCAL_POSITION_NED": 10.0,
            "GLOBAL_POSITION_INT": 5.0,
            "RC_CHANNELS": 2.0,
        },
    ) as ctx:
        if not ctx.gcs.set_param("RAWES_TEL_HZ", 0.5, timeout=5.0):
            pytest.fail("Failed to reduce pumping diagnostic telemetry rate")
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
      t~62..65 s  Lua sends GUIDED setpoints; kinematic still ramps vel to 0
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
        # Post-arm: configure rawes.lua for landing mode.
        # rawes.lua delays body_z capture until KINEMATIC_SETTLE_MS, so this can
        # be applied before kinematic exit without premature guidance capture.
        ctx.log.info("Setting RAWES_MODE + anchor NVI for rawes.lua (landing mode) ...")
        _wait_and_configure_mode_anchor(ctx, mode=4, mode_label="4 (landing)")

        ctx.wait_drain(timeout=1.0, label="post-param")
        yield ctx


@contextlib.contextmanager
def _ic_trapezoid_stack(
    tmp_path,
    *,
    test_name,
    winch_cmd_port,
    run_ground_winch,
    message_rates=None,
):
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
            - RAWES_MODE=3 (MODE_PASSIVE) set immediately after arm; Lua captures
                the level-yaw anchor and applies the relative rotation to the IC.
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
                                    set; level-yaw anchor captured, then slewed to IC.
            t~34 s      GPS fuses (delAngBiasLearned converges); _tdir0 fires.
            t~60 s      kinematic exits; test promotes RAWES_MODE 3 -> 1 (MODE_STEADY).
            t~60+       free flight under ArduPilot + Lua with steady guidance active.
    """
    # Start level at the IC heading so ArduPilot can arm cleanly. Passive mode
    # captures this AHRS quaternion after arming, then applies the relative
    # rotation from this anchor to the IC attitude.
    _ic_R0 = load_ic().R0
    _ic_yaw = math.atan2(_ic_R0[1, 0], _ic_R0[0, 0])
    _cy, _sy = math.cos(_ic_yaw), math.sin(_ic_yaw)
    _R0_level_yaw = np.array([
        [_cy, -_sy, 0.0],
        [_sy,  _cy, 0.0],
        [0.0,  0.0, 1.0],
    ])
    _ic_relative_rpy = _rotation_matrix_to_euler_zyx(_R0_level_yaw.T @ _ic_R0)
    extra = {
        "R0": _R0_level_yaw.tolist(),
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
        # Simplified cyclic response slews from the captured level-yaw anchor to
        # the IC attitude while translation follows the startup trajectory.
        "kinematic_aero_mode": "nul",
        "kinematic_nul_rate_gain_rads_per_rad": 4.0,
        # Mediator-side cyclic handoff smoothing after kinematic release:
        # disabled for tilt-response comparison against the IC-angle-only test.
        "post_release_cyclic_blend_s": 0.0,
        # Enable the mediator's winch command socket so we can run a
        # ground-side tension regulator (mirrors test_create_ic warmup).
        "winch_cmd_port":       winch_cmd_port,
    }
    # MODE_PASSIVE captures the current AHRS quaternion when RAWES_PEN arrives,
    # so capturing before GPS-yaw alignment would freeze the pre-alignment
    # heading (~0 deg) instead of the IC heading (+90 deg).
    _arm_at_sim_s = 14.0

    with _acro_stack(
        tmp_path,
        extra_config=extra,
        test_name=test_name,
        arm_at_sim_s=_arm_at_sim_s,
        message_rates=message_rates,
        require_yaw_alignment=True,
    ) as ctx:
        # MODE_PASSIVE (3) is set immediately after arm, decoupled from anchor
        # calibration below (which needs telemetry/EKF data that can take
        # longer to become ready). Lua captures the armable level-yaw attitude,
        # then the relative offsets below command the IC attitude.
        ctx.log.info("Setting RAWES_MODE=3 (PASSIVE) immediately after arm ...")
        ctx.gcs.set_param("RAWES_MODE", 3, timeout=5.0)

        # Stream IC collective to Lua so MODE_PASSIVE holds the IC collective
        # through GUIDED throttle and omega_spin doesn't droop while the body is kinematically
        # constrained.
        _ic = ctx.initial_state
        if _ic is not None:
            # Seed thrust immediately. Lua captures the settled AHRS quaternion
            # as the passive anchor; the ground sends only relative offsets.

            # Seed Lua with the IC thrust [0..1].
            if "eq_thrust" in _ic:
                _ic_thrust = float(_ic["eq_thrust"])
            else:
                raise KeyError(
                    "initial_state missing thrust seed: eq_thrust"
                )
            ctx.gcs.send_message(NamedValueFloat("RAWES_THR", float(_ic_thrust)))
            ctx.log.info("IC thrust: %.3f", _ic_thrust)

            _roll_offset, _pitch_offset, _yaw_offset = _ic_relative_rpy
            ctx.gcs.send_message(NamedValueFloat("RAWES_ROFF", float(_roll_offset)))
            ctx.gcs.send_message(NamedValueFloat("RAWES_POFF", float(_pitch_offset)))
            ctx.gcs.send_message(NamedValueFloat("RAWES_YOFF", float(_yaw_offset)))
            ctx.gcs.send_message(NamedValueFloat("RAWES_PEN", 1.0))
            ctx.log.info(
                "Lua passive anchor captured; IC-relative offsets: roll=%+.2f pitch=%+.2f yaw=%+.2f deg",
                math.degrees(_roll_offset),
                math.degrees(_pitch_offset),
                math.degrees(_yaw_offset),
            )

            # Stream the IC equilibrium tension and target altitude from the IC.
            # Tension feeds the orientation force balance; altitude is the
            # physics-truth IC altitude (ctx.home_alt_m), not a live EKF
            # reading, avoiding the ~2.5 m EKF vertical convergence-lag bias
            # present at capture time.
            _tension_eq = float(_ic["tension_eq_n"])
            ctx.gcs.send_message(NamedValueFloat("RAWES_TEN", _tension_eq))

            # Anchor location is a static constant, sent EXACTLY ONCE here
            # (after the IC seed values are queued) -- it does not depend on
            # telemetry/EKF availability, so it can be sent right away without
            # gating the MODE_PASSIVE assignment above.
            _send_anchor_location(ctx)

            # Altitude above anchor at IC = ctx.home_alt_m (precomputed in
            # stack_infra._acro_stack as -initial_state["pos"][2]; the anchor
            # sits at the mediator's world-frame origin so this equals the
            # physics-truth altitude directly -- no telemetry/EKF calibration
            # needed).
            _alt_ic = float(ctx.home_alt_m)

            ctx.gcs.send_message(NamedValueFloat("RAWES_ALT", _alt_ic))
            ctx.log.info("IC equilibrium tension: %.0f N  target altitude: %.1f m",
                         _tension_eq, _alt_ic)

            # Seed the yaw-motor trim equilibrium for the IC/release rotor spin
            # rate (same torque_model.equilibrium_throttle() calc physics_core.py
            # uses to initialize the frozen hub ODE state).  rawes.lua holds this
            # directly in H_YAW_TRIM throughout MODE_PASSIVE instead of trying to
            # derive it from a (kinematically-locked, hence meaningless) psi_dot
            # readback -- so the real SERVO9 PWM already matches the yaw-motor
            # ODE's equilibrium by the time of kinematic release, avoiding a
            # step-input torque mismatch that spins the hub. See design/flight_stack.md
            # "Yaw observer in passive mode" and repo memory sitl-param-verify-and-yaw-ff.md.
            _yff_seed = equilibrium_throttle(float(_ic["omega_spin"]), HubParams())
            ctx.gcs.send_message(NamedValueFloat("RAWES_YFF", _yff_seed))
            ctx.log.info("IC yaw-trim equilibrium seed: %.3f", _yff_seed)
        ctx.wait_drain(timeout=1.0, label="post-param")
        ctx.wait_drain(timeout=0.5, label="post-col")

        # Wait for EKF local position before yielding. Lua needs the same fused
        # position state to initialise _tdir0; STATUSTEXT wording is not a
        # synchronization interface.
        ctx.log.info("Waiting for GPS fusion before yielding (up to 60 s) ...")
        _gps_seen = False
        _gps_deadline = ctx.gcs.sim_now() + 60.0
        _gps_cursor = ctx.gcs.current_cursor()
        while ctx.gcs.sim_now() < _gps_deadline:
            _batch = ctx.gcs.read_messages(
                _gps_cursor,
                ["LOCAL_POSITION_NED", "STATUSTEXT"],
                wait=0.5,
                limit=1,
            )
            _gps_cursor = _batch.next_cursor
            _msg = _batch.messages[0] if _batch.messages else None
            if _msg is None:
                continue
            _decoded = decode_message(_msg)
            if isinstance(_decoded, LocalPositionNed):
                ctx.last_local_position_ned = (
                    _decoded.x, _decoded.y, _decoded.z,
                    _decoded.vx, _decoded.vy, _decoded.vz,
                )
                _gps_seen = True
                break
            if isinstance(_decoded, StatusText):
                ctx.all_statustext.append(_decoded.text)
                ctx.log.info("STATUSTEXT [gps-fuse]: %s", _decoded.text)
        if not _gps_seen:
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
