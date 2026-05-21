"""
flight/conftest.py — pytest fixtures for RAWES flight stack integration tests.

Fixtures:
  acro_armed              — full ACRO stack (mediator + arm).
  acro_armed_pumping_lua  — ACRO stack with rawes.lua in pumping mode (SCR_USER6=5).
  acro_armed_landing_lua  — ACRO stack with rawes.lua in landing mode (SCR_USER6=4).
  acro_armed_lua_full     — ACRO stack with rawes.lua in flight mode (SCR_USER6=1).
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

    Division of labour (mirrors test_pump_cycle_lua.py):
      - Test process: PumpingGroundController (10 Hz) — phase state machine.
          * Sends winch set_target commands to mediator via UDP socket.
          * Sends NVF (RAWES_TSP, RAWES_TEN, RAWES_ALT, RAWES_SUB) to Lua via GCS.
      - Mediator: WinchController (400 Hz) — owns tether rest_length physics.
          * Listens on winch_cmd_port for {target_length, target_tension} commands.
          * Sends back {tension_n, rest_length, hub_alt_m} at ~10 Hz.
      - Lua (50 Hz): TensionPI collective + bz_altitude_hold cyclic.

    This mirrors the real hardware architecture: ground station manages phase
    logic and MAVLink NVF delivery; winch node owns motor control.

    """
    import socket as _socket

    # Find a free UDP port for the winch command socket.
    with _socket.socket(_socket.AF_INET, _socket.SOCK_DGRAM) as _s:
        _s.bind(("127.0.0.1", 0))
        _winch_port = _s.getsockname()[1]

    extra = {
        "kinematic_vel_ramp_s": 0.0,
        "winch_cmd_port":       _winch_port,
    }
    with _acro_stack(tmp_path, extra_config=extra,
                     test_name=request.node.name) as ctx:
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

        ctx.wait_drain(timeout=1.0, label="post-param")
        yield ctx


@pytest.fixture
def acro_armed_landing_lua(tmp_path, request):
    """
    Landing stack fixture with rawes.lua active in landing mode (SCR_USER6=4).

    Extends acro_armed_landing:
      - kinematic_vel_ramp_s=20: hub exits kinematic at vel=0, eliminating the
        linear tether jolt. Tether extension at exit ~ 0 m, tension ~ 0 N.
      - rawes.lua installed before SITL starts.
      - SCR_USER1..5 configured post-arm immediately; SCR_USER6=4 set so Lua
        starts in landing mode; KINEMATIC_SETTLE_MS=62000 delays body_z capture
        until EKF has converged.
      What is validated:
          (a) Lua enters landing mode and body_z capture fires on schedule
              ("RAWES land: captured" STATUSTEXT at t~62 s).
          (b) Lua alt_est computation is correct: "RAWES land: final_drop"
              STATUSTEXT fires when alt_est <= LAND_MIN_TETHER_M=2 m.
          (c) Hub descends to floor and tension stays safe (Lua + WinchController).
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

    With dual GPS (EK3_SRC1_YAW=2, default in rawes_sitl_defaults.parm) yaw
    is known from the first GPS fix — no motion is needed.  The kinematic is a
    stationary hold at pos0 (vel0=[0,0,0], linear trajectory) for 80 s.

    Key design points:
      - SCR_USER6=1 set immediately after arm; Lua pre-GPS bypass holds
        col_cruise and zero cyclic until _tdir0 fires (GPS fusion).
      - GPS fuses at ~34 s (delAngBiasLearned with constant-zero gyro).
        _tdir0 fires; orbit tracking active ~46 s before kinematic exits.
      - Fixture waits for GPS fusion before yielding.

    Timeline (from mediator start, speedup=1):
      t=0..80 s   kinematic stationary hold at pos0 (vel=0).
      t~12 s      arm complete; SCR_USER6=1 set; Lua pre-GPS bypass active.
      t~34 s      GPS fuses; _tdir0 fires; Lua orbit tracking active.
      t~34 s      fixture yields (GPS fusion confirmed).
      t=80 s      kinematic exits; Lua already tracking orbit for ~46 s.
      t~80+       free flight under ArduPilot + Lua.
    """
    extra = {
        # Stationary hold at pos0 (tether equilibrium from steady_state_starting.json).
        # vel0=0 + ramp_s=0: hub stays at pos0 throughout the 80 s kinematic.
        "vel0":                 [0.0, 0.0, 0.0],
        "kinematic_vel_ramp_s": 0.0,
        "startup_damp_seconds": 80.0,
        # Enable the mediator's winch command socket so we can run a
        # ground-side tension regulator (mirrors test_create_ic warmup).
        "winch_cmd_port":       14570,
    }
    with _acro_stack(tmp_path, extra_config=extra,
                     test_name=request.node.name) as ctx:
        ctx.log.info("Setting SCR_USER params for rawes.lua ...")
        # The anchor is at world origin; the EKF origin is the hub's first
        # GPS fix = launch position pos0.  So anchor-in-EKF-frame = -pos0.
        # SCR_USER3/4 must encode the horizontal offset (negated launch x/y)
        # or the Lua sees the anchor directly below the hub and computes
        # elevation = 90 deg, producing a saturated cyclic that kicks the
        # body at kinematic_exit.
        _pos0 = ctx.initial_state["pos"] if ctx.initial_state else [0.0, 0.0, -ctx.home_alt_m]
        lua_params = {
            "SCR_ENABLE": 1,              # persist scripting in EEPROM
            "SCR_USER1": 1.0,             # RAWES_KP_CYC   [rad/s / rad]
            "SCR_USER2": 0.40,            # RAWES_BZ_SLEW  [rad/s]
            "SCR_USER3": -float(_pos0[0]),   # anchor North in EKF frame [m]
            "SCR_USER4": -float(_pos0[1]),   # anchor East  in EKF frame [m]
            "SCR_USER5":  ctx.home_alt_m,    # anchor Down  in EKF frame [m]
            # MODE_PASSIVE (3): vehicle stays armed (motor interlock kept
            # high) but Lua emits no rate commands.  ArduPilot's rate PID
            # therefore has no setpoint to wind up against while the body
            # is kinematically locked.  The test must promote to
            # MODE_STEADY (1) immediately after kinematic_exit.
            "SCR_USER6": 3,
        }
        for pname, pvalue in lua_params.items():
            ok = ctx.gcs.set_param(pname, pvalue, timeout=5.0)
            ctx.log.info("  %-12s = %g  ACK=%s", pname, pvalue, ok)
        ctx.wait_drain(timeout=1.0, label="post-param")

        # Compute trim cyclic at the IC operating point and stream it to the
        # Lua via NAMED_VALUE_FLOAT (RAWES_TLN / RAWES_TLT — 10-char names).
        # Without this the wind-driven baseline hub moment kicks the body at
        # kinematic_exit before ArduPilot's rate PID can catch up.
        _ic = ctx.initial_state
        if _ic is not None:
            try:
                import numpy as _np_trim
                from dynbem import create_aero, solve_trim_cyclic
                from tests.simtests._rotor_helpers import load_default_rotor
                _rotor_trim = load_default_rotor()
                _aero_trim  = create_aero(_rotor_trim, model="oye")
                _state_trim = _aero_trim.initial_rotor_state()
                _state_trim.omega_rad_s = float(_ic["omega_spin"])
                _R0_trim    = _np_trim.array(_ic["R0"], dtype=float).reshape(3, 3)
                _wind_trim  = _np_trim.array([0.0, 10.0, 0.0])
                _coll_trim  = float(_ic.get("stack_coll_eq", _ic.get("coll_eq_rad", -0.18)))
                _trim_out   = solve_trim_cyclic(
                    _aero_trim, _state_trim,
                    collective_rad=_coll_trim,
                    R_hub=_R0_trim, v_hub_world=_np_trim.zeros(3),
                    wind_world=_wind_trim,
                    n_inflow_relax=200, dt_relax=1.0/400.0,
                    fix_omega=True, tolerance_Nm=0.2,
                )
                ctx.log.info("Trim cyclic: tlon=%+.5f tlat=%+.5f converged=%s",
                             _trim_out.tilt_lon, _trim_out.tilt_lat, _trim_out.converged)
                ctx.gcs.send_named_float("RAWES_TLN", float(_trim_out.tilt_lon))
                ctx.gcs.send_named_float("RAWES_TLT", float(_trim_out.tilt_lat))
                # IC collective: pinned during MODE_PASSIVE so omega_spin
                # doesn't droop while the body is kinematically locked.
                ctx.gcs.send_named_float("RAWES_COL", float(_coll_trim))
                ctx.log.info("IC collective: coll=%+.4f rad", _coll_trim)
            except Exception as e:    # noqa: BLE001
                ctx.log.warning("Trim cyclic computation failed: %s", e)
        ctx.wait_drain(timeout=0.5, label="post-trim")

        # Wait for GPS fusion before yielding.
        # Lua needs _tdir0 (fires on GPS fusion) to begin orbit tracking.
        # With dual GPS the wait is ~44 s (delAngBiasLearned bottleneck).
        ctx.log.info("Waiting for GPS fusion before yielding (up to 60 s) ...")
        _gps_seen: list[bool] = [False]

        def _gps_fused(text: str | None) -> bool:
            if text and "is using GPS" in text:
                _gps_seen[0] = True
                return True
            return False

        ctx.wait_drain(
            until       = _gps_fused,
            timeout     = 60.0,
            drain_s     = 1.0,
            check_procs = True,
            label       = "gps-fuse",
        )
        if not _gps_seen[0]:
            raise RuntimeError("GPS did not fuse within 60 s — cannot start orbit tracking")
        ctx.log.info("GPS fused — Lua orbit tracking active; yielding to test")

        # ── Ground-side tension-regulating winch ─────────────────────────────
        # Mirrors the test_create_ic warmup pattern: a slow integrator on
        # ``rest_length`` driven by tension error.  Without this the tether
        # spring mode is undamped after kinematic_exit and the kinematic→
        # free-flight transient blows up within ~700 ms (tension peaks
        # >1000 N, SITL crashes).
        #
        # The mediator's WinchController interprets ``target_length`` as
        # a destination; for tension regulation we set it slightly above /
        # below the current ``rest_length`` based on the sign of the
        # tension error, exploiting the WinchController's asymmetric
        # reel-out / reel-in tension-controlled cruise speed.
        import socket as _sock
        import json as _json_w
        import threading as _thr
        import time as _time_w

        _winch_stop = _thr.Event()
        _tension_target_n = 300.0
        _winch_addr  = ("127.0.0.1", 14570)
        _winch_sock  = _sock.socket(_sock.AF_INET, _sock.SOCK_DGRAM)
        _winch_sock.bind(("127.0.0.1", 0))
        _winch_sock.settimeout(0.05)

        # Seed the mediator with an initial command so it knows our address
        # and starts streaming state back.
        _winch_sock.sendto(_json_w.dumps({
            "target_length":  float(_ic.get("rest_length", 100.0)) if _ic else 100.0,
            "target_tension": _tension_target_n,
        }).encode(), _winch_addr)

        def _winch_regulator():
            _rest_local = float(_ic.get("rest_length", 100.0)) if _ic else 100.0
            _tension_now = _tension_target_n
            while not _winch_stop.is_set():
                # Pull any pending state updates from mediator
                try:
                    while True:
                        _data, _ = _winch_sock.recvfrom(256)
                        _state = _json_w.loads(_data)
                        _tension_now = float(_state["tension_n"])
                        _rest_local  = float(_state["rest_length"])
                except (TimeoutError, _sock.timeout, BlockingIOError):
                    pass

                # Asymmetric set-point: target_length above or below current
                # rest_length depending on which way we need to push tension.
                # WinchController reel-out speed = kp*(T-T_target), reel-in
                # speed = kp*(T_target-T) — non-zero only in the matching
                # direction, so flipping the sign of ``target_length - rest``
                # selects the right mode each tick.
                _dT = _tension_now - _tension_target_n
                _target_len = _rest_local + (1.0 if _dT > 0 else -1.0)
                try:
                    _winch_sock.sendto(_json_w.dumps({
                        "target_length":  _target_len,
                        "target_tension": _tension_target_n,
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
