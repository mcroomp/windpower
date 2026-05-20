"""
torque/conftest.py — pytest fixtures for RAWES counter-torque motor stack tests.

Fixtures:
  torque_armed              — constant-RPM torque stack fixture.
  torque_armed_profile      — parametrised torque fixture (profile name via request.param).
  torque_armed_lua          — torque fixture with rawes.lua, armed via RAWES_ARMON (1 hour).
  torque_unarmed_lua        — torque fixture with rawes.lua, unarmed; test controls RAWES_ARMON.
  torque_armed_ddfp_zero    — DDFP fixture with prescribed zero yaw (motor should stay off).
  torque_armed_ddfp_ramp    — DDFP fixture with prescribed 0→10 deg/s yaw ramp (PI must cancel it).
  torque_armed_ddfp         — DDFP fixture with kinematic yaw model (closed-loop regulation).
"""
import pytest

from stack_infra import *  # noqa: F401,F403  — re-export everything for test imports
from stack_infra import (
    _torque_stack,
    _LUA_TORQUE_EXTRA_PARAMS,
    _DDFP_TORQUE_EXTRA_PARAMS,
    _SERVO_TAIL_TORQUE_EXTRA_PARAMS,
)


# ---------------------------------------------------------------------------
# Counter-torque motor stack fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def torque_armed(tmp_path, request):
    """Counter-torque stack fixture (constant RPM). Yields StackContext."""
    import torque_model as _m
    with _torque_stack(
        tmp_path,
        omega_rotor=_m.OMEGA_ROTOR_NOMINAL,
        test_name=request.node.name,
    ) as ctx:
        yield ctx


@pytest.fixture
def torque_armed_profile(request, tmp_path):
    """
    Like torque_armed but accepts a profile name via request.param.

    Usage::

        @pytest.mark.parametrize("torque_armed_profile", ["slow_vary"], indirect=True)
        def test_foo(torque_armed_profile):
            ...
    """
    import torque_model as _m
    profile = getattr(request, "param", "constant")
    with _torque_stack(
        tmp_path,
        omega_rotor=_m.OMEGA_ROTOR_NOMINAL,
        profile=profile,
        test_name=request.node.name,
    ) as ctx:
        yield ctx


def _lua_torque_stack(tmp_path, request, armon_ms):
    """Shared setup for Lua torque fixtures.

    ArduPilot DDFP yaw PID (H_TAIL_TYPE=4) drives SERVO4 (tail_channel=3).
    Lua (rawes.lua, SCR_USER6=0) handles only RAWES_ARM arming and Ch8 keepalive.
    """
    import torque_model as _m
    return _torque_stack(
        tmp_path,
        omega_rotor=_m.OMEGA_ROTOR_NOMINAL,
        tail_channel=3,
        extra_params=_LUA_TORQUE_EXTRA_PARAMS,
        install_scripts=("rawes.lua",),
        test_name=request.node.name,
        armon_ms=armon_ms,
    )


@pytest.fixture
def torque_armed_lua(tmp_path, request):
    """
    Torque stack with rawes.lua passive (SCR_USER6=0, MODE_NONE).

    Yaw is regulated by ArduPilot's ATC_RAT_YAW DDFP PID (H_TAIL_TYPE=4);
    Lua handles only RAWES_ARM arming and Ch8 motor interlock keepalive.
    Armed via RAWES_ARM(1 hour) — Lua owns Ch3/Ch8; no GCS RC override.
    Yields StackContext with vehicle armed and ACRO active.
    """
    with _lua_torque_stack(tmp_path, request, armon_ms=3_600_000) as ctx:
        yield ctx


@pytest.fixture
def torque_armed_lua_yaw(tmp_path, request):
    """
    Torque stack with rawes.lua in MODE_YAW (SCR_USER6=2).

    Bench setup: ArduPilot rides on a motor with a horizontal axle and
    no other motion.  The motor spins the rotor hub at OMEGA_ROTOR_NOMINAL,
    so the body experiences a constant reaction torque about its yaw axis.
    The GB4008 counter-rotation motor on SERVO4 must compensate to hold
    yaw rate ~ 0.

    MODE_YAW differs from the default torque_armed_lua fixture:
      - rawes.lua's run_yaw_pid() bypasses ArduPilot's internal DDFP mixer
        and writes SERVO4 directly via SRV_Channels:set_output_pwm_chan_timeout
      - ATC_RAT_YAW_P/I/D/IMAX + H_YAW_TRIM + SERVO4_MIN/MAX are read by
        the Lua each tick (NOT by ArduPilot's PID); pidtune-by-PARAM_SET
        takes effect immediately
      - cyclic channels held at neutral, collective at COL_CRUISE_FLIGHT_RAD

    Arming: the torque rig has no GPS — armed via RAWES_ARM(1 hour) after
    EKF attitude alignment (no GPS fix required).  See
    ``stack_infra._launch_mediator_torque`` for the no-GPS configuration.
    """
    import math
    # Override SCR_USER6 from the default 0 (MODE_NONE) used by the
    # shared _LUA_TORQUE_EXTRA_PARAMS to 2 (MODE_YAW).
    _yaw_extras = _LUA_TORQUE_EXTRA_PARAMS.update({"SCR_USER6": 2})
    # Bench setpoint: motor stationary during STARTUP (rig won't drive the
    # rotor before ArduPilot arms via RAWES_ARM), then spins up to 120 RPM
    # = 4*pi rad/s (= 12.566 rad/s).  The mediator's universal 10 s ramp
    # at DYNAMIC start (mediator_torque.py:_SPINUP_S) provides the smooth
    # accelerate-from-zero; the test_lua_yaw_regulation observes the
    # subsequent 30 s of constant-RPM hold.
    # Positive omega_rotor: the GB4008 (positive-throttle-only motor) has
    # authority to zero psi_dot when omega_rotor > 0 -- the equilibrium is
    # omega_motor = omega_rotor * gear_ratio = positive throttle.  With
    # negative omega_rotor the motor can only worsen the drift (no reverse).
    _OMEGA_120_RPM = 120.0 * 2.0 * math.pi / 60.0   # rad/s
    with _torque_stack(
        tmp_path,
        omega_rotor=_OMEGA_120_RPM,
        tail_channel=3,
        extra_params=_yaw_extras,
        install_scripts=("rawes.lua",),
        test_name=request.node.name,
        armon_ms=3_600_000,
    ) as ctx:
        yield ctx


@pytest.fixture
def torque_unarmed_lua(tmp_path, request):
    """
    Torque stack with rawes.lua passive (SCR_USER6=0, MODE_NONE).

    Yaw is regulated by ArduPilot's ATC_RAT_YAW DDFP PID (H_TAIL_TYPE=4);
    Lua handles only RAWES_ARM arming and Ch8 motor interlock keepalive.
    Yields StackContext with vehicle UNARMED and ACRO active.
    The test is responsible for sending RAWES_ARM to arm.
    """
    with _lua_torque_stack(tmp_path, request, armon_ms=0) as ctx:
        yield ctx


@pytest.fixture
def torque_armed_servo_tail(tmp_path, request):
    """
    Torque stack with H_TAIL_TYPE=0 (conventional servo tail).

    SERVO4_MIN=1000 / SERVO4_TRIM=1500 / SERVO4_MAX=2000: symmetric servo range.
    ATC_RAT_YAW PID drives SERVO4 away from 1500 µs neutral in response to hub drift.
    No DDFP sign flip.  Yields StackContext with vehicle armed and ACRO active.

    omega_rotor positive: US-convention rotor (CCW from above) drives body CCW;
    PID error positive -> servo above 1500 us.
    """
    import torque_model as _m
    with _torque_stack(
        tmp_path,
        omega_rotor=_m.OMEGA_ROTOR_NOMINAL,
        tail_channel=3,
        extra_params=_SERVO_TAIL_TORQUE_EXTRA_PARAMS,
        test_name=request.node.name,
        startup_hold_s=15.0,
        startup_yaw_rate_deg_s=0.0,
    ) as ctx:
        yield ctx


@pytest.fixture
def torque_armed_ddfp_zero(tmp_path, request):
    """
    DDFP fixture with prescribed zero yaw throughout DYNAMIC.

    Hub never rotates — tests that the motor stays near 800 µs (off) with no
    integrator activity.  startup_hold_s=15 (SITL-seconds) ensures EKF and
    arming complete before DYNAMIC starts.  startup_yaw_rate_deg_s=0 — no
    artificial spin during startup (ArduPilot can arm without it in the torque
    SITL environment).
    """
    import torque_model as _m
    with _torque_stack(
        tmp_path,
        omega_rotor=_m.OMEGA_ROTOR_NOMINAL,
        profile="yaw_zero",

        tail_channel=3,
        extra_params=_DDFP_TORQUE_EXTRA_PARAMS,
        test_name=request.node.name,
        startup_hold_s=15.0,
        startup_yaw_rate_deg_s=0.0,
    ) as ctx:
        yield ctx


@pytest.fixture
def torque_armed_ddfp_ramp(tmp_path, request):
    """
    DDFP motor response to prescribed yaw ramp: psi_dot prescribed 0→10 deg/s
    over 30 s.  ArduPilot should drive motor throttle above the zero-yaw
    equilibrium value in proportion to the error.

    Prescribed yaw is used (psi_dot set directly) rather than the kinematic model
    driven by ArduPilot throttle, so the test isolates the control-law response.
    Assertion: motor throttle > equilibrium + 0.05 at t_dyn = 25-35 s when
    psi_dot is 8-10 deg/s.
    """
    import torque_model as _m
    with _torque_stack(
        tmp_path,
        omega_rotor=_m.OMEGA_ROTOR_NOMINAL,
        profile="yaw_slow_ramp",     # prescribed psi_dot 0→10 deg/s over 30 s

        tail_channel=3,
        extra_params=_DDFP_TORQUE_EXTRA_PARAMS,   # H_YAW_TRIM=0.02, P=0.5, I=0
        test_name=request.node.name,
        startup_hold_s=15.0,
        startup_yaw_rate_deg_s=0.0,
    ) as ctx:
        yield ctx


@pytest.fixture
def torque_armed_ddfp(tmp_path, request):
    """
    DDFP fixture with kinematic yaw model (closed-loop regulation test).

    The kinematic model feeds back through ArduPilot: motor throttle from Ch4 drives
    omega_motor (first-order lag), which drives psi_dot.  No prescribed yaw — this is
    the real closed loop.

    SERVO4_MIN=800 / SERVO4_MAX=2000 (from _DDFP_TORQUE_EXTRA_PARAMS).  SCR_ENABLE=0
    so no Lua scripting is active — ArduPilot's built-in DDFP controller runs alone.
    """
    import torque_model as _m
    with _torque_stack(
        tmp_path,
        omega_rotor=_m.OMEGA_ROTOR_NOMINAL,
        profile="constant",
        tail_channel=3,
        extra_params=_DDFP_TORQUE_EXTRA_PARAMS,
        test_name=request.node.name,
        startup_hold_s=15.0,
        startup_yaw_rate_deg_s=0.0,
        # omega_motor=0 at DYNAMIC start (not updated during STARTUP).
        # Safety clamp caps psi_dot at 500 deg/s (8.727 rad/s) sent to ArduPilot.
        # Motor overshoot stays safe when P * 8.727 + |H_YAW_TRIM| ≤ 0.607, i.e. P ≤ 0.022.
        # P=0.015 gives ~40% faster convergence than P=0.01 (observed ~56 s → ~40 s)
        # while staying clear of the limit-cycle boundary.
        # I=0.001 corrects any residual steady-state offset from SPIN_MIN/SPIN_MAX.
        boot_params={
            "ATC_RAT_YAW_P":    0.015,
            "ATC_RAT_YAW_I":    0.01,
            "ATC_RAT_YAW_IMAX": 0.7,   # must be > 0.505 (= throttle_eq + H_YAW_TRIM) to reach zero error
        },
    ) as ctx:
        yield ctx
