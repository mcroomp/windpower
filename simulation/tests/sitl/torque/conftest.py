"""
torque/conftest.py — pytest fixtures for RAWES counter-torque motor stack tests.

Fixtures:
  torque_armed              — constant-RPM torque stack fixture.
  torque_armed_profile      — parametrised torque fixture (profile name via request.param).
  torque_armed_lua          — torque fixture with rawes.lua in yaw-only mode (SCR_USER6=2, MODE_YAW).
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


@pytest.fixture
def torque_armed_lua(tmp_path, request):
    """
    Like torque_armed but with rawes.lua active in yaw-only mode (SCR_USER6=2, MODE_YAW).

    Key differences from torque_armed:
      - rawes.lua installed to /ardupilot/scripts/ before boot
      - EEPROM wiped so copter-heli.parm defaults apply cleanly
      - SCR_ENABLE=1, SCR_USER6=2 (MODE_YAW), RPM1_TYPE=10, SERVO9_FUNCTION=94
      - mediator_torque.py --tail-channel 8 (reads Ch9)
      - ATC_RAT_YAW_P=0 (Lua is sole feedforward provider, no ArduPilot yaw PID)
    """
    import torque_model as _m
    with _torque_stack(
        tmp_path,
        omega_rotor=_m.OMEGA_ROTOR_NOMINAL,

        tail_channel=8,
        extra_params=_LUA_TORQUE_EXTRA_PARAMS,
        install_scripts=("rawes.lua",),
        test_name=request.node.name,
        # ATC_RAT_YAW_P=0.0: Lua is the sole feedforward provider; ArduPilot yaw PID
        # must be inactive so it doesn't fight the Lua trim output.
        boot_params={"ATC_RAT_YAW_P": 0.0},
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
        extra_params=_DDFP_TORQUE_EXTRA_PARAMS,   # H_YAW_TRIM=-0.419, P=0.5, I=0
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
