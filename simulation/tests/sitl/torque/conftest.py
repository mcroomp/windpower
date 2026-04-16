"""
torque/conftest.py — pytest fixtures for RAWES counter-torque motor stack tests.

Fixtures:
  torque_armed         — constant-RPM torque stack fixture.
  torque_armed_profile — parametrised torque fixture (profile name via request.param).
  torque_armed_lua     — torque fixture with rawes.lua in yaw mode (SCR_USER6=2).
"""
import pytest

from stack_infra import *  # noqa: F401,F403  — re-export everything for test imports
from stack_infra import (
    _torque_stack,
    _LUA_TORQUE_EXTRA_PARAMS,
)


# ---------------------------------------------------------------------------
# Counter-torque motor stack fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def torque_armed(tmp_path, request):
    """Counter-torque stack fixture (constant RPM). Yields TorqueStackContext."""
    import model as _m
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
    import model as _m
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
    Like torque_armed but with rawes.lua active in yaw mode (SCR_USER6=2).

    Key differences from torque_armed:
      - rawes.lua installed to /ardupilot/scripts/ before boot
      - EEPROM wiped so copter-heli.parm defaults apply cleanly
      - SCR_ENABLE=1, SCR_USER6=2, RPM1_TYPE=10, SERVO9_FUNCTION=94
      - mediator_torque.py --lua-mode --tail-channel 8 (reads Ch9, linear mapping)
      - ATC_RAT_YAW_P=0 (Lua is sole feedforward provider, no ArduPilot yaw PID)
    """
    import model as _m
    with _torque_stack(
        tmp_path,
        omega_rotor=_m.OMEGA_ROTOR_NOMINAL,
        lua_mode=True,
        tail_channel=8,
        extra_params=_LUA_TORQUE_EXTRA_PARAMS,
        install_scripts=("rawes.lua",),
        test_name=request.node.name,
        # Boot-time params for the Lua fixture (merged on top of _BASE_TORQUE_BOOT_PARAMS):
        # - RPM1_TYPE=10: JSON RPM backend must be activated at boot (driver init).
        #   Setting post-boot via MAVLink does NOT activate the driver -> rawes.lua gets
        #   nil from get_rpm() -> outputs STARTUP_PWM (1050 us) instead of trim -> hub yaws.
        # - ATC_RAT_YAW_P=0.0: Lua is the sole feedforward provider; ArduPilot yaw PID
        #   must be inactive so it doesn't fight the Lua trim output.
        boot_params={"RPM1_TYPE": 10, "RPM1_MIN": 0, "ATC_RAT_YAW_P": 0.0},
    ) as ctx:
        yield ctx
