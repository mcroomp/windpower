"""
torque/conftest.py — pytest fixtures for RAWES counter-torque motor stack tests.

Fixtures:
  torque_armed              — constant-RPM torque stack fixture.
  torque_armed_profile      — parametrised torque fixture (profile name via request.param).
    torque_armed_lua          — torque fixture with rawes.lua, armed via GCS.
    torque_unarmed_lua        — torque fixture with rawes.lua, unarmed; test controls arm/timer.
  torque_armed_ddfp_zero    — DDFP fixture with prescribed zero yaw (motor should stay off).
  torque_armed_ddfp_ramp    — DDFP fixture with prescribed 0→10 deg/s yaw ramp (PI must cancel it).
  torque_armed_ddfp         — DDFP fixture with kinematic yaw model (closed-loop regulation).
  torque_armed_lua_manual   — MODE_MANUAL fixture; force_params sourced from calibrate._RUN_MODES.
"""
import sys
from pathlib import Path
import pytest

from stack_infra import *  # noqa: F401,F403  — re-export everything for test imports
from stack_infra import (
    _torque_stack,
    _LUA_TORQUE_EXTRA_PARAMS,
    _DDFP_TORQUE_EXTRA_PARAMS,
    _SERVO_TAIL_TORQUE_EXTRA_PARAMS,
)

# Import calibrate._RUN_MODES so the manual-mode fixture uses the SAME
# force_params as the interactive calibrate tool — a single source of truth.
_SCRIPTS_DIR = str(Path(__file__).resolve().parents[3] / "scripts")
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)
from calibrate import _RUN_MODES as _CALIBRATE_RUN_MODES  # noqa: E402

# IC (steady-state tethered-hover) orientation constants — single source of
# truth in mediator_torque (derived from steady_state_starting.json R0).
from mediator_torque import _IC_ROLL_RAD, _IC_PITCH_RAD, _IC_YAW_RAD  # noqa: E402


# ---------------------------------------------------------------------------
# Counter-torque motor stack fixtures
# ---------------------------------------------------------------------------

# IC collective value sent to MODE_YAW so the Lua pins ch3 at the right
# collective while the test observes yaw regulation.
LUA_YAW_IC_COL:   float = -0.150  # rad

@pytest.fixture
def torque_armed(tmp_path, request):
    """Counter-torque stack fixture at the IC (high-tilt) orientation. Yields StackContext.

    Boots from the FLIGHT default params (dual-GPS yaw, GPS pos/vel enabled) via
    profile="ic": the hub is held at the steady-state tethered-hover attitude
    (roll=0, pitch=-63.6 deg) instead of level.  rawes.lua boots in MODE_PASSIVE
    (SCR_USER6=3); the IC operating point is seeded before arm (collective=
    LUA_YAW_IC_COL, RIC/PIC=IC roll/pitch) and the EKF pre-arm attitude is seeded
    from the live yaw.  ArduPilot's DDFP yaw PID regulates hub yaw via SERVO4.
    """
    import torque_model as _m
    with _torque_stack(
        tmp_path,
        omega_rotor=_m.OMEGA_ROTOR_NOMINAL,
        profile="ic",
        test_name=request.node.name,
        passive_init=True,
        passive_col_rad=LUA_YAW_IC_COL,
        passive_roll_rad=_IC_ROLL_RAD,
        passive_pitch_rad=_IC_PITCH_RAD,
        passive_yaw_rad=_IC_YAW_RAD,
        passive_yaw_ff_ki=0.05,
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
        passive_init=True,
        passive_col_rad=LUA_YAW_IC_COL,
    ) as ctx:
        yield ctx


def _lua_torque_stack(tmp_path, request, armon_ms):
    """Shared setup for Lua torque fixtures.

    ArduPilot DDFP yaw PID (H_TAIL_TYPE=4) drives SERVO4 (tail_channel=3).
    Arming is handled by GCS; rawes.lua (SCR_USER6=0) provides optional
    RAWES_ARM disarm timer behavior only.
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

    Yaw is regulated by ArduPilot's ATC_RAT_YAW DDFP PID (H_TAIL_TYPE=4).
    Armed via GCS in ACRO mode; no GCS RC override required for arming.
    Lua RAWES_ARM remains available as an optional disarm timer.
    Yields StackContext with vehicle armed and ACRO active.
    """
    with _lua_torque_stack(tmp_path, request, armon_ms=3_600_000) as ctx:
        yield ctx


@pytest.fixture
def torque_production_vanilla_lua(tmp_path, request):
    """
    Production-like torque fixture using vanilla SITL boot defaults + Lua PASSIVE.

    Uses the default _sitl_stack parameter chain (copter-heli + rawes_common +
    rawes_sitl_defaults), not the torque-specific _BASE_TORQUE_BOOT_PARAMS.

    Rotor profile: starts stationary during STARTUP hold, then spins up to 200 RPM
    and varies slowly around that speed (profile="slow_vary").
    """
    _OMEGA_200_RPM = 200.0 * 2.0 * math.pi / 60.0
    with _torque_stack(
        tmp_path,
        omega_rotor=_OMEGA_200_RPM,
        profile="slow_vary",
        tail_channel=3,
        # Keep Lua loaded and in MODE_PASSIVE. PASSIVE holds the seeded IC state
        # and runs yaw feedforward trim via H_YAW_TRIM while ArduPilot DDFP closes yaw.
        passive_init=True,
        passive_col_rad=LUA_YAW_IC_COL,
        # NOTE: a fixed absolute yaw heading via RAWES_YIC was trialled here but,
        # combined with the slow_vary RPM sweep and the I=0 rate loop, it left a
        # steady yaw-rate residual that tripped the gate.  Free-capture (PASSIVE
        # latches the AHRS yaw on entry) is the default.
        test_name=request.node.name,
        startup_hold_s=15.0,
        startup_yaw_rate_deg_s=0.0,
        use_vanilla_boot_defaults=True,
    ) as ctx:
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

    Arming: the torque rig has no GPS — armed via GCS after
    EKF attitude alignment (no GPS fix required).  See
    ``stack_infra._launch_mediator_torque`` for the no-GPS configuration.
    """
    import math
    # MODE_YAW (SCR_USER6=2) runs rawes.lua's OWN manual yaw PID (run_manual),
    # which reads ATC_RAT_YAW_P/I/D directly and writes SERVO4 -- it does NOT use
    # ArduPilot's DDFP mixer and there is NO Lua trim observer in this mode.  So it
    # needs a nonzero yaw INTEGRAL to build the DC holding throttle itself (the
    # DDFP standard uses I=0 because the observer / AP integrator carries the DC).
    # Keep the standard P/D but restore I for this manual-PID path.
    _yaw_extras = _LUA_TORQUE_EXTRA_PARAMS.update({
        "SCR_USER6":        2,
        "ATC_RAT_YAW_I":    0.01,
        # run_manual's PID builds the FULL ~0.36 holding throttle from its own
        # integrator (no observer in MODE_MANUAL), so it needs a large IMAX --
        # NOT the small standard IMAX that assumes the observer carries the DC.
        "ATC_RAT_YAW_IMAX": 0.7,
    })
    # Bench setpoint: motor stationary during STARTUP (rig won't drive the
    # rotor before ArduPilot arms via RAWES_ARM), then spins up to 120 RPM
    # = 4*pi rad/s (= 12.566 rad/s).  The mediator's universal 10 s ramp
    # at DYNAMIC start (mediator_torque.py:_SPINUP_S) provides the smooth
    # accelerate-from-zero; the test_lua_yaw_regulation_sitl observes the
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
        ctx.gcs.send_named_float("RAWES_COL", LUA_YAW_IC_COL)
        ctx.log.info("Sent IC collective to Lua: col=%+.4f", LUA_YAW_IC_COL)
        yield ctx


@pytest.fixture
def torque_armed_lua_manual(tmp_path, request):
    """
    Torque stack with rawes.lua in MODE_MANUAL (SCR_USER6=2).

    force_params are sourced from calibrate._RUN_MODES["manual"]["force_params"]
    so the stack test always exercises the exact same param set that the
    interactive 'calibrate manual' command applies on hardware.

    MODE_MANUAL: rawes.lua run_manual() drives SERVO4 via the yaw PID
    (bypasses ArduPilot's DDFP mixer) and sets RC1/RC2 from RAWES_TLT/TLN
    NVFs scaled by H_CYC_MAX.  H_FLYBAR_MODE=1 routes the RC overrides
    directly to the swash mixer, skipping the rate PID.
    """
    import math as _math
    _cal = _CALIBRATE_RUN_MODES["manual"]
    _manual_extras = _LUA_TORQUE_EXTRA_PARAMS.update({
        "SCR_USER6": float(_cal["scr_user6"]),
        **{k: float(v) for k, v in _cal["force_params"].items()},
    })
    _OMEGA_120_RPM = 120.0 * 2.0 * _math.pi / 60.0   # 120 RPM in rad/s
    with _torque_stack(
        tmp_path,
        omega_rotor=_OMEGA_120_RPM,
        tail_channel=3,
        extra_params=_manual_extras,
        install_scripts=("rawes.lua",),
        test_name=request.node.name,
        armon_ms=3_600_000,
    ) as ctx:
        ctx.gcs.send_named_float("RAWES_COL", LUA_YAW_IC_COL)
        ctx.log.info("Sent IC collective to Lua: col=%+.4f", LUA_YAW_IC_COL)
        yield ctx


@pytest.fixture
def torque_unarmed_lua(tmp_path, request):
    """
    Torque stack with rawes.lua passive (SCR_USER6=0, MODE_NONE).

    Yaw is regulated by ArduPilot's ATC_RAT_YAW DDFP PID (H_TAIL_TYPE=4).
    Yields StackContext with vehicle UNARMED and ACRO active.
    The test is responsible for arming via GCS and may optionally set RAWES_ARM
    disarm timer.
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
        passive_init=True,
        passive_col_rad=LUA_YAW_IC_COL,
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
        passive_init=True,
        passive_col_rad=LUA_YAW_IC_COL,
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
        passive_init=True,
        passive_col_rad=LUA_YAW_IC_COL,
    ) as ctx:
        yield ctx


@pytest.fixture
def torque_armed_ddfp(tmp_path, request):
    """
    DDFP fixture with kinematic yaw model (closed-loop regulation test).

    The kinematic model feeds back through ArduPilot: motor throttle from Ch4 drives
    omega_motor (first-order lag), which drives psi_dot.  No prescribed yaw — this is
    the real closed loop.

    SERVO4_MIN=800 / SERVO4_MAX=2000 (from _DDFP_TORQUE_EXTRA_PARAMS).  Uses the
    flight GUIDED_NOGPS init technique (passive_init): rawes.lua boots in
    MODE_PASSIVE (SCR_USER6=3) only to seed the IC operating point and hold the
    level pre-arm attitude; yaw is still regulated solely by ArduPilot's built-in
    DDFP controller (the Lua does not touch SERVO4 in MODE_PASSIVE).
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
        passive_init=True,
        passive_col_rad=LUA_YAW_IC_COL,
    ) as ctx:
        yield ctx
