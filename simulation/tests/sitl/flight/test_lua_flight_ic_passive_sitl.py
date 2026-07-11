"""
test_lua_flight_ic_passive_sitl.py -- IC-targeted passive-mode stack test.

Goal
----
Provide a stack-level analogue of the Python IC angle-only test:
- Keep Lua in MODE_PASSIVE (SCR_USER6=3) after kinematic_exit.
- Re-seed IC collective/tension at release.
- Observe a short free-flight window and check IC-style stability metrics.

This intentionally avoids MODE_STEADY capture/altitude-hold behavior so the
result is as comparable as practical to the Python IC-angle test.
"""

from __future__ import annotations

import logging
import math
import sys
from pathlib import Path

import numpy as np
import pytest

pytestmark = pytest.mark.sitl

_SIM_DIR = Path(__file__).resolve().parents[3]
_SITL_DIR = Path(__file__).resolve().parents[1]
_ANALYSIS_DIR = _SIM_DIR / "analysis"
sys.path.insert(0, str(_SIM_DIR))
sys.path.insert(0, str(_SITL_DIR))
sys.path.insert(0, str(_ANALYSIS_DIR))

from stack_infra import (  # noqa: E402
    StackContext,
    assert_no_mediator_criticals,
    get_arducopter_crash_info,
    observe,
)

# Timing
_KINEMATIC_TIMEOUT_S = 60.0
_OBS_SECONDS = 10.0

# IC-comparable thresholds (mirrors test_steady_flight_ic_angle_only intent)
_DRIFT_MAX_M = 15.0
_AXLE_MAX_DEG = 25.0
_MIN_ALT_M = 3.0
_MIN_TENSION_N = 0.01


def _get_ic_collective(ic: dict) -> tuple[float, str]:
    if "coll_eq_rad" in ic:
        return float(ic["coll_eq_rad"]), "coll_eq_rad"
    raise KeyError(
        "initial_state missing collective seed: coll_eq_rad"
    )


def _axle_deg(row) -> float:
    # body_z is the 3rd column of R body->NED
    body_z = np.array([float(row.r02), float(row.r12), float(row.r22)], dtype=float)
    pos = np.array([float(row.pos_x), float(row.pos_y), float(row.pos_z)], dtype=float)
    tlen = float(np.linalg.norm(pos))
    if tlen < 0.1:
        return 0.0
    tdir = -pos / tlen
    dot = float(np.clip(np.dot(body_z, tdir), -1.0, 1.0))
    return math.degrees(math.acos(dot))


def test_lua_flight_ic_passive_sitl(guided_nogps_armed_lua_full: StackContext):
    """Hold IC target in MODE_PASSIVE and verify short-window IC-style stability."""
    ctx = guided_nogps_armed_lua_full
    gcs = ctx.gcs
    log = logging.getLogger("test_lua_flight_ic_passive_sitl")

    log.info("=== waiting for kinematic_exit ===")
    if not ctx.wait_kinematic_done(timeout=_KINEMATIC_TIMEOUT_S):
        pytest.fail(
            f"Kinematic phase did not end within {_KINEMATIC_TIMEOUT_S:.0f}s. "
            "Check mediator log for transition markers."
        )

    ic = ctx.initial_state
    if ic is None:
        pytest.fail("initial_state is required for IC-passive test")

    coll_seed, coll_src = _get_ic_collective(ic)
    ten_seed = float(ic["tension_eq_n"])
    R0 = ic.get("R0")
    if R0 is None:
        pytest.fail("initial_state missing R0 for IC passive attitude seed")
    r20 = float(R0[2][0])
    r21 = float(R0[2][1])
    r22 = float(R0[2][2])
    ic_roll_rad = math.atan2(r21, r22)
    ic_pitch_rad = -math.asin(max(-1.0, min(1.0, r20)))

    # Re-seed IC targets right at release for deterministic passive hold.
    gcs.send_named_float("RAWES_COL", coll_seed)
    gcs.send_named_float("RAWES_TEN", ten_seed)
    gcs.send_named_float("RAWES_RIC", ic_roll_rad)
    gcs.send_named_float("RAWES_PIC", ic_pitch_rad)
    ok = gcs.set_param("SCR_USER6", 3, timeout=5.0)
    log.info(
        "Release seeds: RAWES_COL=%+.4f (%s), RAWES_TEN=%.1f N, IC r/p=(%.2f, %.2f)deg, SCR_USER6=3 ACK=%s",
        coll_seed,
        coll_src,
        ten_seed,
        math.degrees(ic_roll_rad),
        math.degrees(ic_pitch_rad),
        ok,
    )

    all_statustext = list(ctx.all_statustext)
    state = {
        "ekf_yaw_reset": False,
        "max_cyclic": 0,
    }

    t_obs_start = gcs.sim_now()

    def _handle(msg, t_rel):
        if msg is None:
            return None
        mt = msg.get_type()
        if mt == "SERVO_OUTPUT_RAW":
            activity = abs(msg.servo1_raw - 1500) + abs(msg.servo2_raw - 1500)
            if activity > state["max_cyclic"]:
                state["max_cyclic"] = activity
        elif mt == "STATUSTEXT":
            text = msg.text.rstrip("\x00").strip()
            all_statustext.append(text)
            tl = text.lower()
            if "emergency yaw" in tl or "yaw reset" in tl:
                state["ekf_yaw_reset"] = True
        return None

    log.info("=== observing %.0fs in MODE_PASSIVE ===", _OBS_SECONDS)
    observe(
        ctx,
        _OBS_SECONDS,
        _handle,
        msg_types=["SERVO_OUTPUT_RAW", "STATUSTEXT", "ATTITUDE", "LOCAL_POSITION_NED", "EKF_STATUS_REPORT"],
        label="ic-passive-observation",
    )

    try:
        if not ctx.telemetry_log.exists():
            pytest.fail("Missing telemetry.csv for IC-passive analysis")

        from telemetry_csv import read_csv as _read_csv  # noqa: PLC0415

        rows = _read_csv(ctx.telemetry_log)
        window = [r for r in rows if float(r.t_sim) >= t_obs_start]
        if len(window) < 2:
            pytest.fail(
                f"Insufficient telemetry rows in passive observation window: {len(window)}"
            )

        pos0 = np.array([float(window[0].pos_x), float(window[0].pos_y), float(window[0].pos_z)], dtype=float)
        posf = np.array([float(window[-1].pos_x), float(window[-1].pos_y), float(window[-1].pos_z)], dtype=float)
        drift = np.abs(posf - pos0)

        min_alt = float(min(-float(r.pos_z) for r in window))
        min_tension = float(min(float(r.tether_tension) for r in window))
        max_axle = float(max(_axle_deg(r) for r in window))

        log.info(
            "IC-passive metrics: drift_E=%.3fm drift_Z=%.3fm min_alt=%.2fm min_tension=%.2fN max_axle=%.2fdeg max_cyclic=%d",
            drift[1],
            drift[2],
            min_alt,
            min_tension,
            max_axle,
            state["max_cyclic"],
        )

        failures: list[str] = []
        if not np.all(np.isfinite([*pos0, *posf])):
            failures.append("NaN/inf in position samples")
        if drift[1] >= _DRIFT_MAX_M:
            failures.append(f"East drift {drift[1]:.2f} m >= {_DRIFT_MAX_M:.1f} m")
        if drift[2] >= _DRIFT_MAX_M:
            failures.append(f"Vertical drift {drift[2]:.2f} m >= {_DRIFT_MAX_M:.1f} m")
        if min_alt < _MIN_ALT_M:
            failures.append(f"Min altitude {min_alt:.2f} m < {_MIN_ALT_M:.1f} m")
        if min_tension < _MIN_TENSION_N:
            failures.append(f"Tether slack detected: min tension {min_tension:.3f} N")
        if max_axle > _AXLE_MAX_DEG:
            failures.append(f"Axle misalignment {max_axle:.2f} deg > {_AXLE_MAX_DEG:.1f} deg")
        if state["ekf_yaw_reset"]:
            failures.append("EKF yaw reset detected in STATUSTEXT")

        if failures:
            pytest.fail(
                "IC-passive stability checks failed:\n  "
                + "\n  ".join(failures)
                + "\nSTATUSTEXT tail: "
                + str(all_statustext[-20:])
            )

        assert_no_mediator_criticals(ctx.mediator_log)
    finally:
        crash_info = get_arducopter_crash_info(ctx)
        if crash_info:
            log.error(crash_info)
