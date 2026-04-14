"""
test_kinematic_gps.py -- Empirical GPS fusion test: which kinematic vel0 works?

Each variant runs the standard acro_armed startup with a different vel0.
The kinematic phase is extended to 120 s so GPS has time to fuse before
anything interesting happens.  After arm the test simply watches for GPS
fusion events and reports the heading gap.

The compass heading is fixed by body_z = [0.277, 0.877, -0.392] via
build_orb_frame() East-projection convention -> orb_yaw ~ 136.6 deg.

Variants (heading_gap = compass - vel_heading):
  north      vel=[1,0,0]            heading= 0.0 deg  gap=136.6 deg
  east       vel=[0,1,0]            heading=90.0 deg  gap= 46.6 deg
  compass    vel aligned w/compass  heading=136.6 deg gap=  0.0 deg
  southeast  vel=[0.72,-0.69,0]     heading=-43.7 deg gap=180 deg  (opposite)
  current    vel=[0.916,-0.257,0]   heading=-15.7 deg gap=152 deg  (baseline)
  zero       vel=[0,0,0]            heading=undef     gap=undef

Run:
  bash simulation/dev.sh test-stack -v -k test_kinematic_gps
"""
from __future__ import annotations

import math
import sys
import time
from pathlib import Path

import pytest

_SIM_DIR  = Path(__file__).resolve().parents[3]
_SITL_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_SIM_DIR))
sys.path.insert(0, str(_SITL_DIR))

from stack_infra import _acro_stack

# ---------------------------------------------------------------------------
# Compass heading (deg) for the default body_z=[0.277, 0.877, -0.392].
# Computed from build_orb_frame East-projection -> atan2(R[1,0], R[0,0]).
# ---------------------------------------------------------------------------
_COMPASS_DEG = 136.6
_SPEED       = 0.96     # orbital speed [m/s] -- matches config default

def _vel_heading(heading_deg: float, speed: float = _SPEED) -> list:
    """NED vel0 pointing at heading_deg (clockwise from North)."""
    h = math.radians(heading_deg)
    return [round(speed * math.cos(h), 4), round(speed * math.sin(h), 4), 0.0]


# ---------------------------------------------------------------------------
# Variants: (id, vel0, expected_gap_deg)
# ---------------------------------------------------------------------------
_VARIANTS = [
    ("north",    _vel_heading(0.0),          136.6),
    ("east",     _vel_heading(90.0),          46.6),
    ("compass",  _vel_heading(_COMPASS_DEG),   0.0),
    ("current",  [0.916, -0.257, 0.093],     152.2),
    ("zero",     [0.0,   0.0,   0.0],         None),   # no GPS heading at v=0
]


# ---------------------------------------------------------------------------
# Test
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("variant,vel0,expected_gap", _VARIANTS,
                          ids=[v[0] for v in _VARIANTS])
def test_kinematic_gps(tmp_path, request, variant, vel0, expected_gap):
    """
    Run standard acro_armed startup with vel0=<variant>.

    Kinematic duration = 120 s so GPS has plenty of time to fuse before
    kinematic exit.  After arm the test waits 90 s and checks:
      - "EKF3 IMU0 is using GPS" appeared  (GPS fused)
      - "GPS Glitch or Compass error" did NOT appear  (stayed fused)

    Results are printed per-variant for manual comparison.  The test
    PASSES if GPS fused and stayed fused; FAILS (with xfail) for variants
    expected to fail (gap > 30 deg) so the suite stays green while collecting
    empirical data.
    """
    _expect_gps = (expected_gap is not None and expected_gap < 30.0)

    speedup = request.config.getoption("--sitl-speedup", default=1)

    with _acro_stack(
        tmp_path,
        log_name   = f"kin_gps_{variant}",
        log_prefix = "kin_gps",
        test_name  = request.node.name,
        speedup    = speedup,
        extra_config = {
            "vel0":                list(vel0),
            "startup_damp_seconds": 120.0,   # long kinematic -- GPS must fuse here
            "kinematic_vel_ramp_s": 0.0,     # constant vel throughout (clean heading)
        },
    ) as ctx:
        log = ctx.log

        # --- wait 90 s into kinematic; GPS should fuse well before t=90 s ---
        log.info("[%s] waiting 90 s for GPS to fuse during kinematic ...", variant)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 90.0:
            time.sleep(2.0)

        # --- read gcs.log for STATUSTEXT events ---
        gcs_text = ctx.gcs_log.read_text(errors="replace") if ctx.gcs_log.exists() else ""

        gps_fused  = "is using GPS" in gcs_text
        gps_glitch = "GPS Glitch or Compass error" in gcs_text
        gps_origin = "origin set" in gcs_text

        # Derive actual heading gap from telemetry (first non-zero row)
        actual_gap = _read_heading_gap(ctx.telemetry_log)

        vel_str  = f"[{vel0[0]:+.3f}, {vel0[1]:+.3f}, {vel0[2]:+.3f}]"
        gap_str  = f"{expected_gap:.1f}" if expected_gap is not None else "undef"
        result   = "FUSED" if (gps_fused and not gps_glitch) else (
                   "GLITCH" if gps_glitch else "NO_GPS")

        log.info("")
        log.info("=" * 60)
        log.info("  variant      : %s", variant)
        log.info("  vel0 (NED)   : %s", vel_str)
        log.info("  expected gap : %s deg", gap_str)
        log.info("  actual gap   : %s deg", actual_gap)
        log.info("  GPS origin   : %s", gps_origin)
        log.info("  GPS fused    : %s", gps_fused)
        log.info("  GPS glitch   : %s", gps_glitch)
        log.info("  RESULT       : %s", result)
        log.info("=" * 60)

        if _expect_gps:
            assert gps_fused,  f"[{variant}] GPS should fuse (gap={gap_str} deg) but did not"
            assert not gps_glitch, f"[{variant}] GPS fused then glitched"
        else:
            # Variants with large gap are expected to fail GPS fusion.
            # Mark xfail so the suite stays green and we collect the data.
            if not gps_fused or gps_glitch:
                pytest.xfail(
                    f"[{variant}] GPS did not fuse (expected: gap={gap_str} deg > 30 deg)"
                )


def _read_heading_gap(telemetry_log) -> str:
    """Return mean absolute heading_gap_deg from kinematic rows, or '?' if unavailable."""
    try:
        from telemetry_csv import read_csv
        if not telemetry_log or not telemetry_log.exists():
            return "?"
        rows = read_csv(telemetry_log)
        kin_rows = [r for r in rows if r.damp_alpha > 0.0 and r.v_horiz_ms > 0.1]
        if not kin_rows:
            return "?"
        gaps = [abs(r.heading_gap_deg) for r in kin_rows]
        return f"{sum(gaps)/len(gaps):.1f}"
    except Exception:
        return "?"
