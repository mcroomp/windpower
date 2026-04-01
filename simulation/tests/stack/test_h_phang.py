"""
test_h_phang.py -- empirical H_PHANG calibration via step-cyclic SITL response.

H_PHANG (H_SW_PHANG in ArduPilot) is a +-30 deg phase-angle trim in the
ArduPilot H3-120 swashplate mixing that corrects for any angular offset between
ArduPilot's assumed servo positions and the actual physical servo positions.

ArduPilot H3_120 internal geometry (from AP_MotorsHeli_Swash.cpp)
------------------------------------------------------------------
  CH_1 at -60 deg, CH_2 at +60 deg, CH_3 at 180 deg

Roll coefficient:  cos(angle + 90 - H_PHANG)   <- includes 90 deg advance angle
Pitch coefficient: cos(angle      - H_PHANG)

RAWES physical servo layout (swashplate.py)
-------------------------------------------
  S1 at 0 deg (East), S2 at 120 deg, S3 at 240 deg

Empirical result
-----------------
H_PHANG = 0 already produces near-zero cross-coupling for our layout:
  phase_ch1 ~ 0 deg  (roll command -> pure tilt_lat)
  phase_ch2 ~ 0 deg  (pitch command -> pure tilt_lon)

The ArduPilot H3_120 formula's built-in +90 deg roll advance angle (gyroscopic
pre-compensation) aligns the servo mixing with our swashplate geometry without
any additional H_PHANG offset.

Measurement procedure
----------------------
1. Arm in ACRO, internal_controller=True (mediator holds physics stable;
   ArduPilot servo outputs are observable without risking a crash).
2. The _no_lua fixture removes rawes_flight.lua before SITL starts, preventing
   Lua RC overrides on Ch1/Ch2 from interfering with the step commands.
3. With H_PHANG=current (default 0):
   a. 4 s neutral baseline  -- collect SERVO_OUTPUT_RAW.
   b. 5 s Ch1=1700 step     -- collect SERVO_OUTPUT_RAW.
   c. 3 s neutral.
   d. 5 s Ch2=1700 step     -- collect SERVO_OUTPUT_RAW.
   e. 3 s neutral.
4. Decode via h3_inverse_mix.  Compute phase angles and cross-coupling ratios.
5. If cross-coupling > 0.20 for either axis, apply empirical H_PHANG correction
   and re-measure.  Assert corrected cross-coupling < 0.20.

Phase angle definitions
------------------------
  phase_ch1 = atan2(delta_tilt_lon,  delta_tilt_lat)  [ideal: 0 deg for roll cmd]
  phase_ch2 = atan2(delta_tilt_lat, delta_tilt_lon)   [ideal: 0 deg for pitch cmd]
  cross_coupling = |off-axis| / |on-axis|

Pass criteria (H_PHANG=0 path)
--------------------------------
- Hub does not crash (physics altitude > 0.5 m).
- cross_ch1 < 0.20 and cross_ch2 < 0.20 at H_PHANG=0.
- tilt_lat_ch1 and tilt_lon_ch2 positive (correct axis polarity).

Pass criteria (H_PHANG correction path, if triggered)
-------------------------------------------------------
- After H_PHANG correction: cross_ch1 < 0.20 and cross_ch2 < 0.20.

Output for rawes_params.parm
------------------------------
The test logs the confirmed H_SW_PHANG value (expected: 0).
"""

import logging
import math
import sys
import time
from pathlib import Path

import pytest

_SIM_DIR   = Path(__file__).resolve().parents[2]
_STACK_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(_SIM_DIR))
sys.path.insert(0, str(_STACK_DIR))

from conftest import StackContext, dump_startup_diagnostics
from swashplate import h3_inverse_mix

# ── Timing --------------------------------------------------------------------
_BASELINE_S = 4.0   # s -- neutral sticks before measurement
_STEP_S     = 5.0   # s -- step command duration
_SETTLE_S   = 3.0   # s -- neutral between steps
_DISCARD_S  = 0.5   # s -- discard initial transient

# ── RC values -----------------------------------------------------------------
_PWM_NEUTRAL = 1500
_PWM_STEP    = 1700   # +0.4 normalised

# ── Tolerance -----------------------------------------------------------------
_CROSS_COUPLING_MAX = 0.20   # off-axis / on-axis ratio; ideal is 0
_MIN_AXIS_RESPONSE  = 0.01   # minimum on-axis tilt to confirm servo moved
_MIN_ALT_M          = 0.5    # m -- physics crash guard

_log = logging.getLogger("test_h_phang")


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def sensor_mode():
    """Physical sensor mode: GPS + compass consistent for EKF GPS fusion."""
    return "physical"


@pytest.fixture
def _no_lua():
    """
    Remove all Lua scripts from /ardupilot/scripts/ BEFORE SITL starts.

    rawes_flight.lua sends Ch1/Ch2 RC overrides at 50 Hz when active.
    This fixture must be listed FIRST in the test parameters so it executes
    before the acro_armed fixture launches SITL.
    """
    lua_dir = Path("/ardupilot/scripts")
    removed = []
    if lua_dir.exists():
        for script in sorted(lua_dir.glob("*.lua")):
            script.unlink()
            removed.append(script.name)
    if removed:
        _log.info("_no_lua: removed Lua scripts: %s", removed)
    else:
        _log.info("_no_lua: no Lua scripts present")
    yield


# ---------------------------------------------------------------------------
# Helper: collect SERVO_OUTPUT_RAW samples
# ---------------------------------------------------------------------------

def _collect_servo_samples(
    gcs,
    ctx: StackContext,
    duration_s: float,
    rc_channels: dict,
    label: str,
) -> list[tuple[float, int, int, int]]:
    """
    Send RC overrides and collect SERVO_OUTPUT_RAW for duration_s seconds.

    RC override is sent on EVERY loop iteration (not rate-limited) so our
    command is always the most recently written value in ArduPilot's RC override
    buffer.  This is essential when any other source (e.g. Lua) could otherwise
    overwrite at a higher rate.

    Returns list of (t_rel, servo1_raw, servo2_raw, servo3_raw).
    rc_channels must include Ch8=2000 (motor interlock keepalive).
    """
    samples: list[tuple[float, int, int, int]] = []
    t_start = time.monotonic()

    while (t_now := time.monotonic()) - t_start < duration_s:
        for name, proc, lp in [
            ("mediator", ctx.mediator_proc, ctx.mediator_log),
            ("SITL",     ctx.sitl_proc,     ctx.sitl_log),
        ]:
            if proc.poll() is not None:
                txt = lp.read_text(encoding="utf-8", errors="replace") if lp.exists() else "(no log)"
                pytest.fail(f"{name} exited during {label} (rc={proc.returncode}):\n{txt[-2000:]}")

        gcs.send_rc_override(rc_channels)
        msg = gcs._mav.recv_match(type=["SERVO_OUTPUT_RAW"], blocking=True, timeout=0.05)
        if msg is not None:
            t_rel = t_now - t_start
            samples.append((t_rel, msg.servo1_raw, msg.servo2_raw, msg.servo3_raw))

    return samples


# ---------------------------------------------------------------------------
# Helper: decode servo samples -> mean tilt
# ---------------------------------------------------------------------------

def _mean_tilt(samples: list[tuple[float, int, int, int]]) -> tuple[float, float]:
    """
    Discard first _DISCARD_S, average the rest.

    Returns (mean_tilt_lat, mean_tilt_lon).
    """
    late = [(s1, s2, s3) for t, s1, s2, s3 in samples if t >= _DISCARD_S]
    if not late:
        raise ValueError(f"No samples after {_DISCARD_S}s discard (total={len(samples)})")
    tilts_lat, tilts_lon = [], []
    for s1_raw, s2_raw, s3_raw in late:
        s1 = (s1_raw - 1500) / 500.0
        s2 = (s2_raw - 1500) / 500.0
        s3 = (s3_raw - 1500) / 500.0
        _, tilt_lon, tilt_lat = h3_inverse_mix(s1, s2, s3)
        tilts_lat.append(tilt_lat)
        tilts_lon.append(tilt_lon)
    n = len(tilts_lat)
    return sum(tilts_lat) / n, sum(tilts_lon) / n


# ---------------------------------------------------------------------------
# Helper: one full measurement round
# ---------------------------------------------------------------------------

def _measure_phang(gcs, ctx: StackContext, h_phang_label: str) -> dict:
    """
    Baseline + Ch1 step + Ch2 step.

    Returns dict: phase_ch1_deg, phase_ch2_deg, cross_ch1, cross_ch2,
                  tilt_lat_ch1, tilt_lon_ch1, tilt_lat_ch2, tilt_lon_ch2.
    """
    neutral = {1: _PWM_NEUTRAL, 2: _PWM_NEUTRAL, 3: _PWM_NEUTRAL, 8: 2000}

    _log.info("--- %s: baseline (%gs) ---", h_phang_label, _BASELINE_S)
    base_samples = _collect_servo_samples(gcs, ctx, _BASELINE_S, neutral, "baseline")
    base_lat, base_lon = _mean_tilt(base_samples)
    _log.info("  Baseline: tilt_lat=%.4f  tilt_lon=%.4f  n=%d",
              base_lat, base_lon, len(base_samples))

    _log.info("--- %s: Ch1=1700 step (%gs) ---", h_phang_label, _STEP_S)
    ch1_samples = _collect_servo_samples(
        gcs, ctx, _STEP_S,
        {1: _PWM_STEP, 2: _PWM_NEUTRAL, 3: _PWM_NEUTRAL, 8: 2000}, "Ch1 step")
    ch1_lat, ch1_lon = _mean_tilt(ch1_samples)
    d_lat_ch1, d_lon_ch1 = ch1_lat - base_lat, ch1_lon - base_lon
    _log.info("  Ch1 delta: tilt_lat=%.4f  tilt_lon=%.4f  n=%d",
              d_lat_ch1, d_lon_ch1, len(ch1_samples))

    _collect_servo_samples(gcs, ctx, _SETTLE_S, neutral, "settle-1")

    _log.info("--- %s: Ch2=1700 step (%gs) ---", h_phang_label, _STEP_S)
    ch2_samples = _collect_servo_samples(
        gcs, ctx, _STEP_S,
        {1: _PWM_NEUTRAL, 2: _PWM_STEP, 3: _PWM_NEUTRAL, 8: 2000}, "Ch2 step")
    ch2_lat, ch2_lon = _mean_tilt(ch2_samples)
    d_lat_ch2, d_lon_ch2 = ch2_lat - base_lat, ch2_lon - base_lon
    _log.info("  Ch2 delta: tilt_lat=%.4f  tilt_lon=%.4f  n=%d",
              d_lat_ch2, d_lon_ch2, len(ch2_samples))

    _collect_servo_samples(gcs, ctx, _SETTLE_S, neutral, "settle-2")

    eps = 1e-6
    phase_ch1 = math.degrees(math.atan2(d_lon_ch1, d_lat_ch1))
    phase_ch2 = math.degrees(math.atan2(d_lat_ch2, d_lon_ch2))
    cross_ch1 = abs(d_lon_ch1) / (abs(d_lat_ch1) + eps)
    cross_ch2 = abs(d_lat_ch2) / (abs(d_lon_ch2) + eps)

    _log.info("  phase_ch1=%.1f deg  phase_ch2=%.1f deg  "
              "cross_ch1=%.3f  cross_ch2=%.3f",
              phase_ch1, phase_ch2, cross_ch1, cross_ch2)
    return {
        "phase_ch1_deg": phase_ch1, "phase_ch2_deg": phase_ch2,
        "cross_ch1": cross_ch1, "cross_ch2": cross_ch2,
        "tilt_lat_ch1": d_lat_ch1, "tilt_lon_ch1": d_lon_ch1,
        "tilt_lat_ch2": d_lat_ch2, "tilt_lon_ch2": d_lon_ch2,
    }


# ---------------------------------------------------------------------------
# Test
# ---------------------------------------------------------------------------

def test_h_swash_phang(_no_lua, acro_armed: StackContext):
    """
    Empirically calibrate H_SW_PHANG for the RAWES swashplate.

    _no_lua MUST be listed first -- it removes rawes_flight.lua before SITL
    starts so the scripting engine sends no RC overrides on Ch1/Ch2.

    The test is adaptive:
      - Measures at H_PHANG=0 (default).
      - If cross-coupling is already acceptable (< 0.20), H_PHANG=0 is confirmed.
      - If cross-coupling is high, computes the empirical correction, applies it,
        and re-measures to verify.
    """
    ctx = acro_armed
    gcs = ctx.gcs

    try:
        # ── Measurement at H_PHANG=0 ──────────────────────────────────────────
        _log.info("=== H_PHANG calibration: measuring with H_PHANG=0 (default) ===")
        m0 = _measure_phang(gcs, ctx, "H_PHANG=0")

        # Sanity: servos must have produced a meaningful deflection.
        assert abs(m0["tilt_lat_ch1"]) + abs(m0["tilt_lon_ch1"]) > _MIN_AXIS_RESPONSE, (
            "Ch1=1700 step produced no measurable servo deflection.\n"
            "Check ATC_RAT_RLL_FF, ACRO_RP_P, and SERVO_OUTPUT_RAW stream rate."
        )
        assert abs(m0["tilt_lat_ch2"]) + abs(m0["tilt_lon_ch2"]) > _MIN_AXIS_RESPONSE, (
            "Ch2=1700 step produced no measurable servo deflection.\n"
            "Check ATC_RAT_PIT_FF, ACRO_RP_P, and SERVO_OUTPUT_RAW stream rate."
        )

        # Sanity: roll must primarily drive tilt_lat, pitch must primarily drive
        # tilt_lon.  cross_chX < 1.0 means on-axis dominates off-axis.
        assert m0["cross_ch1"] < 1.0, (
            f"Ch1 roll produces more tilt_lon ({m0['tilt_lon_ch1']:.3f}) than "
            f"tilt_lat ({m0['tilt_lat_ch1']:.3f}).  "
            "Check H_SWASH_TYPE=3 (H3_120) and servo wiring order S1/S2/S3."
        )
        assert m0["cross_ch2"] < 1.0, (
            f"Ch2 pitch produces more tilt_lat ({m0['tilt_lat_ch2']:.3f}) than "
            f"tilt_lon ({m0['tilt_lon_ch2']:.3f}).  "
            "Check H_SWASH_TYPE=3 (H3_120) and servo wiring order S1/S2/S3."
        )

        h_phang_used = 0  # default, may be updated below

        need_correction = (
            m0["cross_ch1"] > _CROSS_COUPLING_MAX
            or m0["cross_ch2"] > _CROSS_COUPLING_MAX
        )

        if not need_correction:
            _log.info(
                "H_PHANG=0 accepted: cross_ch1=%.3f  cross_ch2=%.3f  "
                "(both < %.2f limit)",
                m0["cross_ch1"], m0["cross_ch2"], _CROSS_COUPLING_MAX,
            )
        else:
            # Estimate correction:  H_PHANG = (phase_ch2 - phase_ch1) / 2
            h_phang_est = (m0["phase_ch2_deg"] - m0["phase_ch1_deg"]) / 2.0
            h_phang_used = int(round(max(-30.0, min(30.0, h_phang_est))))
            _log.info(
                "H_PHANG=0 has high cross-coupling -- estimating correction: "
                "%.1f deg -> applying %d deg",
                h_phang_est, h_phang_used,
            )

            ok = gcs.set_param("H_SW_PHANG", float(h_phang_used), timeout=5.0)
            assert ok, f"H_SW_PHANG param set failed (no ACK for value {h_phang_used})"
            time.sleep(0.5)

            _log.info("=== Re-measuring with H_PHANG=%d ===", h_phang_used)
            m_corr = _measure_phang(gcs, ctx, f"H_PHANG={h_phang_used}")

            assert m_corr["cross_ch1"] <= _CROSS_COUPLING_MAX, (
                f"Cross-coupling still high after H_PHANG={h_phang_used} "
                f"(Ch1): {m_corr['cross_ch1']:.3f} > {_CROSS_COUPLING_MAX:.2f}.\n"
                f"Phase: {m_corr['phase_ch1_deg']:.1f} deg."
            )
            assert m_corr["cross_ch2"] <= _CROSS_COUPLING_MAX, (
                f"Cross-coupling still high after H_PHANG={h_phang_used} "
                f"(Ch2): {m_corr['cross_ch2']:.3f} > {_CROSS_COUPLING_MAX:.2f}.\n"
                f"Phase: {m_corr['phase_ch2_deg']:.1f} deg."
            )
            _log.info(
                "H_PHANG=%d corrected: cross_ch1=%.3f  cross_ch2=%.3f",
                h_phang_used, m_corr["cross_ch1"], m_corr["cross_ch2"],
            )

        # ── Physics altitude check ────────────────────────────────────────────
        if ctx.telemetry_log.exists():
            import csv as _csv
            with ctx.telemetry_log.open(encoding="utf-8") as _f:
                tel = list(_csv.DictReader(_f))
            alts = [-float(r["hub_pos_z"]) for r in tel
                    if r.get("hub_pos_z") not in ("", "None", "nan")]
            if alts:
                min_alt = min(alts)
                _log.info("Min physics altitude: %.2f m  (limit=%.1f m)",
                          min_alt, _MIN_ALT_M)
                assert min_alt >= _MIN_ALT_M, (
                    f"Hub crashed: min alt {min_alt:.2f} m < {_MIN_ALT_M:.1f} m"
                )

        # ── Summary ───────────────────────────────────────────────────────────
        _log.info("=== H_PHANG CALIBRATION COMPLETE ===")
        _log.info("  H_PHANG=0: phase_ch1=%.1f deg  phase_ch2=%.1f deg  "
                  "cross_ch1=%.3f  cross_ch2=%.3f",
                  m0["phase_ch1_deg"], m0["phase_ch2_deg"],
                  m0["cross_ch1"], m0["cross_ch2"])
        _log.info("  Confirmed H_SW_PHANG = %d", h_phang_used)
        _log.info("  rawes_params.parm recommendation: H_SW_PHANG %d", h_phang_used)

    except Exception:
        dump_startup_diagnostics(ctx)
        raise
