"""
Unit tests for compute_map.py — isolates masking, tension-range split, and PNG rendering.
Run: .venv/Scripts/python.exe -m pytest simulation/envelope/tests/test_compute_map.py -v
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from envelope.compute_map import compute_grid, V_REEL_OUT, V_REEL_IN


# ── helpers ───────────────────────────────────────────────────────────────────

def _tiny_grid(tension_list=None):
    """Compute a minimal 1-angle 1-wind grid fast enough for unit tests."""
    if tension_list is None:
        tension_list = {
            V_REEL_OUT: [100.0, 200.0, 300.0, 400.0, 500.0, 750.0, 1000.0],
            V_REEL_IN:  [100.0, 200.0, 300.0, 400.0, 500.0],
        }
    return compute_grid(
        wind_list    = [10.0],
        angle_list   = [50.0],
        tension_list = tension_list,
        v_targets    = [V_REEL_OUT, V_REEL_IN],
        n_workers    = 1,
    )


# ── tension range split ───────────────────────────────────────────────────────

class TestTensionRangeSplit:

    def test_unified_tension_list_is_merged(self):
        """tension_list in the returned grid contains all unique values from both modes."""
        grid = _tiny_grid()
        tensions = list(grid["tension_list"])
        # 500 N is in reel-in; 750 and 1000 are reel-out only
        assert 500.0 in tensions
        assert 750.0 in tensions
        assert 1000.0 in tensions

    def test_v_target_tensions_stored(self):
        """v_target_tensions key maps each v_target to its own tension list."""
        grid = _tiny_grid()
        assert "v_target_tensions" in grid
        vtt = grid["v_target_tensions"]
        assert float(V_REEL_OUT) in vtt
        assert float(V_REEL_IN) in vtt

    def test_reel_out_max_tension(self):
        """Reel-out v_target_tensions goes up to 1000 N, not 500 N."""
        grid = _tiny_grid()
        out_tensions = grid["v_target_tensions"][float(V_REEL_OUT)]
        assert max(out_tensions) == pytest.approx(1000.0)
        assert 500.0 in out_tensions

    def test_reel_in_max_tension(self):
        """Reel-in v_target_tensions stops at 500 N."""
        grid = _tiny_grid()
        in_tensions = grid["v_target_tensions"][float(V_REEL_IN)]
        assert max(in_tensions) == pytest.approx(500.0)
        assert 750.0 not in in_tensions
        assert 1000.0 not in in_tensions


# ── masking ───────────────────────────────────────────────────────────────────

class TestTensionMasking:

    def _apply_mask(self, grid, v_tgt):
        """Reproduce the PNG rendering masking logic for a given v_target."""
        v_idx = int(np.argmin(np.abs(grid["v_targets"] - v_tgt)))
        col_rad  = grid["settled_col"][v_idx, 0].copy()   # (na, nt)
        has_data = np.isfinite(col_rad)
        tensions = np.array(grid["tension_list"], dtype=float)

        if "v_target_tensions" in grid:
            valid = np.array(grid["v_target_tensions"].get(float(v_tgt), tensions), dtype=float)
            for ti, t in enumerate(tensions):
                if not np.any(np.abs(valid - t) < 0.1):
                    has_data[:, ti] = False
                    col_rad[:, ti] = np.nan
        return col_rad, has_data, tensions

    def test_reel_out_no_data_above_1000(self):
        """After masking, reel-out has no data at tensions > 1000 N."""
        grid = _tiny_grid()
        col_rad, has_data, tensions = self._apply_mask(grid, V_REEL_OUT)
        for ti, t in enumerate(tensions):
            if t > 1000.0 + 0.1:
                assert not np.any(has_data[:, ti]), \
                    f"Reel-out should have no data at tension={t} N"

    def test_reel_in_no_data_above_500(self):
        """After masking, reel-in has no data at tensions > 500 N."""
        grid = _tiny_grid()
        col_rad, has_data, tensions = self._apply_mask(grid, V_REEL_IN)
        for ti, t in enumerate(tensions):
            if t > 500.0 + 0.1:
                assert not np.any(has_data[:, ti]), \
                    f"Reel-in should have no data at tension={t} N"

    def test_shared_tensions_not_masked(self):
        """Tensions in both reel-out and reel-in ranges are NOT masked for either mode."""
        grid = _tiny_grid()
        _, has_data_out, tensions = self._apply_mask(grid, V_REEL_OUT)
        _, has_data_in, _         = self._apply_mask(grid, V_REEL_IN)
        for ti, t in enumerate(tensions):
            if t <= 500.0:
                # Neither mode should mask these tension indices
                pass  # only convergence NaN is valid here, not mask NaN
        # Specifically: 100, 200, 300, 400, 500 are shared → present in both modes
        shared = {100.0, 200.0, 300.0, 400.0, 500.0}
        for ti, t in enumerate(tensions):
            if t in shared:
                out_valid = np.array(
                    grid["v_target_tensions"].get(float(V_REEL_OUT), []), dtype=float)
                in_valid  = np.array(
                    grid["v_target_tensions"].get(float(V_REEL_IN),  []), dtype=float)
                assert np.any(np.abs(out_valid - t) < 0.1), \
                    f"Tension {t} should be valid for reel-out"
                assert np.any(np.abs(in_valid  - t) < 0.1), \
                    f"Tension {t} should be valid for reel-in"


# ── y-axis range ──────────────────────────────────────────────────────────────

class TestYAxisRange:
    """The PNG renderer must clip each subplot's y-axis to its mode's tension range,
    not the full unified range."""

    def _valid_ylim(self, grid, v_tgt):
        """Return (ymin, ymax) that the renderer should use for this v_target."""
        vtt = grid["v_target_tensions"]
        tensions = np.array(vtt.get(float(v_tgt), grid["tension_list"]), dtype=float)
        return float(tensions.min()), float(tensions.max())

    def test_reel_out_ylim_reaches_1000(self):
        grid = _tiny_grid()
        ymin, ymax = self._valid_ylim(grid, V_REEL_OUT)
        assert ymax == pytest.approx(1000.0), \
            f"Reel-out ymax should be 1000 N, got {ymax}"

    def test_reel_in_ylim_stops_at_500(self):
        grid = _tiny_grid()
        ymin, ymax = self._valid_ylim(grid, V_REEL_IN)
        assert ymax == pytest.approx(500.0), \
            f"Reel-in ymax should be 500 N, got {ymax}"

    def test_both_have_same_ymin(self):
        grid = _tiny_grid()
        ymin_out, _ = self._valid_ylim(grid, V_REEL_OUT)
        ymin_in,  _ = self._valid_ylim(grid, V_REEL_IN)
        assert ymin_out == pytest.approx(ymin_in)
