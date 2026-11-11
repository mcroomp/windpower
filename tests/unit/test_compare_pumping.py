import pytest

from analysis.compare_pumping import KEY_COLS, _bucket_rows, _print_comparison
from simulation.telemetry_columns import COLUMNS


def test_comparison_columns_are_canonical():
    assert len(KEY_COLS) > 0
    assert set(KEY_COLS) <= set(COLUMNS)


def test_bucket_averages_and_comparison_output(capsys: pytest.CaptureFixture[str]) -> None:
    rows = [
        {"t_sim": str(t), "phase": "reel_out", **dict.fromkeys(KEY_COLS, str(value))}
        for t, value in [(0.0, 1.0), (0.5, 3.0)]
    ]

    assert _bucket_rows(rows, "reel_out", 1.0) == [
        (0.5, dict.fromkeys(KEY_COLS, pytest.approx(2.0)))
    ]
    _print_comparison(rows, rows, "reel_out", 1.0)
    output = capsys.readouterr().out
    assert "Mean absolute difference per column" in output
    assert all(column in output for column in KEY_COLS)
