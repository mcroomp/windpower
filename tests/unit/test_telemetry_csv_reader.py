from simulation.telemetry_columns import COLUMNS
from simulation.telemetry_csv import read_csv


def test_read_csv_ignores_incomplete_appended_row(tmp_path) -> None:
    path = tmp_path / "telemetry.csv"
    complete = [""] * len(COLUMNS)
    complete[COLUMNS.index("t_sim")] = "1.0"
    partial = ["2.0", "0.0", "2"]
    path.write_text(
        ",".join(COLUMNS) + "\n"
        + ",".join(complete) + "\n"
        + ",".join(partial),
        encoding="utf-8",
    )

    rows = read_csv(path)

    assert [row.t_sim for row in rows] == [1.0]
