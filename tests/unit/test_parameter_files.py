import os
import subprocess
import sys
from pathlib import Path

import pytest

from groundstation.parameter_files import load_parameter_files


def test_parameter_files_merge_in_order(tmp_path):
    base = tmp_path / "base.parm"
    override = tmp_path / "override.parm"
    base.write_text(
        "# comment\nH_COL_MIN 1000\nH_COL_MAX 2000 # inline\n"
        "INCOMPLETE\nINVALID value\nGAIN -2.5e-2\n",
        encoding="utf-8",
    )
    override.write_text("H_COL_MIN 1200\nH_COL_MAX 1557\n", encoding="utf-8")

    assert load_parameter_files([base, tmp_path / "missing.parm", str(override)]) == {
        "H_COL_MIN": 1200.0,
        "H_COL_MAX": 1557.0,
        "GAIN": -0.025,
    }


def test_parameter_files_require_a_file_list():
    with pytest.raises(FileNotFoundError, match="No ArduPilot"):
        load_parameter_files([])


def test_hardware_config_does_not_import_simulation_or_scipy():
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            "import sys\n"
            "from calibrate.params import _config_target_params\n"
            "for use_all in (False, True):\n"
            "    params = _config_target_params(use_all=use_all)\n"
            "    assert params['SERVO1_TRIM'] == 1517\n"
            "    assert params['H_COL_MIN'] == 1342\n"
            "for name in sys.modules:\n"
            "    assert name != 'simulation.param_defaults', name\n"
            "    assert name.split('.')[0] not in ('arduloop', 'scipy'), name\n",
        ],
        cwd=Path(__file__).resolve().parents[2],
        env={**os.environ, "PYTHONPATH": os.pathsep.join(sys.path)},
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr
