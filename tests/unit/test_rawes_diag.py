import re
from pathlib import Path

import pytest

from groundstation.rawes_diag import DIAG_ARRAY_ID, DIAG_ARRAY_NAME, DIAG_KEYS, diag_values
from simulation.rawes_lua_harness import RawesLua

_ROOT = Path(__file__).resolve().parents[2]


def _quoted_keys(text: str) -> list[str]:
    without_comments = re.sub(r"(--|//).*", "", text)
    return re.findall(r'"([A-Z_]+)"', without_comments)


def test_lua_key_order_matches_python_schema():
    source = (_ROOT / "scripts" / "rawes.lua").read_text(encoding="utf-8")
    block = re.search(r"local _diag_nvf_keys = \{(.*?)\n\}", source, re.DOTALL)
    assert block, "_diag_nvf_keys table not found in rawes.lua"
    assert tuple(_quoted_keys(block.group(1))) == DIAG_KEYS


def test_typescript_schema_matches_python_schema():
    source = (_ROOT / "linkhub-ui" / "src" / "diag-array.ts").read_text(encoding="utf-8")
    block = re.search(r"DIAG_KEYS = \[(.*?)\] as const", source, re.DOTALL)
    assert block, "DIAG_KEYS not found in diag-array.ts"
    assert tuple(_quoted_keys(block.group(1))) == DIAG_KEYS
    assert f"DIAG_ARRAY_ID = {DIAG_ARRAY_ID};" in source
    assert f'DIAG_ARRAY_NAME = "{DIAG_ARRAY_NAME}";' in source


def _data(values: dict[str, float]) -> list[float]:
    data = [0.0] * 58
    for key, value in values.items():
        index = DIAG_KEYS.index(key)
        data[0] += 1 << index
        data[index + 1] = value
    return data


def test_diag_values_returns_only_keys_the_mask_marks_as_set():
    values = diag_values(DIAG_ARRAY_ID, _data({"YFF_U": 0.25, "OL_AP": 0.0}))
    assert values == {"YFF_U": 0.25, "OL_AP": 0.0}


def test_diag_values_ignores_other_arrays_and_empty_data():
    assert diag_values(DIAG_ARRAY_ID + 1, _data({"YFF_U": 1.0})) == {}
    assert diag_values(DIAG_ARRAY_ID, []) == {}


def test_diag_values_tolerates_trimmed_and_null_slots():
    mask = (1 << DIAG_KEYS.index("YFF_T")) | (1 << DIAG_KEYS.index("ANCH_D"))
    assert diag_values(DIAG_ARRAY_ID, [float(mask), 0.5]) == {"YFF_T": 0.5, "ANCH_D": 0.0}
    assert diag_values(DIAG_ARRAY_ID, [None, 1.0]) == {}


def test_lua_emits_set_keys_in_one_array_at_tel_hz():
    sim = RawesLua(mode=3)
    sim.vehicle_mode = 20
    sim.healthy = True
    sim.armed = True
    assert sim.diag_array is None
    sim.run(1.2)

    array_id, name, data = sim.diag_array
    assert (array_id, name) == (DIAG_ARRAY_ID, DIAG_ARRAY_NAME)
    assert len(data) == len(DIAG_KEYS) + 1
    values = sim.diag_values()
    assert "OL_COL" in values
    assert values["OL_COL"] == pytest.approx(float(sim.fns.diag_nvf("OL_COL")), rel=1e-6)
    # Keys Lua has not set must not appear as zeros (yaw trim waits for ENTER_PASSIVE,
    # the anchor is unresolved).
    assert "YFF_T" not in values
    assert "ANCH_N" not in values
