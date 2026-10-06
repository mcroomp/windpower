"""Schema of rawes.lua's diagnostic telemetry (a MAVLink DEBUG_FLOAT_ARRAY).

rawes.lua packs all diagnostic floats into one DEBUG_FLOAT_ARRAY instead of one
NAMED_VALUE_FLOAT per value (~5x fewer bytes on the link). The array carries no
names, so this ordered key list is the contract between the Lua emitter and
every reader:

    array_id  = DIAG_ARRAY_ID, name = DIAG_ARRAY_NAME
    data[0]   = bitmask (float-encoded integer) of the keys that were set;
                bit i set => data[1 + i] holds DIAG_KEYS[i]. Unset keys are
                sent as 0.0, so a clear bit -- not a zero value -- means "absent".
    data[1+i] = value of DIAG_KEYS[i]

Keep DIAG_KEYS in sync with ``_diag_nvf_keys`` in scripts/rawes.lua and
``DIAG_KEYS`` in linkhub-ui/src/diag-array.ts (tests/unit/test_rawes_diag.py
checks both). Append new keys at the end; reordering breaks old captures.
"""

from collections.abc import Sequence

DIAG_ARRAY_ID = 1
DIAG_ARRAY_NAME = "RAWES_DIAG"

DIAG_KEYS = (
    "YFF_T", "YFF_U", "YFF_GZ",           # yaw trim observer
    "OL_RSP", "OL_PSP", "OL_YSP",         # outer-loop commanded body rates
    "OL_RER", "OL_PER", "OL_YER",         # body-rate tracking errors
    "OL_AP", "OL_AI", "OL_AD", "OL_COL",  # altitude PID terms + commanded thrust
    "OL_TEN",                             # ramped tension feedforward [N]
    "ANCH_N", "ANCH_E", "ANCH_D",         # resolved anchor NED offset from EKF origin [m]
)

# Float32 represents integers exactly up to 2**24.
assert len(DIAG_KEYS) <= 24


def diag_values(array_id: int, data: Sequence[float | None]) -> dict[str, float]:
    """Decode a RAWES diagnostic array into ``{key: value}`` for the keys that were set.

    Returns an empty dict for any other array_id. ``data`` may be shorter than the
    schema (trailing zeros trimmed); missing slots read as 0.0.
    """
    if array_id != DIAG_ARRAY_ID or not data:
        return {}
    mask = int(data[0] or 0)
    values: dict[str, float] = {}
    for index, key in enumerate(DIAG_KEYS):
        if mask & (1 << index):
            slot = index + 1
            raw = data[slot] if slot < len(data) else 0.0
            values[key] = float(raw or 0.0)
    return values
