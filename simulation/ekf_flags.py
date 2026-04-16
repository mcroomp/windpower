"""
ekf_flags.py -- ArduPilot MAVLink EKF_STATUS_REPORT flag definitions.

Source of truth: ardupilot/libraries/AP_NavEKF3/AP_NavEKF3_Outputs.cpp
  mavlink_msg_ekf_status_report_send_struct()

The EKF_STATUS_REPORT.flags field is NOT a raw copy of the internal
nav_filter_status union bits.  It is populated via the MAVLink
EKF_STATUS_FLAGS enum (ardupilotmega.h), with one exception: the
gps_glitching bit is hardcoded as (1<<15) = 0x8000.

Only the flags that ArduPilot actually sends are defined here.
nav_filter_status bits 10-18 (takeoff_detected, using_gps,
gps_quality_good, etc.) are NOT transmitted.  0x0800 is unused.
"""

from __future__ import annotations

# ---------------------------------------------------------------------------
# EKF_STATUS_FLAGS (MAVLink) as sent by AP_NavEKF3_Outputs.cpp
# ---------------------------------------------------------------------------

EKF_FLAGS: dict[int, str] = {
    0x0001: "attitude",           # EKF_ATTITUDE
    0x0002: "horiz_vel",          # EKF_VELOCITY_HORIZ
    0x0004: "vert_vel",           # EKF_VELOCITY_VERT
    0x0008: "horiz_pos_rel",      # EKF_POS_HORIZ_REL
    0x0010: "horiz_pos_abs",      # EKF_POS_HORIZ_ABS
    0x0020: "vert_pos",           # EKF_POS_VERT_ABS
    0x0040: "terrain_alt",        # EKF_POS_VERT_AGL
    0x0080: "const_pos_mode",     # EKF_CONST_POS_MODE  -- EKF in constant-position fallback
    0x0100: "pred_horiz_pos_rel", # EKF_PRED_POS_HORIZ_REL
    0x0200: "pred_horiz_pos_abs", # EKF_PRED_POS_HORIZ_ABS
    0x0400: "uninitialized",      # EKF_UNINITIALIZED   -- EKF has never been healthy (!initalized)
    0x8000: "gps_glitching",      # (1<<15) hardcoded   -- nav_filter_status.gps_glitching
}

# Flags whose presence indicates a problem
EKF_WARN: frozenset[int] = frozenset({0x0080, 0x0400, 0x8000})

# ---------------------------------------------------------------------------
# GPS fix type names (GPS_RAW_INT.fix_type)
# ---------------------------------------------------------------------------

GPS_FIX: dict[int, str] = {
    0: "NO_FIX",
    1: "NO_FIX",
    2: "2D",
    3: "3D",
    4: "DGPS",
    5: "RTK_FLOAT",
    6: "RTK_FIXED",
}

# ---------------------------------------------------------------------------
# MAVLink HEARTBEAT constants
# ---------------------------------------------------------------------------

MAV_MODE_ARMED = 0x80  # base_mode bit: vehicle is armed

ARDU_MODES: dict[int, str] = {
    0:  "STABILIZE", 1:  "ACRO",     2:  "ALT_HOLD",
    3:  "AUTO",      4:  "GUIDED",   5:  "LOITER",
    6:  "RTL",       9:  "LAND",     16: "POSHOLD",
}

MAV_STATE: dict[int, str] = {
    0: "UNINIT", 1: "BOOT",    2: "CALIB",    3: "STANDBY",
    4: "ACTIVE", 5: "CRITICAL",6: "EMERGENCY",7: "POWEROFF",
    8: "FLIGHT_TERMINATION",
}

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def decode_flags(flags: int) -> str:
    """Return a comma-separated string of all set flag names.

    Unknown bits are shown as hex.  Returns "none" if flags == 0.

    Example
    -------
    >>> decode_flags(0x033f)
    'attitude, horiz_vel, vert_vel, horiz_pos_rel, horiz_pos_abs, vert_pos,
     pred_horiz_pos_rel, pred_horiz_pos_abs'
    """
    known   = [name for mask, name in EKF_FLAGS.items() if flags & mask]
    unknown = [f"0x{mask:04x}" for mask in _unknown_bits(flags)]
    parts   = known + unknown
    return ", ".join(parts) if parts else "none"


def flag_diff(old: int, new: int) -> str:
    """Return a compact diff string showing gained (+) and lost (-) flags.

    Example
    -------
    >>> flag_diff(0x00a7, 0x033f)
    '+[horiz_pos_rel, horiz_pos_abs, pred_horiz_pos_rel, pred_horiz_pos_abs] -[const_pos_mode]'
    """
    gained = new & ~old
    lost   = old & ~new
    parts  = []
    if gained:
        parts.append(f"+[{decode_flags(gained)}]")
    if lost:
        parts.append(f"-[{decode_flags(lost)}]")
    return " ".join(parts)


def has_warn(flags: int) -> bool:
    """Return True if any warning flag is set."""
    return bool(flags & sum(EKF_WARN))


def _unknown_bits(flags: int) -> list[int]:
    known_mask = sum(EKF_FLAGS)
    remainder  = flags & ~known_mask
    return [1 << i for i in range(32) if remainder & (1 << i)]
