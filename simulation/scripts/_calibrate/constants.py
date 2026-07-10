"""
_calibrate/constants.py -- All module-level constants, tables, and sys.path setup.

Every submodule that needs gcs/param_defaults/servo_pwm imports them from here
via relative imports (from .constants import ...).
"""
from __future__ import annotations

import math
import os
import sys

# ---------------------------------------------------------------------------
# sys.path setup — make simulation/ importable (for gcs, param_defaults, etc.)
# ---------------------------------------------------------------------------
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))          # …/scripts/_calibrate/
_SIM_DIR    = os.path.abspath(os.path.join(_SCRIPT_DIR, '..', '..'))  # …/simulation/
if _SIM_DIR not in sys.path:
    sys.path.insert(0, _SIM_DIR)

# Re-exported so submodules can do `from .constants import RawesGCS` etc.
from gcs            import RawesGCS, WallClock         # noqa: E402
from param_defaults import load_ap_params              # noqa: E402
from servo_pwm      import (SWASH_PWM_MIN, SWASH_PWM_NEUTRAL, SWASH_PWM_MAX,
                             MOTOR_PWM_MIN, MOTOR_PWM_MAX)  # noqa: E402

# ---------------------------------------------------------------------------
# GB4008 motor constants (used in diag torque estimates)
# ---------------------------------------------------------------------------
GB4008_KV         = 66.0      # rev/min/V
GB4008_POLES      = 22        # rotor magnet poles = SERVO_BLH_POLES (compile-time default)
GB4008_POLE_PAIRS = GB4008_POLES // 2   # 11 (overridden at connect from FC's SERVO_BLH_POLES)
GB4008_GEAR_RATIO = 10.0      # motor shaft turns per output shaft turn (10:1)

# Kt = 60 / (2*pi*Kv)  [N*m/A at motor shaft]
GB4008_KT = 60.0 / (2.0 * math.pi * GB4008_KV)  # ~0.144 N*m/A

# ---------------------------------------------------------------------------
# RAWES servo output numbers
# ---------------------------------------------------------------------------
SERVO_S1    = 1
SERVO_S2    = 2
SERVO_S3    = 3
# GB4008 anti-rotation motor: bidirectional DShot on AUX 1 = SERVO9 (output 9).
# Single source of truth for the motor output location -- see design/dshot.md.
SERVO_MOTOR         = 9
MOTOR_TEST_INSTANCE = 4   # Heli tail motor NUMBER (Motor4); independent of output

# DShot motor throttle endpoints.
MOTOR_OFF_US  = 1000   # DShot idle = throttle 0 = motor off
MOTOR_FULL_US = 2000   # DShot full throttle

SWASH_SERVOS = (SERVO_S1, SERVO_S2, SERVO_S3)

# ---------------------------------------------------------------------------
# DShot RPM telemetry
# ---------------------------------------------------------------------------
MOTOR_ESC_CHANNEL = SERVO_MOTOR

# ESC_TELEMETRY_x_TO_y decode: message name -> (numeric id, first output channel).
_ESC_TELEM_MSGS = {
    "ESC_TELEMETRY_1_TO_4":  (11030, 1),
    "ESC_TELEMETRY_5_TO_8":  (11031, 5),
    "ESC_TELEMETRY_9_TO_12": (11032, 9),
}

# ---------------------------------------------------------------------------
# H3-120 forward mix constants -- bench rig azimuths.
# ---------------------------------------------------------------------------
_AZ_S1 = math.radians(-60.0)   # SV1: front-right
_AZ_S2 = math.radians( 60.0)   # SV2: front-left
_AZ_S3 = math.radians(180.0)   # SV3: back

# PWM range constants — imported from servo_pwm.py; local aliases for brevity.
PWM_MIN     = SWASH_PWM_MIN
PWM_NEUTRAL = SWASH_PWM_NEUTRAL
PWM_MAX     = SWASH_PWM_MAX

# ---------------------------------------------------------------------------
# Mode / status decode tables
# ---------------------------------------------------------------------------
_COPTER_MODES = {
    0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED",
    5: "LOITER", 6: "RTL", 7: "CIRCLE", 9: "LAND", 11: "DRIFT",
    13: "SPORT", 14: "FLIP", 15: "AUTOTUNE", 16: "POSHOLD", 17: "BRAKE",
    18: "THROW", 19: "AVOID_ADSB", 20: "GUIDED_NOGPS", 21: "SMART_RTL",
}

_SYS_STATUS = {0: "UNINIT", 1: "BOOT", 2: "CALIBRATING", 3: "STANDBY",
               4: "ACTIVE", 5: "CRITICAL", 6: "EMERGENCY", 7: "POWEROFF"}

_LUA_MODES = {0: "none", 1: "steady", 3: "passive", 4: "landing", 5: "pumping"}

# ---------------------------------------------------------------------------
# Param file paths
# ---------------------------------------------------------------------------
_AP_BASE_PARM_PATH     = os.path.join(_SIM_DIR, "tests", "sitl", "copter-heli.parm")
_RAWES_COMMON_PARM_PATH = os.path.join(_SIM_DIR, "tests", "sitl", "rawes_common_defaults.parm")

# Never push hardware-calibrated sensor values from defaults to a real FC.
_CALIBRATION_PARAM_PREFIXES = (
    "INS_ACCOFFS_",
    "INS_ACCSCAL_",
    "INS_ACC2OFFS_",
    "INS_ACC2SCAL_",
    "INS_ACC3OFFS_",
    "INS_ACC3SCAL_",
    "INS_GYROFFS_",
    "INS_GYR2OFFS_",
    "INS_GYR3OFFS_",
    "COMPASS_OFS",
    "COMPASS_DIA",
    "COMPASS_ODI",
    "COMPASS_MOT",
    "BARO1_GND_PRESS",
    "BARO2_GND_PRESS",
    "BARO3_GND_PRESS",
    "BARO1_GND_TEMP",
    "BARO2_GND_TEMP",
    "BARO3_GND_TEMP",
    "GND_ABS_PRESS",
    "GND_TEMP",
    "AHRS_TRIM_",
)

# ---------------------------------------------------------------------------
# Status display param lists
# ---------------------------------------------------------------------------
_KEY_PARAM_NAMES = (
    "FRAME_CLASS",
    "INITIAL_MODE",
    "H_TAIL_TYPE",
    "SCR_ENABLE",
    "SCR_USER6",
    "ARMING_CHECK",
    "BRD_SAFETY_DEFLT",
    "ACRO_TRAINER",
)

_TAIL_PARAM_NAMES = (
    "H_YAW_TRIM",
    "ATC_RAT_YAW_P",
    "ATC_RAT_YAW_I",
    "ATC_RAT_YAW_D",
    "ATC_RAT_YAW_IMAX",
    "ATC_RAT_YAW_FLTT",
    "ATC_RAT_YAW_FLTE",
    "ATC_RAT_YAW_FLTD",
    "H_RSC_MODE",
    "H_RSC_RUNUP_TIME",
    f"SERVO{SERVO_MOTOR}_MIN",
    f"SERVO{SERVO_MOTOR}_MAX",
    f"SERVO{SERVO_MOTOR}_TRIM",
    f"SERVO{SERVO_MOTOR}_FUNCTION",
)

# ---------------------------------------------------------------------------
# Lua script directory
# ---------------------------------------------------------------------------
SCRIPTS_DIR = "/APM/scripts"

# ---------------------------------------------------------------------------
# COM port scan fallback bauds
# ---------------------------------------------------------------------------
_FALLBACK_BAUDS = [57600, 38400, 19200, 9600]

# ---------------------------------------------------------------------------
# Run mode config table
# ---------------------------------------------------------------------------
_TRIM_NVF = {"tlon": "RAWES_TLN", "tlat": "RAWES_TLT", "col": "RAWES_COL"}

# Default passive IC collective [deg blade pitch] when --trim col is not given.
_PASSIVE_IC_COL_DEG = -8.6

_RUN_MODES = {
    "none": {
        "scr_user6":   0,
        "take_servo4": False,
        "gain_keys": {"trim": "H_YAW_TRIM"},
        "doc":        "Lua idle (mode 0), armed-but-quiet; only --gain trim=<value> (H_YAW_TRIM) changes the static yaw-motor throttle.",
    },
    "passive": {
        "scr_user6":   3,
        "flight_mode": 20,       # GUIDED_NOGPS (ArduCopter mode 20)
        "ic_seed":     True,
        "take_servo4": False,
        "gain_keys": {},
        "force_params": {
            "ATC_RAT_YAW_P":  0.0,
            "ATC_RAT_YAW_I":  0.0,
            "ATC_RAT_YAW_D":  0.0,
            "ATC_RAT_YAW_FF": 0.0,
        },
        "doc":        "armed-but-quiet in GUIDED_NOGPS (matches the SITL passive test): seeds the IC (RAWES_COL/RIC/PIC) and holds the IC attitude via the GUIDED angle API.  IC via --trim col=<deg> --roll <deg> --pitch <deg>.",
    },
    "steady": {
        "scr_user6":  1,
        "take_servo4": False,
        "gain_keys":  {},
        "doc":        "steady flight: altitude hold + VZ PI collective",
    },
    "pumping": {
        "scr_user6":  1,
        "take_servo4": False,
        "gain_keys":  {},
        "doc":        "De Schutter pumping cycle (runs in steady mode; ground varies tension)",
    },
    "landing": {
        "scr_user6":  4,
        "take_servo4": False,
        "gain_keys":  {},
        "doc":        "landing (reserved)",
    },
}

# AP yaw-rate PID params that MUST be zero while the Lua owns H_YAW_TRIM.
_AP_YAW_ZERO_PARAMS = ("ATC_RAT_YAW_P", "ATC_RAT_YAW_I",
                       "ATC_RAT_YAW_D", "ATC_RAT_YAW_FF")

# ---------------------------------------------------------------------------
# Oscillation sequences for `--osc {all|s1|s2|s3}`
# ---------------------------------------------------------------------------
_OSC_BASE_COL_DEG = -8.6   # IC operating-point collective baseline
_OSC_DELTA_COL    =  1.08  # Half collective swing (deg) at Delta = 0.3

_OSCILLATE_STEPS_ALL = [
    # (tlon_deg, tlat_deg, col_deg, label)
    ( 0.0,  0.0,  -8.6, "center"),
    (+5.0,  0.0,  -8.6, "tlon +5 (nose-down)"),
    ( 0.0,  0.0,  -8.6, "center"),
    (-5.0,  0.0,  -8.6, "tlon -5 (nose-up)"),
    ( 0.0,  0.0,  -8.6, "center"),
    ( 0.0, +5.0,  -8.6, "tlat +5 (roll-right)"),
    ( 0.0,  0.0,  -8.6, "center"),
    ( 0.0, -5.0,  -8.6, "tlat -5 (roll-left)"),
    ( 0.0,  0.0,  -8.6, "center"),
    ( 0.0,  0.0,  +3.0, "col +3 (positive)"),
    ( 0.0,  0.0,  -8.6, "center"),
    ( 0.0,  0.0, -12.0, "col -12 (negative)"),
    ( 0.0,  0.0,  -8.6, "center"),
]

_OSCILLATE_STEPS_S1 = [
    (   0.0,    0.0, _OSC_BASE_COL_DEG,                   "center"),
    ( -2.22,  +3.85, _OSC_BASE_COL_DEG + _OSC_DELTA_COL, "S1 UP"),
    (   0.0,    0.0, _OSC_BASE_COL_DEG,                   "center"),
    ( +2.22,  -3.85, _OSC_BASE_COL_DEG - _OSC_DELTA_COL, "S1 DOWN"),
    (   0.0,    0.0, _OSC_BASE_COL_DEG,                   "center"),
]
_OSCILLATE_STEPS_S2 = [
    (   0.0,    0.0, _OSC_BASE_COL_DEG,                   "center"),
    ( -2.22,  -3.85, _OSC_BASE_COL_DEG + _OSC_DELTA_COL, "S2 UP"),
    (   0.0,    0.0, _OSC_BASE_COL_DEG,                   "center"),
    ( +2.22,  +3.85, _OSC_BASE_COL_DEG - _OSC_DELTA_COL, "S2 DOWN"),
    (   0.0,    0.0, _OSC_BASE_COL_DEG,                   "center"),
]
_OSCILLATE_STEPS_S3 = [
    (   0.0,    0.0, _OSC_BASE_COL_DEG,                   "center"),
    ( +4.44,    0.0, _OSC_BASE_COL_DEG + _OSC_DELTA_COL, "S3 UP"),
    (   0.0,    0.0, _OSC_BASE_COL_DEG,                   "center"),
    ( -4.44,    0.0, _OSC_BASE_COL_DEG - _OSC_DELTA_COL, "S3 DOWN"),
    (   0.0,    0.0, _OSC_BASE_COL_DEG,                   "center"),
]

_OSCILLATE_TARGETS = {
    "all": _OSCILLATE_STEPS_ALL,
    "s1":  _OSCILLATE_STEPS_S1,
    "s2":  _OSCILLATE_STEPS_S2,
    "s3":  _OSCILLATE_STEPS_S3,
}
_OSCILLATE_STEP_S = 5.0

# ---------------------------------------------------------------------------
# Watch stream labels
# ---------------------------------------------------------------------------
_WATCH_STREAMS = {
    "servos":   "Stream SERVO_OUTPUT_RAW for ch1..8",
    "esc":      "Stream ESC_TELEMETRY for rpm/volt/current/temp",
    "text":     "Stream STATUSTEXT only",
    "attitude": "Stream ATTITUDE (roll/pitch/yaw + body rates)",
    "power":    "Stream BATTERY_STATUS / SYS_STATUS (vbat / current / power)",
}

# ---------------------------------------------------------------------------
# Logging directory
# ---------------------------------------------------------------------------
_LOG_DIR = os.path.join(_SIM_DIR, "logs", "calibrate")
