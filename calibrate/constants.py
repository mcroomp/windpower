"""
calibrate/constants.py -- All module-level constants and tables.

Every submodule that needs gcs/param_defaults/servo_pwm imports them from here
via relative imports (from .constants import ...).
"""
from __future__ import annotations

import math
import os

# Re-exported so existing calibration modules share the one HTTP client.
from linkhub_client.client import LinkHubClient, LinkHubGenerationChanged, WallClock
from linkhub_client.messages import (
    Attitude,
    AttitudeQuaternion,
    EscTelemetry,
    PidTuning,
    BatteryStatus,
    RcChannels,
    SysStatus,
    CommandAck,
    ExtendedSysState,
    Heartbeat,
    LocalPositionNed,
    MavLandedState,
    NamedValueFloat,
    CommandLong,
    DebugFloatArray,
    decode_message,
    Statustext,
)
from simulation.servo_pwm      import (SWASH_PWM_MIN, SWASH_PWM_NEUTRAL, SWASH_PWM_MAX,
                                        MOTOR_PWM_MIN, MOTOR_PWM_MAX)

# Resolve paths from this package rather than importing the simulation runtime.
_REPO_ROOT  = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_SIM_DIR    = os.path.join(_REPO_ROOT, "simulation")


def load_ap_params(*args, **kwargs):
    """Load simulation parameters only when a config operation needs them."""
    from simulation.param_defaults import load_ap_params as _load_ap_params

    return _load_ap_params(*args, **kwargs)

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
# Single source of truth for the motor output location -- see design/hardware.md.
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
# HR3-120 forward mix constants -- physical bench rig azimuths.
# ---------------------------------------------------------------------------
_AZ_S1 = math.radians(-120.0)  # SV1: right-rear
_AZ_S2 = math.radians( 120.0)  # SV2: left-rear
_AZ_S3 = math.radians(   0.0)  # SV3: front

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

_LUA_MODES = {0: "none", 1: "steady", 2: "acro_manual", 3: "passive", 4: "landing"}

# ---------------------------------------------------------------------------
# Param file paths
# ---------------------------------------------------------------------------
_AP_BASE_PARM_PATH     = os.path.join(_REPO_ROOT, "tests", "sitl", "copter-heli.parm")
_RAWES_COMMON_PARM_PATH = os.path.join(_REPO_ROOT, "tests", "sitl", "rawes_common_defaults.parm")
_RAWES_HARDWARE_PARM_PATH = os.path.join(_REPO_ROOT, "hardware", "rawes_hardware_defaults.parm")

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
    "RAWES_MODE",
    "RAWES_YAW_SLP",
    "ARMING_SKIPCHK",
    "BRD_SAFETY_DEFLT",
    "ACRO_TRAINER",
    "FS_THR_ENABLE",
    "FS_GCS_ENABLE",
)

_TAIL_PARAM_NAMES = (
    "H_COL2YAW",
    "H_YAW_TRIM",
    "ATC_RAT_YAW_P",
    "ATC_RAT_YAW_I",
    "ATC_RAT_YAW_D",
    "ATC_RAT_YAW_IMAX",
    "ATC_RAT_YAW_FLTT",
    "ATC_RAT_YAW_FLTE",
    "ATC_RAT_YAW_FLTD",
)

_MOTOR_PATH_PARAM_NAMES = (
    "H_RSC_MODE",
    "H_RSC_RUNUP_TIME",
    "SERVO_BLH_MASK",
    "SERVO_BLH_BDMASK",
    "SERVO_BLH_AUTO",
    "SERVO_BLH_OTYPE",
    "SERVO_BLH_POLES",
    f"SERVO{SERVO_MOTOR}_MIN",
    f"SERVO{SERVO_MOTOR}_MAX",
    f"SERVO{SERVO_MOTOR}_TRIM",
    f"SERVO{SERVO_MOTOR}_FUNCTION",
    "RPM1_TYPE",
    "RPM1_ESC_MASK",
)

# ---------------------------------------------------------------------------
# Lua script directory
# ---------------------------------------------------------------------------
SCRIPTS_DIR = "/APM/scripts"

# ---------------------------------------------------------------------------
# Run mode config table
# ---------------------------------------------------------------------------
# IC-seed-specific trim key (passive mode only); sent as thrust [0..1] directly,
# no radians conversion.
_IC_TRIM_KEYS = {"thr"}

# Default passive IC thrust [0..1] when --trim thr is not given.
# Corresponds to -8.6 deg blade pitch (col_min=-0.28 rad, span=0.38 rad).
_PASSIVE_IC_THRUST = 0.342

_RUN_MODES = {
    "none": {
        "rawes_mode":  0,
        "doc":        "Lua idle (mode 0), armed-but-quiet.",
    },
    "passive": {
        "rawes_mode":  3,
        "flight_mode": 20,       # GUIDED_NOGPS, entered by Lua on ENTER_GUIDED
        "ic_seed":     True,
        "doc":        "interactive GUIDED_NOGPS attitude hold: captures the current quaternion; keys apply relative quaternion offsets and adjust held thrust",
    },
    "acro-manual": {
        "rawes_mode":  2,
        "flight_mode": 1,
        "manual_control": True,
        "doc":        "interactive ACRO flybar control: arrows=roll/pitch, -/=collective; AP yaw compensation remains active",
    },
    "steady": {
        "rawes_mode":  1,
        "doc":        "steady flight: altitude hold + VZ PI collective",
    },
    "pumping": {
        "rawes_mode":  1,
        "doc":        "De Schutter pumping cycle (runs in steady mode; ground varies tension)",
    },
    "landing": {
        "rawes_mode":  4,
        "doc":        "landing (reserved)",
    },
}

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
_LOG_DIR = os.path.join(_SIM_DIR, "logs", "calibrate")  # simulation/logs/calibrate/
