"""
linkhub_client/mav_constants.py -- internal stable wire constants.

Clients talk to LinkHub over finite HTTP/JSON requests only (see client.py) and
must not import pymavlink, even transitively.  These are the small handful of
MAV_CMD / MAV_* enum values calibrate actually sends or compares against;
values are the standard MAVLink common/ardupilotmega dialect numbers (the
same values pymavlink.mavutil.mavlink exposes) and are stable wire constants,
not implementation details of pymavlink.
"""
from __future__ import annotations

import sys
from types import SimpleNamespace

# -- MAV_CMD ------------------------------------------------------------
MAV_CMD_DO_SET_MODE = 176
MAV_CMD_DO_SET_SERVO = 183
MAV_CMD_DO_MOTOR_TEST = 209
MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN = 246
MAV_CMD_COMPONENT_ARM_DISARM = 400
MAV_CMD_SET_MESSAGE_INTERVAL = 511

# -- MAV_MODE_FLAG (bitmask) ---------------------------------------------
MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
MAV_MODE_FLAG_SAFETY_ARMED = 128

STABILIZE = 0
GUIDED = 4
GUIDED_NOGPS = 20

# -- SET_ATTITUDE_TARGET type mask -------------------------------------------
ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE = 1
ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE = 2
ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE = 4
ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE = 64

# -- EKF_STATUS_REPORT flags -------------------------------------------------
EKF_ATTITUDE = 1

# -- MAV_RESULT -----------------------------------------------------------
MAV_RESULT_ACCEPTED = 0
MAV_RESULT_TEMPORARILY_REJECTED = 1
MAV_RESULT_DENIED = 2
MAV_RESULT_UNSUPPORTED = 3
MAV_RESULT_FAILED = 4

# -- MAV_PARAM_TYPE ---------------------------------------------------------
MAV_PARAM_TYPE_INT8 = 2
MAV_PARAM_TYPE_INT16 = 4
MAV_PARAM_TYPE_INT32 = 6
MAV_PARAM_TYPE_REAL32 = 9

# -- MAV_DATA_STREAM --------------------------------------------------------
MAV_DATA_STREAM_EXTENDED_STATUS = 2
MAV_DATA_STREAM_RC_CHANNELS = 3
MAV_DATA_STREAM_RAW_CONTROLLER = 4
MAV_DATA_STREAM_EXTRA1 = 10
MAV_DATA_STREAM_EXTRA3 = 12

# -- MAV_STATE ---------------------------------------------------------------
MAV_STATE_ACTIVE = 4
MAV_STATE_STANDBY = 3

# -- MAV_TYPE / MAV_AUTOPILOT (GCS heartbeat identity) -----------------------
MAV_TYPE_GCS = 6
MAV_TYPE_HELICOPTER = 4
MAV_AUTOPILOT_INVALID = 8
MAV_AUTOPILOT_ARDUPILOTMEGA = 3

# -- Message ids (used only for SET_MESSAGE_INTERVAL requests) --------------
MAVLINK_MSG_ID_ATTITUDE_QUATERNION = 31
MAVLINK_MSG_ID_LOCAL_POSITION_NED = 32
MAVLINK_MSG_ID_RC_CHANNELS = 65
MAVLINK_MSG_ID_ATTITUDE_TARGET = 83

# Existing calibration algorithms use the familiar ``mavutil.mavlink.CONSTANT``
# spelling. This namespace contains constants only; it does not provide or load
# pymavlink.
mavutil = SimpleNamespace(mavlink=sys.modules[__name__])
