"""
linkhub_client/mav_constants.py -- plain integer MAVLink constants.

Clients talk to LinkHub over finite HTTP/JSON requests only (see client.py) and
must not import pymavlink, even transitively.  MAVLink enumerations, bitmasks
and message ids are *not* here: LinkHub exposes enumerations as named, typed
values (``linkhub_client.messages.MavCmd``, ``MavResult``, ``MavModeFlag``, ...)
and every message class carries its id (``Attitude.MAVLINK_ID``).

What remains are ArduPilot custom flight modes, which are plain integers on the
MAVLink wire (HEARTBEAT.custom_mode).
"""
from __future__ import annotations

# -- ArduCopter custom modes (HEARTBEAT.custom_mode) --------------------------
STABILIZE = 0
GUIDED = 4
GUIDED_NOGPS = 20
