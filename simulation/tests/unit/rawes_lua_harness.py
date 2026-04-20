"""
rawes_lua_harness.py  --  Python harness for running rawes.lua in Lua 5.4.

Loads the actual rawes.lua Lua script with a minimal ArduPilot API mock
and exposes a clean Python interface.  Tests set sensor state, run time
forward, and read RC/servo outputs -- with no knowledge of Lua internals.

Time is fake: millis() reads from an internal counter advanced by run().
Simulation runs at Python execution speed with no real sleeping.

Usage:
    from rawes_lua_harness import RawesLua

    sim = RawesLua(mode=1)           # mode 1 = steady_noyaw
    sim.armed    = True
    sim.healthy  = True
    sim.gyro     = [0, 0, 0]
    sim.pos_ned  = None              # GPS not fused yet

    sim.run(0.5)                     # 0.5 sim-seconds at 100 Hz

    sim.pos_ned = [50.0, 0.0, -14.0]   # GPS fuses
    sim.run(1.0)

    assert sim.has_message("GPS")
    assert sim.ch_out[8] == 2000     # motor interlock keepalive
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
from lupa import lua54

# ── File paths ────────────────────────────────────────────────────────────────

_UNIT_DIR    = Path(__file__).resolve().parent
_SCRIPTS_DIR = _UNIT_DIR.parent.parent / "scripts"

_MOCK_LUA     = (_UNIT_DIR    / "mock_ardupilot.lua"   ).read_text(encoding="utf-8")
_RAWES_LUA    = (_SCRIPTS_DIR / "rawes.lua"            ).read_text(encoding="utf-8")
_SURFACE_LUA  = (_SCRIPTS_DIR / "rawes_test_surface.lua").read_text(encoding="utf-8")

# Splice the test surface in at the hook comment so every rawes.lua local
# is in scope when _rawes_fns is built.
_RAWES_WITH_SURFACE = _RAWES_LUA.replace("-- @@UNIT_TEST_HOOK", _SURFACE_LUA, 1)

# Wrap rawes.lua in an anonymous function so its locals stay encapsulated and
# its returned `update` function is captured in _rawes_update global.
_LOAD_RAWES = """\
_rawes_update = (function()
{body}
end)()
"""

# ── SCR_USER param shorthand map ──────────────────────────────────────────────

_PARAM_ALIAS = {
    "mode":     "SCR_USER6",   # flight mode (0-8)
    "kp":       "SCR_USER1",   # cyclic P gain
    "slew":     "SCR_USER2",   # body_z slew rate rad/s
    "anchor_n": "SCR_USER3",   # anchor North m
    "anchor_e": "SCR_USER4",   # anchor East  m
    "anchor_d": "SCR_USER5",   # anchor Down  m
}


# ── _ChOut proxy ──────────────────────────────────────────────────────────────

class _ChOut:
    """Dict-like view of RC channel override outputs.

    Supports integer indexing (1-based, matching ArduPilot channels):
        sim.ch_out[1]   -> roll PWM or None
        sim.ch_out[8]   -> motor interlock PWM or None
    """

    def __init__(self, mock):
        self._mock = mock

    def __getitem__(self, n: int):
        v = self._mock.ch_out[n]
        return int(v) if v is not None else None

    def __repr__(self):
        items = {n: int(self._mock.ch_out[n])
                 for n in range(1, 17)
                 if self._mock.ch_out[n] is not None}
        return f"ChOut({items})"


# ── RawesLua ──────────────────────────────────────────────────────────────────

class RawesLua:
    """ArduPilot-like harness for rawes.lua running in Lua 5.4 via lupa.

    Presents the same interface ArduPilot exposes to the script:
      * AHRS sensor state (attitude, gyro, GPS position, velocity)
      * Vehicle mode and arming state
      * SCR_USER parameters
      * RC channel and servo outputs from the script
      * GCS text messages logged by the script

    Time is simulated: run(seconds) advances an internal millisecond
    counter at 100 Hz (10 ms / tick) and calls the Lua update() each tick.
    No real sleeping occurs -- simulation runs as fast as Python allows.

    Parameters
    ----------
    **params : float
        Initial SCR_USER parameter values using shorthand names:
            mode      -> SCR_USER6   (flight mode, default 0)
            kp        -> SCR_USER1   (cyclic P gain, default 1.0)
            slew      -> SCR_USER2   (body_z slew rate rad/s, default 0.40)
            anchor_n  -> SCR_USER3   (anchor North m, default 0)
            anchor_e  -> SCR_USER4   (anchor East  m, default 0)
            anchor_d  -> SCR_USER5   (anchor Down  m, default 0)
        Full ArduPilot names (e.g. "SCR_USER6") are also accepted.
    """

    # Base tick rate: rawes.lua BASE_PERIOD_MS = 10 ms (100 Hz)
    _TICK_MS = 10

    def __init__(self, **params):
        self._lua = lua54.LuaRuntime()
        self._lua.execute(_MOCK_LUA)
        self._mock = self._lua.globals()._mock

        # Apply params before loading rawes.lua so the startup GCS message
        # (which reads SCR_USER6 at module load time) shows the right mode.
        for k, v in params.items():
            self._mock.params[_PARAM_ALIAS.get(k, k)] = float(v)

        self._lua.execute(_LOAD_RAWES.format(body=_RAWES_WITH_SURFACE))
        self._update_fn = self._lua.globals()._rawes_update
        self._fns       = self._lua.globals()._rawes_fns
        self._t_ms      = 0
        self.ch_out     = _ChOut(self._mock)

    # ── AHRS / vehicle state ──────────────────────────────────────────────

    @property
    def armed(self) -> bool:
        return bool(self._mock.armed)

    @armed.setter
    def armed(self, v: bool):
        self._mock.armed = bool(v)

    @property
    def arm_fail_n(self) -> int:
        """Number of arming:arm() calls that will silently fail before succeeding."""
        return int(self._mock.arm_fail_n)

    @arm_fail_n.setter
    def arm_fail_n(self, n: int):
        self._mock.arm_fail_n = int(n)

    @property
    def arm_call_count(self) -> int:
        """Total number of arming:arm() calls since last reset."""
        return int(self._mock.arm_call_count)

    @property
    def healthy(self) -> bool:
        """AHRS health flag -- script skips attitude maths when False."""
        return bool(self._mock.healthy)

    @healthy.setter
    def healthy(self, v: bool):
        self._mock.healthy = bool(v)

    @property
    def vehicle_mode(self) -> int:
        """ArduPilot flight mode number seen by vehicle:get_mode() (1 = ACRO)."""
        return int(self._mock.mode)

    @vehicle_mode.setter
    def vehicle_mode(self, v: int):
        self._mock.mode = int(v)

    @property
    def gyro(self) -> list[float]:
        """Body-frame gyro rates [x, y, z] rad/s."""
        g = self._mock.gyro
        return [float(g.x), float(g.y), float(g.z)]

    @gyro.setter
    def gyro(self, xyz):
        self._mock.gyro.x = float(xyz[0])
        self._mock.gyro.y = float(xyz[1])
        self._mock.gyro.z = float(xyz[2])

    @property
    def pos_ned(self):
        """Hub position in NED [x, y, z] m, or None when GPS is not fused."""
        p = self._mock.pos_ned
        if p is None:
            return None
        return [float(p.x), float(p.y), float(p.z)]

    @pos_ned.setter
    def pos_ned(self, xyz):
        """Set to [x, y, z] to simulate GPS fusion, or None to drop GPS."""
        if xyz is None:
            self._lua.execute("_mock.pos_ned = nil")
        else:
            x, y, z = float(xyz[0]), float(xyz[1]), float(xyz[2])
            self._mock.pos_ned = self._lua.eval(f"{{x={x}, y={y}, z={z}}}")

    @property
    def vel_ned(self) -> list[float]:
        """Hub velocity in NED [x, y, z] m/s."""
        v = self._mock.vel_ned
        return [float(v.x), float(v.y), float(v.z)]

    @vel_ned.setter
    def vel_ned(self, xyz):
        self._mock.vel_ned.x = float(xyz[0])
        self._mock.vel_ned.y = float(xyz[1])
        self._mock.vel_ned.z = float(xyz[2])

    @property
    def R(self) -> np.ndarray:
        """Rotation matrix body-to-NED (3x3 numpy array, row-major)."""
        r = self._mock.R
        return np.array([
            [r[1], r[2], r[3]],
            [r[4], r[5], r[6]],
            [r[7], r[8], r[9]],
        ])

    @R.setter
    def R(self, mat):
        """Set body-to-NED rotation (3x3 numpy array or nested list)."""
        flat = np.asarray(mat, dtype=float).flatten()
        for i, v in enumerate(flat):
            self._mock.R[i + 1] = float(v)

    # ── Parameters ────────────────────────────────────────────────────────

    def set_param(self, name: str, value: float):
        """Set a SCR_USER parameter by ArduPilot name or shorthand alias.

        Example:
            sim.set_param("mode", 5)          # SCR_USER6 = 5 (pumping)
            sim.set_param("SCR_USER1", 1.5)   # cyclic kp
        """
        self._mock.params[_PARAM_ALIAS.get(name, name)] = float(value)

    def get_param(self, name: str) -> float:
        """Read a SCR_USER parameter by ArduPilot name or shorthand alias."""
        v = self._mock.params[_PARAM_ALIAS.get(name, name)]
        return float(v) if v is not None else 0.0

    # ── Time control ──────────────────────────────────────────────────────

    @property
    def t_ms(self) -> int:
        """Current fake simulation time in milliseconds."""
        return self._t_ms

    @property
    def t_s(self) -> float:
        """Current fake simulation time in seconds."""
        return self._t_ms / 1000.0

    def run(self, seconds: float):
        """Advance simulation by `seconds` at 100 Hz (10 ms per update call).

        No real sleeping -- runs at Python execution speed.
        Each call to update() mirrors one ArduPilot scheduler tick.
        """
        n_ticks = max(1, round(seconds * (1000 / self._TICK_MS)))
        for _ in range(n_ticks):
            self._t_ms += self._TICK_MS
            self._mock.millis_val = self._t_ms
            self._update_fn()

    def tick(self):
        """Single 10 ms update -- useful when stepping through state manually."""
        self.run(self._TICK_MS / 1000.0)

    # ── Internal function access ──────────────────────────────────────────

    @property
    def fns(self):
        """Direct access to rawes.lua internal functions (from test surface).

        All functions are the real Lua implementations with access to the
        same mock AHRS/param state as the running script.

        Example:
            v  = sim.lua_vec(0, 0, 1)
            r  = sim.fns.rodrigues(v, sim.lua_vec(1, 0, 0), math.pi / 2)
            bz = sim.fns.disk_normal_ned()
        """
        return self._fns

    def lua_vec(self, x: float, y: float, z: float):
        """Create a Lua Vector3f with the given NED components."""
        return self._lua.eval(
            f"(function() local v=Vector3f(); v:x({x}); v:y({y}); v:z({z}); return v end)()"
        )

    def vec_to_list(self, v) -> list[float]:
        """Convert a Lua Vector3f returned by fns.* into a Python [x, y, z] list.

        Uses a Lua-side accessor to preserve colon-call semantics (v:x() not v.x()).
        """
        _get = self._lua.eval("function(v) return v:x(), v:y(), v:z() end")
        x, y, z = _get(v)
        return [float(x), float(y), float(z)]

    # ── Outputs ───────────────────────────────────────────────────────────

    def srv_pwm(self, func: int = 94) -> int | None:
        """PWM written to SRV_Channels for ArduPilot function number.

        Default func=94 is Script 1 (the GB4008 anti-rotation motor output).
        Returns None if the script has not set this output yet.
        """
        v = self._mock.srv_out[func]
        return int(v) if v is not None else None

    @property
    def messages(self) -> list[tuple[int, str]]:
        """All gcs:send_text calls as [(level, text), ...] since last clear."""
        n = len(self._mock.gcs_msgs)
        return [
            (int(self._mock.gcs_msgs[i].level), str(self._mock.gcs_msgs[i].msg))
            for i in range(1, n + 1)
        ]

    def has_message(self, text: str) -> bool:
        """True if any gcs:send_text message contains `text`."""
        return any(text in msg for _, msg in self.messages)

    def clear_messages(self):
        """Discard all accumulated gcs:send_text messages."""
        self._lua.execute("_mock.gcs_msgs = {}")

    # ── MAVLink named-float inject ─────────────────────────────────────────

    def send_named_float(self, name: str, value: float) -> None:
        """Inject a NAMED_VALUE_FLOAT into the Lua mavlink inbox.

        Builds the message with pymavlink (same serialiser used by the real GCS)
        and extracts the wire payload (bytes 6..6+len), then prepends 12 null
        bytes to match the mavlink_message_t internal-struct layout that
        ArduPilot's mavlink.receive_chan() returns.  Lua unpacks the payload
        starting at byte 13 (1-indexed) with string.unpack("<If10s", raw, 13).
        """
        from pymavlink import mavutil as _mu  # local import — not always needed

        _mav = _mu.mavlink.MAVLink(None, srcSystem=255, srcComponent=0)
        name_b = name.encode("ascii")[:10].ljust(10, b"\x00")
        msg = _mu.mavlink.MAVLink_named_value_float_message(
            time_boot_ms=0, name=name_b, value=float(value)
        )
        wire = msg.pack(_mav)
        # Extract payload from wire packet (skip 6-byte MAVLink v1 header and
        # 2-byte trailing CRC; payload length is wire[1]).
        payload = wire[6 : 6 + wire[1]]
        # Prepend 12-byte internal-struct header so payload lands at offset 13.
        raw = b"\x00" * 12 + payload
        # Push as a Lua string using hex escapes — safe for all byte values.
        lua_str = "".join(f"\\x{b:02x}" for b in raw)
        self._lua.execute(f'table.insert(_mock.mavlink_inbox, "{lua_str}")')
