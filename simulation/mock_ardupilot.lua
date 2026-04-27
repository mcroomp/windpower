--[[
mock_ardupilot.lua  --  Minimal ArduPilot API stub for rawes.lua unit tests.

Load this script BEFORE rawes.lua.  All state lives in the global `_mock`
table.  Python (via lupa) writes inputs into _mock before each tick and
reads outputs from _mock after each tick.

Inputs (write before tick):
  _mock.armed          bool
  _mock.mode           int   (1 = ACRO)
  _mock.healthy        bool
  _mock.millis_val     int   (fake milliseconds -- no real sleep needed)
  _mock.gyro           {x, y, z}  rad/s
  _mock.pos_ned        {x, y, z} m  or nil (nil = GPS not yet fused)
  _mock.vel_ned        {x, y, z} m/s
  _mock.R              flat row-major 3x3, indices 1..9  (body_to_NED)
  _mock.accel          {x, y, z} m/s^2  body-frame specific force (gravity excluded)
  _mock.params         {SCR_USER1..6, ...}

Outputs (read after tick):
  _mock.ch_out[n]      RC channel n PWM override (nil if not set)
  _mock.srv_out[func]  SRV_Channels output for function number
  _mock.gcs_msgs       array of {level=int, msg=string}
--]]

_mock = {
    armed       = false,
    mode        = 1,            -- 1 = ACRO
    healthy     = false,
    millis_val  = 0,
    gyro        = {x=0.0, y=0.0, z=0.0},
    accel       = {x=0.0, y=0.0, z=0.0},  -- body-frame specific force [m/s^2]
    pos_ned     = nil,           -- nil until GPS fuses
    vel_ned     = {x=0.0, y=0.0, z=0.0},
    -- R: flat row-major body_to_NED 3x3, 1-indexed.  Default = identity.
    R           = {1,0,0, 0,1,0, 0,0,1},
    params      = {
        SCR_USER1 = 1.0,   -- KP_CYC
        SCR_USER2 = 0.40,  -- BZ_SLEW
        SCR_USER3 = 0.0,   -- anchor N
        SCR_USER4 = 0.0,   -- anchor E
        SCR_USER5 = 0.0,   -- anchor D
        SCR_USER6 = 0,     -- mode (0=disabled)
    },
    ch_out      = {},    -- [channel_n] = pwm
    srv_out     = {},    -- [func] = pwm
    gcs_msgs    = {},    -- array of {level, msg}
}

-- ── Vector3f ────────────────────────────────────────────────────────────────
-- Replicates the ArduPilot Lua binding quirks:
--   * Constructor ignores arguments -- use setter methods after.
--   * :normalize() is in-place.
--   * :x(v) sets x and returns new value; :x() returns current value.

Vector3f = {}
Vector3f.__index = Vector3f

function Vector3f.new()
    return setmetatable({_x=0.0, _y=0.0, _z=0.0}, Vector3f)
end

setmetatable(Vector3f, {__call = function(_cls) return Vector3f.new() end})

function Vector3f:x(v)  if v ~= nil then self._x = v end;  return self._x  end
function Vector3f:y(v)  if v ~= nil then self._y = v end;  return self._y  end
function Vector3f:z(v)  if v ~= nil then self._z = v end;  return self._z  end

function Vector3f:length()
    return math.sqrt(self._x^2 + self._y^2 + self._z^2)
end

function Vector3f:normalize()
    local n = self:length()
    if n > 1e-12 then
        self._x = self._x / n
        self._y = self._y / n
        self._z = self._z / n
    end
end

function Vector3f:cross(o)
    local r = Vector3f()
    r:x(self._y * o:z() - self._z * o:y())
    r:y(self._z * o:x() - self._x * o:z())
    r:z(self._x * o:y() - self._y * o:x())
    return r
end

function Vector3f:dot(o)
    return self._x * o:x() + self._y * o:y() + self._z * o:z()
end

-- ── millis() ─────────────────────────────────────────────────────────────────

function millis()
    return _mock.millis_val
end

-- ── ahrs ─────────────────────────────────────────────────────────────────────

ahrs = {}

function ahrs:healthy()
    return _mock.healthy
end

function ahrs:get_gyro()
    local v = Vector3f()
    v:x(_mock.gyro.x); v:y(_mock.gyro.y); v:z(_mock.gyro.z)
    return v
end

function ahrs:get_accel()
    local v = Vector3f()
    v:x(_mock.accel.x); v:y(_mock.accel.y); v:z(_mock.accel.z)
    return v
end

function ahrs:get_relative_position_NED_origin()
    if _mock.pos_ned == nil then return nil end
    local v = Vector3f()
    v:x(_mock.pos_ned.x); v:y(_mock.pos_ned.y); v:z(_mock.pos_ned.z)
    return v
end

function ahrs:get_velocity_NED()
    local v = Vector3f()
    v:x(_mock.vel_ned.x); v:y(_mock.vel_ned.y); v:z(_mock.vel_ned.z)
    return v
end

-- R is stored flat row-major: index 1..9 = R[0,0]..R[2,2]
function ahrs:body_to_earth(v)
    local R = _mock.R
    local r = Vector3f()
    r:x(R[1]*v:x() + R[2]*v:y() + R[3]*v:z())
    r:y(R[4]*v:x() + R[5]*v:y() + R[6]*v:z())
    r:z(R[7]*v:x() + R[8]*v:y() + R[9]*v:z())
    return r
end

function ahrs:earth_to_body(v)  -- R^T @ v
    local R = _mock.R
    local r = Vector3f()
    r:x(R[1]*v:x() + R[4]*v:y() + R[7]*v:z())
    r:y(R[2]*v:x() + R[5]*v:y() + R[8]*v:z())
    r:z(R[3]*v:x() + R[6]*v:y() + R[9]*v:z())
    return r
end

-- ── rc ───────────────────────────────────────────────────────────────────────

local _rc_channels = {}
rc = {}

function rc:get_channel(n)
    if _rc_channels[n] == nil then
        local ch = {_n = n}
        function ch:set_override(pwm)
            _mock.ch_out[self._n] = pwm
        end
        function ch:get_value()
            return _mock.ch_out[self._n] or 1500
        end
        _rc_channels[n] = ch
    end
    return _rc_channels[n]
end

-- ── SRV_Channels ─────────────────────────────────────────────────────────────

SRV_Channels = {}

function SRV_Channels:set_output_pwm(func, pwm)
    _mock.srv_out[func] = pwm
end

-- ── param ────────────────────────────────────────────────────────────────────

param = {}

function param:get(name)
    return _mock.params[name]
end

-- ── gcs ──────────────────────────────────────────────────────────────────────

gcs = {}

function gcs:send_text(level, msg)
    _mock.gcs_msgs[#_mock.gcs_msgs + 1] = {level = level, msg = msg}
end

-- ── arming ───────────────────────────────────────────────────────────────────
-- arm_fail_n: number of arming:arm() calls that will silently fail before the
-- next call succeeds.  Set from Python to test retry behaviour.
-- arm_call_count: total calls to arming:arm() since last reset.

_mock.arm_fail_n    = 0   -- how many arm() calls to silently reject before succeeding
_mock.arm_call_count = 0  -- total arm() calls (for test assertions)

arming = {}

function arming:is_armed()  return _mock.armed end
function arming:disarm()    _mock.armed = false end

function arming:arm()
    _mock.arm_call_count = _mock.arm_call_count + 1
    if _mock.arm_fail_n > 0 then
        _mock.arm_fail_n = _mock.arm_fail_n - 1
        return  -- prearm failure: do not set armed
    end
    _mock.armed = true
end

-- ── vehicle ──────────────────────────────────────────────────────────────────

vehicle = {}

function vehicle:get_mode()  return _mock.mode end

-- ── mavlink ──────────────────────────────────────────────────────────────────
-- Minimal stub for mavlink.init / register_rx_msgid / receive_chan.
-- Python injects raw byte strings into _mock.mavlink_inbox; receive_chan()
-- pops them one at a time in FIFO order, matching ArduPilot's queue semantics.
-- The raw string layout mirrors mavlink_message_t: 12 header bytes followed
-- by the message payload, so string.unpack("<If10s", raw, 13) works correctly.

_mock.mavlink_inbox = {}   -- queue of raw byte strings

mavlink = {}

function mavlink.init(_tx_slots, _rx_slots)
    -- no-op in mock
end

function mavlink.register_rx_msgid(_msgid)
    -- no-op in mock
end

function mavlink.receive_chan()
    local inbox = _mock.mavlink_inbox
    if #inbox == 0 then return nil end
    local raw = table.remove(inbox, 1)   -- pop front (FIFO)
    return raw, 0, 0                     -- raw, chan, timestamp
end
