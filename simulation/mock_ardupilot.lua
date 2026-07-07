--[[
mock_ardupilot.lua  --  Minimal ArduPilot API stub for rawes.lua unit tests.

Load this script BEFORE rawes.lua.  All state lives in the global `_mock`
table.  Python (via lupa) writes inputs into _mock before each tick and
reads outputs from _mock after each tick.

Inputs (write before tick):
  _mock.armed          bool
--  _mock.mode           int   (4 = GUIDED)
  _mock.healthy        bool
  _mock.millis_val     int   (fake milliseconds -- no real sleep needed)
  _mock.gyro           {x, y, z}  rad/s
  _mock.pos_ned        {x, y, z} m  or nil (nil = GPS not yet fused)
  _mock.vel_ned        {x, y, z} m/s
  _mock.R              flat row-major 3x3, indices 1..9  (body_to_NED)
  _mock.accel          {x, y, z} m/s^2  body-frame specific force (gravity excluded)
  _mock.params         {SCR_USER6, ...}  (mode only; slew/anchor via NAMED_VALUE_FLOAT)

Outputs (read after tick):
  _mock.ch_out[n]      RC channel n PWM override (nil if not set)
  _mock.srv_out[func]  SRV_Channels output for function number
  _mock.srv_chan_out[n] SRV_Channels output by physical channel (1-indexed; chan 0-based+1)
  _mock.gcs_msgs       array of {level=int, msg=string}
    _mock.guided_target  {roll_deg,pitch_deg,yaw_deg,climbrate,use_yaw_rate,yaw_rate_degs}
                                             set by set_target_angle_and_climbrate (nil until first call)
    _mock.guided_rate_target {roll_rate,pitch_rate,yaw_rate}
                                             set by set_target_rate_and_throttle
--]]

_mock = {
    armed       = false,
    mode        = 4,            -- 4 = GUIDED
    healthy     = false,
    millis_val  = 0,
    gyro        = {x=0.0, y=0.0, z=0.0},
    accel       = {x=0.0, y=0.0, z=0.0},  -- body-frame specific force [m/s^2]
    pos_ned     = nil,           -- nil until GPS fuses
    vel_ned     = {x=0.0, y=0.0, z=0.0},
    -- R: flat row-major body_to_NED 3x3, 1-indexed.  Default = identity.
    R           = {1,0,0, 0,1,0, 0,0,1},
    params      = {
        -- Mode is the only SCR_USER parameter rawes.lua reads; slew + anchor
        -- are delivered via NAMED_VALUE_FLOAT (RAWES_SLW/ANN/ANE/AND).
        SCR_USER6 = 0,     -- mode (0=disabled)
    },
    ch_out      = {},    -- [channel_n] = pwm
    srv_out     = {},    -- [func] = pwm
    srv_chan_out = {},   -- [chan_1indexed] = pwm (set_output_pwm_chan_timeout; SERVO4_CHAN=3 -> key 4)
    gcs_msgs    = {},    -- array of {level, msg}
    guided_target      = nil, -- set_target_angle_and_* payload (attitude)
    guided_rate_target = nil, -- set_target_rate_and_throttle payload (body rates)
    guided_throttle    = nil, -- throttle [0,1] from set_target_*_and_throttle
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

-- ── uint32_t ─────────────────────────────────────────────────────────────────
-- Replicates the ArduPilot Lua uint32_t userdata returned by millis()/micros().
-- The real binding stores an unsigned 32-bit integer and coerces the *other*
-- operand of every arithmetic/comparison op to uint32_t.  Coercion FAILS with
-- "Unable to coerce to uint32_t" when the operand has a fractional part -- this
-- is exactly what catches `millis() * 0.001` style bugs (multiply by a fraction)
-- at unit-test time instead of only on real hardware.  Negative integers wrap
-- (two's complement) like the real binding, so `now - (-2000)` still works.

local UINT32_MOD = 4294967296  -- 2^32

local Uint32 = {}
Uint32.__index = Uint32

local function _coerce_u32(x)
    if type(x) == "table" and getmetatable(x) == Uint32 then
        return x._v
    end
    if type(x) == "number" then
        if x ~= math.floor(x) then
            error("Unable to coerce to uint32_t", 2)
        end
        return x % UINT32_MOD
    end
    error("Unable to coerce to uint32_t", 2)
end

local function _new_u32(v)
    return setmetatable({_v = v % UINT32_MOD}, Uint32)
end

function Uint32:tofloat() return self._v + 0.0 end
function Uint32:toint()   return math.floor(self._v) end

Uint32.__add      = function(a, b) return _new_u32(_coerce_u32(a) + _coerce_u32(b)) end
Uint32.__sub      = function(a, b) return _new_u32(_coerce_u32(a) - _coerce_u32(b)) end
Uint32.__mul      = function(a, b) return _new_u32(_coerce_u32(a) * _coerce_u32(b)) end
Uint32.__div      = function(a, b) return _new_u32(math.floor(_coerce_u32(a) / _coerce_u32(b))) end
Uint32.__mod      = function(a, b) return _new_u32(_coerce_u32(a) % _coerce_u32(b)) end
Uint32.__lt       = function(a, b) return _coerce_u32(a) <  _coerce_u32(b) end
Uint32.__le       = function(a, b) return _coerce_u32(a) <= _coerce_u32(b) end
Uint32.__eq       = function(a, b) return _coerce_u32(a) == _coerce_u32(b) end
Uint32.__tostring = function(a)    return tostring(a._v) end

-- ── millis() ─────────────────────────────────────────────────────────────────

function millis()
    return _new_u32(_mock.millis_val)
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

-- ZYX Euler angles derived from _mock.R (row-major body-to-NED).
-- R layout: rows are NED axes, so R[1..3]=row0, R[4..6]=row1, R[7..9]=row2.
-- R[:,2] = body_z in NED = (R[3], R[6], R[9]).
-- Standard ZYX decomposition: pitch = asin(-R[7]), roll = atan2(R[8],R[9]), yaw = atan2(R[4],R[1]).
function ahrs:get_roll()
    local R = _mock.R
    return math.atan(R[8], R[9])
end

function ahrs:get_roll_rad()
    return self:get_roll()
end

function ahrs:get_pitch()
    local R = _mock.R
    return math.asin(math.max(-1.0, math.min(1.0, -R[7])))
end

function ahrs:get_pitch_rad()
    return self:get_pitch()
end

function ahrs:get_yaw()
    local R = _mock.R
    return math.atan(R[4], R[1])
end

function ahrs:get_yaw_rad()
    return self:get_yaw()
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

-- set_output_pwm_chan_timeout(chan_0based, pwm, timeout_ms)
-- chan is 0-indexed in ArduPilot; store as 1-indexed for Lua table access.
function SRV_Channels:set_output_pwm_chan_timeout(chan, pwm, timeout_ms)
    _mock.srv_chan_out[chan + 1] = pwm
end

-- get_output_pwm(function_num) -> pwm | nil
-- Returns the last PWM written for a servo function, or nil when the function
-- has no output yet (matches the real binding, which returns nil/false when the
-- function is not available).  Tests inject the applied yaw-motor PWM via
-- _mock.srv_out[func] (harness: set_srv_out).
function SRV_Channels:get_output_pwm(func)
    return _mock.srv_out[func]
end

-- ── param ────────────────────────────────────────────────────────────────────

param = {}

function param:get(name)
    return _mock.params[name]
end

function param:set(name, value)
    _mock.params[name] = value
    return true
end

-- ── gcs ──────────────────────────────────────────────────────────────────────

gcs = {}

function gcs:send_text(level, msg)
    _mock.gcs_msgs[#_mock.gcs_msgs + 1] = {level = level, msg = msg}
end

function gcs:send_named_float(name, value)
    -- no-op stub; telemetry NVFs are not consumed in unit tests
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

function vehicle:set_target_angle_and_climbrate(roll_deg, pitch_deg, yaw_deg, climbrate, use_yaw_rate, yaw_rate_degs)
    _mock.guided_target = {
        roll_deg      = roll_deg,
        pitch_deg     = pitch_deg,
        yaw_deg       = yaw_deg,
        climbrate     = climbrate,
        use_yaw_rate  = use_yaw_rate,
        yaw_rate_degs = yaw_rate_degs,
    }
    _mock.guided_rate_target = nil
    _mock.guided_throttle = nil
    return true
end

function vehicle:set_target_angle_and_rate_and_throttle(roll_deg, pitch_deg, yaw_deg, roll_rate, pitch_rate, yaw_rate, throttle)
    _mock.guided_target = {
        roll_deg  = roll_deg,
        pitch_deg = pitch_deg,
        yaw_deg   = yaw_deg,
        climbrate = nil,
    }
    _mock.guided_rate_target = nil
    _mock.guided_throttle = throttle
    return true
end

function vehicle:set_target_rate_and_throttle(roll_rate, pitch_rate, yaw_rate, throttle)
    _mock.guided_target = nil
    _mock.guided_rate_target = {
        roll_rate  = roll_rate,
        pitch_rate = pitch_rate,
        yaw_rate   = yaw_rate,
    }
    _mock.guided_throttle = throttle
    return true
end

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
