--[[
rawes_yaw_trim.lua — RAWES counter-torque motor feedforward controller

Runs on the Pixhawk 6C (hardware) or ArduPilot SITL via SCR_ENABLE=1.

SITL RPM channel
-----------------
Motor RPM is encoded in battery.voltage by mediator_torque.py:
  - mediator sends motor_rpm (~485) as the "voltage" value
  - voltage < MIN_RPM (50) means RPM data is unavailable (real 4S = 12.6V < 50)
  - Lua reads motor_rpm = battery:voltage(0) and ignores values < MIN_RPM

On hardware (Pixhawk 6C):
  - Use RPM:get_rpm(0) with RPM1_TYPE=5 (DSHOT ESC telemetry from AM32)
  - Remove the battery:voltage(0) hack and restore proper RPM reading

Control law: throttle = trim(RPM, V_bat) + Kp × psi_dot
--]]

local GEAR_RATIO = 80.0 / 44.0
local KV_RAD     = 66.0 * math.pi / 30.0
local R_MOTOR    = 7.5
local K_BEARING  = 0.005
local KP_YAW     = 0.001
local MIN_RPM    = 50    -- RPM below this = no data (real 4S is 12.6V < 50)
local OUT_CHAN   = 8     -- Ch9 (0-indexed)
local PERIOD_MS  = 10   -- 100 Hz
local V_BAT_NOM  = 15.2  -- nominal 4S LiPo voltage for trim computation

local function compute_trim(motor_rpm, v_bat)
    local omega_m    = motor_rpm * math.pi / 30.0
    local omega_a    = omega_m / GEAR_RATIO
    local omega_0    = KV_RAD * v_bat
    local tau_stall  = v_bat / (KV_RAD * R_MOTOR)
    local tau_needed = K_BEARING * omega_a / GEAR_RATIO
    return math.max(0.0, math.min(1.0, tau_needed / tau_stall + omega_m / omega_0))
end

local _diag = 0

local function update()
    -- Motor RPM via battery.voltage encoding (SITL) or RPM:get_rpm(0) (hardware)
    local motor_rpm = battery:voltage(0)  -- mediator sends motor_rpm as voltage

    _diag = _diag + 1
    if _diag % 500 == 1 then
        gcs:send_text(6, string.format(
            "RAWES yaw trim: motor_rpm=%.0f  diag=%d",
            motor_rpm or -1, _diag
        ))
    end

    if not motor_rpm or motor_rpm < MIN_RPM then return update, PERIOD_MS end

    local trim = compute_trim(motor_rpm, V_BAT_NOM)

    local yaw_corr = 0.0
    local gyro = ahrs:get_gyro()
    if gyro then yaw_corr = -KP_YAW * gyro:z() end

    local throttle = math.max(0.0, math.min(1.0, trim + yaw_corr))
    local pwm_us   = math.floor(1000.0 + throttle * 1000.0 + 0.5)

    if _diag % 500 == 1 then
        gcs:send_text(6, string.format(
            "RAWES ACTIVE: trim=%.3f  thr=%.3f  pwm=%d",
            trim, throttle, pwm_us
        ))
    end

    local ok, err = pcall(function()
        SRV_Channels:set_output_pwm(94, pwm_us)
    end)
    if not ok and _diag % 500 == 1 then
        gcs:send_text(3, "RAWES: servo err: " .. tostring(err))
    end

    return update, PERIOD_MS
end

gcs:send_text(6, string.format(
    "RAWES yaw trim: loaded  gear=%.3f  K_bearing=%.4f  Kp=%.4f",
    GEAR_RATIO, K_BEARING, KP_YAW
))
return update, PERIOD_MS
