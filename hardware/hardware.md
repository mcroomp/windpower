# RAWES Pixhawk 6C Hardware Profile

Captured: 2026-04-08 15:42
Port: COM4  Baud: 115200

## Firmware

| Field | Value |
|-------|-------|
| Flight SW version | 4.6.3-255 |
| Middleware version | 0 |
| OS version | 0 |
| Board version | 3670016 |
| Capabilities bitmask | 0x0000FBEF |
| MAVLink2 capable | True |
| SET_ATTITUDE_TARGET capable | True |
| Hardware UID | 25001c001651333337363133000000000000 |

## System

| Field | Value |
|-------|-------|
| Autopilot | 3 (3=ArduPilot) |
| MAV type | 4 (4=Helicopter) |
| Base mode | 0x51 |
| System status | 3 |
| MAVLink version | 3 |

## Sensor Health

| Sensor | Present | Enabled | Healthy |
|--------|---------|---------|---------|
| 3D Gyro | yes | yes | yes |
| 3D Accel | yes | yes | yes |
| 3D Mag | yes | yes | yes |
| Abs pressure | yes | yes | yes |
| RC receiver | yes | yes | yes |
| 3D Gyro 2 | yes | yes | yes |
| 3D Accel 2 | yes | yes | yes |
| 3D Mag 2 | yes | no | **NO** |
| Battery | yes | no | yes |

CPU load: 22.7%  Battery voltage: 15.80 V  Current: 0.44 A  Remaining: 97%

## Power Rails

| Rail | Voltage |
|------|---------|
| Vcc (5V) | 5.233 V |
| Vservo | 8.519 V |

| Flag | State |
|------|-------|
| USB connected | True |
| Brick OK | False |
| Servo power OK | True |

## Memory

| Field | Value |
|-------|-------|
| Free RAM | 65535 bytes |
| Free RAM (32-bit) | 370024 bytes |

## MCU

| Field | Value |
|-------|-------|
| Temperature | 35.2 deg C |
| Voltage | 3.329 V |
| Voltage min | 3.321 V |
| Voltage max | 3.335 V |

## GPS

| Field | Value |
|-------|-------|
| Fix type | No GPS |
| Satellites visible | 0 |
| HDOP | 655.35 |
| VDOP | 655.35 |

## EKF Status

| Field | Value |
|-------|-------|
| Flags | 0x00A7 |
| Attitude initialised | True |
| Velocity horiz valid | True |
| Pos horiz rel valid | True |
| Pos horiz abs valid | False |
| Pos vert abs valid | False |
| Velocity variance | 0.0000 |
| Pos horiz variance | 0.0373 |
| Pos vert variance | 0.2141 |
| Compass variance | 0.0459 |
| Terrain alt variance | 0.0000 |

## Attitude (bench, stationary)

| Axis | Value |
|------|-------|
| Roll | -47.8 deg |
| Pitch | 4.7 deg |
| Yaw | 119.5 deg |
| Roll rate | -0.04 deg/s |
| Pitch rate | 0.70 deg/s |
| Yaw rate | -0.21 deg/s |

## IMU (raw)

| Axis | Accel (mg) | Gyro (mrad/s) | Mag (mGauss) |
|------|-----------|--------------|-------------|
| X | 69 | -1 | -181 |
| Y | 750 | -5 | -363 |
| Z | -681 | 0 | -139 |

## Servo Outputs (current)

| Channel | PWM (us) |
|---------|----------|
| 1 | 1008 |
| 2 | 1314 |
| 3 | 1426 |
| 4 | 1465 |
| 8 | 1100 |
| 9 | 1050 |

## RAWES Parameters

### Frame

| Parameter | Value |
|-----------|-------|
| FRAME_CLASS | 6.0 |
| FRAME_TYPE | 1.0 |

### Swashplate

| Parameter | Value |
|-----------|-------|
| H_SW_TYPE | 3.0 |
| H_SW_COL_DIR | 0.0 |
| H_SW_H3_ENABLE | 0.0 |
| H_SW_LIN_SVO | 0.0 |

### RSC

| Parameter | Value |
|-----------|-------|
| H_RSC_MODE | 1.0 |
| H_RSC_RUNUP_TIME | 1.0 |
| H_RSC_SETPOINT | 70.0 |
| H_RSC_RAMP_TIME | 1.0 |
| H_RSC_CLDWN_TIME | 0.0 |

### Tail / yaw motor

| Parameter | Value |
|-----------|-------|
| H_TAIL_TYPE | 0.0 |
| H_COL2YAW | 0.0 |
| ATC_RAT_YAW_P | 0.0010000000474974513 |
| ATC_RAT_YAW_I | 0.0 |
| ATC_RAT_YAW_D | 0.0 |
| ATC_RAT_YAW_IMAX | 0.0 |

### Roll/Pitch PID limits

| Parameter | Value |
|-----------|-------|
| ATC_RAT_RLL_IMAX | 0.0 |
| ATC_RAT_PIT_IMAX | 0.0 |

### ACRO

| Parameter | Value |
|-----------|-------|
| ACRO_TRAINER | 0.0 |
| ACRO_RP_RATE | 360.0 |
| INITIAL_MODE | 1.0 |

### Failsafes

| Parameter | Value |
|-----------|-------|
| FS_THR_ENABLE | 0.0 |
| FS_GCS_ENABLE | 0.0 |
| FS_EKF_ACTION | 0.0 |

### EKF / compass

| Parameter | Value |
|-----------|-------|
| EK3_SRC1_YAW | 1.0 |
| EK3_MAG_CAL | 3.0 |
| EK3_GPS_CHECK | 31.0 |
| COMPASS_USE | 1.0 |
| COMPASS_ENABLE | 1.0 |

### DShot / BLHeli

| Parameter | Value |
|-----------|-------|
| SERVO_BLH_MASK | 256.0 |
| SERVO_BLH_OTYPE | 5.0 |
| SERVO_BLH_AUTO | 1.0 |

### Servo functions

| Parameter | Value |
|-----------|-------|
| SERVO9_FUNCTION | 94.0 |

### RPM sensor

| Parameter | Value |
|-----------|-------|
| RPM1_TYPE | 5.0 |
| RPM1_MIN | 0.0 |

### Scripting

| Parameter | Value |
|-----------|-------|
| SCR_ENABLE | 1.0 |
| SCR_HEAP_SIZE | 204800.0 |
| SCR_USER1 | 0.0 |
| SCR_USER2 | 0.0 |
| SCR_USER3 | 0.0 |
| SCR_USER4 | 0.0 |
| SCR_USER5 | 0.0 |
| SCR_USER6 | 2.0 |

### Arming / safety

| Parameter | Value |
|-----------|-------|
| ARMING_SKIPCHK | n/a |
| BRD_SAFETYENABLE | n/a |

## Known Firmware Differences vs SITL

| Feature | SITL Docker | This Pixhawk |
|---------|------------|-------------|
| SCR_USER params | SCR_USER1..9 | SCR_USER1..6 only |
| H_SW_PHANG | Exists (set to 0) | Does not exist; phase implicit in H_SW_TYPE |
| RPM source in Lua | battery:voltage(0) (mediator hack) | rpm:get_rpm(0) (DSHOT telemetry) |
| Mode selector param | SCR_USER7 | SCR_USER6 (rawes_hw.lua) |
