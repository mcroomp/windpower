#!/usr/bin/env python3
"""
query_hardware.py -- Query Pixhawk 6C over MAVLink and write hardware.md.

Usage:
    RAWES_HIL_PORT=COM4 simulation/.venv/Scripts/python.exe simulation/scripts/query_hardware.py
"""
from __future__ import annotations
import datetime, math, os, struct, sys, threading, time
from pymavlink import mavutil

PORT = os.environ.get("RAWES_HIL_PORT", "COM4")
BAUD = int(os.environ.get("RAWES_HIL_BAUD", "115200"))
OUT  = os.path.join(os.path.dirname(__file__), "..", "..", "hardware", "hardware.md")

mav = mavutil.mavlink_connection(PORT, baud=BAUD, source_system=255)
mav.wait_heartbeat(timeout=15)
print(f"Connected sysid={mav.target_system}")

def _hb():
    for _ in range(120):
        try:
            mav.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        except Exception:
            break
        time.sleep(1.0)
threading.Thread(target=_hb, daemon=True).start()
mav.mav.request_data_stream_send(
    mav.target_system, mav.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
time.sleep(1.5)

def recv_param(name, timeout=8.0):
    deadline = time.monotonic() + timeout
    last = 0.0
    while time.monotonic() < deadline:
        if time.monotonic() - last > 2.0:
            mav.mav.param_request_read_send(
                mav.target_system, mav.target_component, name.encode(), -1)
            last = time.monotonic()
        msg = mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if msg and msg.param_id.rstrip("\x00") == name:
            return float(msg.param_value)
    return None

lines = []
def L(s=""): lines.append(s)

# ---------------------------------------------------------------------------
L(f"# RAWES Pixhawk 6C Hardware Profile")
L()
L(f"Captured: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M')}")
L(f"Port: {PORT}  Baud: {BAUD}")
L()

# --- Firmware ---
mav.mav.autopilot_version_request_send(mav.target_system, mav.target_component)
av = mav.recv_match(type="AUTOPILOT_VERSION", blocking=True, timeout=5.0)
L("## Firmware")
L()
if av:
    fwv   = av.flight_sw_version
    major = (fwv >> 24) & 0xFF
    minor = (fwv >> 16) & 0xFF
    patch = (fwv >>  8) & 0xFF
    ftype = (fwv      ) & 0xFF
    cap   = av.capabilities
    L("| Field | Value |")
    L("|-------|-------|")
    L(f"| Flight SW version | {major}.{minor}.{patch}-{ftype} |")
    L(f"| Middleware version | {av.middleware_sw_version} |")
    L(f"| OS version | {av.os_sw_version} |")
    L(f"| Board version | {av.board_version} |")
    L(f"| Capabilities bitmask | 0x{cap:08X} |")
    L(f"| MAVLink2 capable | {bool(cap & 2048)} |")
    L(f"| SET_ATTITUDE_TARGET capable | {bool(cap & 64)} |")
    uid = bytes(av.uid2[:18]).hex() if hasattr(av, "uid2") else "n/a"
    L(f"| Hardware UID | {uid} |")
else:
    L("_AUTOPILOT_VERSION not received_")
L()

# --- Heartbeat ---
hb_msg = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=3.0)
L("## System")
L()
if hb_msg:
    L("| Field | Value |")
    L("|-------|-------|")
    L(f"| Autopilot | {hb_msg.autopilot} (3=ArduPilot) |")
    L(f"| MAV type | {hb_msg.type} (4=Helicopter) |")
    L(f"| Base mode | 0x{hb_msg.base_mode:02X} |")
    L(f"| System status | {hb_msg.system_status} |")
    L(f"| MAVLink version | {hb_msg.mavlink_version} |")
L()

# --- Sensor health ---
ss = mav.recv_match(type="SYS_STATUS", blocking=True, timeout=3.0)
L("## Sensor Health")
L()
if ss:
    sensors = [
        (0x000001, "3D Gyro"),
        (0x000002, "3D Accel"),
        (0x000004, "3D Mag"),
        (0x000008, "Abs pressure"),
        (0x000010, "Diff pressure"),
        (0x000020, "GPS"),
        (0x000200, "Motor outputs"),
        (0x000400, "RC receiver"),
        (0x000800, "3D Gyro 2"),
        (0x001000, "3D Accel 2"),
        (0x002000, "3D Mag 2"),
        (0x100000, "Battery"),
    ]
    L("| Sensor | Present | Enabled | Healthy |")
    L("|--------|---------|---------|---------|")
    for bit, name in sensors:
        pres = bool(ss.onboard_control_sensors_present & bit)
        enab = bool(ss.onboard_control_sensors_enabled & bit)
        hlth = bool(ss.onboard_control_sensors_health  & bit)
        if pres:
            L(f"| {name} | yes | {'yes' if enab else 'no'} | {'**NO**' if not hlth else 'yes'} |")
    L()
    L(f"CPU load: {ss.load/10.0:.1f}%  "
      f"Battery voltage: {ss.voltage_battery/1000.0:.2f} V  "
      f"Current: {ss.current_battery/100.0:.2f} A  "
      f"Remaining: {ss.battery_remaining}%")
L()

# --- Power ---
pw = mav.recv_match(type="POWER_STATUS", blocking=True, timeout=3.0)
L("## Power Rails")
L()
if pw:
    L("| Rail | Voltage |")
    L("|------|---------|")
    L(f"| Vcc (5V) | {pw.Vcc/1000.0:.3f} V |")
    L(f"| Vservo | {pw.Vservo/1000.0:.3f} V |")
    flags = pw.flags
    L()
    L("| Flag | State |")
    L("|------|-------|")
    L(f"| USB connected | {bool(flags & 1)} |")
    L(f"| Brick OK | {bool(flags & 2)} |")
    L(f"| Servo power OK | {bool(flags & 4)} |")
L()

# --- Memory ---
mi = mav.recv_match(type="MEMINFO", blocking=True, timeout=3.0)
L("## Memory")
L()
if mi:
    L("| Field | Value |")
    L("|-------|-------|")
    L(f"| Free RAM | {mi.freemem} bytes |")
    if hasattr(mi, "freemem32"):
        L(f"| Free RAM (32-bit) | {mi.freemem32} bytes |")
L()

# --- MCU ---
mcu = mav.recv_match(type="MCU_STATUS", blocking=True, timeout=3.0)
L("## MCU")
L()
if mcu:
    L("| Field | Value |")
    L("|-------|-------|")
    L(f"| Temperature | {mcu.MCU_temperature/100.0:.1f} deg C |")
    L(f"| Voltage | {mcu.MCU_voltage/1000.0:.3f} V |")
    L(f"| Voltage min | {mcu.MCU_voltage_min/1000.0:.3f} V |")
    L(f"| Voltage max | {mcu.MCU_voltage_max/1000.0:.3f} V |")
L()

# --- GPS ---
gps = mav.recv_match(type="GPS_RAW_INT", blocking=True, timeout=3.0)
L("## GPS")
L()
if gps:
    fix_map = {0:"No GPS", 1:"No fix", 2:"2D fix", 3:"3D fix",
               4:"DGPS", 5:"RTK float", 6:"RTK fixed"}
    L("| Field | Value |")
    L("|-------|-------|")
    L(f"| Fix type | {fix_map.get(gps.fix_type, str(gps.fix_type))} |")
    L(f"| Satellites visible | {gps.satellites_visible} |")
    L(f"| HDOP | {gps.eph/100.0:.2f} |")
    L(f"| VDOP | {gps.epv/100.0:.2f} |")
    if gps.fix_type >= 3:
        L(f"| Latitude | {gps.lat/1e7:.6f} deg |")
        L(f"| Longitude | {gps.lon/1e7:.6f} deg |")
        L(f"| Altitude | {gps.alt/1000.0:.1f} m MSL |")
L()

# --- EKF ---
ekf = mav.recv_match(type="EKF_STATUS_REPORT", blocking=True, timeout=3.0)
L("## EKF Status")
L()
if ekf:
    L("| Field | Value |")
    L("|-------|-------|")
    L(f"| Flags | 0x{ekf.flags:04X} |")
    L(f"| Attitude initialised | {bool(ekf.flags & 0x0001)} |")
    L(f"| Velocity horiz valid | {bool(ekf.flags & 0x0002)} |")
    L(f"| Pos horiz rel valid | {bool(ekf.flags & 0x0004)} |")
    L(f"| Pos horiz abs valid | {bool(ekf.flags & 0x0008)} |")
    L(f"| Pos vert abs valid | {bool(ekf.flags & 0x0010)} |")
    L(f"| Velocity variance | {ekf.velocity_variance:.4f} |")
    L(f"| Pos horiz variance | {ekf.pos_horiz_variance:.4f} |")
    L(f"| Pos vert variance | {ekf.pos_vert_variance:.4f} |")
    L(f"| Compass variance | {ekf.compass_variance:.4f} |")
    L(f"| Terrain alt variance | {ekf.terrain_alt_variance:.4f} |")
L()

# --- Attitude ---
att = mav.recv_match(type="ATTITUDE", blocking=True, timeout=3.0)
L("## Attitude (bench, stationary)")
L()
if att:
    L("| Axis | Value |")
    L("|------|-------|")
    L(f"| Roll | {math.degrees(att.roll):.1f} deg |")
    L(f"| Pitch | {math.degrees(att.pitch):.1f} deg |")
    L(f"| Yaw | {math.degrees(att.yaw):.1f} deg |")
    L(f"| Roll rate | {math.degrees(att.rollspeed):.2f} deg/s |")
    L(f"| Pitch rate | {math.degrees(att.pitchspeed):.2f} deg/s |")
    L(f"| Yaw rate | {math.degrees(att.yawspeed):.2f} deg/s |")
L()

# --- IMU ---
imu = mav.recv_match(type="RAW_IMU", blocking=True, timeout=3.0)
L("## IMU (raw)")
L()
if imu:
    L("| Axis | Accel (mg) | Gyro (mrad/s) | Mag (mGauss) |")
    L("|------|-----------|--------------|-------------|")
    L(f"| X | {imu.xacc} | {imu.xgyro} | {imu.xmag} |")
    L(f"| Y | {imu.yacc} | {imu.ygyro} | {imu.ymag} |")
    L(f"| Z | {imu.zacc} | {imu.zgyro} | {imu.zmag} |")
L()

# --- Servo outputs ---
srv = mav.recv_match(type="SERVO_OUTPUT_RAW", blocking=True, timeout=3.0)
L("## Servo Outputs (current)")
L()
if srv:
    L("| Channel | PWM (us) |")
    L("|---------|----------|")
    for i in range(1, 17):
        attr = f"servo{i}_raw"
        val  = getattr(srv, attr, None)
        if val is not None and val != 0:
            L(f"| {i} | {val} |")
L()

# --- Parameters ---
L("## RAWES Parameters")
L()
param_groups = {
    "Frame": [
        "FRAME_CLASS", "FRAME_TYPE",
    ],
    "Swashplate": [
        "H_SW_TYPE", "H_SW_COL_DIR", "H_SW_H3_ENABLE", "H_SW_LIN_SVO",
    ],
    "RSC": [
        "H_RSC_MODE", "H_RSC_RUNUP_TIME", "H_RSC_SETPOINT",
        "H_RSC_RAMP_TIME", "H_RSC_CLDWN_TIME",
    ],
    "Tail / yaw motor": [
        "H_TAIL_TYPE", "H_COL2YAW",
        "ATC_RAT_YAW_P", "ATC_RAT_YAW_I", "ATC_RAT_YAW_D", "ATC_RAT_YAW_IMAX",
    ],
    "Roll/Pitch PID limits": [
        "ATC_RAT_RLL_IMAX", "ATC_RAT_PIT_IMAX",
    ],
    "ACRO": [
        "ACRO_TRAINER", "ACRO_RP_RATE", "INITIAL_MODE",
    ],
    "Failsafes": [
        "FS_THR_ENABLE", "FS_GCS_ENABLE", "FS_EKF_ACTION",
    ],
    "EKF / compass": [
        "EK3_SRC1_YAW", "EK3_MAG_CAL", "EK3_GPS_CHECK",
        "COMPASS_USE", "COMPASS_ENABLE",
    ],
    "DShot / BLHeli": [
        "SERVO_BLH_MASK", "SERVO_BLH_OTYPE", "SERVO_BLH_AUTO",
    ],
    "Servo functions": [
        "SERVO4_FUNCTION", "SERVO4_MIN", "SERVO4_MAX", "SERVO4_TRIM",
        "BRD_IO_DSHOT",
    ],
    "RPM sensor": [
        "RPM1_TYPE", "RPM1_MIN",
    ],
    "Scripting": [
        "SCR_ENABLE", "SCR_HEAP_SIZE",
        "SCR_USER1", "SCR_USER2", "SCR_USER3",
        "SCR_USER4", "SCR_USER5", "SCR_USER6",
    ],
    "Arming / safety": [
        "ARMING_SKIPCHK", "BRD_SAFETYENABLE",
    ],
}

for group, params in param_groups.items():
    L(f"### {group}")
    L()
    L("| Parameter | Value |")
    L("|-----------|-------|")
    for name in params:
        val = recv_param(name, timeout=5.0)
        L(f"| {name} | {val if val is not None else 'n/a'} |")
    L()

# --- Known firmware differences vs SITL ---
L("## Known Firmware Differences vs SITL")
L()
L("| Feature | SITL Docker | This Pixhawk |")
L("|---------|------------|-------------|")
L("| SCR_USER params | SCR_USER1..9 | SCR_USER1..6 only |")
L("| H_SW_PHANG | Exists (set to 0) | Does not exist; phase implicit in H_SW_TYPE |")
L("| RPM source in Lua | battery:voltage(0) (mediator hack) | rpm:get_rpm(0) (DSHOT telemetry) |")
L("| Mode selector param | SCR_USER6 | SCR_USER6 |")
L()

mav.close()

out_path = os.path.abspath(OUT)
os.makedirs(os.path.dirname(out_path), exist_ok=True)
with open(out_path, "w", encoding="utf-8") as f:
    f.write("\n".join(lines))
print(f"Written to {out_path}")
