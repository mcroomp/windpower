"""
rc_hold.py — Hold RC override on a channel continuously.

Usage:
    python rc_hold.py [port] [channel] [pwm]

Defaults: COM4, channel 3, pwm 1500
Press Ctrl-C to stop (releases override).
"""

import sys
import time
from pymavlink import mavutil

port    = sys.argv[1] if len(sys.argv) > 1 else "COM4"
channel = int(sys.argv[2]) if len(sys.argv) > 2 else 3
pwm     = int(sys.argv[3]) if len(sys.argv) > 3 else 1500

print(f"Connecting to {port} ...")
mav = mavutil.mavlink_connection(port, baud=115200, source_system=255)
mav.wait_heartbeat(timeout=10)
print(f"Connected. Holding CH{channel}={pwm} us  (Ctrl-C to stop)")

channels = [0] * 8
channels[channel - 1] = pwm

try:
    while True:
        mav.mav.rc_channels_override_send(
            mav.target_system, mav.target_component,
            *channels,
        )
        time.sleep(0.05)
except KeyboardInterrupt:
    print("\nReleasing override ...")
    mav.mav.rc_channels_override_send(
        mav.target_system, mav.target_component,
        0, 0, 0, 0, 0, 0, 0, 0,
    )
    mav.close()
    print("Done.")
