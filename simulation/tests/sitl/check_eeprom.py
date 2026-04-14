#!/usr/bin/env python3
"""Scan EEPROM binary for SCR_ENABLE value."""
import struct

with open("/ardupilot/eeprom.bin", "rb") as f:
    data = f.read()

print(f"EEPROM size: {len(data)}")

found = False
for i in range(len(data) - 20):
    chunk = data[i:i+16]
    try:
        s = chunk.rstrip(b"\x00").decode("ascii")
        if "SCR_ENABLE" in s or "SCR_" in s:
            val_bytes = data[i+16:i+20]
            if len(val_bytes) == 4:
                val = struct.unpack("<f", val_bytes)[0]
                print(f"  offset {i:6d}: {s!r:20s} = {val}")
                found = True
    except Exception:
        pass

if not found:
    print("SCR_ENABLE not found in EEPROM raw scan")
    print("Scanning for any recognizable param names...")
    count = 0
    for i in range(len(data) - 20):
        chunk = data[i:i+16]
        try:
            s = chunk.rstrip(b"\x00").decode("ascii")
            if len(s) >= 4 and s.replace("_", "").replace(".", "").isalnum():
                val_bytes = data[i+16:i+20]
                if len(val_bytes) == 4:
                    val = struct.unpack("<f", val_bytes)[0]
                    if -1e6 < val < 1e6:
                        print(f"  offset {i:6d}: {s!r:20s} = {val}")
                        count += 1
                        if count > 20:
                            print("  ... (truncated)")
                            break
        except Exception:
            pass
