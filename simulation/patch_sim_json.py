#!/usr/bin/env python3
"""
Patch ArduPilot SIM_JSON.h and SIM_JSON.cpp to expose a 'rpm' JSON field.

The JSON physics backend ignores the 'rpm' field from external physics
providers (like mediator_torque.py).  This patch adds:
  1. A `float rpm_input` field to the JSON::state struct
  2. A keytable entry so the JSON parser reads it
  3. A DataKey bitmask enum value
  4. Copy logic in SIM_JSON.cpp to transfer state.rpm_input → rpm[0]

After patching, waf will recompile only the affected files (~10 s on a
cached build).  No ArduPilot full rebuild needed.
"""
import sys, re

# ── SIM_JSON.h ────────────────────────────────────────────────────────────────

h_path = "/ardupilot/libraries/SITL/SIM_JSON.h"
with open(h_path) as f:
    h = f.read()

if "rpm_input" in h:
    print("SIM_JSON.h already patched — skipping")
else:
    # 1. Add float rpm_input to state struct (after bat_amp)
    h = h.replace(
        "        float bat_amp;\n",
        "        float bat_amp;\n        float rpm_input;\n",
        1
    )

    # 2. Increase keytable size by 1
    old_size = re.search(r'keytable\[(\d+)\]', h)
    if old_size:
        n = int(old_size.group(1))
        h = h.replace(f"keytable[{n}]", f"keytable[{n+1}]", 1)
        print(f"keytable size {n} → {n+1}")

    # 3. Add keytable entry for rpm (after battery current entry)
    old_entry = '        { "battery", "current", &state.bat_amp, DATA_FLOAT, false },'
    new_entry = (old_entry + "\n"
                 + '        {"", "rpm", &state.rpm_input, DATA_FLOAT, false},')
    h = h.replace(old_entry, new_entry, 1)

    # 4. Add RPM_INPUT DataKey enum value (after BAT_AMP)
    h = re.sub(
        r'(BAT_AMP\s*=\s*0x[0-9a-fA-FUL]+)',
        r'\1, RPM_INPUT = 0x0000001000000000ULL',
        h, count=1
    )

    with open(h_path, "w") as f:
        f.write(h)
    print("Patched", h_path)

# ── SIM_JSON.cpp ──────────────────────────────────────────────────────────────

cpp_path = "/ardupilot/libraries/SITL/SIM_JSON.cpp"
with open(cpp_path) as f:
    cpp = f.read()

if "rpm_input" in cpp:
    print("SIM_JSON.cpp already patched — skipping")
else:
    # Insert after battery_current copy block
    old_block = "    if ((received_bitmask & BAT_AMP) != 0) {\n        battery_current = state.bat_amp; \n    }"
    new_block  = (old_block
                  + "\n    // Copy rpm from JSON → SITL motor state (for RPM1_TYPE=10 Lua scripting)\n"
                  + "    if ((received_bitmask & RPM_INPUT) != 0) {\n"
                  + "        sitl->state.rpm[0] = state.rpm_input;\n"
                  + "    } else if (state.rpm_input > 0) {\n"
                  + "        // rpm_input parsed but bitmask not set — force it\n"
                  + "        sitl->state.rpm[0] = state.rpm_input;\n"
                  + "    }\n")
    if old_block in cpp:
        cpp = cpp.replace(old_block, new_block, 1)
        with open(cpp_path, "w") as f:
            f.write(cpp)
        print("Patched", cpp_path)
    else:
        print("WARNING: bat_amp block not found in SIM_JSON.cpp — rpm copy not inserted")

print("Done")
