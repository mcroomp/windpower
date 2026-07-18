"""rc_hold.py is intentionally disabled.

Policy: control must be GUIDED-only. The only allowed RC override path is
the Lua-managed CH8 interlock hold in rawes.lua.
"""

raise SystemExit(
    "rc_hold.py disabled by policy: use GUIDED setpoints; only rawes.lua may "
    "override CH8."
)
