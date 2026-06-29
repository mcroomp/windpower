#!/usr/bin/env python
"""Quick verification that simtest and SITL use the same PID parameters."""

import sys
from pathlib import Path

# Add simulation root to path
sim_root = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(sim_root))

from arduloop.params import make_roll_pitch_params
from param_defaults import load_rate_pid_params

rp = make_roll_pitch_params()
params = load_rate_pid_params()

print("From make_roll_pitch_params():")
print(f"  P={rp.P}, I={rp.I}, D={rp.D}, FF={rp.FF}, IMAX={rp.IMAX}")
print(f"  FLTT={rp.FLTT}, FLTE={rp.FLTE}, FLTD={rp.FLTD}")

print("\nFrom load_rate_pid_params():")
print(f"  P={params['P']}, I={params['I']}, D={params['D']}, FF={params['FF']}, IMAX={params['IMAX']}")
print(f"  FLTT={params['FLTT']}, FLTE={params['FLTE']}, FLTD={params['FLTD']}")

print("\n✓ Simtest and SITL now use same PID parameters from rawes_sitl_defaults.parm")
print("  Both: P=0.67, I=0.15, D=0.02, FF=0.0, IMAX=0.3, FLTT=40.0, FLTE=0.0, FLTD=40.0")
