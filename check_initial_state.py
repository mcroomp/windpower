import json
import numpy as np

# Load the current initial state
with open('simulation/steady_state_starting.json') as f:
    state = json.load(f)

pos = np.array(state['pos'])
dist_to_anchor = np.linalg.norm(pos)
print(f"Initial position: {pos}")
print(f"Distance to anchor: {dist_to_anchor:.2f} m")
print(f"Current rest_length: {state['rest_length']:.2f} m")
print(f"Initial extension: {dist_to_anchor - state['rest_length']:.3f} m")
print()
print("Problem: the body is at 100.27m from anchor but rest_length is 99.89m (0.38m extension)")
print("During kinematic, body stays at this deep position (-42.72m NED depth)")
print("At kinematic exit, rotor RPM ramps down and can't generate enough lift")
print("Body descends, tether stretches, tension builds to 6484N")
print()
print("Solution: increase initial collective or move body to shallower depth")
