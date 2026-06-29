# Aero Model Conventions

This document describes the **canonical interface** between the `dynbem` aero model and the rest of RAWES simulation. All frame conventions, sign rules, and physical meanings are defined here.

## Aero Model (dynbem)

The aero package lives at `c:\repos\aero\`. It implements blade-element-moment (BEM) aerodynamics with optional dynamic-inflow models (Pitt-Peters, Oye, JIT-compiled variants).

**Production model:** `quasi_static` BEM. Use for all simtests, stack-facing physics, IC replay, pumping, landing, and diagnostics unless explicitly comparing aero models.

---

## RotorInputs — What Goes In

`RotorInputs` is the **immutable input struct** passed to `compute_forces()`. All fields use NED world frame or body frame as noted.

```python
@dataclass(frozen=True)
class RotorInputs:
    collective_rad:     float        # collective pitch [rad]; -0.28 to +0.10 typical
    tilt_lon:           float        # longitudinal cyclic tilt [rad]; nose-down > 0
    tilt_lat:           float        # lateral cyclic tilt [rad]; roll-right > 0
    R_hub:              np.ndarray   # (3, 3) hub rotation: NED world → FRD body
    v_hub_world:        np.ndarray   # (3,) hub velocity in NED [m/s]
    wind_world:         np.ndarray   # (3,) ambient wind in NED [m/s]
    omega_rad_s:        float        # rotor spin angular velocity [rad/s]; CCW from above > 0
    rho_kg_m3:          float        # air density [kg/m³]
    t:                  float        # simulation time [s] (used for time-dependent effects if any)
```

### Frame Conventions

| Field | Frame | Convention | Notes |
|-------|-------|-----------|-------|
| `collective_rad` | — | scalar | Pitch angle of all blades equally; typically negative (trailing-edge down) |
| `tilt_lon` | body | scalar | Disk tilt around body-y axis: positive = nose-down (forward stick) |
| `tilt_lat` | body | scalar | Disk tilt around body-x axis: positive = right wing down (roll-right stick) |
| `R_hub` | NED → body | 3×3 rotation matrix | Columns are FRD body axes in NED frame. `R_hub[:, 2]` is hub-axis (down through disk). Must be orthonormal (`det=+1`). |
| `v_hub_world` | NED | (3,) vector | Hub velocity in NED frame [North, East, Down] |
| `wind_world` | NED | (3,) vector | Ambient wind vector in NED frame (direction wind comes FROM) |
| `omega_rad_s` | — | scalar magnitude | Rotor spin in rad/s. Always positive for CCW-from-above. Direction implicit: −body_z (up) by right-hand rule. |
| `rho_kg_m3` | — | scalar | Air density; typically 1.225 at sea level |

### Cyclic Tilt Sign Conventions

**Critical:** `tilt_lon` and `tilt_lat` describe **disk-normal tilts**, not body-frame pitch/roll commands.

- **`tilt_lon > 0`** ⇒ nose-down disk (forward stick, body_z tilts forward toward North)
  - In body frame: rotor hub axis tilts toward negative pitch direction
  - Result: produces nose-down aerodynamic moment (disk pulls nose down)
- **`tilt_lat > 0`** ⇒ roll-right disk (right-wing-down stick, body_z tilts rightward toward East)
  - In body frame: rotor hub axis tilts toward positive roll direction
  - Result: produces right-roll aerodynamic moment (disk rolls right)

See [design/flight_stack.md §3.3](flight_stack.md) for how `HeliCyclicController` maps body-rate errors to these tilts.

---

## RotorOutput — What Comes Out

`RotorOutput` is the **immutable output struct** from `compute_forces()`. All fields use NED world frame as noted.

```python
@dataclass(frozen=True)
class RotorOutput:
    F_world:            np.ndarray   # (3,) force vector in NED [N]
    M_orbital:          np.ndarray   # (3,) moment vector in NED [N·m]
    Q_spin:             float        # shaft torque [N·m]
    M_spin:             np.ndarray   # (3,) gyroscopic moment from rotor spin in NED [N·m]
```

### Frame Conventions & Physical Meaning

| Field | Frame | Convention | Notes |
|-------|-------|-----------|-------|
| `F_world` | NED | (3,) vector | Aerodynamic force in NED. Positive Z is downward, so upward thrust appears as **negative Z**. Total thrust `T = -F_world[2]` (upward). |
| `M_orbital` | NED | (3,) vector | **Orbital (tilt-inducing) moment** in NED frame. Produced by cyclic commands and wind/velocity asymmetries. See sign table below. |
| `Q_spin` | — | scalar | Torque needed to maintain spin against profile drag. Positive for lift-generating rotor; from `quasi_static` it balances drive torque against Q_profile. |
| `M_spin` | NED | (3,) vector | **Gyroscopic moment** from rotor spin `ω` and orbital motion. Computed as `R_hub @ [0, 0, -I_spin * omega] + damp_term`. Applied to hub dynamics to represent reaction moment of spinning rotor. |

### M_orbital Sign Conventions (NED Frame)

The **orbital moment** `M_orbital` is always returned in **NED world frame**, not body frame. To check signs relative to body, transform via `M_body = R_hub.T @ M_orbital`.

| Condition | M_orbital[0] (roll) | M_orbital[1] (pitch) | M_orbital[2] (yaw) | Physical meaning |
|-----------|---|---|---|---|
| `tilt_lat > 0` (right-wing-down disk) | **positive** | ≈ 0 | ≈ 0 | Right-roll moment (disk produces rolling motion) |
| `tilt_lat < 0` (left-wing-down disk) | **negative** | ≈ 0 | ≈ 0 | Left-roll moment |
| `tilt_lon > 0` (nose-down disk) | ≈ 0 | **negative** | ≈ 0 | Nose-down moment (disk pitches nose down) |
| `tilt_lon < 0` (nose-up disk) | ≈ 0 | **positive** | ≈ 0 | Nose-up moment |
| Wind from North, no cyclic | **positive** | ≈ 0 | ≈ 0 | Wind-driven roll moment (can be trimmed by `tilt_lat`) |

**Critical gotcha:** `M_orbital` is in **NED frame**, not body frame. At hover, NED ≈ body, but at tethered equilibrium (~65° tether tilt), they differ significantly.

### F_world Sign Conventions (NED Frame)

| Condition | F_world[2] | Interpretation |
|-----------|-----------|---|
| Hovering with positive collective | **negative** | Upward thrust (negative Z in NED) |
| Zero collective or descending | **positive** (small) or **zero** | Gravity component or drag |

**Thrust scalar:** `T = -F_world[2]` (always extract upward thrust as the negative of NED-Z component).

---

## Key Invariants

### 1. Frame Consistency

- **All input vectors (`v_hub_world`, `wind_world`, input to `compute_forces`) must be in NED frame.**
- **All output vectors (`F_world`, `M_orbital`, `M_spin`) are in NED frame.**
- **Body-frame transformation via `R_hub`:** To convert any NED vector to body: `v_body = R_hub.T @ v_ned`. Inverse: `v_ned = R_hub @ v_body`.

### 2. Rotor Spin Direction (US Convention)

- **Rotor spins CCW viewed from above.** This is non-negotiable — the entire stack (Lua, ArduPilot config, motor wiring, swashplate mixer) assumes this.
- **Input `omega_rad_s` is a positive scalar** — direction is implicit as −body_z (upward in body frame) by right-hand rule.
- **Gyroscopic moment** `M_spin` accounts for the coupling between this spin and orbital motion (disk tilt).

### 3. Cyclic Tilt Mapping

The relationship between cyclic tilts and aerodynamic moments:

```
tilt_lon -> M_orbital_y  (pitch moment)
tilt_lat -> M_orbital_x  (roll moment)
```

At hover (R_hub ≈ I), transforming to body frame gives:
```
M_body_x ≈ M_orbital_x  (roll moment unchanged)
M_body_y ≈ M_orbital_y  (pitch moment unchanged)
```

At tethered equilibrium (65° tether tilt), the transformation is non-trivial — **always use `R_hub.T @ M_orbital` to get body moments**.

### 4. Wind Convention

**Wind vector points in the direction wind comes FROM** (standard meteorology). Example:
- `wind_world = [0, 10, 0]` ⇒ wind from North (10 m/s, flows South)
- In body frame at heading ψ: `wind_body = R_hub.T @ wind_world`

The aero model internally computes relative wind as `V_rel = v_hub - wind_world` (hub velocity minus wind).

---

## Integration with Physics & Controller

### In `physics_core.py`

```python
inputs = RotorInputs(
    collective_rad=controller_out.collective,
    tilt_lon=controller_out.tilt_lon,
    tilt_lat=controller_out.tilt_lat,
    R_hub=hub_state["R"],
    v_hub_world=hub_state["vel"],
    wind_world=wind_vector,
    omega_rad_s=hub_state["omega_spin"],
    rho_kg_m3=1.225,
    t=t_sim,
)

aero_result, aero_state = aero.compute_forces(inputs, aero_state)

# In dynamics, forces are applied directly to hub:
F_net = aero_result.F_world + other_forces
M_net = aero_result.M_orbital + other_moments + aero_result.M_spin
```

### In Telemetry (`telemetry_csv.py`)

```python
M = np.asarray(aero_result.M_orbital, dtype=float)  # NED frame
# M[0] = roll moment, M[1] = pitch moment, M[2] = yaw moment
# Written as-is to CSV (NED frame)
```

**Note:** Telemetry stores `M_orbital` in NED frame directly. To interpret in body frame for analysis, apply the stored `R_hub` at that frame: `M_body = R_hub.T @ M_orbital`.

### In `controller.py` (via `compute_rate_cmd`)

The controller computes desired disk-axis (`body_z`) orientation from tether equilibrium, then maps to rate commands. The aero model then produces moments that drive the body toward this desired orientation.

**Key coupling:** Controller uses cyclic to tilt disk → aero produces moment → moment rotates hub → hub gyro feeds back to controller → closes loop.

---

## Testing & Validation

### Unit Test: `tests/test_cyclic.py` (in `c:\repos\aero\`)

Canonical test of cyclic moment signs at hover:

```python
def test_tilt_lat_positive_gives_roll_right_moment():
    """tilt_lat > 0 => M_orbital_x > 0 (roll right)."""
    res, _ = model.compute_forces(
        RotorInputs(tilt_lat=radians(2.0), ..., R_hub=eye(3), ...),
        state
    )
    assert res.M_orbital[0] > 1.0  # NED frame, positive roll moment
```

At hover (`R_hub = I`), NED frame moments equal body-frame moments, so this directly validates the sign convention.

### Regression Test: `simulation/tests/unit/test_cyclic_direction.py`

Validates cyclic derivatives at saved IC (tethered equilibrium):

```python
def test_cyclic_moment_derivative_signs_at_static_ic():
    # Compute dm/dlat via finite difference
    m_pos = aero.compute_forces(..., ic, tilt_lat=+eps, ...).M_orbital
    m_neg = aero.compute_forces(..., ic, tilt_lat=-eps, ...).M_orbital
    dm_dlat = (m_pos - m_neg) / (2*eps)
    
    # Transform to body frame for sign check
    dm_dlat_body = R_hub.T @ dm_dlat
    assert dm_dlat_body[0] > 100.0  # roll moment derivative
```

**Critical:** Must transform NED moments to body frame via `R_hub.T` before checking signs at non-hover attitudes.

---

## Common Mistakes

1. **Forgetting to transform `M_orbital` to body frame at non-hover attitudes.**
   - At hover: NED ≈ body, mistake hidden.
   - At tethered hover: 65° tether tilt, transforms are large, mistake causes wrong sign checks.
   - **Fix:** Always use `M_body = R_hub.T @ M_orbital` when validating sign conventions.

2. **Confusing `omega_rad_s` direction.**
   - Input is a positive scalar (magnitude only).
   - Direction is implicit: −body_z (up) by right-hand rule (CCW from above).
   - **Fix:** Never negate `omega_rad_s`; if spin direction is wrong, the bug is upstream (motor wiring, harness, physical setup).

3. **Wind vector sign confusion.**
   - `wind_world = [0, 10, 0]` means wind FROM North (flows South).
   - Relative wind to hub: `V_rel = v_hub - wind_world`.
   - **Fix:** Always document wind as "FROM" direction; aero model expects this convention.

4. **Not ensuring `R_hub` is orthonormal.**
   - Euler integration or serialization roundtrips can introduce small errors.
   - Aero model assumes `det(R_hub) ≈ +1` and `R_hub @ R_hub.T ≈ I`.
   - **Fix:** Normalize before passing: `R_hub = scipy.spatial.Rotation.from_matrix(R_hub).as_matrix()`.

---

## Links

- [c:\repos\aero\dynbem\README.md](file:///c:/repos/aero/dynbem/README.md) — dynbem package reference
- [c:\repos\aero\tests\test_cyclic.py](file:///c:/repos/aero/tests/test_cyclic.py) — Canonical cyclic sign tests
- [design/simulation.md § Aero](#) — Integration in simulation loop
- [design/flight_stack.md § Controller](#) — How controller drives cyclic

