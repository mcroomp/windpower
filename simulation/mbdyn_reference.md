# MBDyn Integration Reference

This document records everything about the MBDyn co-simulation that was part of the
RAWES simulation stack. MBDyn has been removed from the runtime loop and replaced by
a pure-Python RK4 6-DOF integrator (`dynamics.py`). This document exists so the
integration can be restored when the simulation needs more physical fidelity than a
single rigid body (e.g., separate spinning hub + non-rotating electronics, flexible
tether, explicit blade bodies).

---

## What MBDyn Is

MBDyn is a general-purpose free multibody dynamics solver from Politecnico di Milano.
It integrates rigid and flexible multibody systems described by a declarative input
file (`.mbd`). The time integration method used here is BDF-2 / Adams-Moulton with
configurable spectral radius damping.

- Source: `public.gitlab.polimi.it/DAER/mbdyn`
- Language: C++, runs as a standalone Linux/macOS process
- Input: `.mbd` file + optional `.set` include file
- Output: binary streams via UNIX domain sockets (in our setup)
- Version required: 1.7.x+

MBDyn was chosen because it provides a validated rigid-body integrator with gravity,
convergence iteration, and flexible external-force injection, without us implementing
Newton-Euler from scratch. The RAWES hub is a 6-DOF rigid body; MBDyn handled the
equations of motion, timestep, and re-orthogonalisation internally.

---

## Why It Was Removed

MBDyn requires a running Linux process and UNIX domain sockets, which are unavailable
on Windows. This forced the simulation into Docker even for simple unit-level runs.
The physics MBDyn performed — a single 6-DOF rigid body with gravity — is a
straightforward RK4 integration. Removing MBDyn makes the full stack run natively on
Windows, eliminates the Docker dependency for local development, and removes process
synchronisation complexity. `dynamics.py` replicates the same equations with SVD
re-orthogonalisation and produces an identical state dict.

When to restore MBDyn:
- Splitting the model into a spinning hub body + non-rotating electronics body
  (the anti-rotation motor coupling becomes an inter-body force)
- Adding explicit blade bodies with flapping / lag DOFs
- Adding flexible tether with distributed mass and drag
- Needing MBDyn's validated DAE solver for kinematic constraints

---

## Socket Protocol

Two UNIX domain stream sockets (AF_UNIX, SOCK_STREAM). MBDyn creates both sockets
(`create, yes`); Python connects as client.

### Forces socket  `/tmp/rawes_forces.sock`

Python → MBDyn. Each simulation step, Python sends exactly **6 × float64 = 48 bytes**,
native endian, no framing, no header.

| Byte offset | Channel | Description |
|-------------|---------|-------------|
| 0–7         | 1       | Fx  East  [N] |
| 8–15        | 2       | Fy  North [N] |
| 16–23       | 3       | Fz  Up    [N] |
| 24–31       | 4       | Mx  [N·m] |
| 32–39       | 5       | My  [N·m] |
| 40–47       | 6       | Mz  [N·m] |

All values in the ENU world frame. This is the combined aerodynamic + tether wrench.
Gravity is NOT included — MBDyn applies gravity internally via a `gravity:` element.

Initial value (sent once before SITL connects, to prevent free-fall):
`[0.0, 0.0, 49.05, 0.0, 0.0, 0.0]` — equal and opposite to weight.

### State socket  `/tmp/rawes_state.sock`

MBDyn → Python. Each simulation step, MBDyn sends exactly **18 × float64 = 144 bytes**,
native endian, no framing.

**IMPORTANT: The comment in `rotor.mbd` lines 178–181 lists the order as
`[pos, vel, R, omega]`. This is WRONG. The actual MBDyn `motion` stream output order
is `[pos, R, vel, omega]`. Trust the Python unpack in `mbdyn_interface.py`, not the
`.mbd` comment.**

| Float64 index | Content |
|---------------|---------|
| 0–2           | pos: px, py, pz  [m] ENU |
| 3–11          | R: rotation matrix body→world, row-major (R00 R01 R02 R10 R11 R12 R20 R21 R22) |
| 12–14         | vel: vx, vy, vz  [m/s] ENU |
| 15–17         | omega: ωx, ωy, ωz  [rad/s] **world frame** |

Note: `omega` is in the **world frame**, not the body frame. `sensor.py` rotates it to
the body frame before packaging gyro data (`omega_body = R.T @ omega_world`).

### Synchronisation

Strictly lock-step per simulation step:
1. Python sends 48-byte force packet
2. MBDyn integrates one timestep
3. MBDyn sends 144-byte state packet
4. Python reads state

There is no buffering or pipelining. If Python does not read, MBDyn blocks.

---

## MBDyn Model Structure

All model entities are defined in `simulation/mbdyn/rotor.mbd` with parameters from
`simulation/mbdyn/rawes.set`.

### Nodes

| Label | ID | Type | Description |
|-------|----|------|-------------|
| HUB_NODE | 1 | dynamic | Hub rigid body, starts at (0, 0, 50 m), initial spin 28 rad/s about Z |
| GROUND_NODE | 2 | static | World origin, never moves |

### Bodies

| Label | ID | Attached to | Mass | Inertia |
|-------|----|-------------|------|---------|
| HUB_BODY | 1 | HUB_NODE | 5.0 kg | diag(5, 5, 10) kg·m², CoM at node origin |

### Joints

| Label | ID | Type | Description |
|-------|----|------|-------------|
| GROUND_CLAMP | 1 | clamp | Fixes GROUND_NODE to inertial frame |

The tether rod joint that previously existed here was removed. The tether is a
tension-only element computed in Python (`TetherModel` in `mediator.py`) and injected
as part of the external force.

### Force Elements

| Label | ID | Type | Channels |
|-------|----|------|---------|
| FORCE_ELEM | 1 | absolute force on HUB_NODE | driver channels 1, 2, 3 → Fx, Fy, Fz |
| COUPLE_ELEM | 2 | absolute couple on HUB_NODE | driver channels 4, 5, 6 → Mx, My, Mz |

### File Driver (incoming forces)

| Label | ID | Socket | Channels |
|-------|----|--------|---------|
| FORCES_DRIVER | 1 | `/tmp/rawes_forces.sock` (stream) | 6: Fx Fy Fz Mx My Mz |

Initial values: `[0.0, 0.0, WEIGHT, 0.0, 0.0, 0.0]` where `WEIGHT = M_ROTOR * G = 49.05 N`.

### Output Element (outgoing state)

| Label | ID | Socket |
|-------|----|--------|
| STATE_OUTPUT | 1 | `/tmp/rawes_state.sock` (stream) |

Output flags: `position, orientation matrix, velocity, angular velocity` for HUB_NODE.

### Gravity

`gravity: uniform, 0.0, 0.0, -1.0, const, 9.81` — applied to all dynamic nodes.

### Integration Settings

| Setting | Value |
|---------|-------|
| Method | `ms, 0.6` (BDF-2 / Adams-Moulton, spectral radius 0.6) |
| Timestep | 2.5e-3 s (400 Hz) |
| Tolerance | 1e-6 |
| Max iterations | 20 |
| Derivatives tolerance | 1e-4 |
| Linear solver | `naive, colamd` |

---

## Parameter Values (`rawes.set`)

```
DT          = 2.5e-3   s     (400 Hz)
T_FINAL     = 300.0    s
G           = 9.81     m/s²
RHO         = 1.22     kg/m³
X0, Y0      = 0.0, 0.0 m
Z0          = 50.0     m
OMEGA0      = 28.0     rad/s
N_BLADES    = 4
BLADE_LEN   = 2.0      m
R_ROOT      = 0.5      m
R_TIP       = 2.5      m
CHORD       = 0.15     m
DISK_AREA   = 19.635   m²
M_ROTOR     = 5.0      kg
Ixx = Iyy   = 5.0      kg·m²
Izz         = 10.0     kg·m²
WEIGHT      = 49.05    N
```

Aerodynamic and swashplate parameters are also in `rawes.set` but are used by
`aero.py` directly in Python, not by MBDyn.

---

## How to Restore the MBDyn Integration

### Prerequisites

1. Install MBDyn 1.7.x on Linux (or WSL2):
   ```bash
   git clone https://public.gitlab.polimi.it/DAER/mbdyn.git
   cd mbdyn && ./configure && make && sudo make install
   ```

2. Confirm UNIX socket support is compiled in (default).

### Code changes in `mediator.py`

1. Add import: `from mbdyn_interface import MBDynInterface`
2. Remove import: `from dynamics import RigidBodyDynamics`
3. Restore CLI args:
   ```python
   parser.add_argument("--mbdyn-force-sock", default="/tmp/rawes_forces.sock")
   parser.add_argument("--mbdyn-state-sock",  default="/tmp/rawes_state.sock")
   ```
4. Replace `RigidBodyDynamics(...)` instantiation with:
   ```python
   mbdyn = MBDynInterface(
       force_sock_path=args.mbdyn_force_sock,
       state_sock_path=args.mbdyn_state_sock,
   )
   ```
5. After `sitl.bind()`, add:
   ```python
   mbdyn.connect()
   mbdyn.send_forces(np.array([0.0, 0.0, 49.05, 0.0, 0.0, 0.0]))
   try:
       hub_state = mbdyn.recv_state()
   except Exception:
       pass
   ```
6. In the main loop, replace `dynamics.step(forces[0:3], forces[3:6], DT_TARGET)` with:
   ```python
   mbdyn.send_forces(forces)
   hub_state = mbdyn.recv_state()
   ```
7. In the finally block, add:
   ```python
   try:
       mbdyn.close()
   except Exception:
       pass
   ```
8. **Remove gravity from `dynamics.py`** — MBDyn applies gravity internally.
   Or: keep `dynamics.py` but subtract gravity from forces before calling MBDyn
   (since MBDyn adds it back). The cleanest approach is to ensure the external force
   packet sent to MBDyn never includes gravity.

### Running

In one terminal (Linux/WSL2):
```bash
cd simulation/mbdyn
mbdyn -f rotor.mbd
```

In another terminal:
```bash
python3 mediator.py
```

MBDyn waits for Python to connect to the forces socket before advancing time.

---

## What Was Not Modeled in MBDyn

The MBDyn deck was intentionally kept minimal. The following were absent and remain
absent from the Python `dynamics.py` replacement:

- Separate spinning hub body vs. non-rotating electronics body
- Blade bodies with flapping, lag, or pitch DOFs
- Explicit swashplate kinematic linkage
- Distributed tether mass or drag (tether is a Python spring force)
- Wake or induced-flow dynamics
- Gyroscopic coupling between blades and hub

When any of the above is needed, MBDyn is the right tool to restore.
