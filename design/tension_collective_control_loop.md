# Tension / Collective Control Loop

**Reference:** De Schutter J., Leuthold R., Diehl M. (2018).
"Optimal Control of a Rigid-Wing Rotary Kite System for Airborne Wind Energy."
IFAC Proceedings, Vol. 51, No. 13, pp. 523-528.

---

## Problem

Given:
- Hub position $\vec{r}$ (NED) → determines tether direction and elevation angle $\theta_{\text{el}}$
- Commanded tether tension $T_{\text{cmd}}$ [N], received from the winch
- Vehicle mass $m$ [kg]

Find: disk tilt angle $\xi$ (the angle of the *disk normal* relative to the tether) and disk normal vector $\vec{b}_z$ that hold steady altitude.

**Assumptions:** quasi-static equilibrium (no translational acceleration), aerodynamic drag / in-plane H-force neglected. The result is an **equilibrium (trim) map**. Using a *static* map is justified by timescale separation: the inner attitude loop settles much faster than the operating point (tension, commanded elevation) drifts, so the kite is always near the instantaneous equilibrium. *Control consequence of 0-drag assumption:* Because real-world drag pushes the kite downwind, the zero-drag map systematically under-commands forward tilt. The vehicle compensates by settling at a slightly lower equilibrium elevation ($\theta_{\text{el}}$) than the pure kinematic model predicts.

---

## Solution

### Force Balance

Three distinct quantities govern the map:
- **Rotor thrust** $T_R$: The kite's aerodynamic force along the disk normal.
- **Actual tension** $T_t$: The true mechanical load (measured by the winch load cell).
- **Commanded tension** $T_{\text{cmd}}$: The reference value sent over MAVLink. The kite computes its tilt using $T_{\text{cmd}}$, assuming the winch will track it ($T_t \to T_{\text{cmd}}$).

Resolve the steady-state force balance along and perpendicular to the tether (in the vertical plane). With gravity $m g$ down and elevation angle $\theta_{\text{el}}$:

- **Perpendicular to tether:** $\;T_R \sin(\xi) = m g \cos(\theta_{\text{el}})$
- **Along tether:** $\;T_R \cos(\xi) = T_t + m g \sin(\theta_{\text{el}})$

Dividing eliminates the (unmeasured) rotor thrust and gives the equilibrium tilt angle in terms of tether tension:

$$\boxed{\;\tan(\xi) = \frac{m g \cos(\theta_{\text{el}})}{T_t + m g \sin(\theta_{\text{el}})}\;}$$

The kite evaluates this at the **commanded** tension $T_{\text{cmd}}$ (substitute $T_{\text{cmd}}$ for $T_t$); the substitution is exact whenever the winch has driven actual tension to its command.

### Implementation Approximation

When $T_{\text{cmd}} \gg m g \sin(\theta_{\text{el}})$ (high tension), the gravity-along-tether term is dropped:

$$\tan(\xi) \approx \frac{m g \cos(\theta_{\text{el}})}{T_{\text{cmd}}}$$

This is the form built by [`compute_bz_altitude_hold()`](../simulation/controller.py#L671) (a `tdir + (g·cosθ/T)·e_perp` construction — a **tangent**, not a sine). At the design point the dropped term is $m g \sin 60° \approx 21$ N, ~7% of 300 N.

**Key insight:** as $T_{\text{cmd}}$ increases, $\xi$ decreases (disk stays closer to the tether); as $T_{\text{cmd}}$ decreases, $\xi$ must increase. The $1/T$ dependence means the angle command is highly sensitive at low tension — see [Robustness](#robustness-and-stability).

---

## Disk Normal Vector

From hub position $\vec{r} = [r_N, r_E, r_D]$ (NED), compute:

**Elevation angle** (hub is above the anchor, so $r_D < 0$ in NED):
$$\sin(\theta_{\text{el}}) = \frac{-r_D}{|\vec{r}|}$$

**Tether direction** (hub to anchor at origin):
$$\hat{\vec{t}} = -\frac{\vec{r}}{|\vec{r}|} = \begin{bmatrix} -\cos(\theta_{\text{el}}) \cos(\alpha) \\ -\cos(\theta_{\text{el}}) \sin(\alpha) \\ \sin(\theta_{\text{el}}) \end{bmatrix}$$

where $\alpha = \arctan2(r_E, r_N)$ is azimuth.

**Elevation-perpendicular direction** (in vertical plane):
$$\hat{\vec{e}}_{\perp} = \begin{bmatrix} \sin(\theta_{\text{el}}) \cos(\alpha) \\ \sin(\theta_{\text{el}}) \sin(\alpha) \\ \cos(\theta_{\text{el}}) \end{bmatrix}$$

**Disk normal** (tangent construction, gravity-along-tether dropped):
$$\vec{b}_z = \frac{\hat{\vec{t}} + \frac{mg\cos(\theta_{\text{el}})}{T_{\text{cmd}}}\, \hat{\vec{e}}_{\perp}}{\left\|\,\hat{\vec{t}} + \frac{mg\cos(\theta_{\text{el}})}{T_{\text{cmd}}}\, \hat{\vec{e}}_{\perp}\right\|}$$

---

## Example: Design Operating Point

- $m = 2.5$ kg, $g = 9.81$ m/s², $\theta_{\text{el}} = 60°$, $T_{\text{cmd}} = 300$ N

Exact (with gravity-along-tether term):
$$\xi = \arctan\left(\frac{12.26}{300 + 21.2}\right) \approx 2.2°$$

Approximate (implementation form):
$$\xi = \arctan\left(\frac{12.26}{300}\right) \approx 2.3°$$

Low tilt $\xi$ → efficient cyclic trim. The two forms agree to ~0.1° at high tension.

---

## Control Architecture

The angle map is an **equilibrium (trim) map**, not pure feedforward: it is *feedforward in the elevation command* (driven by the target altitude) and *feedforward in the commanded tension* $T_{\text{cmd}}$, but *feedback in the measured azimuth* (from position). The kite does **not** close any loop on tension — it consumes the commanded $T_{\text{cmd}}$ as a clean reference. The tension feedback lives entirely at the winch (below).

### Ground Control: Winch Realizes the Commanded Tension

The commanded tension $T_{\text{cmd}}$ is the shared setpoint: it is sent to the kite (to pick the angle) and is the winch's own target. The winch **modulates reel speed** $v_{\text{reel}}$ to drive *actual* tension $T_t$ toward $T_{\text{cmd}}$, closing a tension loop on its load cell:

$$v_{\text{reel}} = \Pi\big(T_{\text{cmd}} - T_t\big) \quad\text{(reel out to shed tension, reel in to build it)}$$

This is the single source of tension feedback in the system. The kite is fully insulated from load-cell noise because it only ever sees the commanded value.

### Onboard Control: Disk Orientation (feedforward)

The kite's autopilot runs **rawes.lua at 50 Hz**. Lua computes the disk normal `bz_goal` from measured position and the *commanded* tension (`bz_altitude_hold`), converts it to an absolute roll/pitch target (`bz_ned_to_roll_pitch`), and commands it through the GUIDED **set-angle** API:

```
vehicle:set_target_angle_and_rate_and_throttle(roll, pitch, yaw, 0, 0, 0, col_thrust)
```

Lua only *decides* the target angle. **ArduPilot's native attitude controller** (`ATC_ANG_*_P` outer loop + rate PIDs, 400 Hz) actually *holds* that angle against disturbances — Lua passes zero feedforward rate. This replaced an earlier rate-only cascade that diverged at the high-tilt / low-tension reel-in point.

### Altitude Hold Loop (feedback, in Lua)

The altitude controller also lives in **rawes.lua**, separate from the attitude path. It is a PID on altitude error whose output is **collective pitch** (sent as the `col_thrust` argument above):

$$\text{col} = \text{col}_{\text{trim}} + K_p\,e_{\text{alt}} + K_i\!\int e_{\text{alt}}\,dt - K_d\, v_{z}$$

- $\text{col}_{\text{trim}}$ — equilibrium collective feedforward (anchors the loop so it does not rely on integral action for the gravity disturbance).
- Integral state is clamped so that $\text{col}_{\text{trim}} + K_i\!\int e$ stays within $[\text{col}_{\min}, \text{col}_{\max}]$ (anti-windup against collective saturation).
- Vertical-velocity damping $K_d v_z$ is derivative-on-measurement (rate feedback, no setpoint-derivative kick) and is **gain-scheduled** down while body rates are high (`vz_gate`), so attitude transients do not inject collective noise.
- Output is slew-rate limited (`COL_SLEW_MAX`) before mapping to throttle.

### System Coupling & Robustness

$T_{\text{cmd}}$ is the shared variable tying the system together: the ground winch drives actual tension $T_t \to T_{\text{cmd}}$ via reel speed, while the kite assumes $T_t = T_{\text{cmd}}$ to calculate disk tilt and trims collective to hold altitude. This creates specific stability constraints:

- **Positive-Feedback Altitude Path:** $\text{Altitude} \to \theta_{\text{el}} \to \text{tilt} \to \text{thrust} \to \text{altitude}$ is regenerative at high bounds. The deployed code breaks this algebraic loop by filtering the elevation input (`_el_rad` slewed at 0.40 rad/s). *Consequence:* The angle map is sluggish against sudden downdrafts, forcing the fast **collective PID** to handle all immediate transient altitude rejection.
- **Bandwidth Separation:** To prevent limit cycles, the system enforces a strict hierarchy: $\omega_{\text{schedule}} \ll \omega_{\text{winch}} < \omega_{\text{attitude}}$. The winch must be fast enough to track schedule changes without lag, but slower than the kite's aerodynamic attitude loop so it doesn't "chase" rapid transients.
- **Tracking Error & Singularity:** Winch tracking lag ($T_t \neq T_{\text{cmd}}$) directly injects a disk-angle bias into the kite. Furthermore, tilt sensitivity spikes at low tension ($1/T$ singularity), requiring the code to clamp tension at 1.0 N to prevent extreme tilt commands.

---

## Proposed Enhancement: Tension Schedule (Shared S-Curve Reference)

Instead of broadcasting an *instantaneous* tension command, the winch sends a **tension schedule** — a segment $(T_1, \Delta t)$ giving the new commanded tension and the time over which to reach it. Both the winch and the kite evaluate the *same* smooth $T_{\text{cmd}}(\tau)$ profile against a synchronized clock: the kite re-aims its disk along the profile, while the winch drives actual tension along the same profile via reel speed.

This is a **2-DOF structure**: the shared schedule is the *feedforward reference* that both ends track, and the winch's reel-speed tension loop (load-cell feedback) is what makes *actual* tension realize that reference (see [Robustness](#robustness-and-stability)). The kite stays feedforward-only on tension.

### S-Curve Implementation

A bare linear tension ramp creates a step in $\dot{T}_{\text{cmd}}$, causing a jerk in the angle setpoint and cyclic torque. To prevent this, tensions should be interpolated using a **quintic smoothstep** (S-curve), which provides $C^2$ continuity (zero rate and acceleration at endpoints):

$$T_{\text{cmd}}(\tau) = T_0 + (T_1 - T_0)\, s\left(\frac{\tau}{\Delta t}\right), \quad s(u) = 6u^5 - 15u^4 + 10u^3$$

The peak transition rate is mathematically bounded to $\dot{T}_{\text{cmd},\max} = 1.875\, \frac{T_1 - T_0}{\Delta t}$.

**Design Integration Limits:**
- **Synchronization:** The winch and kite must share MAVLink time. If a new schedule arrives mid-transit, it must start from the *current* interpolated $T_{\text{cmd}}$ to avoid a jump.
- **Actuator Limits:** The ground planner must pick a $\Delta t$ feasible for the winch motor. Tracking lag defeats the feedforward coordination.
- **Rate Authority:** The schedule should own the transition rate. Local kite slews (`_el_rad`, `COL_SLEW_MAX`) become safety saturations only, avoiding double-filtering.
- **Discrete Sampling:** Though 50 Hz/10 Hz loops slightly violate absolute $C^2$ precision, the quintic polynomial successfully eliminates the 1-tick jerk.
