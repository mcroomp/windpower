# Servo Flaps — Design Reference

## What Is a Servo Flap?

A servo flap is a small airfoil mounted on the trailing edge of a rotor blade at approximately 75% span. Rather than mechanically changing blade pitch at the hub, deflecting the flap generates aerodynamic forces that twist the flexible blade itself, indirectly adjusting the angle of attack.

Control is indirect: **flap deflection → blade twist → pitch change → lift adjustment**.

---

## Mechanical Principle

- The flap is connected via push-pull rods through an azimuth (swashplate) assembly to pilot inputs
- **Collective control:** all flaps deflect equally → uniform lift increase/decrease
- **Cyclic control:** flap deflection varies per blade position as the rotor turns → rotor disk tilts → directional control

The key insight is that the aerodynamic work happens at 75% span where lift is already being generated, so the force required to move the flap is far smaller than the resulting pitch-change force on the blade.

---

## Key Advantages Relevant to This Design

| Advantage | Detail |
|-----------|--------|
| **No hydraulics** | Light mechanical linkages are sufficient — no powered control system needed |
| **Reduced control forces** | Small flap force produces large blade pitch change via aerodynamic leverage |
| **Simplified hub** | Eliminates complex pitch-change bearings at the hub |
| **Vibration absorption** | Blade flexing dissipates stress rather than transmitting it through rigid linkages |
| **Passive stability** | Flap responds automatically to aerodynamic changes (e.g. gusts, load shifts) |
| **Scalability** | Servo flap ~6 lbs vs main blade ~200 lbs — force ratio scales favorably |

---

## Patent Reference: US3217809 (Kaman Aircraft / Bossler, 1965)

The canonical servo-flap rotor control system. Key design elements:

### Control Architecture
- Two **cyclic control columns** — tilt rotor disk for directional control
- Two **collective columns** — adjust all blades simultaneously for thrust
- A **swashplate (azimuth) assembly** below the rotor shaft converts stationary pilot inputs into rotating blade commands

### Servo-Flap Linkage Path
```
Pilot input
  → Swashplate (azimuth ring, 3-point support)
  → Vertical control rods (20) to rotor hub
  → Rocklever (38) pivoted on hub
  → Link (42) → horn (44) → torsion rod (47) through blade body
  → Servo flap (46) deflects
  → Blade twists aerodynamically → pitch changes
```

### Decoupling — Critical Design Property
The three linkage trains are geometrically proportioned so that:
- The cyclic input arm connection distance = **2× the collective crank arm distance**
- This ensures cyclic inputs produce **pure swashplate tilt** with zero collective cross-coupling
- Collective inputs produce **pure collective pitch** with zero cyclic cross-coupling

No cross-coupling between collective and cyclic is the foundational requirement for predictable control.

### Static Load Isolation
The 3-point symmetric support design causes centrifugal blade loads, feathering torque, and aerodynamic forces to react against the stationary structure — **not fed back to the pilot**. Control feel is light and consistent regardless of rotor loading.

### Optional Automatic Correction
Bell crank positions can be adjusted to automatically introduce small corrective cyclic pitch changes when collective pitch changes — compensating for rotor drag or tail rotor thrust shifts without pilot input.

---

## Application Notes for This Design

- The EMAX GB4008 motor (66KV, hollow shaft) is well-suited as a servo-flap actuator — the hollow shaft allows a control rod to pass through the blade axis
- Low-KV / high-torque characteristic matches the low-speed, high-precision deflection requirement of a servo flap
- The REVVitRC ESC with AM32 firmware supports fine low-end throttle control (sinusoidal startup, variable PWM) which maps well to precise flap positioning
- The Pixhawk flight controller can manage collective + cyclic mixing in software, reducing mechanical complexity in the linkage

---

## Sources

- Patent US3217809 — https://patents.google.com/patent/US3217809
- Kaman servo flap explainer — https://www.helis.com/howflies/servo.php
