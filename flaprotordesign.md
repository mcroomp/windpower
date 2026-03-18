# Flap Rotor Blade Design

## Rotor Geometry

| Parameter | Value |
|-----------|-------|
| Blade length | 1500mm |
| Boom | 1000mm |
| **Total radius** | **2600mm** |
| Chord | 100mm |
| Aspect ratio | 10:1 (good range: 8:1–12:1) |

---

## Airfoil Selection

Four candidates evaluated at Re ≈ 127,301 (10m/s, 0.2m chord, 50% span / 2m).

### Candidate 1 — NACA 8H12 ("autogyro profile")
- Classic rotorcraft airfoil, universally known for autogyro use
- **REJECTED:**
  - Reflexed trailing edge — hard to cut on foam cutter
  - Cm varies significantly at Alpha >5° → flutter risk

### Candidate 2 — Clark-Y
- Max thickness: 11.72%, zero lift AoA: −3.51°
- Flat bottom from ~10% chord to trailing edge — easy to manufacture
- Good baseline but lower lift than SG6042

### Candidate 3 — SG6042 ✓ SELECTED
- Thickness: 10%, zero lift AoA: −4.92°
- Slight undercamber toward trailing edge
- **"Generated about twice the lifting force as Clark-Y at most all wind velocities"** (wind tunnel + flight test confirmed)
- Superior Cl/Cd ratio at operating Reynolds numbers

### Candidate 4 — SG6040 (root section)
- Thicker root airfoil from the SG6040–SG6043 series
- Used at blade root where chord is wider
- Material: EPP RG30

### Airfoil Design Rules
- Thickness: 13–16% of chord (never exceed 16% — blades over 16% hard to spin up)
- Narrow leading edge radius, thin trailing edge preferred
- Bottom-flattened Clark-YS with slightly reflexed trailing edge prevents leading edge tuck-under

---

## Blade Twist

**Decision: NO TWIST**

Rationale:
- In forward flight, AoA variation from rotational position (3:00 vs 9:00 on the disk) far exceeds any practical twist angle
- Twist primarily benefits powered hover — less relevant for wind-driven autorotation
- In autorotation, each blade section acts as a windmill brake; reverse twist would theoretically be optimal, but any fixed twist is a compromise
- NACA-TN-1666 data: −8° twist vs 0° twist showed **little performance difference** in power-off vertical descent or forward glide

---

## Servo Flap (Physical Build)

| Parameter | Value |
|-----------|-------|
| First prototype flap | 900 × 55mm |
| Mass | 42g |
| Airfoil | SG6042 |
| Span position | ~60% (at ~900mm from root on 1500mm blade) |

Flap is attached via screws to the blade trailing edge. Photos confirm hinge line at approximately 10cm from trailing edge with a small balsa/wood spar as flap structure.

---

## Reynolds Number at Operating Conditions

- Fluid velocity: 10 m/s
- Characteristic dimension (chord): 0.2m
- 50% span radius: 2m → 45 RPM
- Air at 25°C: ρ = 1.184 kg/m³, ν = 1.571×10⁻⁵ m²/s
- **Re ≈ 127,301**

This is low-Re territory — airfoil selection sensitive to laminar separation; SG6042 is specifically designed for this range.

---

## Material

- **EPP RG30** (expanded polypropylene) — blade body
- Lightweight, flexible, foam-cuttable

---

## Tools & References

- Airfoil plotter: http://airfoiltools.com/plotter/index
- SG6042 data: http://airfoiltools.com/airfoil/details?airfoil=sg6042-il
- Wing design app: https://www.diyrcwings.com/app/
- RC autogyro aerodynamic design (RC Modeler Aug 2001): https://www.researchgate.net/publication/301588181_Radio_controlled_autogiro_aerodynamic_design
- Blade twist discussion: https://www.rotaryforum.com/threads/ideal-twist-and-autorotation.32480/
