# FlapRotor Blade Design — Data Summary

**Document:** FlapRotor Blade Design.pdf
**Project:** someAWE Labs / Christof Beaupoil RAWES hardware
**Content:** Airfoil selection rationale, blade geometry decisions, and first flap prototype

---

## Airfoil Selection

Three candidates evaluated:

| Airfoil | Zero-lift AoA | Thickness | Decision |
|---------|--------------|-----------|----------|
| NACA 8H12 | — | — | Rejected — reflexed TE hard to foam-cut; C_M varies at α > 5° (flutter risk) |
| Clark-Y | −3.51° | ~11.72% | Rejected — inferior lift at operating Re |
| **SG6042** | **−4.92°** | **10%** | **Selected** |

**Selection rationale:** "The SG6042 generated about twice the lifting force as the Clark-Y airfoil at most all wind velocities"
(wind tunnel tests: Les Garber, RC Modeler August 2001, pp. 70–76)

**SG6042 source:** http://airfoiltools.com/airfoil/details?airfoil=sg6042-il
**Polar source:** diyrcwings.com (Đorđe Mijailović), multiple Re curves

---

## Blade Geometry (Actual Hardware)

| Parameter | Value | Notes |
|-----------|-------|-------|
| Main airfoil | SG6042 | Outer span |
| Root airfoil | SG6040 | Thicker root section |
| Blade span | 1500 mm | Per blade (root to tip) |
| Boom length | 1000 mm | Hub centre to blade root |
| **Total radius** | **2600 mm** | Boom + blade span |
| **Chord** | **200 mm** | Constant (no taper) |
| Blade material | EPP RG30 | 30 g/L foam density |
| Twist | **None** | Explicitly stated "NO TWIST" |
| Number of blades | 4 | 90° spacing |

**Radius note:** Total radius from hub centre = 1000 mm boom + ~1600 mm (1500 mm blade + hub adapter) = 2600 mm.
The simulation uses r_root = 500 mm (hub outer radius), blade length 2000 mm → R = 2500 mm. Confirm with hardware.

**Twist rationale (p. 2–3):** AoA variation through rotation cycle is larger than any reasonable twist angle.
Referenced NACA-TN-1666: −8° twist had "little effect" vs 0° in autorotation.

---

## Operating Reynolds Number (p. 14)

Calculated for: 10 m/s wind, 45 RPM, 50% span (r = 1.0 m), c = 200 mm

| Parameter | Value |
|-----------|-------|
| Air temperature | 25°C |
| ρ | 1.184 kg/m³ |
| μ | 1.86×10⁻⁵ kg/(m·s) |
| ν | 1.571×10⁻⁵ m²/s |
| **Re** | **127,301** |

---

## SG6042 Polar Data (from diyrcwings.com plots, p. 9–10)

Visual reads from polar charts — approximate values:

| Parameter | Value | Condition |
|-----------|-------|-----------|
| C_L at α = 0° | ~0.4–0.5 | Re ≈ 127k |
| C_L,max | ~1.2–1.4 | α ≈ 10–12° |
| Zero-lift AoA | ~−5° | Consistent with −4.92° |
| C_D,min | ~0.01–0.02 | Linear range |
| C_D rise | Steep above α ≈ 10° | |

**Note:** C_L at α = 0° of ~0.4–0.5 does **not** match the Weyel thesis C_L,0 = 0.11.
The thesis value was read from an SG6040 polar, not SG6042, and at a different Re.

---

## First Flap Prototype

| Parameter | Value |
|-----------|-------|
| Dimensions | 900 mm × 55 mm |
| Mass | 42 g |
| Airfoil | SG6042 |

---

## Aspect Ratio Guidance (p. 7)

- 10:1 considered good; range 8:1 to 12:1
- Blade thickness: best 13–16% of chord
- SG6042 at 10% is within acceptable range

---

## Gaps vs Simulation Model

| Item | Document value | beaupoil_2026.yaml value | Action needed |
|------|---------------|--------------------------|---------------|
| Chord | 200 mm | 150 mm | **Update YAML to 0.20 m** |
| Total radius | 2600 mm | 2500 mm | Confirm with hardware |
| C_L at α=0 | ~0.4–0.5 (SG6042 polar) | CL0 = 0.11 (SG6040 thesis) | **Measure or fit SG6042 polar** |
| Drag polar form | not specified | CD0 + CL²/(π·AR·e) | No SG6042 drag data in doc |
| Oswald efficiency | not in document | 0.8 (assumed) | Not measured |
| C_M,γ | not in document | −0.35 (GUESS) | Weyel Table 2 gives −0.5 for thesis geometry |
