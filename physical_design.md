# RAWES — Physical Design Reference

## Assembly Overview

The RAWES is a tethered 4-blade autorotating rotor. The rotor hub is the outer rotating shell. Inside it, from top to bottom, is the full assembly — swashplate and all electronics. The axle runs through the center top to bottom, with the tether attached at the bottom.

### Cross-Section Layout (top to bottom)

```
[ rotor hub top bearing ]        ← hub spins on axle here

  blade─[flap]  blade─[flap]
  blade─[flap]  blade─[flap]     ← 4× blades, free-pitching within damper
       ▲              ▲
       └── push-rods (×4) ───────► upper swashplate ring (rotates with hub)
                                   push-rod anchor points

  ┌─────────────────────────┐
  │   upper swashplate ring │  ← rotates with rotor hub
  ├─────────────────────────┤
  │   swashplate bearing    │  ← allows upper/lower rings to rotate
  ├─────────────────────────┤     relative to each other
  │   lower swashplate ring │  ← NON-rotating; S1/S2/S3 tilt this
  ├─────────────────────────┤
  │   S1  S2  S3  servos    │  ← tilt lower ring (collective + cyclic)
  ├─────────────────────────┤
  │   Pixhawk 6C            │
  │   REVVitRC ESC          │
  │   GB4008 Motor ─────────────► 80:44 spur gear ──► counter-torque on hub
  │   UBEC / PM / BM        │     keeps this assembly non-rotating
  │   4S LiPo 15.2V         │
  │   SiK Radio / RP3-H Rx  │
  └─────────────────────────┘

[ rotor hub bottom bearing ]     ← hub spins on axle here

         axle (top to bottom through everything)
              │
         tether (1.9mm)
              │
        [ ground / winch ]
```

### Rotation Summary

| Rotates (wind-driven)     | Stationary (GB4008 counter-torque)        |
|---------------------------|-------------------------------------------|
| Rotor hub (outer shell)   | Lower swashplate ring                     |
| 4× blades + flaps         | Servos S1, S2, S3                         |
| Push-rods                 | Pixhawk, ESC, motor, battery, radios      |
| Upper swashplate ring     | Axle                                      |

---

## Rotor

### Geometry

| Parameter        | Value         |
|------------------|---------------|
| Blade count      | 4 (90° apart) |
| Blade length     | 2000 mm       |
| Boom             | ~500 mm       |
| Total radius     | ~2500 mm      |
| Total diameter   | ~5000 mm      |
| Chord            | ~200 mm       |
| Aspect ratio     | ~10:1         |
| Rotor mass       | 5 kg          |
| Blade twist      | None          |
| Tether attachment| Bottom of axle|

### Blade Airfoil — SG6042 (selected)

Selected over NACA 8H12 and Clark-Y at Re ≈ 127,301 (10 m/s wind, 0.2 m chord, 50% span).

| Candidate    | Outcome   | Reason                                                        |
|--------------|-----------|---------------------------------------------------------------|
| NACA 8H12    | Rejected  | Reflexed trailing edge (hard to foam-cut); flutter risk >5°  |
| Clark-Y      | Rejected  | Lower lift; flat bottom is easier to make but underperforms  |
| SG6042       | Selected  | ~2× lift of Clark-Y at operating Re; designed for low-Re     |
| SG6040       | Root only | Thicker root section from same series                        |

SG6042 properties:
- Thickness: 10%, zero-lift AoA: −4.92°
- Slight undercamber toward trailing edge
- Designed for laminar-flow, low-Re performance
- Material: EPP RG30 (expanded polypropylene foam)

### Blade Twist

**None.** Rationale:
- AoA variation from rotational position (3:00 vs 9:00 on disk) dominates over any practical fixed twist angle
- Twist benefits powered hover; less relevant for wind-driven autorotation
- NACA-TN-1666: −8° vs 0° twist shows little performance difference in autorotation

### Reynolds Number at Operating Conditions

- Wind speed: 10 m/s
- Chord: 0.2 m
- 50% span radius: ~1.25 m → ~45 RPM at λ = 7
- Air at 25°C: ρ = 1.184 kg/m³, ν = 1.571×10⁻⁵ m²/s
- **Re ≈ 127,301** — low-Re regime; airfoil selection is sensitive to laminar separation

---

## Pitch Control — Servo Flap via Swashplate

Blade pitch is controlled **indirectly**. There are no pitch bearings at the blade root. Instead:

```
Servos S1/S2/S3
  → tilt lower swashplate ring (collective + cyclic)
  → upper swashplate ring (rotating) follows tilt
  → push-rods (×4) transmit motion to trailing-edge flaps
  → flap deflection generates aerodynamic moment
  → flexible blade twists → angle of attack changes
```

Sign convention: γ < 0 (upward flap) → positive moment → blade pitches UP

### Why Servo Flaps

| Advantage           | Detail                                                              |
|---------------------|---------------------------------------------------------------------|
| No hub pitch bearing| Eliminates complex bearings at blade root                          |
| Low servo torque    | Servos move only small flaps, not blades directly                  |
| Vibration absorption| Blade flexing dissipates stress rather than transmitting it rigidly|
| Passive stability   | Flap responds automatically to aerodynamic changes                 |
| Force leverage      | Aerodynamic work at 75% span; flap force << resulting pitch force  |

### Flap Geometry (first prototype)

| Parameter     | Value           |
|---------------|-----------------|
| Span          | 900 mm          |
| Chord         | 55 mm           |
| Mass          | 42 g            |
| Airfoil       | SG6042          |
| Span position | ~60% (~900 mm from root on 2000 mm blade) |
| Hinge line    | ~10 cm from trailing edge                |

Flap attaches via screws to blade trailing edge. Small balsa/wood spar as flap structure.

### Swashplate Linkage (Kaman-style, ref. US Patent US3217809)

```
Pilot input (Pixhawk PWM)
  → Servos S1, S2, S3 (3-point support on lower ring)
  → Lower swashplate ring tilts (stationary)
  → Upper swashplate ring follows tilt (rotates with hub)
  → Push-rods (×4) to each blade's flap horn
  → Flap deflects
  → Blade twists aerodynamically → pitch changes
```

3 servos control collective + cyclic for all 4 blades — no individual blade actuators.

**Decoupling property (from US3217809):**
- Cyclic input arm distance = 2× collective crank arm distance
- Ensures cyclic inputs produce pure swashplate tilt with zero collective cross-coupling
- Collective inputs produce pure collective pitch with zero cyclic cross-coupling

---

## Anti-Rotation Motor

The **EMAX GB4008** gimbal motor keeps the lower swashplate ring, servos, and electronics stationary against rotor drag torque.

| Parameter      | Value                          |
|----------------|--------------------------------|
| Model          | EMAX GB4008                    |
| Type           | Brushless gimbal, hollow shaft |
| KV             | 66KV                           |
| Turns          | 90T                            |
| Pole/stator    | 24N22P                         |
| Resistance     | 7.5 Ω                          |
| Weight         | 101 g                          |
| Mounting       | 19×19 mm or 25×25 mm           |
| Gear reduction | 80:44 spur gear (~1.82× torque)|

The hollow shaft allows mechanical linkage through the blade axis. The motor only needs to match the drag torque of the rotor — it does not drive rotation.

---

## Electronics & Power

### Power Architecture

```
Battery 4S 15.2V
  ├── XT30 → BM/PM (Battery Monitor + Power Module) → XT60 → ESC → GB4008 Motor
  └── XT30 → UBEC → 8.0V → Pixhawk servo rail → S1, S2, S3 servos
                         └── Pixhawk FMU (via PM)
```

Note: Pixhawk 6C PM powers the FMU electronics but does **not** power the servo output rail. The UBEC must connect to the servo rail (pin 1 of any servo output). This connection is required.

### Communication Architecture

```
MissionPlanner (PC) ──SiK Radio V3 (433/915MHz)────────► Pixhawk 6C
Boxer M2 RC         ──ExpressLRS 2.4GHz──► RP3-H Rx ───► Pixhawk 6C
Pixhawk 6C          ──PWM (S1, S2, S3)──────────────────► Swashplate servos
Pixhawk 6C          ──DSHOT/PWM──────────► REVVitRC ESC ► GB4008 Motor
```

### Component List

**Flight Controller — Holybro Pixhawk 6C**
- ArduPilot / MissionPlanner
- Manages collective + cyclic swashplate mixing in software
- S1, S2, S3 → swashplate servo outputs
- PM powers FMU; servo rail powered separately by UBEC

**ESC — REVVitRC 50A AM32**
- AM32 firmware (native port)
- Motor protocols: Servo PWM, DSHOT 300, DSHOT 600
- Bi-directional DSHOT supported
- Built-in BEC 7.4V/8.4V (not used — UBEC used instead)
- Sinusoidal startup mode for smooth low-speed motor control
- Continuous: 50A, voltage: 3–6S (9–26V)
- Dimensions: 44×18×14 mm, weight: ~23 g

**UBEC**
- Output: 8.0V
- Powers: Pixhawk servo rail + S1, S2, S3 servos
- Input: battery via XT30

**Servos S1, S2, S3 — DS113MG V6.0 Digital Metal Gear Micro Servo**
- Type: Digital, metal gear, micro
- Role: Tilt lower swashplate ring (collective + cyclic control)
- 3× servos in 3-point arrangement

**Battery**
- 4S LiPo, 15.2V nominal
- Capacity: 450 mAh (confirmed — short flight time acceptable for current phase)

**Telemetry — Holybro SiK Telemetry Radio V3**
- MAVLink, bidirectional
- Frequency: 433 MHz or 915 MHz
- Power: 100 mW or 500 mW
- Range: 300 m+
- Interface: 3.3V UART, 6-pin JST-GH
- Dimensions: 28×53×10.7 mm, weight: 23.5 g

**RC Receiver — RadioMaster RP3-H ExpressLRS 2.4GHz**
- Protocol: ExpressLRS 2.4GHz
- MCU: ESP8285, RF chip: SX1281
- Output: CRSF / S.Bus
- Refresh rate: 25–500 Hz / F1000Hz
- Voltage: 4.5–8.4V DC
- Dimensions: 28×24×8 mm, weight: 5.1 g

**RC Transmitter — RadioMaster Boxer M2**
- Firmware: EdgeTX
- RF: ExpressLRS 2.4GHz internal module
- Channels: up to 16
- Gimbals: Hall effect V4.0
- Display: 128×64 monochrome LCD
- Weight: 532.5 g

---

## Power Budget

### Power Sources

| Source   | Input          | Output    | Connector | Powers                                |
|----------|----------------|-----------|-----------|---------------------------------------|
| Battery  | —              | 15.2V 4S  | XT30 ×2   | Everything (via PM and UBEC)          |
| PM       | 15.2V battery  | ~5.3V     | JST-GH    | Pixhawk FMU only                      |
| UBEC     | 15.2V battery  | 8.0V      | XT30 in, servo rail out | S1/S2/S3 + Pixhawk servo rail |

### Per-Component Power Requirements

| Component             | Supplied by    | Voltage     | Idle current     | Peak current        | Notes                                   |
|-----------------------|----------------|-------------|------------------|---------------------|-----------------------------------------|
| GB4008 Motor          | ESC → battery  | 15.2V (4S)  | ~0 A             | ~2 A (est.)         | 7.5Ω winding; only opposes drag torque — not driving rotation |
| REVVitRC 50A ESC      | Battery        | 15.2V (4S)  | ~0.1 A           | 50 A max            | Current limit set by ESC firmware       |
| Pixhawk 6C FMU        | PM             | 4.1–5.7V   | ~0.25 A (est.)   | ~0.5 A (est.)       | PM provides regulated 5.3V; servo rail powered separately |
| DS113MG V6.0 (S1)     | UBEC → servo rail | 8.0V     | ~0.05 A          | 1.6 A stall         | Rated 4.8–8.4V; stall at 8.4V = 1.6 A  |
| DS113MG V6.0 (S2)     | UBEC → servo rail | 8.0V     | ~0.05 A          | 1.6 A stall         | "                                       |
| DS113MG V6.0 (S3)     | UBEC → servo rail | 8.0V     | ~0.05 A          | 1.6 A stall         | "                                       |
| Holybro SiK Radio V3  | Pixhawk TELEM  | 5.0V        | 0.025 A RX       | 0.1 A TX            | Powered from Pixhawk TELEM port         |
| RadioMaster RP3-H Rx  | Pixhawk RC port| 4.5–8.4V   | ~0.1 A (est.)    | ~0.15 A (est.)      | Connected to Pixhawk RC input at 5V     |

### Servo Rail Budget (UBEC @ 8V)

| Load              | Idle    | Peak (all 3 stall) |
|-------------------|---------|--------------------|
| 3× DS113MG        | ~0.15 A | ~4.8 A             |
| Pixhawk servo rail| ~0.05 A | ~0.05 A            |
| **Total**         | ~0.2 A  | ~4.85 A            |

UBEC must be rated for at least **5 A continuous** at 8V. Verify UBEC model/rating — not yet confirmed.

### Battery Runtime Estimate (450 mAh)

| Scenario                  | Battery current (est.) | Runtime      |
|---------------------------|------------------------|--------------|
| Idle / bench test         | ~1 A                   | ~27 min      |
| Active flight (motor + servos) | ~5–8 A            | ~4–6 min     |
| Motor at full counter-torque   | ~8–12 A (est.)    | ~2–3 min     |

> These are rough estimates. Actual motor current depends on rotor drag torque, which scales with rpm² and air density.

### Open Items — Power

- [ ] Confirm UBEC model and current rating (must support ≥5 A at 8V for 3× stall servos)
- [ ] Confirm RP3-H power source (Pixhawk RC port 5V vs. servo rail 8V — check voltage compatibility)
- [ ] Measure actual GB4008 counter-torque current at operating rotor RPM

---

## Key Design Constraints

| Constraint                        | Value              |
|-----------------------------------|--------------------|
| Max rotor tilt (good tracking)    | ≤ 15°              |
| Min tip speed ratio               | λ ≥ 7              |
| AoA for linear model validity     | < 15–20°           |
| Tether diameter                   | 1.9 mm             |
| Max tether length                 | 300 m              |
| Min altitude                      | 10 m               |
| Rated power output                | ~3 kW at 10 m/s    |

---

## Sources

- US Patent US3217809 (Kaman/Bossler, 1965) — https://patents.google.com/patent/US3217809
- Kaman servo flap explainer — https://www.helis.com/howflies/servo.php
- SG6042 airfoil data — http://airfoiltools.com/airfoil/details?airfoil=sg6042-il
- RC autogyro aerodynamic design (RC Modeler Aug 2001) — https://www.researchgate.net/publication/301588181_Radio_controlled_autogiro_aerodynamic_design
- Blade twist discussion — https://www.rotaryforum.com/threads/ideal-twist-and-autorotation.32480/
- EMAX GB4008 — https://emaxmodel.com/products/emax-gb4008-brushless-gimbal-motor-hollow-shaft-66kv-90t
- REVVitRC 50A AM32 — https://justcuzrobotics.com/products/revvitrc-50a-am32-esc
- AM32 firmware — https://am32.ca/
- Holybro SiK Radio V3 — https://holybro.com/products/sik-telemetry-radio-v3
- RadioMaster RP3-H — https://radiomasterrc.com/products/rp3-h-expresslrs-2-4ghz-nano-receiver
- RadioMaster Boxer M2 — https://radiomasterrc.com/products/boxer-radio-controller-m2
