# RAWES -- Physical Design Reference

## 1. Overview / Assembly Summary

The RAWES is a tethered 4-blade autorotating rotor. The rotor hub is the outer rotating shell.
Inside it, from top to bottom, is the full assembly -- swashplate and all electronics. The axle
runs through the center top to bottom, with the tether attached at the bottom.

### Cross-Section Layout (top to bottom)

```
[ rotor hub top bearing ]        <- hub spins on axle here

  blade-[flap]  blade-[flap]
  blade-[flap]  blade-[flap]     <- 4x blades, free-pitching within damper
       ^              ^
       +-- push-rods (x4) -------> upper swashplate ring (rotates with hub)
                                   push-rod anchor points

  +-------------------------+
  |   upper swashplate ring |  <- rotates with rotor hub
  +-------------------------+
  |   swashplate bearing    |  <- allows upper/lower rings to rotate
  +-------------------------+     relative to each other
  |   lower swashplate ring |  <- NON-rotating; S1/S2/S3 tilt this
  +-------------------------+
  |   S1  S2  S3  servos    |  <- tilt lower ring (collective + cyclic)
  +-------------------------+
  |   Pixhawk 6C            |
  |   REVVitRC ESC          |
  |   GB4008 Motor ------+------> 80:44 spur gear --> counter-torque on hub
  |   UBEC / PM / BM        |     keeps this assembly non-rotating
  |   4S LiPo 15.2V         |
  |   SiK Radio / RP3-H Rx  |
  +-------------------------+

[ rotor hub bottom bearing ]     <- hub spins on axle here

         axle (top to bottom through everything)
              |
         tether (1.9mm)
              |
        [ ground / winch ]
```

### Rotation Summary

| Rotates (wind-driven)     | Stationary (GB4008 counter-torque)        |
|---------------------------|-------------------------------------------|
| Rotor hub (outer shell)   | Lower swashplate ring                     |
| 4x blades + flaps         | Servos S1, S2, S3                         |
| Push-rods                 | Pixhawk, ESC, motor, battery, radios      |
| Upper swashplate ring     | Axle                                      |

---

## 2. Rotor Geometry

| Parameter        | Value         |
|------------------|---------------|
| Blade count      | 4 (90 apart)  |
| Blade length     | 2000 mm       |
| Boom             | ~500 mm       |
| Total radius     | ~2500 mm      |
| Total diameter   | ~5000 mm      |
| Chord            | ~200 mm       |
| Aspect ratio     | ~10:1         |
| Rotor mass       | 5 kg          |
| Blade twist      | None          |
| Tether attachment| Bottom of axle|

**Note on hardware geometry:** The physical hardware design document records boom 1000 mm and blade
span 1500 mm (total radius 2600 mm). The simulation uses r_root=500 mm and blade_length=2000 mm
(total radius 2500 mm). Confirm with hardware before relying on either value.

---

## 3. Blade Design

### Airfoil Selection -- SG6042 (selected)

Four candidates evaluated at Re ~127,301 (10 m/s wind, 0.2 m chord, 50% span):

| Candidate    | Outcome   | Reason                                                        |
|--------------|-----------|---------------------------------------------------------------|
| NACA 8H12    | Rejected  | Reflexed trailing edge (hard to foam-cut); flutter risk >5   |
| Clark-Y      | Rejected  | Lower lift; flat bottom is easier to make but underperforms  |
| SG6042       | Selected  | ~2x lift of Clark-Y at operating Re; designed for low-Re     |
| SG6040       | Root only | Thicker root section from same series                        |

SG6042 properties:
- Thickness: 10%, zero-lift AoA: -4.92 deg
- Slight undercamber toward trailing edge
- Designed for laminar-flow, low-Re performance
- Material: EPP RG30 (expanded polypropylene foam)

Selection rationale: "The SG6042 generated about twice the lifting force as the Clark-Y airfoil at
most all wind velocities" (wind tunnel + flight test, RC Modeler Aug 2001).

Sources:
- SG6042 airfoil data: http://airfoiltools.com/airfoil/details?airfoil=sg6042-il
- RC autogyro aerodynamic design (RC Modeler Aug 2001): https://www.researchgate.net/publication/301588181_Radio_controlled_autogiro_aerodynamic_design

### Blade Twist

**None.** Rationale:
- AoA variation from rotational position (3:00 vs 9:00 on disk) dominates over any practical
  fixed twist angle
- Twist benefits powered hover; less relevant for wind-driven autorotation
- NACA-TN-1666: -8 deg vs 0 deg twist shows little performance difference in autorotation

### Airfoil Design Rules (from hardware design notes)

- Thickness: 13-16% of chord (never exceed 16% -- blades over 16% hard to spin up)
- Narrow leading edge radius, thin trailing edge preferred
- Bottom-flattened Clark-YS with slightly reflexed trailing edge prevents leading edge tuck-under

### Reynolds Number at Operating Conditions

- Wind speed: 10 m/s
- Chord: 0.2 m
- 50% span radius: ~1.25 m -> ~45 RPM at tip speed ratio 7
- Air at 25 degC: rho = 1.184 kg/m^3, nu = 1.571e-5 m^2/s
- **Re ~127,301** -- low-Re regime; airfoil selection is sensitive to laminar separation

### SG6042 Polar Data (from hardware design document, approximate)

| Parameter | Value | Condition |
|-----------|-------|-----------|
| C_L at alpha=0 | ~0.4-0.5 | Re ~127k |
| C_L,max | ~1.2-1.4 | alpha ~10-12 deg |
| Zero-lift AoA | ~-5 deg | Consistent with -4.92 deg |
| C_D,min | ~0.01-0.02 | Linear range |
| C_D rise | Steep above alpha ~10 deg | |

**Note:** C_L at alpha=0 of ~0.4-0.5 does NOT match the Weyel thesis C_L,0=0.11 (read from SG6040
polar at different Re). See theory/flap_dynamics.md for Weyel model parameters.

### Material

EPP RG30 (expanded polypropylene foam) -- blade body. Lightweight, flexible, foam-cuttable.

---

## 4. Swashplate and Pitch Control

Blade pitch is controlled **indirectly**. There are no pitch bearings at the blade root. Instead:

```
Servos S1/S2/S3
  -> tilt lower swashplate ring (collective + cyclic)
  -> upper swashplate ring (rotating) follows tilt
  -> push-rods (x4) transmit motion to trailing-edge flaps
  -> flap deflection generates aerodynamic moment
  -> flexible blade twists -> angle of attack changes
```

Sign convention: gamma < 0 (upward flap) -> positive moment -> blade pitches UP

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

---

## 5. Servo Flap Mechanism (Kaman / US Patent US3217809)

A servo flap is a small airfoil mounted on the trailing edge of a rotor blade at approximately
75% span. Rather than mechanically changing blade pitch at the hub, deflecting the flap generates
aerodynamic forces that twist the flexible blade itself, indirectly adjusting the angle of attack.

Control is indirect: **flap deflection -> blade twist -> pitch change -> lift adjustment**.

### Mechanical Principle

- The flap is connected via push-pull rods through a swashplate assembly to pilot inputs
- **Collective control:** all flaps deflect equally -> uniform lift increase/decrease
- **Cyclic control:** flap deflection varies per blade position as the rotor turns -> rotor disk
  tilts -> directional control

The aerodynamic work happens at 75% span where lift is already being generated, so the force
required to move the flap is far smaller than the resulting pitch-change force on the blade.

### Swashplate Linkage Path (Kaman-style, ref. US Patent US3217809)

```
Pilot input (Pixhawk PWM)
  -> Servos S1, S2, S3 (3-point support on lower ring)
  -> Lower swashplate ring tilts (stationary)
  -> Upper swashplate ring follows tilt (rotates with hub)
  -> Push-rods (x4) to each blade's flap horn
  -> Flap deflects
  -> Blade twists aerodynamically -> pitch changes
```

3 servos control collective + cyclic for all 4 blades -- no individual blade actuators.

### Decoupling -- Critical Design Property (from US3217809)

The three linkage trains are geometrically proportioned so that:
- The cyclic input arm connection distance = **2x the collective crank arm distance**
- This ensures cyclic inputs produce **pure swashplate tilt** with zero collective cross-coupling
- Collective inputs produce **pure collective pitch** with zero cyclic cross-coupling

No cross-coupling between collective and cyclic is the foundational requirement for predictable
control.

### Static Load Isolation

The 3-point symmetric support design causes centrifugal blade loads, feathering torque, and
aerodynamic forces to react against the stationary structure -- **not fed back to the pilot**.
Control feel is light and consistent regardless of rotor loading.

### Key Advantages

| Advantage | Detail |
|-----------|--------|
| No hydraulics | Light mechanical linkages are sufficient -- no powered control system needed |
| Reduced control forces | Small flap force produces large blade pitch change via aerodynamic leverage |
| Simplified hub | Eliminates complex pitch-change bearings at the hub |
| Vibration absorption | Blade flexing dissipates stress rather than transmitting it through rigid linkages |
| Passive stability | Flap responds automatically to aerodynamic changes (e.g. gusts, load shifts) |

### Sources

- Patent US3217809: https://patents.google.com/patent/US3217809
- Kaman servo flap explainer: https://www.helis.com/howflies/servo.php

---

## 6. Anti-Rotation Motor (GB4008)

The **EMAX GB4008** gimbal motor is mechanically geared to the spinning axle via an 80:44
spur gear. It counter-rotates against the rotor, keeping the lower swashplate ring,
servos, and electronics at a fixed heading. Speed control ensures the motor's
counter-rotation exactly cancels the axle spin seen by the stationary assembly.

| Parameter      | Value                          |
|----------------|--------------------------------|
| Model          | EMAX GB4008                    |
| Type           | Brushless gimbal, hollow shaft |
| KV             | 66KV                           |
| Turns          | 90T                            |
| Pole/stator    | 24N22P                         |
| Resistance     | 7.5 ohm                        |
| Cable gauge    | 0.19 mm                        |
| Weight         | 101 g                          |
| Mounting       | 19x19 mm or 25x25 mm           |
| Gear reduction | 80:44 spur gear (~1.82x torque)|

The hollow shaft allows mechanical linkage through the blade axis. The motor does not drive
rotor rotation -- it counter-rotates via the gear coupling, and we control its speed to keep
the stationary assembly at a fixed heading.

Application notes:
- Low-KV / high-torque characteristic matches the low-speed, high-precision deflection requirement
- The REVVitRC ESC with AM32 firmware supports fine low-end throttle control (sinusoidal startup,
  variable PWM)

Source: https://emaxmodel.com/products/emax-gb4008-brushless-gimbal-motor-hollow-shaft-66kv-90t

---

## 7. Electronics Layout and Power Architecture

### Power Architecture

```
Battery 4S 15.2V
  +-- XT30 -> BM/PM (Battery Monitor + Power Module) -> XT60 -> ESC -> GB4008 Motor
  +-- XT30 -> UBEC -> 8.0V -> Pixhawk servo rail -> S1, S2, S3 servos
                           +-- Pixhawk FMU (via PM)
```

Note: Pixhawk 6C PM powers the FMU electronics but does **not** power the servo output rail.
The UBEC must connect to the servo rail (pin 1 of any servo output). This connection is required.

### Communication Architecture

```
MissionPlanner (PC) --SiK Radio V3 (433/915MHz)--------> Pixhawk 6C
Boxer M2 RC         --ExpressLRS 2.4GHz--> RP3-H Rx ---> Pixhawk 6C
Pixhawk 6C          --PWM (S1, S2, S3)-----------------> Swashplate servos
Pixhawk 6C          --DSHOT/PWM----------> REVVitRC ESC -> GB4008 Motor
```

### Component Specifications

**Flight Controller -- Holybro Pixhawk 6C**
- ArduPilot / MissionPlanner
- Manages collective + cyclic swashplate mixing in software
- S1, S2, S3 -> swashplate servo outputs
- PM powers FMU; servo rail powered separately by UBEC

**ESC -- REVVitRC 50A AM32**
- AM32 firmware (native port, not converted from BlHeli32)
- Motor protocols: Servo PWM, DSHOT 300, DSHOT 600
- Bi-directional DSHOT supported
- Built-in BEC 7.4V/8.4V (not used -- UBEC used instead)
- Sinusoidal startup mode for smooth low-speed motor control
- Continuous: 50A, voltage: 3-6S (9-26V)
- Dimensions: 44x18x14 mm, weight: ~23 g
- Telemetry: voltage, current, temperature (KISS standard)
- Sources: https://justcuzrobotics.com/products/revvitrc-50a-am32-esc
           https://am32.ca/

**UBEC**
- Output: 8.0V
- Powers: Pixhawk servo rail + S1, S2, S3 servos
- Input: battery via XT30

**Servos S1, S2, S3 -- DS113MG V6.0 Digital Metal Gear Micro Servo**
- Type: Digital, metal gear, micro
- Role: Tilt lower swashplate ring (collective + cyclic control)
- 3x servos in 3-point arrangement
- Power: 8.0V via UBEC through Pixhawk servo rail

**Battery**
- 4S LiPo, 15.2V nominal
- Capacity: 450 mAh (confirmed -- short flight time acceptable for current phase)

**Telemetry -- Holybro SiK Telemetry Radio V3**
- MAVLink, bidirectional
- Frequency: 433 MHz or 915 MHz
- Power: 100 mW (20 dBm) or 500 mW (27 dBm)
- Range: 300 m+
- Receive sensitivity: -121 dBm @ 2kbps / -105 dBm @ 64kbps
- RF data rates: 2-250 kbps (default 64 kbps)
- Interface: 3.3V UART, 6-pin JST-GH
- Dimensions: 28x53x10.7 mm, weight: 23.5 g
- Source: https://holybro.com/products/sik-telemetry-radio-v3

**RC Receiver -- RadioMaster RP3-H ExpressLRS 2.4GHz**
- Protocol: ExpressLRS 2.4GHz
- MCU: ESP8285, RF chip: SX1281
- Frequency range: 2.404-2.479 GHz
- Telemetry TX power: Max 100 mW
- Antennas: 2x 150 mm 2.4GHz antenna (diversity)
- Output: CRSF / S.Bus
- Refresh rate: 25-500 Hz / F1000Hz
- Voltage: 4.5-8.4V DC
- Dimensions: 28x24x8 mm, weight: 5.1 g
- Source: https://radiomasterrc.com/products/rp3-h-expresslrs-2-4ghz-nano-receiver

**RC Transmitter -- RadioMaster Boxer M2**
- Firmware: EdgeTX
- RF: ExpressLRS 2.4GHz internal module
- Channels: up to 16
- Gimbals: Hall effect V4.0
- Display: 128x64 monochrome LCD
- Battery: 2S 7.4V LiPo or 2x 18650 Li-Ion (not included)
- Weight: 532.5 g
- Source: https://radiomasterrc.com/products/boxer-radio-controller-m2

---

## Power Budget

### Power Sources

| Source   | Input          | Output    | Connector | Powers                                |
|----------|----------------|-----------|-----------|---------------------------------------|
| Battery  | --             | 15.2V 4S  | XT30 x2   | Everything (via PM and UBEC)          |
| PM       | 15.2V battery  | ~5.3V     | JST-GH    | Pixhawk FMU only                      |
| UBEC     | 15.2V battery  | 8.0V      | XT30 in, servo rail out | S1/S2/S3 + Pixhawk servo rail |

### Per-Component Power Requirements

| Component             | Supplied by    | Voltage     | Idle current     | Peak current        | Notes                                   |
|-----------------------|----------------|-------------|------------------|---------------------|-----------------------------------------|
| GB4008 Motor          | ESC -> battery | 15.2V (4S)  | ~0 A             | ~2 A (est.)         | 7.5 ohm winding; geared counter-rotation for heading hold |
| REVVitRC 50A ESC      | Battery        | 15.2V (4S)  | ~0.1 A           | 50 A max            | Current limit set by ESC firmware       |
| Pixhawk 6C FMU        | PM             | 4.1-5.7V    | ~0.25 A (est.)   | ~0.5 A (est.)       | PM provides regulated 5.3V             |
| DS113MG V6.0 (S1)     | UBEC -> servo rail | 8.0V   | ~0.05 A          | 1.6 A stall         | Rated 4.8-8.4V; stall at 8.4V = 1.6 A |
| DS113MG V6.0 (S2)     | UBEC -> servo rail | 8.0V   | ~0.05 A          | 1.6 A stall         |                                         |
| DS113MG V6.0 (S3)     | UBEC -> servo rail | 8.0V   | ~0.05 A          | 1.6 A stall         |                                         |
| Holybro SiK Radio V3  | Pixhawk TELEM  | 5.0V        | 0.025 A RX       | 0.1 A TX            | Powered from Pixhawk TELEM port         |
| RadioMaster RP3-H Rx  | Pixhawk RC port| 4.5-8.4V   | ~0.1 A (est.)    | ~0.15 A (est.)      | Connected to Pixhawk RC input at 5V     |

### Servo Rail Budget (UBEC @ 8V)

| Load              | Idle    | Peak (all 3 stall) |
|-------------------|---------|--------------------|
| 3x DS113MG        | ~0.15 A | ~4.8 A             |
| Pixhawk servo rail| ~0.05 A | ~0.05 A            |
| **Total**         | ~0.2 A  | ~4.85 A            |

UBEC must be rated for at least **5 A continuous** at 8V. Verify UBEC model/rating -- not yet confirmed.

### Battery Runtime Estimate (450 mAh)

| Scenario                  | Battery current (est.) | Runtime      |
|---------------------------|------------------------|--------------|
| Idle / bench test         | ~1 A                   | ~27 min      |
| Active flight (motor + servos) | ~5-8 A            | ~4-6 min     |
| Motor at full counter-torque   | ~8-12 A (est.)    | ~2-3 min     |

These are rough estimates. Actual motor current depends on bearing and swashplate friction,
which scales with rpm.

### Open Items -- Power

- [ ] Confirm UBEC model and current rating (must support >=5 A at 8V for 3x stall servos)
- [x] Confirm RP3-H power source: powered from servo rail at 8V (working voltage DC 4.5-8.4V --
      confirmed compatible)
- [ ] Measure actual GB4008 counter-torque current at operating rotor RPM

---

## Key Design Constraints

| Constraint                        | Value              |
|-----------------------------------|--------------------|
| Max rotor tilt (good tracking)    | <=15 deg           |
| Min tip speed ratio               | lambda >= 7        |
| AoA for linear model validity     | <15-20 deg         |
| Tether diameter                   | 1.9 mm             |
| Max tether length                 | 300 m              |
| Min altitude                      | 10 m               |
| Rated power output                | ~3 kW at 10 m/s    |

---

## Sources

- US Patent US3217809 (Kaman/Bossler, 1965): https://patents.google.com/patent/US3217809
- Kaman servo flap explainer: https://www.helis.com/howflies/servo.php
- SG6042 airfoil data: http://airfoiltools.com/airfoil/details?airfoil=sg6042-il
- RC autogyro aerodynamic design (RC Modeler Aug 2001): https://www.researchgate.net/publication/301588181_Radio_controlled_autogiro_aerodynamic_design
- Blade twist discussion: https://www.rotaryforum.com/threads/ideal-twist-and-autorotation.32480/
- EMAX GB4008: https://emaxmodel.com/products/emax-gb4008-brushless-gimbal-motor-hollow-shaft-66kv-90t
- REVVitRC 50A AM32: https://justcuzrobotics.com/products/revvitrc-50a-am32-esc
- AM32 firmware: https://am32.ca/
- Holybro SiK Radio V3: https://holybro.com/products/sik-telemetry-radio-v3
- RadioMaster RP3-H: https://radiomasterrc.com/products/rp3-h-expresslrs-2-4ghz-nano-receiver
- RadioMaster Boxer M2: https://radiomasterrc.com/products/boxer-radio-controller-m2
