# Physical System

| Parameter | Value |
|-----------|-------|
| Blade count | 4 (90° apart) |
| Blade length | 2000 mm, rotor radius ~2500 mm |
| Rotor mass | 5 kg, airfoil SG6042 |
| Tether | Dyneema SK75, 1.9 mm, max 300 m |
| Anti-rotation motor | EMAX GB4008 66KV, 80:44 spur gear |
| Servos S1/S2/S3 | DS113MG V6.0 |
| Flight controller | Holybro Pixhawk 6C |
| Battery | 4S LiPo 15.2V, 450 mAh |

**Critical:** Only the spinning outer hub shell + blades contribute to `I_spin`. Motor torque is an **internal** force — never add as external couple.

## Ground Winch Motor/Generator

Separate from the GB4008. Sized to deliver 435 N at 0.40 m/s reel-out and 226 N at 0.80 m/s reel-in with ~200 W continuous.

| Parameter | Value | Derivation |
|-----------|-------|------------|
| Motor type | 400 W BLDC servo, 48 V | P = 435 N × 0.40 m/s = 174 W, add 15% margin |
| Rated speed | 3000 RPM | — |
| Rated torque | 1.27 N·m | P / ω = 400 / (3000 × 2π/60) |
| Peak torque | 3.8 N·m | 3× rated, typical BLDC |
| Gearbox | 20:1 planetary, η = 0.85 | — |
| Drum radius | 50 mm | — |
| Reel-out speed | 0.40 m/s → 1528 RPM motor | v / r / (2π/60) / gear_ratio |
| Reel-in speed | 0.80 m/s → 3056 RPM motor | near rated speed |
| T_reel_out (435 N) | drum 21.75 N·m → motor 1.28 N·m | ≈ rated torque ✓ |
| T_hard_max (496 N) | drum 24.8 N·m → motor 1.46 N·m | 115% rated → current-limit here |
| T_reel_in (226 N) | drum 11.3 N·m → motor 0.66 N·m | 52% rated, ample margin |

**Winch simulation parameters derived from this spec:**

- `T_soft_max = 470 N`, `T_hard_max = 496 N` — reel-out generator taper (matches motor 115% current-limit ceiling)
- `T_reel_in_start = 250 N` — gate: AP must reduce tension before reel-in motor engages
- `T_soft_reel_in = 350 N`, `T_hard_reel_in = 496 N` — reel-in upper taper: motor current-limits and tapers to stop as tension rises toward 496 N (same ceiling as reel-out; shared hardware limit)
- `T_soft_min = 30 N`, `T_hard_min = 10 N` — slack boost: below 30 N reel-in speed increases toward 2× nominal to take up slack; models the drum coasting freely when tether goes slack
