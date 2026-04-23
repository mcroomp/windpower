# ArduPilot Swashplate Control: Signal Flow & Parameters

Traces the full path from RC3 collective stick input to servo PWM output for a
single-rotor helicopter using the H3-120 swashplate type.

---

## 1. RC3 Collective → Normalized 0..1

**Source:** [`AC_InputManager_Heli.cpp:62`](../../ardupilot/libraries/AC_InputManager/AC_InputManager_Heli.cpp#L62)
`get_pilot_desired_collective(int16_t control_in)`

`control_in` is the raw RC3 value in the range **0..1000**.

In **Acro mode** (no expo), the conversion is linear:
```
collective_out = control_in / 1000.0
```

In **Stabilize mode** a 3-segment piecewise curve remaps the stick range using
four percentage kneepoints (all expressed as % of total collective travel):

| Parameter        | Default | Meaning                                     |
|-----------------|---------|---------------------------------------------|
| `IM_STB_COL_1`  | —       | Collective % at stick 0% (bottom)           |
| `IM_STB_COL_2`  | —       | Collective % at stick 40%                   |
| `IM_STB_COL_3`  | —       | Collective % at stick 60%                   |
| `IM_STB_COL_4`  | —       | Collective % at stick 100% (top)            |

Segment slopes:
```
segment 0–40 % :  slope = (STB_COL_2 - STB_COL_1) / 0.4
segment 40–60%:  slope = (STB_COL_3 - STB_COL_2) / 0.2
segment 60–100%: slope = (STB_COL_4 - STB_COL_3) / 0.4
```

Output is blended between the acro curve and the stabilize curve via a
0.5-second ramp. Result: `collective_out` ∈ **[0.0, 1.0]**.

---

## 2. H_COL_MIN / H_COL_MAX: Scaling to Swashplate Coordinates

**Source:** [`AP_MotorsHeli_Single.cpp:415`](../../ardupilot/libraries/AP_Motors/AP_MotorsHeli_Single.cpp#L415)
inside `move_actuators()`

| Parameter   | Stored as         | Default | Units |
|-------------|------------------|---------|-------|
| `H_COL_MIN` | `_collective_min` | 1250    | µs PWM |
| `H_COL_MAX` | `_collective_max` | 1750    | µs PWM |

Conversion to swashplate collective value:
```cpp
// AP_MotorsHeli_Single.cpp:415-416
float collective_scalar     = (H_COL_MAX - H_COL_MIN) * 0.001f;
float collective_out_scaled = collective_out * collective_scalar
                            + (H_COL_MIN - 1000) * 0.001f;
```

With defaults (COL_MIN=1250, COL_MAX=1750):
```
collective_scalar     = (1750 - 1250) * 0.001 = 0.500
collective_out_scaled = collective_out * 0.500 + 0.250

  collective_out = 0.0  →  collective_out_scaled = 0.250   (≡ 1250 µs)
  collective_out = 0.5  →  collective_out_scaled = 0.500   (≡ 1500 µs)
  collective_out = 1.0  →  collective_out_scaled = 0.750   (≡ 1750 µs)
```

Interpretation: `collective_out_scaled` lives on a 0..1 axis where
0 = 1000 µs and 1 = 2000 µs. COL_MIN/COL_MAX carve out the sub-range
that the collective actually occupies.

### Blade-pitch angle parameters (informational, not used in mixing)

| Parameter        | Default  | Units | Meaning                               |
|-----------------|----------|-------|---------------------------------------|
| `H_COL_ANG_MIN` | −90 °    | deg   | Blade pitch angle at `H_COL_MIN` PWM  |
| `H_COL_ANG_MAX` | +90 °    | deg   | Blade pitch angle at `H_COL_MAX` PWM  |
| `H_COL_ZERO_THRST` | 0 °  | deg   | Blade pitch at zero thrust            |
| `H_COL_LAND_MIN`   | see def | deg   | Min blade pitch when landed           |
| `H_COL_HOVER`      | ~0.5    | 0..1  | Hover collective (0=COL_MIN,1=COL_MAX)|

---

## 3. H3-120 Swashplate Mixing Matrix

**Source:** [`AP_MotorsHeli_Swash.cpp:147`](../../ardupilot/libraries/AP_Motors/AP_MotorsHeli_Swash.cpp#L147)
`calculate_roll_pitch_collective_factors()` — case `SWASHPLATE_TYPE_H3_120`

```cpp
// AP_MotorsHeli_Swash.cpp:151-153
add_servo_angle(CH_1, -60.0, 1.0);
add_servo_angle(CH_2,  60.0, 1.0);
add_servo_angle(CH_3, 180.0, 1.0);
```

`add_servo_angle(num, angle_deg, collective)` expands via:
```cpp
// AP_MotorsHeli_Swash.cpp:181-186
rollFactor  = cos(radians(angle + 90))
pitchFactor = cos(radians(angle))
```

And `add_servo_raw` scales all cyclic factors by **0.45**:
```cpp
// AP_MotorsHeli_Swash.cpp:196-198
_rollFactor[num]       = roll  * 0.45
_pitchFactor[num]      = pitch * 0.45
_collectiveFactor[num] = collective          // always 1.0 for H3-120
```

Resulting mixing matrix for H3-120 (Motor1 left, Motor2 right, Motor3 rear):

| Servo | Azimuth | Roll factor | Pitch factor | Collective factor |
|-------|---------|-------------|-------------|-----------------|
| CH_1  | −60°    | +0.3897     | +0.2250     | 1.0             |
| CH_2  | +60°    | −0.3897     | +0.2250     | 1.0             |
| CH_3  | 180°    |  0.0000     | −0.4500     | 1.0             |

**Derivation:**

```
CH_1 (−60°):  roll = cos(30°)  * 0.45 = +0.866 * 0.45 = +0.3897
              pitch= cos(−60°) * 0.45 = +0.500 * 0.45 = +0.2250

CH_2 (+60°):  roll = cos(150°) * 0.45 = −0.866 * 0.45 = −0.3897
              pitch= cos(+60°) * 0.45 = +0.500 * 0.45 = +0.2250

CH_3 (180°):  roll = cos(270°) * 0.45 =  0.000 * 0.45 =  0.0000
              pitch= cos(180°) * 0.45 = −1.000 * 0.45 = −0.4500
```

### Mixing equation

**Source:** [`AP_MotorsHeli_Swash.cpp:213`](../../ardupilot/libraries/AP_Motors/AP_MotorsHeli_Swash.cpp#L213)

```cpp
// AP_MotorsHeli_Swash.cpp:232-238
_output[i] = rollFactor[i]       * roll
           + pitchFactor[i]      * pitch
           + collectiveFactor[i] * collective_out_scaled;

// rescale 0..1 → −1..+1 for PWM calculation
_output[i] = 2.0f * _output[i] - 1.0f;
```

### Phase angle compensation

| Parameter      | Default | Range    | Effect                                                  |
|---------------|---------|----------|---------------------------------------------------------|
| `H_SW_H3_PHANG` | 0°    | −30..30° | Subtracts from every servo's azimuth before computing factors |

### Collective direction

| Parameter    | Values          | Effect                                      |
|-------------|-----------------|---------------------------------------------|
| `H_SW_COL_DIR` | 0=Normal, 1=Rev | Reversed: `collective = 1 − collective` before mixing |

### Cyclic limiting

| Parameter   | Default | Units       | Effect                                              |
|-------------|---------|-------------|-----------------------------------------------------|
| `H_CYC_MAX` | 2500    | centidegrees | Limits combined roll+pitch vector to this fraction of 4500 before mixing |

```cpp
// AP_MotorsHeli_Single.cpp:371-376
if (norm(pitch, roll) > H_CYC_MAX / 4500.0) {
    ratio = (H_CYC_MAX / 4500.0) / norm(pitch, roll);
    roll  *= ratio;
    pitch *= ratio;
}
```

### Optional linearization

| Parameter   | Values         | Effect                                                                     |
|-------------|----------------|----------------------------------------------------------------------------|
| `H_SW_LIN_SVO` | 0=Off, 1=On | Applies `asin(0.766 * x) * 1.146` to each output to compensate servo arm rotation |

---

## 4. Normalized Output → PWM

**Source:** [`AP_MotorsHeli_Swash.cpp:272`](../../ardupilot/libraries/AP_Motors/AP_MotorsHeli_Swash.cpp#L272)
`rc_write(uint8_t chan, float swash_in)`

```cpp
// swash_in is the rescaled output in −1..+1
uint16_t pwm = (uint16_t)(1500 + 500 * swash_in);
SRV_Channels::set_output_pwm_trimmed(function, pwm);
```

| `swash_in` | Raw PWM |
|-----------|---------|
| −1.0      | 1000 µs |
|  0.0      | 1500 µs |
| +1.0      | 2000 µs |

---

## 5. SRV_Channel Trim & Endpoint Clamping

**Source:** [`SRV_Channel_aux.cpp:358`](../../ardupilot/libraries/SRV_Channel/SRV_Channel_aux.cpp#L358)
`set_output_pwm_trimmed(function, value)`

```cpp
// Normal (not reversed):
value2 = value - 1500 + servo_trim

// Reversed:
value2 = 1500 - value + servo_trim

// Clamped to [servo_min, servo_max]
output_pwm = constrain(value2, servo_min, servo_max)
```

### SRV_Channel parameters

| Parameter        | Default | Meaning                                               |
|-----------------|---------|-------------------------------------------------------|
| `SERVO#_MIN`    | 1100 µs | Lower endpoint the channel is clamped to              |
| `SERVO#_MAX`    | 1900 µs | Upper endpoint the channel is clamped to              |
| `SERVO#_TRIM`   | 1500 µs | Trim offset applied around the 1500 µs center         |
| `SERVO#_REVERSED` | 0   | Reverses direction of servo travel                    |

> **Important:** Swashplate servos override `servo_min`/`servo_max` to **1000/2000 µs** in
> `add_servo_raw()` (line 208) regardless of the user-set SERVO parameters. The servo range
> is also forced to 1000 counts (±500 from trim) via `SRV_Channels::set_range(function, 1000)`.
> This means SERVO#_MIN and SERVO#_MAX do not restrict swash servo travel; only SERVO#_TRIM
> and SERVO#_REVERSED take effect.

---

## 6. Complete Signal Flow Summary

```
RC3 stick (0..1000 µs)
    │
    ▼  AC_InputManager_Heli::get_pilot_desired_collective()
    │  3-segment curve (Stabilize) or linear (Acro)
    │  Parameters: IM_STB_COL_1/2/3/4, IM_ACRO_COL_EXP
    ▼
collective_out  [0.0 .. 1.0]
    │
    ▼  move_actuators()  AP_MotorsHeli_Single.cpp:415
    │  collective_scalar     = (H_COL_MAX - H_COL_MIN) * 0.001
    │  collective_out_scaled = collective_out * collective_scalar
    │                        + (H_COL_MIN - 1000) * 0.001
    │  Parameters: H_COL_MIN (def 1250 µs), H_COL_MAX (def 1750 µs)
    ▼
collective_out_scaled  [~0.25 .. ~0.75 at defaults]
    │
    ▼  AP_MotorsHeli_Swash::calculate()  Swash.cpp:213
    │  output[i] = rollFactor[i]*roll + pitchFactor[i]*pitch
    │            + collectiveFactor[i]*collective_out_scaled
    │  output[i] = 2*output[i] - 1          ← rescale to −1..+1
    │  Parameters: H_SW_TYPE=H3_120, H_SW_COL_DIR, H_SW_H3_PHANG
    │              H_CYC_MAX (def 2500 cd), H_SW_LIN_SVO
    ▼
swash_in[0..2]  [−1.0 .. +1.0]
    │
    ▼  rc_write()  Swash.cpp:272
    │  pwm = 1500 + 500 * swash_in
    ▼
raw_pwm  [1000 .. 2000 µs]
    │
    ▼  set_output_pwm_trimmed()  SRV_Channel_aux.cpp:358
    │  trimmed = pwm − 1500 + SERVO#_TRIM  (or reversed)
    │  clamped to [servo_min=1000, servo_max=2000]  ← forced by swash setup
    │  Parameters: SERVO1/2/3_TRIM (def 1500), SERVO#_REVERSED
    ▼
final PWM output to servo hardware
```

---

## 7. All Relevant Parameters (Reference)

### Collective range
| Parameter       | Default  | Units | Description                          |
|----------------|----------|-------|--------------------------------------|
| `H_COL_MIN`    | 1250     | µs    | PWM at minimum collective pitch      |
| `H_COL_MAX`    | 1750     | µs    | PWM at maximum collective pitch      |
| `H_COL_ANG_MIN`| −90.0    | deg   | Blade angle at H_COL_MIN (must be set by user) |
| `H_COL_ANG_MAX`| +90.0    | deg   | Blade angle at H_COL_MAX (must be set by user) |
| `H_COL_ZERO_THRST` | 0.0 | deg   | Blade pitch at zero thrust           |
| `H_COL_LAND_MIN` | param  | deg   | Min pitch when landed                |
| `H_COL_HOVER`  | ~0.5     | 0..1  | Hover collective, learned or set     |
| `H_HOVER_LEARN`| 2        | enum  | 0=disabled, 1=learn, 2=learn+save    |

### Swashplate type & mixing
| Parameter         | Default | Description                               |
|------------------|---------|-------------------------------------------|
| `H_SW_TYPE`      | 3 (H3_120) | 0=H3 generic, 1=H1, 2=H3_140, 3=H3_120, 4=H4_90, 5=H4_45 |
| `H_SW_COL_DIR`   | 0 (Normal) | 0=Normal, 1=Reversed                   |
| `H_SW_LIN_SVO`   | 0       | 0=Off, 1=Linearize servo mechanical throw |
| `H_SW_H3_ENABLE` | 0       | Auto-set when H3 generic type selected    |
| `H_SW_H3_SV1_POS`| −60°   | Servo 1 azimuth (H3 generic only)         |
| `H_SW_H3_SV2_POS`| +60°   | Servo 2 azimuth (H3 generic only)         |
| `H_SW_H3_SV3_POS`| 180°   | Servo 3 azimuth (H3 generic only)         |
| `H_SW_H3_PHANG`  | 0°      | Phase angle compensation (all H3 types)   |
| `H_CYC_MAX`      | 2500 cd | Maximum cyclic authority (centidegrees)   |

### Input manager / pilot curve
| Parameter         | Default | Description                                   |
|------------------|---------|-----------------------------------------------|
| `IM_STB_COL_1`   | —       | Stabilize collective % at stick 0%            |
| `IM_STB_COL_2`   | —       | Stabilize collective % at stick 40%           |
| `IM_STB_COL_3`   | —       | Stabilize collective % at stick 60%           |
| `IM_STB_COL_4`   | —       | Stabilize collective % at stick 100%          |
| `IM_ACRO_COL_EXP`| 0       | Expo factor for acro collective (0=disabled)  |

### SRV_Channel (per swash servo, e.g. SERVO1..SERVO3)
| Parameter       | Default | Description                                      |
|----------------|---------|--------------------------------------------------|
| `SERVO#_MIN`   | 1100 µs | Lower clamp endpoint (overridden to 1000 for swash) |
| `SERVO#_MAX`   | 1900 µs | Upper clamp endpoint (overridden to 2000 for swash) |
| `SERVO#_TRIM`  | 1500 µs | Trim offset applied in `set_output_pwm_trimmed`  |
| `SERVO#_REVERSED` | 0    | Reverses PWM travel direction                    |

### Servo manual override
| Parameter    | Values | Description                                       |
|-------------|--------|---------------------------------------------------|
| `H_SV_MAN`  | 0=Auto, 1=Passthrough, 2=Max coll, 3=Zero thrust, 4=Min coll | Manual swash override for setup |
| `H_SV_TEST` | 0–10   | Number of boot-up servo test cycles               |

---

## 8. Handling Rotated or Mirrored Servos

There is no single "rotate swash by N degrees" parameter. Instead ArduPilot
provides several independent tools that combine to accommodate any physical
servo orientation.

### 8.1 `SERVO#_REVERSED` — flip a single servo

**Source:** [`SRV_Channel_aux.cpp:366`](../../ardupilot/libraries/SRV_Channel/SRV_Channel_aux.cpp#L366)

Setting `SERVO#_REVERSED = 1` for a swash servo inverts its motion around its
trim point inside `set_output_pwm_trimmed()`:

```
Normal:   output = pwm − 1500 + SERVO#_TRIM
Reversed: output = 1500 − pwm + SERVO#_TRIM
```

Effect: a servo that would move to 1700 µs instead moves to 1300 µs (for
TRIM=1500). Use this when a servo is physically installed upside-down or
link geometry inverts its direction.

> The clamping to [1000, 2000] is applied **after** the reversal, so the full
> servo travel is still used.

### 8.2 `H_SW_COL_DIR` — invert collective on all servos simultaneously

**Source:** [`AP_MotorsHeli_Swash.cpp:222`](../../ardupilot/libraries/AP_Motors/AP_MotorsHeli_Swash.cpp#L222)

```cpp
if (_collective_direction == COLLECTIVE_DIRECTION_REVERSED) {
    collective = 1 - collective;
}
```

This flips the collective contribution for the **entire swashplate** before
the mixing equation runs. Use this when the swash moves down for high collective
due to mechanical linkage arrangement (e.g. HR3-120 vs H3-120).

| `H_SW_COL_DIR` | Effect                                                    |
|----------------|----------------------------------------------------------|
| 0 (Normal)     | collective increases → all servos move toward their max  |
| 1 (Reversed)   | collective increases → all servos move toward their min  |

### 8.3 `H_SW_H3_PHANG` — phase angle correction (H3 types only)

**Source:** [`AP_MotorsHeli_Swash.cpp:125`](../../ardupilot/libraries/AP_Motors/AP_MotorsHeli_Swash.cpp#L125)

The phase angle is **subtracted** from every servo's azimuth before the
`cos()` factors are computed:

```cpp
// For H3_120:
add_servo_angle(CH_1, -60.0 - phase_angle, 1.0);
add_servo_angle(CH_2,  60.0 - phase_angle, 1.0);
add_servo_angle(CH_3, 180.0 - phase_angle, 1.0);
```

A positive `H_SW_H3_PHANG` rotates the cyclic control frame **clockwise**
when viewed from above. Use this when pitching the swash forward induces an
unwanted roll (common when the rotor head has mechanical phase lag).

| Phase angle | Practical effect                                          |
|-------------|----------------------------------------------------------|
| 0° (default)| Standard orientation                                     |
| +N°         | Cyclic authority rotated N° clockwise (corrects forward-pitch-induces-right-roll) |
| −N°         | Cyclic authority rotated N° counter-clockwise            |

Range: −30° to +30°.

### 8.4 `H_SW_TYPE = H3 Generic` with custom `H3_SV#_POS` — arbitrary servo placement

**Source:** [`AP_MotorsHeli_Swash.cpp:122`](../../ardupilot/libraries/AP_Motors/AP_MotorsHeli_Swash.cpp#L122)

When `H_SW_TYPE = 0` (H3 Generic), each servo's azimuth is freely configurable:

| Parameter        | Default | Description                                |
|-----------------|---------|--------------------------------------------|
| `H_SW_H3_SV1_POS` | −60°  | Azimuth of servo 1 (0° = nose direction)   |
| `H_SW_H3_SV2_POS` | +60°  | Azimuth of servo 2                         |
| `H_SW_H3_SV3_POS` | 180°  | Azimuth of servo 3                         |

The mixing factors are recalculated from these angles using the same
`add_servo_angle()` function. Any 3-servo layout can be described this way.
Phase angle correction (`H_SW_H3_PHANG`) also applies.

### 8.5 HR3-120 (mirror image of H3-120)

There is no separate `HR3_120` type. The source comment at
[`AP_MotorsHeli_Swash.cpp:149`](../../ardupilot/libraries/AP_Motors/AP_MotorsHeli_Swash.cpp#L149) states:

> *"HR3-120 uses reversed servo and collective direction in heli setup"*

Configure it by:
1. Set `H_SW_TYPE = 3` (H3_120) — same azimuth table.
2. Set `H_SW_COL_DIR = 1` (Reversed) — flips collective for the whole plate.
3. Set `SERVO#_REVERSED = 1` on each servo that runs in the wrong direction.

The combination of collective direction reversal and individual servo reversals
reconstructs the mirror-image mixing without needing a separate swash type.

### 8.6 Servos at non-standard or mirrored azimuth positions

When the physical swashplate has servos at different azimuth positions from
the H3-120 standard (−60°, +60°, 180°), the mixing matrix must be updated to
match. The correct approach is **H3 Generic mode** (`H_SW_TYPE = 0`) with the
three `H3_SV#_POS` parameters set to the actual measured positions.

**How to measure azimuth position:** Standing behind the helicopter looking
forward, 0° points straight to the nose. Angles increase clockwise when viewed
from above (i.e. right side is positive). The azimuth is the angle from the
nose to the servo attachment point on the swashplate ring.

**Example: standard H3-120 vs left-right mirror**

```
Standard H3-120:            Mirrored (H3-120 plate installed reversed):
  SV1 = −60° (left-front)     SV1 = +60° (right-front)
  SV2 = +60° (right-front)    SV2 = −60° (left-front)
  SV3 = 180° (rear)           SV3 = 180° (rear)
```

With the mirrored layout, plugging in `H3_SV1_POS = +60` and
`H3_SV2_POS = −60` swaps the roll factors of CH_1 and CH_2:

```
Standard SV1 (−60°): rollFactor = cos(30°)  * 0.45 = +0.3897
Mirrored SV1 (+60°): rollFactor = cos(150°) * 0.45 = −0.3897
```

The pitch and collective factors are unchanged for these positions. The net
effect is roll reversal: commanding roll-right drives CH_1 down and CH_2 up
instead of up and down. Without correcting `H3_SV#_POS`, the helicopter will
roll the wrong way.

**Example: swashplate rotated by 30°**

If the entire plate is rotated 30° clockwise from the standard layout:
```
SV1 = −30°  (was −60°)
SV2 = +90°  (was +60°)
SV3 = 210°  (was 180°)
```

Set `H_SW_TYPE = 0` and:
```
H_SW_H3_SV1_POS = −30
H_SW_H3_SV2_POS = +90
H_SW_H3_SV3_POS = 210
```

The mixing factors recompute automatically:
```
SV1 (−30°): roll = cos(60°)*0.45 = +0.225,  pitch = cos(−30°)*0.45 = +0.390
SV2 (+90°): roll = cos(180°)*0.45 = −0.450, pitch = cos(90°)*0.45  =  0.000
SV3 (210°): roll = cos(300°)*0.45 = +0.225, pitch = cos(210°)*0.45 = −0.390
```

**Important:** When using H3 Generic, `H_SW_H3_PHANG` is still subtracted from
every angle before the cosine is computed. Set `PHANG = 0` unless you need
additional phase correction on top of the corrected positions.

**Procedure for any non-standard layout:**
1. Set `H_SW_TYPE = 0`.
2. Measure the actual azimuth of each servo attachment point.
3. Set `H_SW_H3_SV1_POS`, `H_SW_H3_SV2_POS`, `H_SW_H3_SV3_POS` to those angles.
4. Use `H_SW_COL_DIR` if collective moves the wrong direction.
5. Use `SERVO#_REVERSED` only if a single servo's link is mechanically inverted
   (not for correcting swapped-position errors — use the `SV#_POS` params for that).

### 8.7 Decision guide

| Symptom                                       | Fix                              |
|----------------------------------------------|----------------------------------|
| One servo moves opposite to expected direction | `SERVO#_REVERSED = 1`           |
| All servos correct but collective inverted    | `H_SW_COL_DIR = 1`              |
| Pitch input causes roll (or vice versa)       | Adjust `H_SW_H3_PHANG`          |
| Servos at non-standard azimuth positions      | `H_SW_TYPE=0` + set `H3_SV#_POS` to measured angles |
| Plate left-right mirrored (e.g. SV1↔SV2 swapped) | `H_SW_TYPE=0`, swap SV1/SV2 azimuth values |
| Entire plate rotated N degrees                | `H_SW_TYPE=0`, add N to all `H3_SV#_POS` values |
| Mirror-image head (HR3-120 manufacturer label)| H3_120 + `COL_DIR=1` + per-servo reversal, or H3 Generic with measured angles |
