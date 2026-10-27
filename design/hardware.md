# RAWES Hardware Reference

Single owner for the current airframe: geometry, control hardware, swash and
servo mapping, and the yaw-motor/DShot path. Every value below is backed by the
file named next to it; **numeric parameter values live in the parm files, not
here**:

- shared and SITL defaults: [rawes_common_defaults.parm](../tests/sitl/rawes_common_defaults.parm)
- hardware-only overrides: [rawes_hardware_defaults.parm](../hardware/rawes_hardware_defaults.parm)
- rotor schema and derived inertia: [beaupoil_2026.yaml](../simulation/rotor_definitions/beaupoil_2026.yaml)
- calibration constants: [constants.py](../calibrate/constants.py)

Arming and safe-off procedures are owned by [arming.md](arming.md); the
calibration workflow by [calibration.md](calibration.md).

## Rotor geometry

From the active rotor definition:

| Quantity | Value | Evidence |
|---|---:|---|
| Blade count | 4 | `n_blades` |
| Root cutout radius | 0.5 m | `root_cutout_m` |
| Modeled radial span (cutout to tip) | 2.0 m | `radius_m - root_cutout_m` |
| Physical foam-blade length in the description | 1.5 m | rotor definition description; not the modeled radial span |
| Total rotor radius | 2.5 m | `radius_m` |
| Chord | 0.20 m | `chord_m` |
| Twist | 0° | `twist_deg` |

The rotor definition describes a 1.5 m foam blade on a 1.0 m boom; the model
treats 0.5–2.5 m from the hub centre as the aerodynamic span (2.0 m). The
definition asks for tape-measure confirmation of the physical dimensions.

## Components

| Component | Identity | Evidence |
|---|---|---|
| Flight controller | Pixhawk 6C | [HARDWARE_STARTUP.md](../HARDWARE_STARTUP.md) |
| Swash servos | DS113MG V6.0 class (simulated slew 545 deg/s, travel 100 deg) | `servo_slew_rate_deg_s`, `servo_travel_deg` in the rotor definition |
| Yaw motor | EMAX GB4008, 66 RPM/V, 22 poles, 10:1 gear | `GB4008_KV`, `GB4008_POLES`, `GB4008_GEAR_RATIO` |
| Yaw ESC | REVVitRC 50A, AM32 firmware, bidirectional DShot | [rawes_common_defaults.parm](../tests/sitl/rawes_common_defaults.parm) DShot block |

Vendor datasheet data (dimensions, weights, radio specs, power budgets) is not
maintained in this repository.

## Swashplate and servos

The physical layout is H3-120, implemented with ArduPilot's `H_SW_TYPE` H3-120
mode. Servos 1-3 are mapped to Motor1-Motor3 and reversed for the physical
airframe. Trims, collective endpoints (`H_COL_MIN/MAX`, `H_COL_ANG_MIN/MAX`),
and `H_CYC_MAX` are measured airframe values; read them from
[rawes_hardware_defaults.parm](../hardware/rawes_hardware_defaults.parm).
Swash geometry and sign mapping are implemented in
[swashplate.py](../simulation/swashplate.py).

The browser's calculated S1/S2/S3 envelope covers both draft collective
endpoints and the full configured cyclic circle for H3-120. ArduPilot limits
the cyclic vector to `H_CYC_MAX/4500` before the 0.45 mixer gain (equivalent
to +/- `H_CYC_MAX/10` microseconds per servo before trim and linearization).
The estimate includes loaded collective direction, servo trims/reversal,
optional swash linearization, and output-endpoint clipping. Reload refreshes
those dependencies; unsupported mappings or missing calibration show
unavailable, not assumed defaults. It is not measured travel or proof of
mechanical clearance; the safety procedure remains in [arming.md](arming.md).

## Yaw motor and DShot telemetry

The GB4008 anti-rotation motor is Motor4 on AUX 1 (output 9),
`SERVO9_FUNCTION=36`, driven through the DDFP tail path (`H_TAIL_TYPE`) with
bidirectional DShot and ESC RPM telemetry. The `SERVO_BLH_*`, `SERVO_DSHOT_*`,
`RPM1_*`, and `BRD_IO_DSHOT` values are the DShot block in
[rawes_common_defaults.parm](../tests/sitl/rawes_common_defaults.parm).

Wiring: AUX 1 signal and ground to the ESC signal and signal ground; power the
ESC from the battery path, not the servo rail.

RPM conversion: eRPM / (poles / 2) gives motor RPM; dividing again by the 10:1
gear ratio gives rotor RPM. The pole count must match `SERVO_BLH_POLES`.

Output 9 must stay mapped at boot (see the safe-off invariant in
[arming.md](arming.md)).

**SITL exclusion.** ArduCopter-heli SITL does not compile the BLHeli backend and
drives output 9 as plain PWM, so the stack harness excludes only the names in
`SITL_UNSUPPORTED_PARAMS` in [stack_utils.py](../tests/sitl/stack_utils.py).
The exact list is maintained in
[sitl_testing.md](sitl_testing.md#dshotblheli-parameters-excluded-from-sitl-verification).
Every other parameter, including `SERVO9_FUNCTION`, is still verified.

To validate on hardware: confirm the output mapping and DShot masks against the
parm file, then check that `watch esc` in `calibrate` shows RPM updating while
the motor runs.

## Rotor-spin inertia

`I_spin` is derived from blade mass plus the spinning hub shell only when the
rotor definition leaves `I_spin_kgm2: null`
([rotor_physics.py](../simulation/rotor_physics.py)). The stationary assembly is
excluded; the simulation-side treatment is in [simulation.md](simulation.md).
