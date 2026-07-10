# Motor Control - GB4008 + REVVitRC AM32

This document is the current-state reference for yaw-motor control and RPM telemetry.
It intentionally avoids migration history and focuses on active configuration.

## Active Configuration

The GB4008 anti-rotation motor runs on AUX 1 (output 9) as Motor4 with bidirectional DShot.

| Parameter | Value | Purpose |
|---|---|---|
| H_TAIL_TYPE | 3 | DDFP CW yaw-motor path |
| SERVO9_FUNCTION | 36 | Motor4 mapped to output 9 |
| SERVO9_MIN | 1000 | Motor off |
| SERVO9_MAX | 2000 | Motor full throttle |
| SERVO9_TRIM | 1000 | Idle |
| SERVO4_FUNCTION | 0 | Output 4 released |

## DShot + RPM Telemetry

| Parameter | Value | Purpose |
|---|---|---|
| SERVO_BLH_MASK | 256 | Register output 9 with BLHeli/DShot subsystem |
| SERVO_BLH_BDMASK | 256 | Enable bidirectional DShot on output 9 |
| SERVO_BLH_AUTO | 0 | Manual mask configuration |
| SERVO_BLH_OTYPE | 5 | DShot300 |
| SERVO_BLH_POLES | 22 | Correct pole count for RPM conversion |
| SERVO_DSHOT_ESC | 1 | AM32 RPM telemetry decode |
| SERVO_DSHOT_RATE | 0 | 1 kHz command rate |
| RPM1_TYPE | 5 | ESC telemetry source |
| RPM1_ESC_MASK | 256 | Route output-9 ESC telemetry to RPM1 |
| BRD_IO_DSHOT | 0 | FMU output path |

## Yaw Controller Interface

The yaw motor command path is governed by heli yaw settings and the active Lua/AP control mode.
For stack behavior and control ownership see [flight_stack.md](flight_stack.md).

## Wiring Summary

Use the FMU PWM connector for output 9 (AUX 1):

- Signal: AUX 1 signal pin to ESC signal
- Ground: AUX 1 ground pin to ESC signal ground
- Power: ESC powered from battery path (not servo rail)

## RPM Conversion Notes

| Constant | Value |
|---|---|
| Motor Kv | 66 RPM/V |
| Gear ratio | 10:1 |
| Pole count | SERVO_BLH_POLES = 22 |
| eRPM -> motor RPM | divide by (poles/2) |
| eRPM -> rotor RPM | divide by (poles/2 * gear_ratio) |

## Validation Checklist

1. Confirm output mapping (`SERVO9_FUNCTION=36`, `SERVO4_FUNCTION=0`).
2. Confirm DShot masks (`SERVO_BLH_MASK=256`, `SERVO_BLH_BDMASK=256`).
3. Confirm ESC telemetry routing (`RPM1_TYPE=5`, `RPM1_ESC_MASK=256`).
4. Verify `watch esc` in calibrate shows RPM updates while motor runs.

## Related Docs

- [calibration.md](calibration.md) - calibrate.py commands and workflows
- [flight_stack.md](flight_stack.md) - control ownership and mode behavior
- [components.md](components.md) - hardware components and constraints
