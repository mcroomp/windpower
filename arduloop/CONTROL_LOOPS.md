# ArduLoop Control Loop & PID Architecture

A visual guide to the two-layer attitude control stack ported from ArduPilot.

---

## High-Level Block Diagram

```mermaid
flowchart TD
    subgraph LUA["rawes.lua (50 Hz)"]
        CMD["set_target_angle_and_climbrate\n(roll, pitch, yaw, climbrate)"]
    end

    subgraph OUTER["GuidedAttitudeController — OUTER LOOP (400 Hz)"]
        QC["_q_commanded\n(from Lua)"]
        AT["_attitude_target\n(slewed internal)"]
        AVT["_ang_vel_target\n(feedforward)"]

        SHAPE["input_shaping_angle (per axis)\nsqrt_controller + accel limit\nATC_INPUT_TC from .parm"]
        QC --> SHAPE
        AT --> SHAPE
        SHAPE -->|"updates"| AVT

        subgraph RUNQUAT["attitude_controller_run_quat"]
            TVRA["1. thrust_vector_rotation_angles\n → att_error [roll, pitch, yaw]"]
            SQRT["2. sqrt_controller per axis\nP = ATC_ANG_*_P (from .parm)\n→ P_correction (rad/s)"]
            FF["3. feedforward =\nrot_t2b × _ang_vel_target"]
            BLEND["4. Blend by thrust_error_angle:\n• < 30°: full FF all axes\n• 30–60°: partial, yaw→gyro\n• > 60°: no roll/pitch FF"]
            TVRA --> SQRT --> BLEND
            FF --> BLEND
        end

        SHAPE --> RUNQUAT
        AVT --> FF
    end

    subgraph INNER["HeliRateController — INNER LOOP (400 Hz)"]
        PID_R["AC_PID Roll"]
        PID_P["AC_PID Pitch"]
        PID_Y["AC_PID Yaw"]
        PIRO["PIRO_COMP\nRotate roll/pitch I-terms\nby yaw_rate × dt"]
        TRIM["Hover Roll Trim\nroll += trim × collective"]
        SWASH["SwashH3 Phase Rotation\nH_SW_H3_PHANG"]
        CLIP["Output Clipping ±1"]

        PID_R --> PIRO
        PID_P --> PIRO
        PID_Y --> PIRO
        PIRO --> TRIM --> SWASH --> CLIP
    end

    subgraph PLANT["Physical Plant"]
        SERVO["Servos / Swashplate / Rotor"]
    end

    CMD --> QC
    BLEND -->|"rate_target\n(roll, pitch, yaw) rad/s"| PID_R
    BLEND --> PID_P
    BLEND --> PID_Y
    CLIP -->|"roll_cyclic"| SERVO
    CLIP -->|"pitch_cyclic"| SERVO
    CLIP -->|"yaw_cmd"| SERVO
    SERVO -.->|"gyro (body rates)"| TVRA
    SERVO -.->|"gyro"| PID_R
    SERVO -.->|"gyro"| PID_P
    SERVO -.->|"gyro"| PID_Y
```

---

## Per-Axis AC_PID Signal Flow (Detail)

```mermaid
flowchart LR
    subgraph TP["Target Path"]
        RT[/"rate_target"/] --> NTF["NTF Notch\n(optional)"]
        NTF --> FLTT["FLTT LPF\n(20 Hz)"]
        FLTT --> TGT["_target"]
    end

    subgraph EP["Error Computation"]
        TGT2["_target"] --> DIFF[" − "]
        GYRO[/"gyro_rate"/] --> DIFF
        DIFF --> NEF["NEF Notch\n(3.77 Hz)"]
        NEF --> FLTE["FLTE LPF"]
        FLTE --> ERR["_error"]
    end

    subgraph GAINS["PID Gains"]
        ERR2["_error"] --> P_GAIN["P = Kp × _error"]
        ERR2 --> D_GAIN["D = Kd × FLTD(d_error/dt)"]
        ERR2 --> I_GAIN["I = ∫ Ki × _error dt\n(±IMAX, anti-windup)"]
        TGT3["_target"] --> FF_GAIN["FF = Kff × _target"]
        TGT3 --> DFF_GAIN["DFF = Kdff × d_target/dt"]
    end

    subgraph OUT["Output Stage"]
        PDMX["PDMX Limiter\nscale P+D if exceeds"]
        SUM["Σ = P + D + I + FF + DFF"]
        RESULT[/"axis_command"/]
        P_GAIN --> PDMX
        D_GAIN --> PDMX
        PDMX --> SUM
        I_GAIN --> SUM
        FF_GAIN --> SUM
        DFF_GAIN --> SUM
        SUM --> RESULT
    end

    TGT --> TGT2
    TGT --> TGT3
    ERR --> ERR2
```

---

## Outer Loop: Attitude Error Decomposition

```mermaid
flowchart TD
    AT["_attitude_target (q)"] --> TVRA
    QB["q_body (measured)"] --> TVRA

    TVRA["thrust_vector_rotation_angles()\n───────────────────────\nbody_z_target vs body_z_actual\ncross product → rotation axis\n→ roll/pitch error (body frame)\nremaining rotation → yaw error"]

    TVRA --> AE["att_error = roll, pitch, yaw"]
    AE --> SQRT["sqrt_controller (per axis)\n───────────────────────\nSmall error: rate = error × P\nLarge error: rate = √(2 × a × e)\n───────────────────────\nP = ATC_ANG_*_P (from .parm)\na = ATC_ACC_*_MAX (fallback ATC_ACCEL_*)"]

    SQRT --> PC["P_correction (rad/s)"]
    AVT["_ang_vel_target"] --> ROT["rot_t2b × _ang_vel_target"]
    ROT --> BFF["blended feedforward"]

    PC --> ADD["( + )"]
    BFF --> ADD
    ADD --> RATE["rate_target → HeliRateController"]
```

---

## Input Shaping: How _attitude_target Slews

```mermaid
sequenceDiagram
    participant Lua as rawes.lua (50 Hz)
    participant Cmd as _q_commanded
    participant Shape as input_shaping_angle
    participant AT as _attitude_target
    participant AVT as _ang_vel_target

    Lua->>Cmd: set_target(roll, pitch, yaw)
    Note over Cmd: Jumps instantly

    loop Every 400 Hz tick
        Shape->>Shape: error = axis_angle(AT⁻¹ × Cmd)
        Shape->>Shape: desired_vel = sqrt_controller(error, 1/TC, ACCEL_MAX)
        Shape->>AVT: _ang_vel_target = accel_clamp(desired_vel)
        AVT->>AT: _attitude_target *= from_rotvec(AVT × dt)
    end

    Note over AT: Converges to _q_commanded
    Note over AT: ~1.5 s for 65 deg step
```

---

## Module Dependency Graph

```mermaid
flowchart TD
    G["guided.py\nGuidedAttitudeController"] -->|owns| H["attitude_heli.py\nHeliRateController"]
    H -->|owns| PID["pid.py\nAC_PID (×3 axes)"]
    H -->|owns| SW["swash.py\nSwashH3"]
    PID -->|owns| F["filters.py\nLowPassFilter1p\nNotchFilter"]
```

---

## Parameter Quick Reference

### Outer Loop (GuidedAttitudeParams)

| Parameter | Default | Role |
|-----------|---------|------|
| `ATC_ANG_RLL_P` | loaded from .parm | Attitude P-gain: rad/s per rad error |
| `ATC_ANG_PIT_P` | loaded from .parm | Same for pitch |
| `ATC_ANG_YAW_P` | loaded from .parm | Same for yaw |
| `ATC_ACCEL_R_MAX` | loaded from .parm | AP 4.7 `ATC_ACC_R_MAX`, fallback `ATC_ACCEL_R_MAX` |
| `ATC_ACCEL_P_MAX` | loaded from .parm | AP 4.7 `ATC_ACC_P_MAX`, fallback `ATC_ACCEL_P_MAX` |
| `ATC_ACCEL_Y_MAX` | loaded from .parm | AP 4.7 `ATC_ACC_Y_MAX`, fallback `ATC_ACCEL_Y_MAX` |
| `ATC_RATE_R/P/Y_MAX` | 0 (unlimited) | Max body rate cap (deg/s) |
| `ATC_INPUT_TC` | loaded from .parm | Slew time constant |

### Inner Loop (RateAxisParams per axis)

| Parameter | Roll/Pitch | Yaw | Role |
|-----------|-----------|-----|------|
| `P` | loaded from .parm | loaded from .parm | Proportional gain |
| `I` | loaded from .parm | loaded from .parm | Integral gain |
| `D` | loaded from .parm | loaded from .parm | Derivative gain |
| `FF` | loaded from .parm | loaded from .parm | Feedforward on target |
| `IMAX` | loaded from .parm | loaded from .parm | Integrator clamp |
| `FLTT` | loaded from .parm | loaded from .parm | Target signal LPF |
| `FLTE` | 0 Hz | 0 Hz | Error signal LPF |
| `FLTD` | loaded from .parm | loaded from .parm | Derivative LPF |
| `NEF` | 3.77 Hz | — | Error notch (tether spring) |
| `NTF` | — | — | Target notch |
| `PDMX` | 0 | 0 | PD sum limit (0=off) |

### Heli-Specific

| Parameter | Default | Role |
|-----------|---------|------|
| `H_SW_H3_PHANG` | 0 deg | Swashplate phase angle |
| `ATC_HOVR_ROL_TRM` | 0 cd | Collective-scaled roll trim |
| `ATC_PIRO_COMP` | Off | Rotate I-terms with yaw rate |

---

## Timing & Execution Context

| Layer | Rate | Caller | Output |
|-------|------|--------|--------|
| rawes.lua target command | 50 Hz | Lua `update()` | quaternion target |
| GuidedAttitudeController.update | 400 Hz | Physics runner | rate targets |
| HeliRateController.update | 400 Hz | (inside above) | cyclic + yaw |
| Plant / Servo actuation | 400 Hz | Physics dynamics | torques |

---

## Key Design Insight: Why Two Quaternion States?

ArduPilot maintains **two** attitude quaternions:

1. **`_q_commanded`** — what Lua asked for (jumps instantly at 50 Hz)
2. **`_attitude_target`** — internal slewed target (advances smoothly at 400 Hz)

The **attitude error** used for the P-loop is computed between `_attitude_target`
and `q_body` (actual), NOT between `_q_commanded` and `q_body`. This means:

- Large commanded steps don't produce huge instant rate demands
- The input shaping acts as a reference-model prefilter
- Feedforward (`_ang_vel_target`) provides the expected rate while slewing
- The P-loop only corrects tracking error relative to the slewed reference

This is critical for RAWES: the 65° tether tilt means initial commanded angles
are large, and without slewing the rate PIDs would instantly saturate.
