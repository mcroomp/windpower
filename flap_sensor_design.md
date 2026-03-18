# Flap Sensor Design — RAWES Bench Measurement System

## Purpose

Characterise the swashplate → push-rod → flap deflection chain on the physical hardware.
Mounts on the rotating hub. Measures all 4 trailing-edge flap angles simultaneously via WiFi
while the rotor is rotated by hand. Data is used to:

- Verify the sinusoidal relationship: `γ(θ) = γ₀ + A·cos(θ − Ψ)`
- Extract the gain `Δγ / Δδ` (flap angle per degree of swashplate tilt) for each servo command
- Detect push-rod length mismatches, binding, or asymmetry between blades

---

## Component List

| Qty | Part | Model | Approx Cost | Notes |
|-----|------|-------|-------------|-------|
| 1 | Microcontroller | LOLIN32 Lite (ESP32) | €6 | Built-in LiPo connector + charger |
| 1 | I2C multiplexer | TCA9548A breakout | €2 | Adafruit #2717 or AliExpress equivalent |
| 4 | IMU sensor | MPU-6050 breakout (GY-521) | €4 | One per flap |
| 1 | Battery | LiPo 3.7V 500mAh JST-PH 2.0 | €3 | ~5 hrs runtime |
| 1 | Wire | 28AWG 4-conductor ribbon cable, 10m | €2 | Runs along blade underside |
| — | Misc | Heatshrink, zip ties, double-sided foam tape | €1 | Mounting and cable management |

**Total: ~€18**

---

## Power Design

```
LiPo 3.7V 500mAh
       │
       │ JST-PH 2.0
       ▼
  LOLIN32 Lite
  (onboard charging + 3.3V regulator)
       │
       │ 3.3V regulated out  (~200mA total draw)
       ├──► TCA9548A VIN
       ├──► MPU-6050 VCC ×4  (via TCA9548A channel wiring)
       └──► LOLIN32 internal (WiFi ~150mA peak)

Charging: plug USB-C into LOLIN32 while assembled.
          LiPo charges at ~500mA. No disassembly needed.
```

| Device | Current draw |
|--------|-------------|
| LOLIN32 (WiFi active) | ~150mA avg, 250mA peak |
| TCA9548A | <5mA |
| MPU-6050 ×4 | ~15mA total |
| **Total** | **~170mA avg** |

500mAh LiPo → ~3 hours continuous use.

---

## Wiring Diagram

```
                       LOLIN32 Lite
                      ┌─────────────────┐
            LiPo ──── │ JST             │
                       │                 │
                       │ 3.3V ───────────┼────────────────────────── 3.3V rail
                       │ GND  ───────────┼────────────────────────── GND rail
                       │                 │
                       │ GPIO21 (SDA) ───┼──┐
                       │ GPIO22 (SCL) ───┼──┼──┐
                       └─────────────────┘  │  │
                                            │  │
                            TCA9548A        │  │
                           ┌────────────┐   │  │
                    3.3V ──│ VIN        │   │  │
                     GND ──│ GND        │   │  │
                           │ SDA ───────│───┘  │
                           │ SCL ───────│──────┘
                           │ A0  ───────│── GND  ┐
                           │ A1  ───────│── GND  ├── I2C address 0x70
                           │ A2  ───────│── GND  ┘
                           │            │
                           │ SD0/SC0 ───│────────────────► MPU-6050 Blade 1
                           │ SD1/SC1 ───│────────────────► MPU-6050 Blade 2
                           │ SD2/SC2 ───│────────────────► MPU-6050 Blade 3
                           │ SD3/SC3 ───│────────────────► MPU-6050 Blade 4
                           └────────────┘


Each MPU-6050 (identical wiring ×4):

                           ┌─────────────┐
                   3.3V ───│ VCC         │
                    GND ───│ GND         │
        TCA9548A SDx ───── │ SDA         │
        TCA9548A SCx ───── │ SCL         │
                    GND ───│ AD0         │  (I2C address 0x68 — same for all,
                           │ INT         │   OK because each is on a separate
                           └─────────────┘   TCA9548A channel)

Cable to each MPU-6050: 4 wires — 3.3V, GND, SDA, SCL
```

---

## I2C Address Map

| Device | Address | Set by |
|--------|---------|--------|
| TCA9548A | 0x70 | A0/A1/A2 all GND |
| MPU-6050 Blade 1 | 0x68 | AD0 GND, on channel 0 |
| MPU-6050 Blade 2 | 0x68 | AD0 GND, on channel 1 |
| MPU-6050 Blade 3 | 0x68 | AD0 GND, on channel 2 |
| MPU-6050 Blade 4 | 0x68 | AD0 GND, on channel 3 |

All MPU-6050s share address 0x68 — this is fine because the TCA9548A ensures only
one channel is active at a time.

---

## Physical Layout on Hub

```
              Blade 1 (0°)
                  │
      MPU-6050 ───┤ ← glued flat on flap surface, wires along blade underside
                  │
                  │ 2m cable run
                  │
Blade 4 (270°) ──[HUB]── Blade 2 (90°)
                  │
                  │   ┌─────────────────────────┐
                  │   │  LOLIN32 Lite            │
                  │   │  TCA9548A (stacked below)│  ← mounted centrally on hub top
                  │   │  LiPo (below board)      │     with double-sided foam tape
                  │   └─────────────────────────┘
                  │
              Blade 3 (180°)
```

### Mounting notes

- Mount LOLIN32 + TCA9548A + LiPo as a single stack at the hub centre, directly on the
  rotor spin axis. Central mounting minimises rotational imbalance.
- TCA9548A can be stacked directly under the LOLIN32 using header pins.
- LiPo sits flat underneath, secured with double-sided foam tape.
- USB-C port on LOLIN32 should face outward for easy access when charging.

---

## Cable Routing

```
Hub centre                                          Blade tip (~2m)
    │                                                     │
    │── 4-wire ribbon ──────────────────────────────── MPU-6050
         secured with zip ties every 150mm
         along blade underside (non-lifting surface)
         small service loop (~50mm) at blade root
         to absorb flex during rotation
```

Wire colours (suggested):
| Wire | Colour |
|------|--------|
| 3.3V | Red |
| GND | Black |
| SDA | Blue |
| SCL | Yellow |

At the hub, 4 bundles (one per blade) converge. Cable-tie the bundle to the hub body
before splitting to the TCA9548A.

---

## Measurement Procedure

### Setup

1. Assemble hub with all 4 blades and flaps fitted
2. Connect LOLIN32 to PC via USB to configure WiFi SSID/password (first time only)
3. Power on — LOLIN32 connects to WiFi and starts streaming UDP
4. Open Python receiver script on PC — confirm data is arriving
5. Disconnect USB, verify WiFi stream continues (battery powered)

### Calibration (zero offset)

1. With swashplate level (all servos at neutral PWM), hold hub stationary
2. Record 2 seconds of data — this is the zero-offset for each sensor
3. Note: MPU-6050 accelerometer gives absolute tilt, so zero must be recorded per-session

### Measurement run

1. Command a fixed swashplate tilt via MissionPlanner servo output test
   (e.g. S1=1600µs, S2=1400µs, S3=1500µs for a specific tilt direction)
2. Rotate hub slowly by hand — one full revolution in ~5–10 seconds
3. Keep rotation speed roughly constant — no need to stop at specific angles
4. Python script records: `timestamp, blade1_angle, blade2_angle, blade3_angle, blade4_angle`
5. Repeat for several tilt magnitudes (e.g. ±100µs, ±200µs, ±300µs servo offset)

### Analysis

Fit a cosine to each blade's angle vs time:

```
γ_i(t) = γ₀ + A·cos(2π·t/T + φ)
```

- `A` = flap amplitude for that servo command
- `φ` = phase (should be 90° apart between adjacent blades)
- `γ₀` = mean offset (should be ~0 at neutral)

Plot `A` vs servo PWM offset → this is your `Δγ / Δδ` gain curve.

---

## Data Format

UDP packets (ASCII, newline terminated):

```
timestamp_ms,blade1_deg,blade2_deg,blade3_deg,blade4_deg
1234,2.31,-1.45,3.12,-2.89
1254,2.45,-1.32,3.28,-2.71
```

CSV file saved by Python receiver:

```
flap_data_YYYYMMDD_HHMMSS.csv
```

---

## Software Stack

```
LOLIN32 (Arduino sketch)          PC (Python script)
─────────────────────────         ──────────────────────────────
setup():                          sock = socket.socket(UDP)
  Wire.begin(21, 22)              sock.bind(('', 4210))
  tca.begin()
  mpu[0..3].begin()               while True:
  WiFi.begin(ssid, pass)            data = sock.recv(256)
  udp.begin(4210)                   row = parse(data)
                                    writer.writerow(row)
loop() every 20ms:
  angles = []
  for ch in 0..3:
    tca.select(ch)
    angles.append(mpu[ch].getAngle())
  udp.send(timestamp + angles)
```

---

## Anomaly Detection (post-measurement)

With all 4 blades logged, check for:

| Check | Expected | Indicates if wrong |
|-------|----------|--------------------|
| All 4 amplitudes equal | `A₁ = A₂ = A₃ = A₄` | Push-rod length mismatch |
| Phase spacing | 90° between adjacent blades | Blade mounting angle error |
| Linearity | `A` proportional to servo PWM offset | Linkage binding or slop |
| Symmetry | Same `A` for +tilt and −tilt | Swashplate geometry asymmetry |
| Noise | Smooth cosine, low residual | Loose push-rod connection |

---

## Known Limitations

- **Bench only at high RPM**: Cable runs along 2m blades are fine for slow hand rotation.
  At operating speed (35 rad/s), centrifugal force on the cable and connectors needs
  proper strain relief. Not designed for in-flight use as-is.
- **Static angle only**: MPU-6050 accelerometer measures static tilt. Valid for slow
  rotation (bench). At speed, centrifugal acceleration dominates — gyroscope integration
  would be needed for dynamic measurement.
- **Flap → blade pitch not captured**: This rig measures flap deflection angle only.
  The aerodynamic conversion from flap angle to blade pitch requires airflow and must
  be characterised separately (in-flight step response or wind tunnel).
