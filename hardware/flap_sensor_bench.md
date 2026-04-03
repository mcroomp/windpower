# Flap Sensor Design -- RAWES Bench Measurement System

## Purpose

Characterise the swashplate -> push-rod -> flap deflection chain on the physical hardware.
Mounts on the rotating hub. Measures all 4 trailing-edge flap angles simultaneously via WiFi
while the rotor is rotated by hand. Data is used to:

- Verify the sinusoidal relationship: gamma(theta) = gamma_0 + A*cos(theta - Psi)
- Extract the gain delta_gamma / delta_delta (flap angle per degree of swashplate tilt) for each servo command
- Detect push-rod length mismatches, binding, or asymmetry between blades

---

## Component List

| Qty | Part | Model | Approx Cost | Notes |
|-----|------|-------|-------------|-------|
| 1 | Microcontroller | LOLIN32 Lite (ESP32) | 6 EUR | Built-in LiPo connector + charger |
| 1 | I2C multiplexer | TCA9548A breakout | 2 EUR | Adafruit #2717 or AliExpress equivalent |
| 4 | IMU sensor | MPU-6050 breakout (GY-521) | 4 EUR | One per flap |
| 1 | Battery | LiPo 3.7V 500mAh JST-PH 2.0 | 3 EUR | ~5 hrs runtime |
| 1 | Wire | 28AWG 4-conductor ribbon cable, 10m | 2 EUR | Runs along blade underside |
| -- | Misc | Heatshrink, zip ties, double-sided foam tape | 1 EUR | Mounting and cable management |

**Total: ~18 EUR**

---

## Power Design

```
LiPo 3.7V 500mAh
       |
       | JST-PH 2.0
       v
  LOLIN32 Lite
  (onboard charging + 3.3V regulator)
       |
       | 3.3V regulated out  (~200mA total draw)
       +---> TCA9548A VIN
       +---> MPU-6050 VCC x4  (via TCA9548A channel wiring)
       +---> LOLIN32 internal (WiFi ~150mA peak)

Charging: plug USB-C into LOLIN32 while assembled.
          LiPo charges at ~500mA. No disassembly needed.
```

| Device | Current draw |
|--------|-------------|
| LOLIN32 (WiFi active) | ~150mA avg, 250mA peak |
| TCA9548A | <5mA |
| MPU-6050 x4 | ~15mA total |
| **Total** | **~170mA avg** |

500mAh LiPo -> ~3 hours continuous use.

---

## Wiring Diagram

```
                       LOLIN32 Lite
                      +-----------------+
            LiPo ---- | JST             |
                       |                 |
                       | 3.3V -----------+------------------------------ 3.3V rail
                       | GND  -----------+------------------------------ GND rail
                       |                 |
                       | GPIO21 (SDA) ---+-+
                       | GPIO22 (SCL) ---+-+-+
                       +-----------------+  | |
                                            | |
                            TCA9548A        | |
                           +------------+   | |
                    3.3V --| VIN        |   | |
                     GND --| GND        |   | |
                           | SDA -------+---+ |
                           | SCL -------+-----+
                           | A0  -------+-- GND  +
                           | A1  -------+-- GND  +-- I2C address 0x70
                           | A2  -------+-- GND  +
                           |            |
                           | SD0/SC0 ---+---------> MPU-6050 Blade 1
                           | SD1/SC1 ---+---------> MPU-6050 Blade 2
                           | SD2/SC2 ---+---------> MPU-6050 Blade 3
                           | SD3/SC3 ---+---------> MPU-6050 Blade 4
                           +------------+


Each MPU-6050 (identical wiring x4):

                           +-------------+
                   3.3V ---| VCC         |
                    GND ---| GND         |
        TCA9548A SDx ----- | SDA         |
        TCA9548A SCx ----- | SCL         |
                    GND ---| AD0         |  (I2C address 0x68 -- same for all,
                           | INT         |   OK because each is on a separate
                           +-------------+   TCA9548A channel)

Cable to each MPU-6050: 4 wires -- 3.3V, GND, SDA, SCL
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

All MPU-6050s share address 0x68 -- this is fine because the TCA9548A ensures only
one channel is active at a time.

---

## Physical Layout on Hub

```
              Blade 1 (0 deg)
                  |
      MPU-6050 ---+ <- glued flat on flap surface, wires along blade underside
                  |
                  | 2m cable run
                  |
Blade 4 (270 deg)--[HUB]-- Blade 2 (90 deg)
                  |
                  |   +-------------------------+
                  |   |  LOLIN32 Lite            |
                  |   |  TCA9548A (stacked below)|  <- mounted centrally on hub top
                  |   |  LiPo (below board)      |     with double-sided foam tape
                  |   +-------------------------+
                  |
              Blade 3 (180 deg)
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
    |                                                     |
    +-- 4-wire ribbon -------------------------------- MPU-6050
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
3. Power on -- LOLIN32 connects to WiFi and starts streaming UDP
4. Open Python receiver script on PC -- confirm data is arriving
5. Disconnect USB, verify WiFi stream continues (battery powered)

### Calibration (zero offset)

1. With swashplate level (all servos at neutral PWM), hold hub stationary
2. Record 2 seconds of data -- this is the zero-offset for each sensor
3. Note: MPU-6050 accelerometer gives absolute tilt, so zero must be recorded per-session

### Measurement run

1. Command a fixed swashplate tilt via MissionPlanner servo output test
   (e.g. S1=1600us, S2=1400us, S3=1500us for a specific tilt direction)
2. Rotate hub slowly by hand -- one full revolution in ~5-10 seconds
3. Keep rotation speed roughly constant -- no need to stop at specific angles
4. Python script records: timestamp, blade1_angle, blade2_angle, blade3_angle, blade4_angle
5. Repeat for several tilt magnitudes (e.g. +-100us, +-200us, +-300us servo offset)

### Analysis

Fit a cosine to each blade's angle vs time:

```
gamma_i(t) = gamma_0 + A*cos(2*pi*t/T + phi)
```

- A = flap amplitude for that servo command
- phi = phase (should be 90 deg apart between adjacent blades)
- gamma_0 = mean offset (should be ~0 at neutral)

Plot A vs servo PWM offset -> this is your delta_gamma / delta_delta gain curve.

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
---------------------------------  ----------------------------------
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
| All 4 amplitudes equal | A1 = A2 = A3 = A4 | Push-rod length mismatch |
| Phase spacing | 90 deg between adjacent blades | Blade mounting angle error |
| Linearity | A proportional to servo PWM offset | Linkage binding or slop |
| Symmetry | Same A for +tilt and -tilt | Swashplate geometry asymmetry |
| Noise | Smooth cosine, low residual | Loose push-rod connection |

---

## Manual Measurement Procedure (Digital Level)

Simpler alternative to the automated sensor rig. Requires only a digital level,
MissionPlanner connected to the Pixhawk, and a marker on the hub for azimuth reference.
Suitable for initial characterisation -- ~20 minutes of work.

### Equipment

- Digital level / inclinometer
- MissionPlanner connected to Pixhawk via USB or telemetry
- Marker pen or tape to mark azimuth positions on hub (0, 90, 180, 270 deg)
- Notebook or spreadsheet to record readings

### Step 1 -- Mark azimuth positions

With the hub assembled, mark 4 positions on the outer rim corresponding to each blade
root: 0, 90, 180, 270 deg. These are the positions you will rotate to for each reading.

```
        Blade 1
          |  <- mark "0" here on hub rim
          |
Blade 4 --*-- Blade 2
   270     |    90
          |
        Blade 3
         180
```

### Step 2 -- Zero reference (swashplate level)

1. Set all servos to neutral in MissionPlanner (S1=S2=S3=1500us)
2. Verify swashplate is level -- place digital level on lower swashplate ring and confirm ~0 deg
3. Rotate hub to each azimuth position and record flap angle at each

| Azimuth | Flap angle (neutral) |
|---------|---------------------|
| 0 deg   | ___ deg             |
| 90 deg  | ___ deg             |
| 180 deg | ___ deg             |
| 270 deg | ___ deg             |

These are your zero offsets. Subtract them from all subsequent readings.

### Step 3 -- Measure flap response vs servo command

Command a fixed tilt via MissionPlanner servo output test. Suggested test points:

| Test | S1 (us) | S2 (us) | S3 (us) | Description |
|------|---------|---------|---------|-------------|
| A    | 1600    | 1500    | 1500    | S1 only, +100us |
| B    | 1700    | 1500    | 1500    | S1 only, +200us |
| C    | 1400    | 1500    | 1500    | S1 only, -100us |
| D    | 1300    | 1500    | 1500    | S1 only, -200us |
| E    | 1500    | 1600    | 1500    | S2 only, +100us |
| F    | 1500    | 1500    | 1600    | S3 only, +100us |

For each test:
1. Enter servo PWM values in MissionPlanner
2. Rotate hub to 0 deg -- hold still -- read and record flap angle
3. Rotate to 90 deg -- hold still -- read and record
4. Repeat for 180 and 270 deg
5. Subtract zero offsets from step 2

### Step 4 -- Record sheet

```
Test | S1   | S2   | S3   | 0 deg | 90 deg | 180 deg | 270 deg
-----|------|------|------|-------|--------|---------|--------
 A   | 1600 | 1500 | 1500 | ___ deg  | ___ deg  | ___ deg  | ___ deg
 B   | 1700 | 1500 | 1500 | ___ deg  | ___ deg  | ___ deg  | ___ deg
 C   | 1400 | 1500 | 1500 | ___ deg  | ___ deg  | ___ deg  | ___ deg
 D   | 1300 | 1500 | 1500 | ___ deg  | ___ deg  | ___ deg  | ___ deg
 E   | 1500 | 1600 | 1500 | ___ deg  | ___ deg  | ___ deg  | ___ deg
 F   | 1500 | 1500 | 1600 | ___ deg  | ___ deg  | ___ deg  | ___ deg
```

### Step 5 -- Analysis

For each test row, the 4 readings should follow a cosine:

```
gamma(theta) = A * cos(theta - Psi)
```

Quick manual check -- the values at 0 and 180 deg should be equal and opposite,
and the values at 90 and 270 deg should be equal and opposite:

```
gamma(0 deg)  ~ -gamma(180 deg)
gamma(90 deg) ~ -gamma(270 deg)
```

The amplitude A is:

```
A = (gamma(0 deg) - gamma(180 deg)) / 2
```

Plot A vs servo PWM offset for each servo -> this is your delta_gamma / delta_delta gain.
Linearity check: doubling the PWM offset should double A.

### What to look for

| Observation | Meaning |
|-------------|---------|
| gamma(0) ~ -gamma(180) and gamma(90) ~ -gamma(270) | Geometry is symmetric -- good |
| One azimuth reads significantly different from expected | Push-rod length error or binding at that position |
| A not proportional to PWM offset | Nonlinearity in linkage -- check for slop or binding |
| S2 and S3 tests give different A for same PWM offset | Unequal servo arm lengths or attachment radii |
| All readings near zero regardless of servo command | Push-rods not connected or swashplate not moving |

---

## ArduPilot Configuration

This section lists the ArduPilot parameters that must be set, and which ones
come directly from the manual measurements above.

Reference: ArduPilot Traditional Helicopter setup wiki -- verify exact parameter
names and function numbers against your ArduCopter firmware version.

### Step 1 -- Frame type

```
FRAME_CLASS = 11        # Traditional Helicopter
FRAME_TYPE  = 0         # (default)
```

### Step 2 -- Swashplate geometry

```
H_SWASH_TYPE = 0        # H3-120: 3 servos equally spaced at 120 deg
```

Verify by checking the physical servo mounting angles on the lower swashplate ring.

### Step 3 -- Servo output wiring

| Pixhawk output | Connects to | ArduPilot role |
|----------------|-------------|----------------|
| MAIN OUT 1 | S1 servo | Swashplate servo 1 |
| MAIN OUT 2 | S2 servo | Swashplate servo 2 |
| MAIN OUT 3 | S3 servo | Swashplate servo 3 |
| MAIN OUT 8 | REVVitRC ESC | RSC (GB4008 counter-torque motor) |

### Step 4 -- Servo calibration

```
SERVO1_MIN      = 1000   # us -- safe minimum
SERVO1_MAX      = 2000   # us -- safe maximum
SERVO1_TRIM     = 1500   # us -- neutral (verify: swashplate level at this value)

SERVO2_MIN      = 1000
SERVO2_MAX      = 2000
SERVO2_TRIM     = 1500

SERVO3_MIN      = 1000
SERVO3_MAX      = 2000
SERVO3_TRIM     = 1500
```

SERVO_REVERSED -- set from measurement data:
After entering trim values, command a positive collective in MissionPlanner
(H_SV_MAN=1, increase collective slider). All 3 servos should move in the same direction.
If any servo moves the wrong way, set SERVO1/2/3_REVERSED = 1.

### Step 5 -- Collective pitch range

```
H_COL_MIN = 1250    # us -- minimum collective (flat/negative pitch)
H_COL_MAX = 1750    # us -- maximum collective (positive pitch)
H_COL_MID = 1500    # us -- zero collective (neutral, matches SERVO_TRIM)
```

### Step 6 -- Cyclic pitch range

This value comes directly from the manual measurement:

```
H_CYC_MAX = 4.0     # degrees -- start conservative, increase after testing
```

How to set it:
1. From your measurement: note the PWM offset that gives a safe maximum flap deflection
2. Convert flap angle to approximate blade pitch using the model ratio from the Weyel 2025
   model (CM_gamma / CM_beta ~= -0.5/0.002 -> ~250:1 reduction, refined from in-flight ID)
3. Start conservatively at 4.0 degrees and increase only after verifying stable flight

### Step 7 -- RSC configuration (wind-driven rotor)

```
H_RSC_MODE     = 0      # Disabled -- no rotor speed control
H_RSC_SETPOINT = 0      # Not used
H_RSC_RUNUP_TIME = 0    # No runup wait on arming
```

The GB4008 ESC (REVVitRC) is driven manually via RC channel or fixed PWM to maintain
counter-torque. This is separate from the ArduPilot RSC system.

### Step 8 -- Verify with H_SV_MAN

```
H_SV_MAN = 1           # Enable manual servo test mode
```

In MissionPlanner -> Initial Setup -> Mandatory Hardware -> Heli Setup:
- Move collective slider -> all 3 servos should rise/fall together
- Move roll cyclic -> swashplate should tilt left/right
- Move pitch cyclic -> swashplate should tilt fore/aft
- Verify direction matches physical expectation from measurement data

Set back to normal after verification: H_SV_MAN = 0

### Parameter summary -- what comes from measurements

| Parameter | Source |
|-----------|--------|
| SERVO1/2/3_TRIM | Manual measurement Step 2 (neutral PWM) |
| SERVO1/2/3_REVERSED | Manual measurement -- servo direction check |
| H_COL_MIN/MAX | Mechanical travel limits from measurement |
| H_CYC_MAX | Amplitude A from measurement Step 5 analysis |
| H_SWASH_TYPE | Physical servo mounting geometry |
| H_RSC_MODE = 0 | RAWES specific -- wind drives rotor, not motor |

---

## Known Limitations

- **Bench only at high RPM**: Cable runs along 2m blades are fine for slow hand rotation.
  At operating speed (35 rad/s), centrifugal force on the cable and connectors needs
  proper strain relief. Not designed for in-flight use as-is.
- **Static angle only**: MPU-6050 accelerometer measures static tilt. Valid for slow
  rotation (bench). At speed, centrifugal acceleration dominates -- gyroscope integration
  would be needed for dynamic measurement.
- **Flap -> blade pitch not captured**: This rig measures flap deflection angle only.
  The aerodynamic conversion from flap angle to blade pitch requires airflow and must
  be characterised separately (in-flight step response or wind tunnel).
