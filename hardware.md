# Hardware Stack

## Communication Flow

```
MissionPlanner (PC) ──SiK Radio V3────────────────> Pixhawk
Boxer Controller    ──ExpressLRS 2.4GHz──> RP3-H ──> Pixhawk
Pixhawk             ────────────────────────────> REVVitRC ESC ──> GB4008 Motor
```

---

## Motor — EMAX GB4008 Brushless Gimbal Motor

- **URL:** https://emaxmodel.com/products/emax-gb4008-brushless-gimbal-motor-hollow-shaft-66kv-90t
- **Type:** Brushless gimbal motor, hollow shaft
- **KV Rating:** 66KV
- **Turns:** 90T
- **Pole/Stator config:** 24N22P
- **Resistance:** 7.5 ohms
- **Cable gauge:** 0.19mm
- **Weight:** 101g
- **Mounting:** 19×19mm or 25×25mm base center pitch
- **Notes:** Hollow shaft design allows mechanical linkage through the center; optimized for smooth, low-speed gimbal control

---

## ESC — REVVitRC 50A AM32

- **URL:** https://justcuzrobotics.com/products/revvitrc-50a-am32-esc
- **Continuous current:** 50A
- **Voltage range:** 3–6S (9–26V)
- **Dimensions:** 44×18×14mm
- **Weight:** ~23g (without motor wires)
- **Firmware:** AM32 (native port, not converted from BlHeli32)
- **Built-in BEC:** 7.4V or 8.4V (switchable), can power receivers and servos
- **Telemetry:** Yes — voltage, current, temperature
- **Cutoffs:** Low voltage, current limit, thermal
- **USB programmer:** Included
- **Default mode:** Brushless Weapon (single direction, braking disabled)
- **Notes:** Optimized for low-end torque and sensorless startup; suited for 3lb weapon motors and 12lb drive systems

### AM32 Firmware

- **URL:** https://am32.ca/ — https://github.com/am32-firmware/AM32
- **Motor protocols:** Servo PWM, DSHOT 300, DSHOT 600
- **Bi-directional DSHOT:** Supported
- **Telemetry:** KISS standard ESC telemetry
- **Variable PWM frequency:** Yes
- **Sinusoidal startup mode:** Yes (for larger motor smooth acceleration)
- **Supported MCUs:** STSPIN32F0, STM32F051, STM32G071, GD32E230, AT32F415, AT32F421
- **Configuration tools:** Web-based AM32 Configurator, desktop (Windows/Linux), Online-ESC Configurator (WebSerial)
- **Update methods:** Betaflight passthrough, single-wire serial, Arduino

---

## Servos S1, S2, S3 — DS113MG V6.0 Digital Metal Gear Micro Servo

- **Type:** Digital, metal gear, micro
- **Role:** Tilt lower swashplate ring (collective + cyclic control)
- **Count:** 3× (3-point support on lower swashplate ring)
- **Power:** 8.0V via UBEC through Pixhawk servo rail

---

## Flight Controller — Pixhawk

- **Ecosystem:** ArduPilot / MissionPlanner
- **Ground control software:** MissionPlanner (PC)

---

## Telemetry — Holybro SiK Telemetry Radio V3

- **URL:** https://holybro.com/products/sik-telemetry-radio-v3
- **Purpose:** Bidirectional MAVLink link between MissionPlanner (PC) and Pixhawk
- **Frequency options:** 433MHz or 915MHz
- **Power options:** 100mW (20 dBm) or 500mW (27 dBm)
- **Range:** 300m+ typical
- **Receive sensitivity:** −121 dBm @ 2kbps / −105 dBm @ 64kbps
- **RF data rates:** 2–250 kbps (default 64 kbps)
- **Dimensions:** 28×53×10.7mm (without antenna)
- **Weight:** 23.5g (with antenna)
- **Antenna connector:** RP-SMA female
- **Interface:** 3.3V UART via 6-pin JST-GH
- **USB:** Micro-USB (Type-C adapter included)
- **Power input:** 5V DC; TX 100mA, RX 25mA
- **Operating temp:** −40°C to 85°C
- **Protocol:** MAVLink, standard framing

---

## RC Receiver — RadioMaster RP3-H ExpressLRS 2.4GHz Nano

- **URL:** https://radiomasterrc.com/products/rp3-h-expresslrs-2-4ghz-nano-receiver
- **Protocol:** ExpressLRS 2.4GHz
- **MCU:** ESP8285
- **RF Chip:** SX1281
- **Frequency range:** 2.404–2.479 GHz
- **Telemetry TX power:** Max 100mW
- **Antennas:** 2× 150mm 2.4GHz antenna (diversity)
- **Refresh rate:** 25Hz–500Hz / F1000Hz
- **Working voltage:** 4.5–8.4V DC
- **Battery voltage detection:** 4.0–35V DC
- **Bus interface:** CRSF / S.Bus
- **Dimensions:** 28×24×8mm
- **Weight:** 5.1g (with antennas)
- **Firmware:** ExpressLRS v3.3.1 (pre-installed)

---

## RC Transmitter — RadioMaster Boxer (M2)

- **URL:** https://radiomasterrc.com/products/boxer-radio-controller-m2
- **Firmware:** EdgeTX
- **RF system:** ExpressLRS 2.4GHz internal module (M2 variant)
- **Frequency:** 2.400–2.480 GHz
- **Channels:** Up to 16 (receiver dependent)
- **Display:** 128×64 monochrome LCD
- **Gimbals:** Full-size V4.0 Hall effect (upgradable to AG01 CNC Hall)
- **Battery:** 2S 7.4V LiPo or 2× 18650 Li-Ion (not included)
- **Voltage range:** 6.6–8.4V DC
- **Dimensions:** 235×178×77mm
- **Weight:** 532.5g
