# calibrate — Hardware Calibration Tool

`calibrate` is an HTTP-only client of LinkHub. LinkHub owns the Pixhawk
serial/UDP connection, MAVLink parsing and transmission, journaling, stateful
MAVLink transactions, DataFlash transfer, MAVFTP, and the optional Bluetooth
rotor-motor link. `calibrate` owns operator workflows and RAWES-specific bench
sequencing on top of that API.

This document owns calibration commands and operator workflows. Current
arm/disarm, passive-startup, and safe-off policy is owned by
[arming.md](arming.md). LinkHub transport, journal, and HTTP contract are owned
by [linkhub.md](linkhub.md). The repository ownership map lives at
[README.md#documentation-map](../README.md#documentation-map). Chronological
hardware evidence belongs in [HARDWARE_STARTUP.md](../HARDWARE_STARTUP.md).

## Connection

```bash
python -m calibrate
python -m calibrate <verb> [args]
```

Top-level options are:

```text
--server URL
--connection COMx
--baud N
--motor-name-prefix PREFIX
--force | -f
```

When the default local endpoint (`http://127.0.0.1:8999`) is not running,
`calibrate` can start the release LinkHub binary automatically. LinkHub itself
performs serial discovery and heartbeat confirmation; `--connection` and
`--baud` only restrict which candidates it probes. `--server` points to an
already-running LinkHub instance.

Per [arming.md](arming.md), the first hardware operation in a session should
omit `--connection` and `--baud` so LinkHub can auto-detect the live Pixhawk
connection. Reuse the detected link only for later commands in that same
session.

LinkHub reports readiness after the first heartbeat. The main service and
vehicle-status endpoints used by calibration are:

- `GET /v1/status`
- `GET /v1/mavlink/status`
- `GET /v1/mavlink/messages`
- `POST /v1/mavlink/commands`
- `GET|PUT /v1/mavlink/parameters`
- `GET|PUT /v1/mavlink/parameters/{name}`
- `GET|PUT|DELETE /v1/mavlink/files`
- `POST /v1/mavlink/directories`
- `GET /v1/mavlink/logs`
- `GET /v1/motor`, `PUT /v1/motor`, `POST /v1/motor/stop`, `POST /v1/motor/reconnect`
  when the optional Bluetooth motor backend is enabled

MAVLink enumerations and bitmasks are named values on LinkHub's wire, not
numbers. `calibrate` uses the typed `linkhub_client.messages` members
(`MavCmd`, `MavResult`, `MavModeFlag`, `MavState`, ...) and tests flags with
`flag in message.base_mode` rather than bit masks. Rejected arm/disarm commands
print the result name (for example `MAV_RESULT_FAILED`).

The hardware-focused environment is intentionally lightweight, so:

```bash
uv run python -m calibrate --help
```

starts quickly. Install the optional simulation environment only when needed:

```bash
uv sync --dev --extra simulation
```

The workstation uses `UV_NO_SYNC=1`, so `uv run` does not re-sync
automatically; run `uv sync` explicitly when dependencies change.

`--force` / `-f` skips interactive confirmation prompts. It does **not** weaken
cleanup: every exit path still goes through the canonical safe-off flow owned by
[arming.md](arming.md).

---

## Output channel mapping

| Output | Component | SERVO function |
|---|---|---|
| 1 | S1 — right-rear swashplate servo | 33 (`Motor1`) |
| 2 | S2 — left-rear swashplate servo | 34 (`Motor2`) |
| 3 | S3 — front/elevator swashplate servo | 35 (`Motor3`) |
| 9 | GB4008 anti-rotation motor (AUX 1) | 36 (`Motor4`) |

The physical layout is HR3-120. ArduPilot has no separate HR3 selector:
`H_SW_TYPE=3` selects H3-120, while `SERVO1/2/3_REVERSED=1` and
`H_SW_COL_DIR=1` implement the front-elevator arrangement. The Pixhawk arrow
and vehicle +X both point toward the centre of gravity (`AHRS_ORIENTATION=0`).
Hardware overrides live in
[`hardware/rawes_hardware_defaults.parm`](../hardware/rawes_hardware_defaults.parm).

Swashplate PWM range is 1000 µs (min) … 1500 µs (neutral) … 2000 µs (max), but
ArduPilot's heli mixer overwrites `SERVOn_MIN/MAX` on every output tick. Use
`swash range <min_us> <max_us>` to write `H_COL_MIN/H_COL_MAX`, which is the
actual travel limiter the heli mixer respects.

Motor PWM range is 1000 µs (off) … 2000 µs (full throttle). `SERVO9_MIN/MAX`
limits that output.

For the current hardware-specific swash trims, collective limits, and cyclic
limit, read
[`hardware/rawes_hardware_defaults.parm`](../hardware/rawes_hardware_defaults.parm)
rather than copying those values into workflow notes.

---

## CLI shape

```text
python -m calibrate [--server URL] [--connection COMx] [--baud N] [--force] <verb> [args...]
```

Two verbs are long-running and always write a CSV under
`simulation/logs/calibrate/`:

- `run`
- `watch`

Everything else is one-shot.

## `run <name>`

```text
run <name> [--duration N] [--force]
```

`run` selects a Lua mode, performs the required startup sequence, arms through
MAVLink, streams observation rows to console and CSV, and always exits through
canonical safe-off. ESC or Ctrl-C aborts cleanly. Without `--duration`, the run
is unbounded.

### Run modes

| CLI mode | `RAWES_MODE` used by calibrate | Notes |
|---|---:|---|
| `none` | 0 | Armed-but-quiet bench mode |
| `acro-manual` | 2 | ACRO flybar passthrough from normalized RAWES controls |
| `passive` | 3 | Passive hold entered through `ENTER_GUIDED` (31010) then `ENTER_PASSIVE` (31011) |
| `steady` | 1 | Steady flight / altitude-hold behavior |
| `pumping` | 1 | Ground-side pumping schedule; Lua still runs steady mode |
| `landing` | 4 | Reserved |

`rawes.lua` also defines `RAWES_MODE=5` (`takeoff`), but calibrate does not
expose a `run takeoff` CLI mode.

### Common options

- `--duration N` — bounded run duration in seconds.
- `--force` — bypass ArduPilot pre-arm checks for a secured bench. Startup,
  disarm, and safe-off behavior are unchanged.

### Passive-only options

| Option | Meaning |
|---|---|
| `--trim thr=V` | Seed `RAWES_THR` in thrust space `[0,1]` before arming |
| `--roll DEG` / `--pitch DEG` / `--yaw DEG` | Initial passive offsets relative to the Lua-captured anchor |
| `--protocol-debug` | Text-only passive protocol trace |
| `--auto-sequence` | Automatic bounded passive exercise |
| `--step-hold S` | Dwell per `--auto-sequence` step |
| `--settle-rate-deg-s N` | Maximum body rate allowed during passive qualification |
| `--settle-time S` | Continuous quiet interval required before passive capture |
| `--settle-timeout S` | Maximum time to wait for passive qualification |
| `--rotor-motor` | Connect the external Bluetooth rotor motor |
| `--rotor-speed N` | Initial external rotor-drive speed `0..100` |
| `--rotor-direction cw|ccw` | External rotor-drive direction |

Only `thr` is accepted in `--trim`; invalid keys are rejected.

For passive runs, `calibrate` also ensures `GUID_OPTIONS` bit 3 is enabled so
GUIDED attitude targets use thrust semantics rather than climb-rate semantics.

Current passive startup sequencing, landed-state handling, and safe-off policy
are owned by [arming.md](arming.md). The CLI-specific passive wire contract is
owned by [flight_stack.md](flight_stack.md).

Examples:

```bash
# Bench check: hold IC swashplate, observer active, 30 s
python -m calibrate --server http://127.0.0.1:8999 run passive --duration 30 --force --trim thr=0.342

# Steady-flight bench run (ESC to stop)
python -m calibrate --server http://127.0.0.1:8999 run steady

# Passive run with initial offsets relative to the captured anchor
python -m calibrate --server http://127.0.0.1:8999 run passive --duration 20 --trim thr=0.342 --roll 3 --pitch -25

# Passive protocol trace with the external Bluetooth rotor motor
python -m calibrate --server http://127.0.0.1:8999 run passive --rotor-motor --rotor-speed 10 --rotor-direction cw --protocol-debug
```

During `run passive --rotor-motor`, keyboard controls are:

- `m` — start/stop the external rotor motor
- `[` / `]` — decrease/increase speed by 5 percentage points
- `0` — emergency stop

Every exit path sends the safe-stop command before disconnecting Bluetooth and
performing the normal vehicle shutdown.

The Bluetooth helper can also be exercised independently:

```bash
uv run python -m calibrate.bldc_ble
uv run python -m calibrate.bldc_ble --run-seconds 5 --speed 10 --direction cw
```

## `watch <stream> [--duration N]`

`watch` is read-only. It never arms and never changes vehicle state. Default
duration is 10 s.

| Stream | Telemetry consumed | CSV columns |
|---|---|---|
| `servos` | `SERVO_OUTPUT_RAW` (requested through the RC stream) | `t_s`, `s1_us` … `s8_us` |
| `esc` | `ESC_TELEMETRY_*` | `t_s`, `erpm`, `mech_rpm`, `rotor_rpm`, `voltage_v`, `current_a`, `temp_c` |
| `text` | `STATUSTEXT` | `t_s`, `severity` (MAVLink name, e.g. `MAV_SEVERITY_INFO`), `text` |
| `attitude` | `ATTITUDE` | `t_s`, roll/pitch/yaw + body rates |
| `power` | `BATTERY_STATUS`, `SYS_STATUS` | `t_s`, `vbat_v`, `current_a`, `power_w` |

`watch servos` records outputs 1–8 only. For output 9 or full journal evidence,
use `status` or the LinkHub journal.

Examples:

```bash
python -m calibrate --server http://127.0.0.1:8999 watch servos --duration 15
python -m calibrate --server http://127.0.0.1:8999 watch attitude --duration 60
python -m calibrate --server http://127.0.0.1:8999 watch text
```

---

## One-shot verbs

### `status`

Print a live snapshot of:

- heartbeat-derived armed state, flight mode, and system status name,
- battery state,
- EKF status (named `EKF_*` flags),
- active `SERVO_OUTPUT_RAW` channels,
- key RAWES / motor-path / yaw-control parameters,
- `[DIFF]` markers against the shared parameter defaults.

### `set <name> <value>` / `get <name> [<name> ...]`

Read or write ArduPilot parameters. `get` accepts multiple names so one
connection can fetch a whole diagnostic group.

```bash
python -m calibrate --server http://127.0.0.1:8999 set H_COL_MAX 1700
python -m calibrate --server http://127.0.0.1:8999 get RAWES_MODE H_SV_MAN H_FLYBAR_MODE
```

### `config check [--all]` / `config fix [--all]`

Compare the live FC parameters against the canonical defaults loaded from:

- `tests/sitl/copter-heli.parm`
- `tests/sitl/rawes_common_defaults.parm`
- `hardware/rawes_hardware_defaults.parm`

Default scope is the RAWES common and hardware overrides. `--all` also includes
the full `copter-heli.parm` baseline.

- `config check` previews differences.
- `config fix` writes the differences through LinkHub's batch parameter API and
  then verifies them.

### `swash`

```text
swash <coll%> [lon%] [lat%]
swash range <min_us> <max_us>
swash neutral [n]
swash info
swash fit-range <min_us> <max_us> --cyclic N --allow-full-range
swash test [--duration N] --allow-full-range
swash test off
```

`swash` is for disarmed swash geometry checks and travel fitting. `swash test`
is the native `H_SV_MAN=5` oscillation path. Full-envelope oscillation requires
disconnected servos and explicit `--allow-full-range`.

### `servo`

```text
servo <ch> <pwm>
servo mode <name|0..5> [--duration N] [--allow-full-range]
servo sweep [--duration N]
servo hold <ch> <pwm> [--duration N]
```

Native mode names are:

- `automated` (0)
- `passthrough` (1)
- `max` (2)
- `zero` (3)
- `min` (4)
- `oscillate` (5)

Swash-channel raw commands temporarily disconnect the swash output functions and
restore them afterward. All servo setup commands require a disarmed vehicle.

### `motor <pwm_us> [--duration N]` / `motor off`

Drive the yaw-motor output through `MAV_CMD_DO_MOTOR_TEST` after arming. The
value is **PWM microseconds**, not a percentage. It must lie within the live
`SERVO9_MIN..SERVO9_MAX` range; commands above 5% of the configured span prompt
unless `--force` is set.

```bash
python -m calibrate --server http://127.0.0.1:8999 motor 1050 --duration 5
python -m calibrate --server http://127.0.0.1:8999 motor off
```

### `arm [--duration N] [--force]`

Arm for a bounded interval (default 5 s), then disarm and restore safe-off.
`--force` uses ArduPilot's force-arm magic value and explicitly bypasses pre-arm
checks.

### `disarm` / `reboot`

- `disarm` requests normal MAVLink disarm, falls back to forced disarm if the
  vehicle rejects a normal disarm from an active flight state, and then applies
  the canonical safe-off invariant owned by [arming.md](arming.md).
- `reboot` sends `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN`.

### `battery monitor off|on [type]`

Disable battery monitoring for USB-only bench work with `battery monitor off`.
Restore it with `battery monitor on`; the default type is `4` (analog voltage
and current). ArduPilot marks `BATT_MONITOR` reboot-required, so the command
reports that a reboot is needed but does not reboot automatically.

### `script upload <file>` / `script list` / `script remove <name>`

Upload, list, or remove Lua scripts under `/APM/scripts`. `upload` writes the
file through LinkHub's MAVFTP API and then toggles `SCR_ENABLE 1→0→1` to
restart the scripting engine.

Use `script upload` only over a positively verified direct Pixhawk USB
connection; do not deploy Lua over the radio.

### `logs list` / `logs fetch [--id N] [--dir D]`

LinkHub owns the full DataFlash transaction (`LOG_ENTRY`, chunk requests,
reassembly, retries, timeouts, `LOG_REQUEST_END`). `logs fetch` downloads the
latest log by default, or the selected `--id`, to
`simulation/logs/calibrate/` unless `--dir` is supplied.

---

## Logging and evidence

Every `run` and `watch` session writes a CSV under
`simulation/logs/calibrate/`. The header records the verb, mode/stream,
duration, timestamps, selected passive options, and the LinkHub start cursor.
At exit, calibration prints the corresponding LinkHub cursor range.

Calibration does **not** mirror MAVLink traffic into a local JSONL queue. The
LinkHub journal is the authoritative transport record. Use `linkhub query` over
that cursor range for RX/TX evidence, sparse message extraction, and diagnostics.
See [linkhub.md](linkhub.md) for journal semantics and query examples.

`run` additionally records the passive-handoff and controller evidence it needs:

- RAWES diagnostic `NAMED_VALUE_FLOAT`s (`YFF_*`, `OL_*`, etc.),
- `ATTITUDE_TARGET` and `PID_TUNING` when ArduPilot emits them,
- actual and target attitude quaternions,
- quaternion error metrics.

For passive runs, `ATTITUDE_QUATERNION` and `ATTITUDE_TARGET` are requested
explicitly with `MAV_CMD_SET_MESSAGE_INTERVAL`; they are not assumed to arrive
via legacy stream groups.

## Passive interactive controls

For `run passive`, the operator-facing controls are:

- Left/Right — relative roll ±5°
- Up/Down — relative pitch ±5°
- `,` / `.` — relative yaw ∓/±5°
- `-` / `=` — held thrust ∓/±0.05 within `[0,1]`
- Space — reset relative roll/pitch/yaw offsets to zero
- ESC — exit and restore safe-off

`--protocol-debug` suppresses the 3D view and prints the live protocol trace.
`--auto-sequence` performs a bounded passive exercise automatically.

## Agent and automation access

Agents and automation talk to LinkHub's HTTP API directly. There is no
calibration-specific MCP server or secondary transport owner. Generic transport
state lives under `/v1/mavlink`; RAWES-specific startup sequencing and safe-off
policy remain in the calibration client.

## Typical calibration sequence

```bash
# 0. Start or attach to LinkHub and inspect live state
python -m calibrate status

# 1. Diff against canonical params; apply if needed
python -m calibrate config check
python -m calibrate config fix
python -m calibrate reboot

# 2. Verify live state after reconnect
python -m calibrate status

# 3. Swashplate neutral + mixing check (disarmed)
python -m calibrate swash neutral
python -m calibrate swash 50 0 0
python -m calibrate swash 0 0 50

# 4. Limit swash travel if servos cannot take full range
python -m calibrate swash range 1300 1700
python -m calibrate set H_CYC_MAX 1000

# 5. Swash motion check
python -m calibrate servo sweep --duration 12

# 6. Motor spin check
python -m calibrate motor 1050 --duration 5
python -m calibrate watch esc --duration 10

# 7. On verified direct USB only, upload Lua and confirm it is present
python -m calibrate script upload scripts\rawes.lua
python -m calibrate script list

# 8. Quiet armed passive bench check
python -m calibrate run passive --duration 30 --trim thr=0.342
```
