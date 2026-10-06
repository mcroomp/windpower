# LinkHub

LinkHub is the Rust MAVLink transport and diagnostic gateway. It manages the
HTTP API, journal, MAVLink connection, and message-rate configuration exposed
by the current LinkHub binary.

[design/linkhub.md](../design/linkhub.md) owns architecture, journal
semantics, and API semantics. This README stays operational: how to build
LinkHub, launch it, generate the checked protocol artifacts, query a saved
journal, and call the live HTTP API.

## Build

From the repository root:

```powershell
cargo build --manifest-path .\linkhub\Cargo.toml --release
```

The executable is `.\linkhub\target\release\linkhub.exe`.

Bluetooth is excluded from ordinary builds. To include the optional motor
backend explicitly:

```powershell
cargo build --manifest-path .\linkhub\Cargo.toml --release --features bluetooth
```

LinkHub uses Mavio with its default features disabled. The
`mavio-dialect` path dependency generates a filtered typed dialect for the
messages LinkHub projects as structured JSON, plus a compact full
ArduPilotMega message ID/name/CRC registry. Other valid ArduPilotMega messages
remain checksum-validated and journaled with empty `fields`; add a message to
`mavio-dialect/build.rs` when LinkHub must expose its decoded fields or accept
its name in message-operation APIs.

For iterative development, run a subcommand directly:

```powershell
cargo run --manifest-path .\linkhub\Cargo.toml -- serve --connection tcp:127.0.0.1:5760
```

## Generate the checked protocol artifacts

`linkhub schema` can print the live JSON schema to stdout and/or refresh the
checked schema and generated client files:

```powershell
cargo run --manifest-path .\linkhub\Cargo.toml -- schema `
  --output .\linkhub\schema\protocol-v1.schema.json `
  --python-output .\linkhub_client\src\linkhub_client\generated_protocol.py `
  --typescript-output .\linkhub-ui\src\generated\protocol.ts
```

Supported flags:

| Flag | Meaning |
| --- | --- |
| `--output <path>` | Write the JSON schema instead of printing it. |
| `--python-output <path>` | Regenerate the Python protocol types. |
| `--typescript-output <path>` | Regenerate the TypeScript protocol types. |

## Run the service

### Common examples

SITL:

```powershell
.\linkhub\target\release\linkhub.exe serve `
  --connection tcp:127.0.0.1:5760 `
  --data-dir .\simulation\logs\linkhub
```

Hardware auto-discovery:

```powershell
.\linkhub\target\release\linkhub.exe serve `
  --connection auto `
  --data-dir .\simulation\logs\linkhub
```

Static UI serving:

```powershell
.\linkhub\target\release\linkhub.exe serve `
  --connection tcp:127.0.0.1:5760 `
  --data-dir .\simulation\logs\linkhub `
  --static-dir .\linkhub-ui\dist `
  --no-cache
```

### `serve` flags

Defaults come from `linkhub\src\main.rs`.

| Flag | Default | Meaning |
| --- | --- | --- |
| `--connection` | `auto` | `tcp:HOST:PORT`, bare `HOST:PORT`, `auto`, or a serial-port restriction such as `COM7` / `serial:COM7`. Serial values still go through heartbeat-based discovery. |
| `--listen` | `127.0.0.1` | HTTP bind address. |
| `--port` | `8999` | HTTP port. |
| `--data-dir` | `linkhub-data` | Parent directory where each run creates `<run-id>\run.json` and `<run-id>\journal\`. |
| `--static-dir` | unset | Optional directory served as fallback static assets. |
| `--no-cache` | off | Sends `Cache-Control: no-store` on every response. |
| `--run-id` | random UUID | Override the per-run UUID. |
| `--flush-ms` | `60000` | Journal flush interval in milliseconds. |
| `--chunk-mb` | `4` | Maximum chunk size in MiB before flushing. |
| `--source-system` | `255` | MAVLink source system ID used by LinkHub-originated messages. |
| `--source-component` | `0` | MAVLink source component ID used by LinkHub-originated messages. |
| `--baud` | unset | Restrict discovery to one baud rate. |
| `--discovery-bauds` | `115200,57600,38400,19200,9600` | Baud list scanned when `--baud` is not supplied. |
| `--discovery-timeout-ms` | `3000` | Heartbeat probe timeout per port/baud candidate. |
| `--motor-name-prefix` | unset | Enable Bluetooth motor support for devices whose name matches the prefix. |
| `--motor-scan-timeout-ms` | `10000` | Bluetooth motor scan timeout. |
| `--motor-heartbeat-ms` | `500` | Bluetooth motor heartbeat interval. |
| `--motor-max-command-timeout-ms` | `10000` | Maximum allowed motor command timeout. |

### Discovery notes

- The default discovery baud list is `115200,57600,38400,19200,9600`.
- Each candidate gets up to 3 seconds to produce a heartbeat.
- Restricting a port and/or baud never bypasses heartbeat confirmation.
- Serial discovery keeps the successful probe handle open for the live link.
- Open serial connections tolerate empty reads and only treat 5 seconds of
  silence as a lost port.
- `/v1/mavlink/status` reports the current `phase`, `connection`, `port`,
  `baud`, `attempts`, `consecutive_failures`, `last_error_stage`, `error`,
  and the most recent serial `last_scan` report.

## HTTP API usage

### Route inventory

Routes come from `linkhub\src\http.rs`.

| Route | Methods | Notes |
| --- | --- | --- |
| `/health/live` | `GET` | Liveness probe. |
| `/health/ready` | `GET` | Ready when MAVLink is connected and has a heartbeat. |
| `/v1/status` | `GET` | Service/run/journal status. |
| `/v1/schema` | `GET` | Live protocol schema JSON. |
| `/v1/journal/flush` | `POST` | Flush the active in-memory journal chunk. |
| `/v1/records` | `GET` | Mixed journal view over diagnostic + MAVLink records. |
| `/v1/diagnostics/events` | `GET`, `POST` | Read or ingest diagnostic events. |
| `/v1/mavlink/frames` | `GET`, `POST` | Read decoded frame records or inject raw base64 frames. |
| `/v1/mavlink/messages` | `GET`, `POST` | Read decoded message records or send a decoded message by name/fields. |
| `/v1/mavlink/commands` | `POST` | Execute `COMMAND_LONG` with finite timeout. |
| `/v1/mavlink/message-requests` | `POST` | Request one message by name. |
| `/v1/mavlink/version` | `GET` | Request `AUTOPILOT_VERSION`. |
| `/v1/mavlink/capabilities` | `GET` | Capability summary. |
| `/v1/mavlink/components` | `GET` | Known heartbeat senders. |
| `/v1/mavlink/parameters` | `GET`, `PUT` | List all parameters or set a batch. |
| `/v1/mavlink/parameters/{name}` | `GET`, `PUT` | Read or write one parameter. |
| `/v1/mavlink/message-rates` | `PUT` | Set explicit message-rate requests. |
| `/v1/mavlink/message-rates/{message}` | `GET` | Read one configured message interval. |
| `/v1/mavlink/files` | `GET`, `PUT`, `DELETE` | MAVFTP list/download/upload/remove by `path`. |
| `/v1/mavlink/directories` | `POST` | MAVFTP create-directory operation. |
| `/v1/mavlink/logs` | `GET` | List DataFlash logs. |
| `/v1/mavlink/logs/{id}` | `GET` | Download one DataFlash log. |
| `/v1/mavlink/status` | `GET` | Live link + vehicle status snapshot. |
| `/v1/motor` | `GET`, `PUT` | Optional Bluetooth motor status and set-running command. |
| `/v1/motor/stop` | `POST` | Optional Bluetooth motor stop command. |
| `/v1/motor/reconnect` | `POST` | Optional Bluetooth motor reconnect. |

### Common query/body parameters

- Journal-backed `GET` projections use opaque cursors of the form `v1:<sequence>`.
- `/v1/records`, `/v1/diagnostics/events`, and `/v1/mavlink/frames` use:
  `after`, `wait_ms`, `limit`, plus filters such as `classes`, `source`,
  `event`, `level`, `direction`, `message_ids`, and `messages`.
- `/v1/mavlink/messages` uses `after`, `wait_ms`, `limit`, `direction`,
  `message_ids`, `messages`, `collapse`, `max_lag_ms`, `expected_generation`,
  range selectors `since_ns`, `until_ns`, `last_ms`, `through`, and content
  filters `eq` and `contains`.
- `timeout_ms` is accepted by parameter reads/lists, message-interval reads,
  capabilities, version, log list/download, and several POST/PUT bodies.
- `target_system` / `target_component` are accepted where the request is sent
  to a MAVLink target, such as commands, version requests, and message requests.
- `PUT /v1/mavlink/message-rates` accepts a JSON object mapping message names
  to numeric Hz values or `null`. The response reports `message`, `rate_hz`,
  `interval_us`, and `after_cursor` for each configured entry.
- `GET /v1/mavlink/files` requires `path`; add `download=true` to download raw
  bytes instead of listing a directory, and `verify_crc=false` to skip the
  post-download CRC check.
- `GET /v1/mavlink/logs/{id}` accepts `timeout_ms` and `max_retries`.
- Parameter names are trimmed and uppercased, then must match
  `[A-Z][A-Z0-9_]{0,15}`. Malformed names are rejected before LinkHub sends a
  MAVLink parameter request.

### Status snapshots

`GET /v1/mavlink/status` serializes the live `LinkStatus` plus:

- `cursor`: the current journal tail as `v1:<sequence>`;
- `sim_clock`: the latest simulation clock snapshot;
- `generation`: `v1:<run-id>:<clock-epoch>`.

The status object includes:

- connectivity and target state: `connected`, `ready`, `phase`, `connection`,
  `port`, `baud`, `clock_epoch`, `target_system`, `target_component`,
  `base_mode`, `custom_mode`, `system_status`, `latest_time_boot_ms`;
- counters: `received_messages`, `transmitted_messages`, `received_bytes`,
  `transmitted_bytes`, `framing_errors`, `discarded_bytes`;
- rate samples: `rx_bps`, `tx_bps`;
- troubleshooting fields: `last_received_ns`, `error`, `last_error_stage`,
  `last_error_ns`, `attempts`, `consecutive_failures`, `connected_since_ns`,
  `last_scan`.

`rx_bps` and `tx_bps` are measured from validated MAVLink frame bytes, sampled
once per second, averaged over the most recent 3-second window, and reported as
`null` while disconnected and until the first sample after reconnect.

### Example requests

Read readiness and live transport state:

```powershell
Invoke-RestMethod http://127.0.0.1:8999/health/ready
Invoke-RestMethod http://127.0.0.1:8999/v1/mavlink/status
```

Read recent received attitude messages:

```powershell
Invoke-RestMethod `
  "http://127.0.0.1:8999/v1/mavlink/messages?last_ms=120000&messages=ATTITUDE&direction=rx&limit=100"
```

Read one parameter:

```powershell
Invoke-RestMethod `
  "http://127.0.0.1:8999/v1/mavlink/parameters/RAWES_MODE?timeout_ms=3000"
```

Set one parameter:

```powershell
Invoke-RestMethod `
  -Method Put `
  -ContentType "application/json" `
  -Body '{"value":1,"type":6,"timeout_ms":3000}' `
  http://127.0.0.1:8999/v1/mavlink/parameters/RAWES_MODE
```

Flush the journal:

```powershell
Invoke-RestMethod -Method Post http://127.0.0.1:8999/v1/journal/flush
```

## Query a saved run

`linkhub query` reads immutable `.lhc` chunks directly. The positional path may
be:

- a run directory containing `journal\`;
- the `journal\` directory itself; or
- a data directory containing exactly one run.

Root syntax:

```powershell
.\linkhub\target\release\linkhub.exe query <path> [--after <sequence>] [--through <sequence>] <subcommand>
```

`--after` and `--through` bound the offline scan in journal-sequence space.

### Subcommands

| Subcommand | Key flags | Meaning |
| --- | --- | --- |
| `types` | none | Count first/last occurrence per message type + direction. |
| `show` | `--type`, `--dir`, `--since`, `--until`, `--eq`, `--contains`, `--fields`, `--limit`, `--json` | Print matching decoded MAVLink messages. |
| `count` | `--type`, `--dir`, `--since`, `--until`, `--eq`, `--contains`, `--by` | Count matches, optionally grouped by one field. |
| `stats` | `--type`, `--dir`, `--since`, `--until`, `--eq`, `--contains`, `--field` | Numeric min/max/mean/median/stdev for one field. |
| `armed` | none | Print armed/disarmed transitions from received heartbeats. |
| `statustext` | `--since`, `--until`, `--min-severity` | Print reassembled `STATUSTEXT` lines. |
| `nvf` | `--name`, `--dir`, `--since`, `--until` | Print `NAMED_VALUE_FLOAT` rows. |
| `param` | `--id`, `--since`, `--until` | Print parameter traffic. |
| `diagnostics` | `--source`, `--event`, `--level`, `--contains`, `--limit`, `--json` | Read structured diagnostic events. |

Examples:

```powershell
.\linkhub\target\release\linkhub.exe query `
  .\simulation\logs\linkhub\RUN_ID types
.\linkhub\target\release\linkhub.exe query `
  .\simulation\logs\linkhub\RUN_ID show --type ATTITUDE --dir rx --limit 20 --json
.\linkhub\target\release\linkhub.exe query `
  .\simulation\logs\linkhub\RUN_ID diagnostics --source linkhub.link
```

`show --json` emits one decoded record per JSON line.

## Message-rate and bandwidth status

Implemented today:

- explicit rate requests through `PUT /v1/mavlink/message-rates`;
- RX/TX throughput measurement from validated MAVLink frame bytes;
- 1-second sampling with a 3-second moving average window;
- `rx_bps` / `tx_bps` reported as `null` while disconnected.

Proposed, not implemented:

- USB/radio bandwidth profiles;
- priority tiers;
- rate leases;
- adaptive shedding or automatic delivered-rate arbitration.

## Repository scripts

The repository also contains `scripts\linkhub_stress.py` for LinkHub HTTP and
telemetry stress testing.
