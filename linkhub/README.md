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

The MAVLink dialect is generated at build time from ArduPilot's own definitions,
vendored in `dialect/definitions` (no submodule or network access is needed to
build). LinkHub uses the `linkhub-dialect` crate (`mavlink-core` plus
`mavlink-bindgen`; see [dialect/README.md](dialect/README.md)) for framing,
CRC, and message types. MAVLink
enumerations and bitmasks appear in JSON as `{"type": "MAV_X_NAME"}` objects and
`" | "`-joined name strings; see
[design/linkhub.md](../design/linkhub.md#mavlink-value-representation).

For iterative development, run a subcommand directly:

```powershell
cargo run --manifest-path .\linkhub\Cargo.toml -- serve --connection tcp:127.0.0.1:5760
```

The development and test profiles keep line-level debug information for
LinkHub while omitting debug information from third-party dependencies. This
reduces normal compile/link time and target-directory growth without affecting
optimized release builds.

## Generate the checked protocol artifacts

`linkhub-clientgen` (`linkhub/clientgen`) generates the Python and TypeScript
types for every message and enumeration in the vendored ArduPilot definitions.
Rerun it after changing `linkhub/dialect/definitions`:

```powershell
cargo run --manifest-path .\linkhub\Cargo.toml -p linkhub-clientgen
```

It rewrites `linkhub_client\src\linkhub_client\generated_protocol.py` and
`linkhub-ui\src\generated\protocol.ts`. `cargo test --workspace` fails when
either file is stale, and checks the declared field shapes against what the
dialect serializes for every message.

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
| `/v1/mavlink/files` | `GET`, `PUT`, `DELETE` | MAVFTP list/upload/remove by `path`. |
| `/v1/mavlink/directories` | `POST` | MAVFTP create-directory operation. |
| `/v1/mavlink/logs` | `GET` | List DataFlash logs. |
| `/v1/mavlink/transfers` | `GET`, `POST` | List background file/log downloads, or start one. |
| `/v1/mavlink/transfers/{id}` | `GET`, `DELETE` | Poll one transfer; cancel it, or forget it once finished. |
| `/v1/mavlink/transfers/{id}/content` | `GET` | The verified bytes of a completed transfer. |
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
  `interval_us`, and `after_cursor` for each configured entry. `0` disables a
  message; `null` asks for the vehicle's default interval, which comes from the
  `MAVn_*` stream parameters and is "off" when those are 0, so it will not
  bring back a stream that another client had requested at runtime. The
  vehicle may refuse a rate (for example 400 Hz on ArduCopter 4.7.1 returns
  `MAV_RESULT_DENIED` and the request fails with 400).
- `GET /v1/mavlink/files` requires `path` and lists a directory. Downloads
  are transfers (below).
- Parameter names are trimmed and uppercased, then must match
  `[A-Z][A-Z0-9_]{0,15}`. Malformed names are rejected before LinkHub sends a
  MAVLink parameter request.

### Transfers

Downloads run as background jobs, so one HTTP request never has to outlive a
slow or lossy radio link: start, poll, then collect.

```
POST   /v1/mavlink/transfers   {"kind":"ftp_download","path":"/APM/LOGS/00000017.BIN"}
POST   /v1/mavlink/transfers   {"kind":"log_download","log_id":17}
GET    /v1/mavlink/transfers/{id}           progress, rate, loss stats
GET    /v1/mavlink/transfers/{id}/content   200 bytes | 409 not complete | 410 evicted
DELETE /v1/mavlink/transfers/{id}           202 cancel requested | 200 finished job removed
```

- `ftp_download` accepts `verify_crc` (default `true`) and `stall_timeout_ms`
  (default 15000, fail after this long without a contiguous byte).
- `log_download` accepts `packet_timeout_ms` (default 2000) and `max_retries`
  (default 10).
- `POST` returns `202` with a status object: `id`, `kind`, `target`, `state`
  (`queued`, `running`, `complete`, `failed`, `cancelled`), `total_bytes`,
  `done_bytes`, `received_bytes`, `elapsed_ms`, `rate_bytes_per_s`, `error`,
  `content_available` and `stats`.
- `stats` counts `packets_received`, `duplicate_packets`, `gaps`,
  `requests_sent`, `bursts`, `repair_requests`, `retransmits`, `timeouts` and
  `naks`. Duplicates should stay near zero and `gaps` track link loss.
- Content is only released after the transfer completes, and after the CRC
  check for MAVFTP, so a truncated download is never mistaken for a whole one.
  LinkHub keeps the last 32 finished transfers within 128 MiB.
- Transfers are serialized per protocol: one MAVFTP job at a time (queued jobs
  wait), and DataFlash downloads run alongside MAVFTP.
- MAVFTP uses `BurstReadFile` (the autopilot streams up to 2000 chunks per
  request) and repairs holes with a few pipelined reads, then always sends
  `TerminateSession`. DataFlash downloads re-request only the first missing
  range. Each transfer also journals `linkhub.transfer` diagnostics
  (`transfer.started`, `.complete`, `.failed`, `.cancelled`) with the final
  stats, so `linkhub query diagnostics` shows how a run went.

### Status snapshots

`GET /v1/mavlink/status` serializes the live `LinkStatus` plus:

- `cursor`: the current journal tail as `v1:<sequence>`;
- `sim_clock`: the latest simulation clock snapshot;
- `generation`: `v1:<run-id>:<clock-epoch>`.

The status object includes:

- connectivity and target state: `connected`, `ready`, `phase`, `connection`,
  `port`, `baud`, `clock_epoch`, `target_system`, `target_component`,
  `base_mode` (bitmask string), `custom_mode` (integer), `system_status`
  (enumeration object), `latest_time_boot_ms`;
- counters: `received_messages`, `transmitted_messages`, `received_bytes`,
  `transmitted_bytes`, `dropped_frames` (frames that passed their CRC but did not
  decode as a typed dialect message, for example an unknown enum value);
- rate samples: `rx_bps`, `tx_bps`;
- per-message-name breakdown: lifetime `received_bytes_by_message` /
  `transmitted_bytes_by_message` (bytes) and windowed `rx_bps_by_message` /
  `tx_bps_by_message` (bits per second, keyed by MAVLink message name such as
  `ATTITUDE`);
- troubleshooting fields: `last_received_ns`, `error`, `last_error_stage`,
  `last_error_ns`, `attempts`, `consecutive_failures`, `connected_since_ns`,
  `last_scan`.

`rx_bps` and `tx_bps` are measured from validated MAVLink frame bytes, sampled
once per second, averaged over the most recent 3-second window, and reported as
`null` while disconnected and until the first sample after reconnect. The
per-message rate maps use the same window and are empty while disconnected;
message types with no traffic in the window are omitted.

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
  -Body '{"value":1,"type":{"type":"MAV_PARAM_TYPE_INT32"},"timeout_ms":3000}' `
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
