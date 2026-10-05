# LinkHub Architecture

LinkHub is the setup-neutral Rust gateway for MAVLink, future device links, and
structured diagnostics. It is the active MAVLink owner for SITL and hardware
calibration; the former Python service has been removed.

## Ownership

One LinkHub process owns each configured physical link. For MAVLink this means:

- one TCP or serial connection;
- one GCS heartbeat source;
- one outbound frame writer;
- one component and capability model;
- one transaction-correlation layer;
- one message-rate configuration owner;
- one lossless record of every complete RX and TX frame accepted by LinkHub.

Clients use finite HTTP/JSON requests and do not link a MAVLink implementation.

## Unified journal

Raw transport facts and diagnostics share one globally ordered journal. Record
classes currently include:

- `mavlink.rx`;
- `mavlink.tx`;
- `diagnostic.event`.

Future transports add record classes rather than separate logging systems.
Every record has a global sequence, ingestion timestamp, optional correlation
ID, and versioned payload. HTTP cursors are opaque encodings of the global
sequence.

The journal actor is the only sequence allocator and chunk writer. Records
accumulate in an in-memory chunk. A chunk is encoded as MessagePack and written
as one immutable `.lhc` file when it reaches the configured record/byte limit
or flush interval. Losing the active in-memory chunk on a process or machine
crash is an accepted design tradeoff; LinkHub does not fsync or repair partial
tail files. A clean shutdown flushes the active chunk.

For a replay, the actor first fixes one tail sequence and snapshots the active
chunk; the reader then merges that snapshot with completed chunks through the
fixed tail. This ordering prevents a periodic flush from moving records between
the disk and RAM views while a replay is being assembled. Public reads are
finite batches; LinkHub keeps no per-client subscription or queue.

## Diagnostics

All producers use the same `DiagnosticEvent` envelope:

- run ID;
- source and source-instance ID;
- monotonic source sequence for idempotency;
- source wall and monotonic timestamps;
- optional simulation timestamp and quality;
- level, category, stable event name, and human message;
- correlation and causation IDs;
- structured fields;
- related global journal sequences.

External retries are deduplicated by `(run_id, source_instance,
source_sequence)`. A diagnostic operation can reference the exact MAVLink RX or
TX sequences that caused it.

High-rate numeric simulation samples do not become diagnostic events.
`telemetry.csv` remains the dense physics/control signal artifact. The LinkHub
journal provides semantic ordering and raw transport evidence.

## Task and lock model

Mutable resources have one Tokio task owner:

- the MAVLink link task owns its socket and official-library codec;
- the journal task owns sequence allocation and the active chunk;
- operation state machines own protocol-specific correlation state.

HTTP handlers read immutable status snapshots and communicate through bounded
channels. They do not hold a socket, archive, or transaction lock while waiting
on a client. A broadcast notification is only a wake-up mechanism; the journal
cursor remains authoritative.

## HTTP API

The foundation provides:

- `GET /health/live`;
- `GET /health/ready`;
- `GET /v1/status`;
- `GET /v1/schema`;
- `GET /v1/records`;
- `GET|POST /v1/diagnostics/events`;
- `GET|POST /v1/mavlink/frames`;
- `GET|POST /v1/mavlink/messages`;
- `POST /v1/mavlink/commands`;
- `POST /v1/mavlink/message-requests`;
- `GET /v1/mavlink/version`, `/capabilities`, and `/components`;
- `GET|PUT /v1/mavlink/parameters`;
- `GET|PUT /v1/mavlink/parameters/{name}`;
- `GET|PUT /v1/mavlink/message-rates`;
- MAVFTP file and directory operations;
- DataFlash list and download operations;
- optional Bluetooth motor status, command, stop, and reconnect operations;
- `GET /v1/mavlink/status`.

Generic, diagnostics, raw MAVLink frames, and decoded MAVLink messages are
finite projections over the same cursor space. Every GET accepts an explicit
`after` cursor and returns:

- a bounded `records` array;
- `next_cursor`, the last journal position scanned even when no record matched.

An optional bounded `wait_ms` keeps one HTTP request open until a match arrives
or the deadline expires. It does not create server-side subscription state.
Response `limit` bounds the number of matching records; LinkHub never advances
`next_cursor` past a matching record it did not return. Raw frame bodies are
base64 in JSON; completed journal chunks keep the original binary frame bytes.
Journal traversal is also bounded independently of the response limit. A sparse
filtered read can therefore return an empty batch with an advanced cursor before
reaching the live tail; clients continue from `next_cursor` rather than
rescanning irrelevant high-rate telemetry.

Decoded-message reads accept `collapse=true` for consumers that only need
current state. Within each bounded batch, LinkHub keeps the newest recognized
snapshot per link, direction, sender, message type, and message-specific key
(for example `NAMED_VALUE_FLOAT.name` or `PID_TUNING.axis`). Event and
transaction messages such as `STATUSTEXT`, `COMMAND_ACK`, parameter traffic,
and file transfers are never collapsed. Calibration recording uses filtered
batches without collapse because its CSV and smoothness checks require every
sample.

Python clients do not mirror the journal in a local message queue and do not
retain a global receive cursor. Each logical operation owns an opaque cursor and
passes it explicitly on every finite batch read. Heartbeat-derived mode/arming
state and the latest vehicle boot time are exposed by
`/v1/mavlink/status`, so polling state never depends on which telemetry a client
chooses to consume. Calibration CSV headers record their LinkHub start cursor,
and SITL preserves the run's `linkhub/<run-id>/journal/` directory. New runs do
not export a duplicate `mavlink.jsonl`.

## Generated protocol boundary

LinkHub's Rust descriptor in `linkhub/src/protocol.rs` owns the public enum
codes, known MAVLink message field types, aliases, and defaults. It generates:

- `linkhub/schema/protocol-v1.schema.json`;
- `linkhub_client/src/linkhub_client/generated_protocol.py`;
- the live `GET /v1/schema` response.

Regenerate checked artifacts with:

```powershell
cargo run --manifest-path .\linkhub\Cargo.toml -- schema `
  --output .\linkhub\schema\protocol-v1.schema.json `
  --python-output .\linkhub_client\src\linkhub_client\generated_protocol.py
```

Rust tests fail when either checked artifact drifts from the descriptor.
Known messages decode to generated dataclasses and numeric MAVLink enums decode
to generated forward-compatible `IntEnum` values. Unknown enum values become
`UNKNOWN_<value>` pseudo-members; unknown message types remain `RawMessage`.
Do not hand-edit the generated artifacts or duplicate these public types in
Python.

Rate leases remain future work; calibration uses explicit message-rate
configuration.

## Offline journal query

`linkhub query` reads immutable `.lhc` chunks directly. Pass a journal
directory, its parent run directory, or a data directory containing exactly
one run:

```powershell
.\linkhub\target\release\linkhub.exe query <journal-path> types
.\linkhub\target\release\linkhub.exe query <journal-path> armed
.\linkhub\target\release\linkhub.exe query <journal-path> statustext
.\linkhub\target\release\linkhub.exe query <journal-path> nvf --name YFF_U
.\linkhub\target\release\linkhub.exe query <journal-path> param --id RAWES_MODE
```

All commands accept a cursor-bounded journal slice through the parent query
options `--after <sequence>` and `--through <sequence>`. Calibration prints
the corresponding `v1:<sequence>` start/end cursors.

`show`, `count`, and `stats` support message type, RX/TX direction, relative
ingest-time range, exact field, and case-insensitive content filters. `show
--json` emits flat decoded MAVLink JSON Lines with journal sequence, host
ingest time, and LinkHub simulation-clock metadata. This is the composition
boundary for PowerShell or `jq`; no project-specific query script is needed:

```powershell
# Yaw PID P/I contribution ranges.
$yawPid = .\linkhub\target\release\linkhub.exe query <journal-path> show `
  --type PID_TUNING --dir rx --eq axis=3 --json |
  ForEach-Object { $_ | ConvertFrom-Json }
$yawPid | ForEach-Object { [math]::Abs($_.P) } |
  Measure-Object -Maximum

# Largest adjacent output-9 step.
$pwm = @(
  .\linkhub\target\release\linkhub.exe query <journal-path> show `
  --type SERVO_OUTPUT_RAW --dir rx --json |
  ForEach-Object { ($_ | ConvertFrom-Json).servo9_raw }
)
1..($pwm.Count - 1) |
  ForEach-Object { [math]::Abs($pwm[$_] - $pwm[$_ - 1]) } |
  Measure-Object -Maximum
```

The equivalent `jq` composition remains available in any shell:

```bash
linkhub query <journal-path> show \
  --type PID_TUNING --dir rx --eq axis=3 --json |
  jq -s '{max_abs_p: (map(.P | fabs) | max), max_abs_i: (map(.I | fabs) | max)}'
```

`statustext` reassembles ArduPilot's multi-chunk messages before printing.
`diagnostics` queries structured LinkHub diagnostic events by source, event,
level, or content. Use the live HTTP DataFlash endpoints before shutting down
a SITL container; offline journal queries and DataFlash recovery are
complementary.

## Protocol boundary

Rust implementation types are private. Public JSON DTOs are mirrored by the
dependency-free `linkhub_client` Python package and protected by
cross-language conformance fixtures and tests. Python clients contain no
MAVLink framing or dialect dependency.

The package and public classes use LinkHub naming throughout.

## Current MAVLink boundary

The current Rust implementation:

- uses the official `mavlink/rust-mavlink` ArduPilotMega dialect;
- validates MAVLink 1 and MAVLink 2 CRCs while framing arbitrary TCP reads;
- preserves exact frame bytes;
- records system, component, message, protocol, signing, and direction metadata;
- owns GCS heartbeat transmission;
- journals RX and TX in the same global order;
- exposes finite raw and decoded message batches;
- supports the typed messages used by calibration and SITL;
- correlates command ACKs and serializes command transactions;
- supports named and bulk parameter reads plus named writes;
- configures and queries message intervals;
- owns TCP and serial MAVLink transports;
- implements MAVFTP and DataFlash state machines;
- optionally owns the Bluetooth motor link with command-expiry safe stop.

MAVLink 2 signature verification, UDP transport, and expiring rate leases
remain future extensions.

## Cutover validation

SITL and hardware calibration cutover are complete. Hardware validation covered
automatic serial discovery, decoded telemetry, capability/component discovery,
parameter reads and batch writes, message-interval queries, MAVFTP
upload/download/CRC/remove, DataFlash list and download, and force arm/disarm
with actuators disconnected. Motor protocol, timeout, and safe-stop behavior are
covered by hardware-free Rust backend tests.

The reusable hardware stress harness additionally verifies malformed-request
and operation-timeout isolation, concurrent TX journaling, bounded-wait
requests, lossless multi-reader batch replay, and requested telemetry
throughput.
