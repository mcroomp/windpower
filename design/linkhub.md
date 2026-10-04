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

Python clients do not mirror the journal in a local message queue and do not
retain a global receive cursor. Each logical operation owns an opaque cursor and
passes it explicitly on every finite batch read. Heartbeat-derived mode/arming
state and the latest vehicle boot time are exposed by
`/v1/mavlink/status`, so polling state never depends on which telemetry a client
chooses to consume. Test MAVLink JSONL artifacts are exported from the journal
cursor range with an explicit export cursor instead of being assembled by a
client receive thread.

Rate leases remain future work; calibration uses explicit message-rate
configuration.

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
