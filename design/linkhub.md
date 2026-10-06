# LinkHub Architecture

LinkHub is the Rust transport and diagnostic gateway used by the current
RAWES MAVLink stack.

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

Every record has a global sequence, ingestion timestamp, optional correlation
ID, and versioned payload. HTTP cursors are opaque encodings of the global
sequence.

The journal actor is the only sequence allocator and chunk writer. Records
accumulate in an in-memory chunk. A chunk is written as one immutable `.lhc`
file when it reaches the configured record/byte limit (4096 records / 4 MiB)
or the 60-second flush interval. Live readers never wait for a flush: HTTP
reads merge flushed chunks with the in-memory chunk. Each file is the magic
`LHCHNK02`, the minimum and maximum host ingest time (two little-endian `u64`
nanosecond values), then the MessagePack chunk; time-range reads use this
header to skip files without decoding them. Files with any other magic are
rejected rather than skipped. LinkHub does not fsync or repair partial tail
files. A clean shutdown flushes the active chunk.

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

LinkHub can optionally serve compiled static browser assets from the directory
configured by `--static-dir` or `LINKHUB_STATIC_DIR`. Static serving is a
deployment convenience only: browser applications still use the same finite
HTTP/JSON API.

## HTTP API

The live router currently exposes:

| Route | Methods | Purpose |
| --- | --- | --- |
| `/health/live` | `GET` | Liveness probe. |
| `/health/ready` | `GET` | Ready when MAVLink is connected and has a heartbeat. |
| `/v1/status` | `GET` | Service/run/journal status. |
| `/v1/schema` | `GET` | Generated protocol schema. |
| `/v1/journal/flush` | `POST` | Flush the active in-memory chunk. |
| `/v1/records` | `GET` | Mixed journal projection over diagnostic + MAVLink records. |
| `/v1/diagnostics/events` | `GET`, `POST` | Diagnostic-event projection and diagnostic ingestion. |
| `/v1/mavlink/frames` | `GET`, `POST` | Raw/decoded MAVLink frame projection and raw frame injection. |
| `/v1/mavlink/messages` | `GET`, `POST` | Decoded MAVLink message projection and decoded message send. |
| `/v1/mavlink/commands` | `POST` | `COMMAND_LONG` execution. |
| `/v1/mavlink/message-requests` | `POST` | One-shot message request by name. |
| `/v1/mavlink/version` | `GET` | `AUTOPILOT_VERSION` request. |
| `/v1/mavlink/capabilities` | `GET` | Capability summary. |
| `/v1/mavlink/components` | `GET` | Known heartbeat senders. |
| `/v1/mavlink/parameters` | `GET`, `PUT` | Bulk parameter list / batch set. |
| `/v1/mavlink/parameters/{name}` | `GET`, `PUT` | Single parameter read/write. |
| `/v1/mavlink/message-rates` | `PUT` | Explicit message-rate requests. |
| `/v1/mavlink/message-rates/{message}` | `GET` | Read one configured interval. |
| `/v1/mavlink/files` | `GET`, `PUT`, `DELETE` | MAVFTP list/download/upload/remove. |
| `/v1/mavlink/directories` | `POST` | MAVFTP create-directory operation. |
| `/v1/mavlink/logs` | `GET` | DataFlash log list. |
| `/v1/mavlink/logs/{id}` | `GET` | DataFlash log download. |
| `/v1/mavlink/status` | `GET` | Live link + vehicle status snapshot. |
| `/v1/motor` | `GET`, `PUT` | Optional Bluetooth motor status and set-running command. |
| `/v1/motor/stop` | `POST` | Optional Bluetooth motor stop. |
| `/v1/motor/reconnect` | `POST` | Optional Bluetooth motor reconnect. |

Route-by-route invocation examples, request bodies, and CLI usage live in
[linkhub/README.md](../linkhub/README.md); this document keeps the shared API
semantics.

Generic records, diagnostics, raw MAVLink frames, and decoded MAVLink messages
are finite projections over the same cursor space. Every journal-backed GET
projection accepts an explicit `after` cursor and returns:

- a bounded `records` array;
- `next_cursor`, the last journal position scanned even when no record matched;
- `next_clock`, the LinkHub simulation-clock snapshot at that cursor.

The projection-specific filter surface is:

- `GET /v1/records`: `after`, `classes`, `source`, `event`, `level`,
  `direction`, `message_ids`, `messages`, `wait_ms`, `limit`;
- `GET /v1/diagnostics/events` and `GET /v1/mavlink/frames`: the same filter
  model, projected to only diagnostic or only MAVLink-frame records;
- `GET /v1/mavlink/messages`: `after`, `direction`, `message_ids`, `messages`,
  `wait_ms`, `limit`, `collapse`, `max_lag_ms`, `expected_generation`,
  one-shot range bounds `since_ns`, `until_ns`, `last_ms`, `through`, plus
  decoded-field/content filters `eq` and `contains`.

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
and file transfers are never collapsed.

Python clients do not mirror the journal in a local message queue and do not
retain a global receive cursor. Each logical operation owns an opaque cursor and
passes it explicitly on every finite batch read. Heartbeat-derived mode/arming
state and the latest vehicle boot time are exposed by
`/v1/mavlink/status`, so polling state never depends on which telemetry a client
chooses to consume.

`/v1/mavlink/status` also exposes `generation = v1:<run-id>:<clock-epoch>`.
The run ID changes when the service is replaced; the clock epoch changes both
when the MAVLink transport (re)connects -- including after a vehicle reboot --
and the moment a previously live connection drops, so the token changes as
soon as the link becomes untrustworthy rather than only once it is
reacquired. Browser clients use this ETag-like value to invalidate
reconstructed command and telemetry state. It intentionally does not include
the journal cursor.

The same status snapshot includes lifetime `received_bytes` and
`transmitted_bytes` counters for validated MAVLink frames (including protocol
headers, checksums, and signatures), plus `rx_bps` and `tx_bps` in bits per
second. These exclude UART framing, TCP/IP overhead, and radio framing.
The link-owning task samples counters once per monotonic second and averages
over the latest three seconds (the available shorter window during startup).
Sampling uses actual elapsed time, including scheduler delays, and continues
while idle so rates decay to zero. Rates are null while disconnected and
until the first sample after reconnect; lifetime counters are retained.
The browser only formats this snapshot, never estimates rates from its
filtered/collapsed telemetry stream or HTTP response sizes.

A multi-step sequence that polls for a state transition across several
requests (e.g. confirming arm/disarm) can pass its captured `generation` as
`expected_generation` on `GET /v1/mavlink/messages`. LinkHub itself aborts
that read with `409 generation_changed` the moment its live generation no
longer matches, instead of returning a batch -- so the caller can stop
immediately rather than keep polling a link, or a vehicle, it can no longer
trust. `calibrate`'s arm/disarm sequences (`calibrate/hw.py`) use this so an
unexpected mid-sequence service restart, serial reconnect, or vehicle reboot
aborts the sequence instead of silently waiting out its timeout. Do not pass
`expected_generation` for single, one-shot reads -- they have no state to
protect.

## Generated protocol boundary

LinkHub's Rust descriptor in `linkhub/src/protocol.rs` owns the public enum
codes, known MAVLink message field types, aliases, and defaults. It generates:

- `linkhub/schema/protocol-v1.schema.json`;
- `linkhub_client/src/linkhub_client/generated_protocol.py`;
- `linkhub-ui/src/generated/protocol.ts`;
- the live `GET /v1/schema` response.

Regenerate checked artifacts with:

```powershell
cargo run --manifest-path .\linkhub\Cargo.toml -- schema `
  --output .\linkhub\schema\protocol-v1.schema.json `
  --python-output .\linkhub_client\src\linkhub_client\generated_protocol.py `
  --typescript-output .\linkhub-ui\src\generated\protocol.ts
```

Rust tests fail when either checked artifact drifts from the descriptor.
Known messages decode to generated dataclasses and numeric MAVLink enums decode
to generated forward-compatible `IntEnum` values. Unknown enum values become
`UNKNOWN_<value>` pseudo-members; unknown message types remain `RawMessage`.
Do not hand-edit the generated artifacts or duplicate these public types in
Python or TypeScript.

LinkHub's MAVLink wire implementation uses Mavio with default features
disabled. The `linkhub/mavio-dialect` path dependency owns the filtered typed
ArduPilotMega message set used for structured JSON projection and named message
operations, keeping generated code out of LinkHub's frequently rebuilt
compilation unit. Its build script also generates a compact registry for every
ArduPilotMega message ID, name, and `CRC_EXTRA`: messages outside the typed set
are still checksum-validated and journaled losslessly, but their `fields`
object is empty and clients therefore see them as `RawMessage`. Add a message
to the filtered set only when LinkHub needs to decode its fields or address it
by name; do not enable Mavio's complete generated ArduPilotMega dialect.

## Message-rate policy and bandwidth

**Current implementation:** clients configure explicit message intervals through
LinkHub's message-rate API. LinkHub measures `rx_bps` and `tx_bps` in the link
task with 1-second samples and a 3-second moving average. Rates are null while
disconnected and until the first sample after reconnect.

**Proposed, not implemented:** USB/radio bandwidth profiles, priority tiers,
rate leases, and adaptive shedding.

## Journal query

For a running LinkHub, prefer the live HTTP range query on
`GET /v1/mavlink/messages`; `linkhub query` is the offline reader for
immutable `.lhc` chunks of stopped runs.

The query CLI accepts a journal directory, its parent run directory, or a data
directory containing exactly one run. Parent options `--after <sequence>` and
`--through <sequence>` bound the slice in journal-sequence space.

Subcommands are `types`, `show`, `count`, `stats`, `armed`, `statustext`,
`nvf`, `param`, and `diagnostics`.

- `show`, `count`, and `stats` support message-type, RX/TX, relative-ingest
  time, exact field (`--eq`), and case-insensitive content (`--contains`)
  filters.
- `show --json` emits decoded MAVLink JSON Lines with journal sequence, host
  ingest time, and LinkHub simulation-clock metadata; this is the intended
  composition boundary for PowerShell or `jq`.
- `statustext` reassembles ArduPilot's multi-chunk messages before printing.
- `diagnostics` queries structured LinkHub diagnostic events by source, event,
  level, or content.

Invocation examples and per-subcommand flags live in
[linkhub/README.md](../linkhub/README.md).
