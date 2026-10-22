# LinkHub Architecture

LinkHub is the Rust transport and diagnostic gateway used by the current
RAWES MAVLink stack.

## Ownership

One LinkHub process owns each configured physical link. For MAVLink this means:

- one TCP, serial, or BLE connection;
- one GCS heartbeat source;
- one outbound frame writer;
- one component and capability model;
- one transaction-correlation layer;
- one message-rate configuration owner;
- one lossless record of every complete RX and TX frame accepted by LinkHub.

Clients use finite HTTP/JSON requests and do not link a MAVLink implementation.

The DroneBridge BLE transport (`ble:DEVICE_NAME_OR_ADDRESS`) connects as a BLE central to
the ESP32 GATT serial service. Notification payloads and writes are treated as
one raw MAVLink byte stream; outgoing writes are fragmented to fit the
negotiated ATT payload size. BLE support is built into LinkHub. The BLE
peripheral must be advertising, in Bluetooth LE mode, and configured to forward
MAVLink from its UART.

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
| `/v1/mavlink/files` | `GET`, `PUT`, `DELETE` | MAVFTP list/start-upload/remove. |
| `/v1/mavlink/directories` | `POST` | MAVFTP create-directory operation. |
| `/v1/mavlink/logs` | `GET` | DataFlash log list. |
| `/v1/mavlink/transfers[/{id}[/content]]` | `GET`, `POST`, `DELETE` | Background MAVFTP uploads and MAVFTP/DataFlash downloads: poll, collect downloads, cancel. |

File transfers are jobs rather than long-lived or streamed responses because
HTTP cannot reliably represent link stalls or mid-body failures. A job's
progress and loss statistics are its own state and it survives client
disconnects. Download content is released only once it is whole and verified;
uploads complete only after the remote CRC matches. The engines live in
`linkhub/src/transfer/` (including the pure, deterministic MAVFTP burst reader)
and `linkhub/src/operations/{files,logs}.rs`.
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
The same status snapshot breaks bytes and rates down by MAVLink message name
(`received_bytes_by_message`, `transmitted_bytes_by_message`,
`rx_bps_by_message`, `tx_bps_by_message`) using the same sampler and window;
idle message types are omitted from the rate maps.
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

`linkhub-clientgen` (`linkhub/clientgen`) generates the client-facing types
directly from the vendored ArduPilot definitions, parsed with `mavlink-bindgen`'s
own parser, so there is no hand-maintained list of messages or enumeration
members:

- `linkhub_client/src/linkhub_client/generated_protocol.py`;
- `linkhub-ui/src/generated/protocol.ts`.

Regenerate them with:

```powershell
cargo run --manifest-path .\linkhub\Cargo.toml -p linkhub-clientgen
```

Rust tests fail when either checked artifact drifts from the definitions, and
they compare the declared shape of every field of every message with what the
real dialect serializes. Every message decodes to a generated dataclass whose
fields default to zero values, enumerations to generated `WireEnum`
(`StrEnum`) members whose names drop the prefix shared by every entry
(`MavCmd.COMPONENT_ARM_DISARM`), and bitmasks to `frozenset`s of members. Names
the generated enumerations do not list become pseudo-members named after the wire
name. Message classes are the PascalCase of the MAVLink name (`Statustext`,
`CommandLong`), are keyword-only in Python, and the TypeScript field interfaces
require every base field and leave extension fields optional.
Do not hand-edit the generated artifacts or duplicate these public types in
Python or TypeScript.

### MAVLink value representation

LinkHub uses the `mavlink-protocol` crate (`linkhub/mavlink-protocol`) for framing, CRC,
and message types. It is the `mavlink` crate's own recipe, `mavlink-core` plus
`mavlink-bindgen`, with the `ardupilotmega` dialect (and `mav2-message-extensions`)
generated from ArduPilot's definitions ([ArduPilot/mavlink](https://github.com/ArduPilot/mavlink),
vendored in `linkhub/mavlink-protocol/definitions` at the commit the simulated ArduPilot
release builds from), so LinkHub's message and
enumeration set matches the firmware's rather than a crates.io snapshot.
The protocol crate owns `codec`: concrete MAVLink frame encoding/decoding,
typed-message JSON conversion, and explicit JSON adapters for dialect enums and
bitmasks. Application structs use those adapters rather than directly invoking
generated Serde implementations. The async frame reader accepts a boxed
`AsyncRead + Unpin + Send` (including borrowed streams), so framing and full-message
serialization code generation stays in the protocol crate and is cached across
application-only builds. Generated message types remain available for constructing
commands and matching responses.

The same crate owns command/acknowledgement, parameter, message-request and
message-interval transactions, MAVFTP packets and stop-and-wait/burst transfer
drivers, and DataFlash `LOG_*` listing/download reassembly and retries. Shared
transfer statistics are protocol values, not dependencies on the application
job registry. `client::new_protocol` accepts the host's `Arc<dyn Transport>` and
returns `Arc<dyn client::Protocol>`; the concrete implementation is private.
The public trait and its implementation use `#[async_trait]`, which turns their
`async fn` methods into boxed `Send` futures. These non-inlined entry points keep
concrete protocol state machines inside the protocol crate; internal async helpers
remain concrete. Erasing the transport or client alone does not erase future
state machines: the boxed-future boundary is also required. Callers still use
`.await`, with borrowed inputs and typed results/errors unchanged. Boxing allocates
per operation, not per transfer packet; the codec frame reader is not part of this
transaction API. Cloned protocol handles share command,
parameter and file/log transaction locks and MAVFTP sequence state.

LinkHub remains the sole connection, transmission, and journal owner. Its adapter
supplies target selection and observed messages and performs every actual send
through the existing link/journal boundary. HTTP/CLI projections, transfer job
IDs, job spawning, cancellation integration, content retention, and user-visible
progress/diagnostic events remain application responsibilities. Protocol cleanup
(MAVFTP session termination and `LOG_REQUEST_END`) must still run on failed or
cancelled transfers; extraction must not turn cancellation into an unobserved
background operation.
Messages cross the HTTP API, the journal, and `linkhub query` in the generated
types' serde JSON form, with the message tag removed from `fields`:

- an **enumeration** is `{"type": "MAV_X_NAME"}`, for example HEARTBEAT
  `system_status` is `{"type": "MAV_STATE_ACTIVE"}`;
- a **bitmask** is a string of ` | `-joined names such as
  `"MAV_MODE_FLAG_SAFETY_ARMED | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED"`, and `""`
  when no flag is set;
- a dialect field named `type` is exposed as `mavtype`;
- plain integers, floats, text, and arrays stay JSON numbers, strings, and
  arrays, and extension fields are included.

LinkHub-owned JSON follows the same rule: link and component `base_mode` is a
bitmask string and `system_status`, `vehicle_type` and `autopilot` are
enumeration objects; command requests take `command` as an enumeration and
return `command` and `result` as enumerations; parameter `type` is an
enumeration; capability `capabilities` is a bitmask string. Requests that
build a message (`POST /v1/mavlink/messages`) take `fields` in this form, and a
numeric enumeration value or a missing non-extension field is rejected.

### Parameter values

The protocol API represents ArduPilot parameters as
`client::ParameterValue::{Int8(i8), Int16(i16), Int32(i32), Real32(f32)}`.
Settings and results carry that value rather than a separate numeric value and
independently selectable type. The wire type is derived from the enum.
The enum describes the logical value, not its transport.
`encode_classic`/`decode_classic` implement the float-based message path;
its precision restrictions do not limit the values the enum or HTTP JSON can
represent. Packed-file encoding can use the same enum without those conversions.
HTTP JSON remains flat: `"value": -42, "type": {"type": "MAV_PARAM_TYPE_INT16"}`.
Integer values serialize as JSON integers. A missing write type defaults to
`REAL32`; explicit integer types require integral, in-range values.
Non-finite values, unsupported types and overflowing `REAL32` inputs are errors.

The pinned Copter-4.7.1 flight-controller parameter server implements classic
`PARAM_REQUEST_READ`, `PARAM_REQUEST_LIST`, `PARAM_SET` and `PARAM_VALUE`, not
`PARAM_EXT_*`. Its scalar types are the four above, and it numerically converts
integers to/from the classic float field rather than using bytewise encoding.
See upstream [parameter dispatch](https://github.com/ArduPilot/ardupilot/blob/Copter-4.7.1/libraries/GCS_MAVLink/GCS_Common.cpp#L4368),
[writes](https://github.com/ArduPilot/ardupilot/blob/Copter-4.7.1/libraries/GCS_MAVLink/GCS_Param.cpp#L265)
and [scalar types](https://github.com/ArduPilot/ardupilot/blob/Copter-4.7.1/libraries/AP_Param/AP_Param.h#L187).
LinkHub rejects integer writes that cannot be represented exactly as `f32`
instead of silently changing the requested value. Integer replies must be
integral and in range; their original precision cannot be recovered if the
vehicle already rounded it during transmission. These transaction APIs do not
claim general MAVLink bytewise or extended-parameter support; raw message
construction and journal access remain available separately.

ArduPilot also provides a lossless alternative over MAVFTP: its virtual
`@PARAM/param.pck` filesystem packs the native scalar bytes for reads and accepts
packed parameter uploads. In Copter-4.7.1,
[the packer copies native value bytes](https://github.com/ArduPilot/ardupilot/blob/Copter-4.7.1/libraries/AP_Filesystem/AP_Filesystem_Param.cpp#L246),
and [uploads explicitly preserve all Int32 bits](https://github.com/ArduPilot/ardupilot/blob/Copter-4.7.1/libraries/AP_Filesystem/AP_Filesystem_Param.cpp#L607)
when the uploaded and destination types both match Int32. This is a distinct
ArduPilot-specific packed-file protocol, not `PARAM_EXT_*` or bytewise classic
parameter encoding. The typed parameter transaction API currently uses classic
messages; it does not yet implement packed-file parameter reads/writes.

The dialect cannot represent an enumeration value outside it. A frame
that passes its CRC but does not decode as a typed message (for example an
unknown enum value) is dropped without ending the link; the first drop per
message ID is logged. Corrupt frames are skipped while `mavlink-core`
resynchronizes the byte stream. Keeping the dialect in step with the pinned
ArduPilot release is enforced by tests and described in
[protocol README](../linkhub/mavlink-protocol/README.md). All dialect-defined
message names are valid for message operations and query filters; unknown names
are rejected.

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
