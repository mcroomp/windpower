# LinkHub

LinkHub is the Rust transport and diagnostic gateway and the single MAVLink
owner for both SITL and hardware calibration.

It owns one MAVLink connection, writes exact RX and TX frames plus structured
diagnostic events into one globally ordered chunk journal, and exposes filtered
cursor batches over HTTP. Completed chunks are immutable MessagePack files.
The active chunk is buffered in RAM and may be lost if the process crashes.

## Run

```powershell
cargo run --manifest-path .\linkhub\Cargo.toml -- serve `
  --connection tcp:127.0.0.1:5760 `
  --data-dir .\simulation\logs\linkhub `
  --static-dir .\linkhub-ui\dist `
  --no-cache
```

The server listens on `127.0.0.1:8999` by default.
When `--static-dir` (or `LINKHUB_STATIC_DIR`) is set, LinkHub serves that
directory at `/` after matching its health and `/v1` API routes.
After UI source changes, run `npm run build` in `linkhub-ui` and refresh the
browser. LinkHub reads the rebuilt files from the static directory on each
request, so it does not need to be restarted. Add `--no-cache` to send
`Cache-Control: no-store` on every response (UI files and API), so a plain
refresh always loads the latest build instead of a browser-cached copy.

Journal chunks are written to disk every `--flush-ms` (default 60000) or when
a chunk reaches 4096 records / `--chunk-mb`. Live HTTP reads include records
not yet flushed.

For hardware, `--connection auto` (the default) is the normal way to run:
LinkHub scans every local serial port, probing each at `--discovery-bauds`
(default `115200,57600,38400,19200,9600`, 3 s each via `--discovery-timeout-ms`)
until one answers with a MAVLink heartbeat. It rescans from scratch on every
reconnect, so an unplugged cable, a renumbered COM port, or an autopilot that
powers on later are all found automatically — not being connected yet is an
ordinary, expected state, not an error. Passing `--connection COM7` and/or
`--baud 57600` does not bypass this: it only *restricts* which port and/or
baud rate are probed; the heartbeat confirmation and reconnect rescan still
apply. `GET /v1/mavlink/status` reports the `port`/`baud` currently in use, or
`null` for both while LinkHub is scanning or disconnected.
An idle serial line (Windows completes reads with zero bytes, e.g. while a
freshly rebooted autopilot sends only its 1 Hz heartbeat) is not treated as
end-of-stream; an open serial link is dropped and rescanned only after 5 s
with no bytes at all.
The port that answered the probe is handed to the live link still open, never
closed and reopened: on Windows an immediate reopen fails with "Access is
denied" while the probe's handle is being released.

Link diagnostics: `GET /v1/mavlink/status` also reports `phase`
(`scanning`/`opening`/`acquiring`/`ready`/`backoff`), `last_error_stage`
(`scan`/`open`/`acquire`/`link`), `consecutive_failures`, `attempts`, and
`last_scan` (every enumerated port with USB VID:PID/serial, and per-probe
stage, error, bytes read and frames parsed). Each lifecycle transition is
journaled as a diagnostic event with source `linkhub.link`
(`link.discovered`, `link.scan_failed`, `link.opened`, `link.open_failed`,
`link.ready`, `link.lost`, `link.acquire_failed`); identical consecutive
failures are collapsed with a `repeat_count`. Inspect them live with
`GET /v1/diagnostics/events` or afterwards with
`linkhub query <run> diagnostics --source linkhub.link`.
`python -m calibrate` starts LinkHub this way automatically when the default
local endpoint is not already live.
Build with `--features bluetooth` to enable the optional motor backend.

Query a persisted journal without starting the service:

```powershell
.\target\release\linkhub.exe query <run-or-journal-path> types
.\target\release\linkhub.exe query <run-or-journal-path> show `
  --type PID_TUNING --dir rx --json |
  ForEach-Object { $_ | ConvertFrom-Json }
```

The native query surface and `jq` examples are documented in
[`design/linkhub.md`](../design/linkhub.md).

Run the reusable, actuator-safe HTTP/telemetry stress harness against an
already-running LinkHub:

```powershell
python scripts/linkhub_stress.py --server http://127.0.0.1:8999
```

It checks malformed-request isolation, parameter-timeout isolation, concurrent
TX journaling, bounded-wait requests, identical lossless batch replay,
and observed high-rate ATTITUDE throughput. The prior ATTITUDE interval is
restored before exit.

## Current HTTP surface

- `GET /health/live`
- `GET /health/ready`
- `GET /v1/status`
- `GET /v1/schema`
- `POST /v1/journal/flush`
- `GET /v1/records?after=v1:<sequence>&wait_ms=<bounded>`
- `GET /v1/mavlink/status`
- `GET|POST /v1/mavlink/messages`
- `POST /v1/mavlink/commands`
- `POST /v1/mavlink/message-requests`
- `GET /v1/mavlink/version`
- `GET /v1/mavlink/capabilities`
- `GET /v1/mavlink/components`
- `GET|PUT /v1/mavlink/parameters`
- `GET|PUT /v1/mavlink/parameters/{name}`
- `GET|PUT /v1/mavlink/message-rates`
- `GET|PUT|DELETE /v1/mavlink/files`
- `POST /v1/mavlink/directories`
- `GET /v1/mavlink/logs`
- `GET /v1/mavlink/logs/{id}`
- `GET|PUT /v1/motor`
- `POST /v1/motor/stop`
- `POST /v1/motor/reconnect`
- `GET /v1/mavlink/frames`
- `POST /v1/mavlink/frames`
- `GET /v1/diagnostics/events`
- `POST /v1/diagnostics/events`

The generic, MAVLink, and diagnostic streams are projections over the same
cursor space. The MAVLink frame API deals in exact base64-encoded frames. The
typed API uses the official `mavlink/rust-mavlink` ArduPilotMega dialect and is
compatible with the dependency-free Python client and generated TypeScript
browser wrappers. MAVFTP, DataFlash, serial
transport, component/capability discovery, and optional Bluetooth motor control
are implemented in Rust.

`GET /v1/mavlink/status` includes a `generation` token composed from the
LinkHub run ID and MAVLink clock epoch. The clock epoch advances both when a
connection is (re)acquired and the moment a previously live connection drops,
so a caller sees the change as soon as the link becomes untrustworthy rather
than only once it is reacquired (which can take much longer). Clients discard
reconstructed state when this token changes, covering service replacement,
serial reconnection, and vehicle reboot without treating ordinary cursor
advancement as a new generation.

Typed message reads accept comma-separated `messages`, `direction`, `limit`,
`collapse=true`, optional `max_lag_ms`, and optional `expected_generation`
query parameters. A multi-step sequence that polls for a state transition
across several requests (e.g. confirming arm/disarm) can capture its
`generation` baseline once up front and pass it as `expected_generation` on
every read; if LinkHub's live generation no longer matches, the request fails
with `409 generation_changed` (body includes `expected_generation`,
`current_generation`, and the current `cursor`) instead of returning a batch,
so the caller can abort immediately instead of continuing to poll a link that
is no longer trustworthy. Leave it unset for single, one-shot reads -- there
is no state to protect and the check would only add overhead.

LinkHub scans the journal in bounded raw batches and advances the cursor past
nonmatching records. When the first unread record is older than `max_lag_ms`,
LinkHub returns no records and advances directly to the current journal tail.
Collapse keeps only the newest snapshot for recognized state telemetry within
a batch while preserving event and transaction messages such as `STATUSTEXT`
and `COMMAND_ACK`.

### Range queries

Typed message reads also accept these filters:

- `since_ns` / `until_ns`: inclusive host ingest-time bounds, in the same
  wall-clock nanoseconds as each record's `received_time_ns`;
- `last_ms`: shorthand for `since_ns = now - last_ms` (exclusive with
  `since_ns`);
- `through`: inclusive upper cursor bound (`v1:<sequence>`);
- `eq`: comma-separated `field=value` pairs matched against decoded fields;
  values parse as JSON when possible (`severity=0`, `value=1.0`), otherwise
  as strings (`name=RAWES_TEN`);
- `contains`: case-insensitive substring of the message name or its decoded
  fields.

`eq` and `contains` also apply to live reads. Supplying any of `since_ns`,
`until_ns`, `last_ms`, or `through` makes the request a one-shot range scan:
it scans from `after` (default the start of the run) until `limit` matches,
the range end, or the journal tail, and rejects `wait_ms` and `max_lag_ms`.
A batch with fewer than `limit` records means the range is exhausted;
otherwise repeat with `after=<next_cursor>` and the same bounds. Example, the
last two minutes of capture-related traffic:

```powershell
Invoke-RestMethod "http://127.0.0.1:8999/v1/mavlink/messages?last_ms=120000&direction=rx&messages=ATTITUDE_TARGET,STATUSTEXT&limit=10000"
Invoke-RestMethod "http://127.0.0.1:8999/v1/mavlink/messages?last_ms=120000&eq=severity=0&contains=crash"
```
