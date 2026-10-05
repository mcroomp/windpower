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
  --static-dir .\linkhub-ui\dist
```

The server listens on `127.0.0.1:8999` by default.
When `--static-dir` (or `LINKHUB_STATIC_DIR`) is set, LinkHub serves that
directory at `/` after matching its health and `/v1` API routes.
After UI source changes, run `npm run build` in `linkhub-ui` and refresh the
browser. LinkHub reads the rebuilt files from the static directory on each
request, so it does not need to be restarted.
For hardware, pass a serial port and baud, for example
`--connection COM7 --baud 57600`. `python -m calibrate` starts LinkHub and scans
serial ports automatically when the default local endpoint is not already live.
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
LinkHub run ID and MAVLink clock epoch. Clients discard reconstructed state
when this token changes, covering both service replacement and link
reconnection/vehicle reboot without treating ordinary cursor advancement as a
new generation.

Typed message reads accept comma-separated `messages`, `direction`, `limit`,
`collapse=true`, and optional `max_lag_ms` query parameters. LinkHub scans the
journal in bounded raw batches and advances the cursor past nonmatching records.
When the first unread record is older than `max_lag_ms`, LinkHub returns no
records and advances directly to the current journal tail. Collapse keeps only
the newest snapshot for recognized state telemetry within a batch while
preserving event and transaction messages such as `STATUSTEXT` and `COMMAND_ACK`.
