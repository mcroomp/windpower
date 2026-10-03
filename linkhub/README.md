# LinkHub

LinkHub is the Rust transport and diagnostic gateway and the single MAVLink
owner for both SITL and hardware calibration.

It owns one MAVLink connection, writes exact RX and TX frames plus structured
diagnostic events into one globally ordered chunk journal, and exposes filtered
cursor streams over HTTP. Completed chunks are immutable MessagePack files.
The active chunk is buffered in RAM and may be lost if the process crashes.

## Run

```powershell
cargo run --manifest-path .\linkhub\Cargo.toml -- serve `
  --connection tcp:127.0.0.1:5760 `
  --data-dir .\simulation\logs\linkhub
```

The server listens on `127.0.0.1:8999` by default.
For hardware, pass a serial port and baud, for example
`--connection COM7 --baud 57600`. `python -m calibrate` starts LinkHub and scans
serial ports automatically when the default local endpoint is not already live.
Build with `--features bluetooth` to enable the optional motor backend.

Run the reusable, actuator-safe HTTP/telemetry stress harness against an
already-running LinkHub:

```powershell
python scripts/linkhub_stress.py --server http://127.0.0.1:8999
```

It checks malformed-request isolation, parameter-timeout isolation, concurrent
TX journaling, follower connection churn, identical lossless reader streams,
and observed high-rate ATTITUDE throughput. The prior ATTITUDE interval is
restored before exit.

## Current HTTP surface

- `GET /health/live`
- `GET /health/ready`
- `GET /v1/status`
- `GET /v1/records?after=v1:<sequence>&follow=true`
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
compatible with the dependency-free Python client. MAVFTP, DataFlash, serial
transport, component/capability discovery, and optional Bluetooth motor control
are implemented in Rust.
