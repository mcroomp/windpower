# LinkHub Python Protocol Client

`linkhub_client` is the dependency-free Python HTTP/JSON client for
[LinkHub](../linkhub/README.md). In this repository it is imported by
calibration code, ground-station code, simulation helpers, and tests. It does
not contain a MAVLink parser, socket transport, serial support, or hardware
discovery logic.

## Import and setup

Within this repository, import the package as:

```python
from linkhub_client import LinkHubClient
```

The checked generated protocol file lives at
`linkhub_client\src\linkhub_client\generated_protocol.py`. It is generated from
ArduPilot's MAVLink definitions by `linkhub-clientgen`
(`cargo run --manifest-path .\linkhub\Cargo.toml -p linkhub-clientgen`); do not
hand-edit it.

## Minimal example

```python
from linkhub_client import LinkHubClient

hub = LinkHubClient("http://127.0.0.1:8999")
hub.connect()
cursor = hub.current_cursor()
batch = hub.read_messages(cursor, "ATTITUDE", wait=1.0)
cursor = batch.next_cursor
attitudes = batch.messages
```

Reads are finite HTTP requests with an explicit input cursor. Each response
returns matching records plus a scanned-through `next_cursor`, even when there
were no matches. The caller owns that cursor; the client has no global receive
cursor, queue, follower, background receive thread, or local MAVLink state
machine.

## Package-level exports

`linkhub_client\src\linkhub_client\__init__.py` exports these names at package
top level:

- `LinkHubClient`
- `LinkHubMotorController`
- `LinkHubError`
- `MessageBatch`
- `DiagnosticBatch`
- `WallClock`
- `TelemetryRecord`
- `DiagnosticEvent`
- `DiagnosticLevel`
- `DiagnosticRecord`
- `SimClock`
- `SimTimeQuality`

## Other public modules

- `linkhub_client.messages` exports the decoded message helpers, including
  `RawMessage`, `EscTelemetry`, `decode_message`, and the generated message
  classes/enums re-exported there.
- `linkhub_client.generated_protocol` contains the generated dataclasses,
  enums, and protocol metadata used by `decode_message`.
- `linkhub_client.client` defines `LinkHubGenerationChanged`, which is raised
  when a read using `expected_generation=` receives LinkHub's
  `generation_changed` HTTP error.

## Typed MAVLink values

MAVLink enumerations and bitmasks are typed, not integers. The enumerations
(`MavCmd`, `MavResult`, `MavParamType`, `MavState`, `MavType`, ...) are
`StrEnum`s whose value is the full MAVLink name, so `MavCmd.DO_SET_MODE ==
"MAV_CMD_DO_SET_MODE"`; names outside the generated members become
pseudo-members instead of raising. Bitmask fields (`MavModeFlag`,
`EkfStatusFlags`, `AttitudeTargetTypemask`, ...) decode to `frozenset`s, so test
membership instead of masking bits:

```python
from linkhub_client import LinkHubClient
from linkhub_client.messages import MavCmd, MavModeFlag, MavResult

hub = LinkHubClient()
hub.connect()
if MavModeFlag.SAFETY_ARMED in hub.vehicle_status()["base_mode"]:
    result = hub.command(MavCmd.COMPONENT_ARM_DISARM, [0.0, 0.0])
    assert result["result"] is MavResult.ACCEPTED
```

On the wire an enumeration is `{"type": "<NAME>"}`, a bitmask is a
`" | "`-joined string, and a dialect field named `type` is `mavtype`
(`Heartbeat.mavtype`); see
[design/linkhub.md](../design/linkhub.md#mavlink-value-representation).
`linkhub_client.mav_constants` keeps only values that are plain integers on the
wire (copter modes and message ids). `MAV_DATA_STREAM` is an enumeration here
too: use `MavDataStream` for `RequestDataStream.req_stream_id`.

## `LinkHubClient` surface

Core connection and journal methods:

- `connect(timeout=15.0)`
- `close()` (currently a no-op)
- `generation` property
- `linkhub_status()`
- `vehicle_status()`
- `current_cursor()`
- `flush_journal()`
- `read_messages(...)`
- `send_diagnostics(...)`
- `read_diagnostics(...)`

MAVLink command and parameter helpers:

- `send_message(message)`
- `command(command: MavCmd, params=None, timeout=3.0)`
- `get_param(name, timeout=3.0)`
- `set_param(name, value, timeout=3.0, param_type: MavParamType | None = None)`
- `fetch_all_params(timeout=15.0)`
- `fetch_all_param_records(timeout=15.0)`
- `set_params(parameters, timeout=15.0, retries=2)`
- `set_mode(custom_mode, timeout=10.0)`
- `is_armed` property
- `arm(timeout=10.0, force=False)`
- `disarm(timeout=10.0, force=False)`
- `sim_now()`
- `sim_sleep(duration_s, check=None)`
- `set_message_rates(rates, timeout=10.0)`
- `set_message_interval(message_id, interval_us)`

MAVFTP, DataFlash, and optional motor helpers:

- `list_files(path)`
- `download_file(remote_path, local_path, verify_crc=True, stall_timeout=15.0, progress=None)`
- `upload_file(local_path, remote_path)`
- `remove_file(path)`
- `create_directory(path)`
- `capabilities()`
- `components()`
- `list_logs(timeout=5.0)`
- `download_log(log_id, local_path, timeout=2.0, max_retries=10, progress=None)`
- `transfers()`, `transfer(id)`, `cancel_transfer(id)`

`download_file` and `download_log` start a LinkHub transfer, poll it (calling
`progress` with each status dict), write the verified content atomically, and
cancel the transfer on any error or Ctrl-C. See the
[transfer API](../linkhub/README.md#transfers).
- `motor_status()`
- `motor_set(speed_percent, direction, timeout_ms=2000)`
- `motor_stop()`
- `motor_reconnect()`

## Generation-aware reads

`LinkHubClient.generation` captures LinkHub's opaque
`v1:<run-id>:<clock-epoch>` token at `connect()` time. Pass that token as
`expected_generation=` on `read_messages()` when you are polling for a state
transition across multiple requests and want LinkHub to fail fast if the
service restarts, the transport reconnects, or the vehicle reboots mid-sequence.

If that happens, the client raises `LinkHubGenerationChanged` from
`linkhub_client.client` and carries:

- `baseline`
- `current`
- `cursor`

The cursor lets callers skip directly to the new tail instead of draining
backlog from an older link generation.

## `LinkHubMotorController`

`LinkHubMotorController` is a small logical wrapper around LinkHub's optional
Bluetooth motor endpoints. Its public methods are:

- `connect()`
- `set_speed(speed)`
- `start()`
- `stop()`
- `close()`

While enabled, it renews the motor command once per second. It does not talk to
Bluetooth hardware directly; it only uses the HTTP API exposed by LinkHub.

For cursor semantics, journal behavior, and the shared protocol boundary, see
[design/linkhub.md](../design/linkhub.md). For where the client fits in the
wider system, see [design/flight_stack.md](../design/flight_stack.md).
