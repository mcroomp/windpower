# LinkHub UI

The LinkHub UI is a static TypeScript/Three.js browser client that talks to
LinkHub over its HTTP API.

## Build and launch

From the repository root, launch a previously built UI plus LinkHub:

```powershell
.\run_linkhub_ui.cmd COM7 115200
```

For SITL:

```powershell
.\run_linkhub_ui.cmd tcp:127.0.0.1:5760
```

Add `--build` to install UI dependencies, build the UI, and build LinkHub with
the `bluetooth` Cargo feature:

```powershell
.\run_linkhub_ui.cmd --build COM7 115200 8999
```

`run_linkhub_ui.cmd` also:

- accepts `LINKHUB_CONNECTION`, `LINKHUB_BAUD`, and `LINKHUB_PORT`;
- defaults the HTTP port to `8999`;
- serves `linkhub-ui\dist` through LinkHub's `--static-dir`;
- adds `--no-cache` so a browser refresh picks up a new build immediately.

## `npm` scripts

`linkhub-ui\package.json` currently defines exactly three scripts:

| Command | What it runs | Notes |
| --- | --- | --- |
| `npm run build` | `tsc --noEmit && vite build` | Type-checks, then writes static assets to `dist\`. |
| `npm run dev` | `vite --host 127.0.0.1` | Frontend-only Vite dev server bound to loopback. The command does not set a port, so Vite chooses its default port. |
| `npm run test` | `vitest run` | One-shot test run. |

There is no checked-in Vite proxy configuration and the frontend uses relative
`/v1/...` fetches, so `npm run dev` is only a frontend server. If you need the
live LinkHub API, either serve the built UI through LinkHub or put your own
reverse proxy in front of the dev server.

## Browser interface

Supported terminal commands come from `src\terminal.ts`:

```text
status
config check [--all]
config apply [--all]
run passive [--force] [--duration S] [--trim thr=0.342]
            [--roll DEG] [--pitch DEG] [--yaw DEG]
stop
help
```

During a passive run, the browser hotkeys are:

```text
Arrows = roll/pitch
Minus or equals = thrust
Comma or period = yaw
Space = reset
Esc = stop
```

`config check` compares the live vehicle against:

- `tests\sitl\rawes_common_defaults.parm`;
- `hardware\rawes_hardware_defaults.parm`;
- and, with `--all`, `tests\sitl\copter-heli.parm`.

Calibration-only sensor values are excluded. `config apply` writes only
differing parameters in one batch, verifies them, and refuses while armed or
while a passive run is active.

## Telemetry behavior

When LinkHub reports the connection as ready, the UI requests these display
message rates through `PUT /v1/mavlink/message-rates`:

- `ATTITUDE`: 25 Hz
- `ATTITUDE_QUATERNION`: 25 Hz
- `ATTITUDE_TARGET`: 25 Hz
- `SERVO_OUTPUT_RAW`: 25 Hz
- `LOCAL_POSITION_NED`: 10 Hz
- `BATTERY_STATUS`: 2 Hz

The UI also consumes other messages when present:

- `HEARTBEAT` for mode/armed state;
- `NAMED_VALUE_FLOAT` named `YFF_U` for yaw-motor display;
- `EKF_STATUS_REPORT` for status text;
- `ESC_TELEMETRY_1_TO_4` for rotor-speed display.

Reads use `GET /v1/mavlink/messages` with `collapse=true` and
`max_lag_ms=1000`, so the browser does not replay stale backlog beyond one
second. A generation change clears reconstructed state and causes the display
message-rate configuration to be requested again.

The yellow reference arrow is driven by `ATTITUDE_TARGET`. Its quaternion `q`
is interpreted as `[w, x, y, z]`, and the arrow points along the target
upper-axis direction (opposite FRD body-down), not the current rotor axle. It
is hidden until a target arrives and cleared on disconnect or generation
change.

Position, attitude, target, and RPM are display-only smoothed. The smoothing
time constant is 80 ms, stale telemetry resets after one second, and a
generation change clears reconstructed state.

## Throughput and rate-policy status

Implemented today:

- explicit message-rate requests from the browser;
- display of LinkHub-reported `rx_bps` / `tx_bps`;
- LinkHub-side 1-second sampling and 3-second averaging;
- `null` throughput while disconnected.

Proposed, not implemented:

- USB/radio bandwidth profiles;
- priority tiers;
- rate leases;
- adaptive shedding.

The browser formats the throughput values reported by LinkHub; it does not
estimate rates from filtered telemetry or HTTP payload size.

## Safety and boundaries

Bench commands may change vehicle mode and Lua state. Follow the primary
[arming](../design/arming.md) and [calibration](../design/calibration.md)
procedures; this README is not a hardware operating procedure.

For the system boundary, see:

- [flight stack ownership](../design/flight_stack.md);
- [LinkHub transport and journal semantics](../design/linkhub.md).
