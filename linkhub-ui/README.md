# LinkHub UI

Static TypeScript browser client for LinkHub. It provides a live Three.js
vehicle-state view and a command terminal with `status`, `config check|apply`,
`run passive`, and `stop`.

From the repository root, launch the existing LinkHub executable and compiled
UI with:

```powershell
.\run_linkhub_ui.cmd COM7 115200
```

Add `--build` to install UI dependencies and rebuild both the UI and LinkHub
before launching:

```powershell
.\run_linkhub_ui.cmd --build COM7 115200
```

For SITL:

```powershell
.\run_linkhub_ui.cmd tcp:127.0.0.1:5760
```

The connection can alternatively be supplied through `LINKHUB_CONNECTION`.
The optional third argument overrides the HTTP port.
The launcher does not invoke or depend on `python -m calibrate`.
It runs LinkHub with `--no-cache`, so after `npm run build` a normal browser
refresh always shows the new UI.

The equivalent manual commands are:

```powershell
Set-Location .\linkhub-ui
npm install
npm run build

cargo run --manifest-path ..\linkhub\Cargo.toml -- serve `
  --connection COM7 --baud 57600 `
  --data-dir ..\simulation\logs\linkhub `
  --static-dir .\dist --no-cache
```

Open `http://127.0.0.1:8999/`. LinkHub serves only the compiled files and
continues to expose its existing transport-generic HTTP/JSON API. The browser
owns RAWES-specific passive sequencing and reconstructs active targets from the
journal after reconnect. A change to the status `generation` token invalidates
that reconstructed state. Message reads ask LinkHub to enforce a one-second
maximum lag; the server advances stale reads to the current journal tail instead
of sending backlog for the browser to replay.

The 3D view uses a blue sky background and green ground plane with a reference grid.
The yellow arrow in front of the axle shows the target rotor-axis direction from
`ATTITUDE_TARGET` (the upper-axis direction, opposite FRD body-down). Its origin
uses the vehicle pose; the target quaternion is decoded from MAVLink's `q`
array in `[w, x, y, z]` order. The arrow
direction follows the target, not the current axle.
It is hidden until a target is received and cleared on disconnect or generation change.
The overlay shows "No capture target" while the arrow is hidden; it does not
substitute the current attitude for a missing target.
It interpolates position with lerp and attitude/target orientation with
shortest-arc quaternion slerp, using an 80 ms exponential smoothing time constant
independent of display refresh rate. The first observation, a gap exceeding one
second, or a new link generation starts a fresh pose rather than blending across
unrelated samples. This is display-only smoothing: command and telemetry values
remain unchanged. No position or attitude is extrapolated beyond the latest
sample, and stale RPM decays to zero after one second without an update.

Opening the live view requests its attitude, target, servo, position, and battery
streams as soon as the link is ready, and requests them again for each new link
generation. No passive run or arm command is required. Stream-configuration
failures are surfaced in the terminal. Use the LinkHub endpoint (normally
`http://127.0.0.1:8999/`); the Vite development server alone has no MAVLink API.

Telemetry is decoded on arrival rather than in the render loop. The view reuses
its math objects, shares blade resources, skips draws when the scene is unchanged,
and pauses animation in hidden tabs. Overlay updates are coalesced to at most
10 Hz; GPU resources and telemetry subscriptions are released on disposal.

Supported terminal commands:

```text
status
config check [--all]
config apply [--all]
run passive [--force] [--duration S] [--trim thr=0.342]
            [--roll DEG] [--pitch DEG] [--yaw DEG]
stop
help
```

`config check` mirrors `python -m calibrate config check`: it compares the
vehicle's parameters with `tests/sitl/rawes_common_defaults.parm` plus
`hardware/rawes_hardware_defaults.parm` (`--all` also includes
`tests/sitl/copter-heli.parm`), excluding sensor-calibration values. The
`.parm` files are bundled at build time, so rebuild the UI after editing them.
`config apply` writes only the differing parameters in one batch and verifies
them; it refuses while armed or during a passive run.

The browser bench route enters GUIDED_NOGPS and passive hold through the
Lua-handled `ENTER_GUIDED`/`ENTER_PASSIVE` commands (see
`design/flight_stack.md`). `ENTER_PASSIVE` carries a zero yaw-trim seed, which
keeps Lua's adaptive yaw trim at zero during the stationary hold while leaving
ArduPilot's ordinary yaw PID available for transient correction.
