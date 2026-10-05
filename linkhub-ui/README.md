# LinkHub UI

Static TypeScript browser client for LinkHub. It provides a live Three.js
vehicle-state view and a command terminal with `status`, `run passive`, and
`stop`.

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

The equivalent manual commands are:

```powershell
Set-Location .\linkhub-ui
npm install
npm run build

cargo run --manifest-path ..\linkhub\Cargo.toml -- serve `
  --connection COM7 --baud 57600 `
  --data-dir ..\simulation\logs\linkhub `
  --static-dir .\dist
```

Open `http://127.0.0.1:8999/`. LinkHub serves only the compiled files and
continues to expose its existing transport-generic HTTP/JSON API. The browser
owns RAWES-specific passive sequencing and reconstructs active targets from the
journal after reconnect. A change to the status `generation` token invalidates
that reconstructed state.

Supported terminal commands:

```text
status
run passive [--force] [--duration S] [--trim thr=0.342]
            [--roll DEG] [--pitch DEG] [--yaw DEG]
stop
help
```
