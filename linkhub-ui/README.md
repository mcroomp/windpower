# LinkHub UI

The LinkHub UI is a static TypeScript/Three.js browser client that talks to
LinkHub over its HTTP API.
Parameter methods preserve the flat numeric `value` and MAVLink `type` JSON;
JavaScript numbers represent all supported Int32 values exactly. Classic
MAVLink encoding limits still apply at the vehicle boundary; see
[Parameter values](../design/linkhub.md#parameter-values).

## Build and launch

### Standalone offline demo / GitHub Pages

The demo is the **full normal UI**, with the same HTML, styling, scene, terminal,
telemetry store, passive controller, and diagnostic panels. The demo Vite build
replaces only the `runtime-api` factory. `LinkHubApi` uses an injected in-memory
`DemoBackend` fetch transport instead of network HTTP. The normal build retains
the real HTTP transport; neither build monkey-patches global `fetch`.
The data source is the checked-in
[pumping-cycle.json](src/demo/pumping-cycle.json) replay fixture. The local
journal uses LinkHub's message-specific snapshot keys when collapsing state.

```powershell
npm --prefix linkhub-ui run dev:demo
npm --prefix linkhub-ui run build:demo
```

The build writes `dist-demo/`, separately from the live UI's `dist/`. Serve the
output with any static HTTP server; double-clicking the HTML via `file://` is
not supported by browser module security. Relative asset URLs support both
the GitHub project-site subdirectory and other static hosts. Once loaded, the
demo needs no network connection; it does not install a service worker for
offline reloads.

The fixture owns source metadata, timing, phases, channel availability, and
parameter values. The UI shows available channels without synthesizing absent
measurements. The header displays phase/playback time, the camera follows the
vehicle, and ground alignment uses the physics/estimator altitude offset.
`SIM_RPM` is explicitly simulation-only rotor RPM, not measured yaw-motor RPM.
`RAWES_TEL_HZ` is a browser-local display setting in the demo.

This is **read-only replay**. The full terminal and panels remain present:
status, parameter reads/listing, config inspection, journal browsing, profile
selection, and Yaw/Wobble analysis use their normal production paths. Commands,
arming/disarming, mode changes, raw sends, calibration, reboot, and vehicle
parameter writes return an explicit read-only error without changing playback.
Nothing can reach hardware.

Profiles thin browser-local delivery only; they never increase the recorded
sample rate. Missing messages stay missing. The throughput popover estimates
traffic at 64 bytes per delivered frame, not measured link bandwidth. Boot and
arrival timestamps are rebased monotonically each loop so the production chart
readers keep working. The natural end-to-start cut is not an invented blending
trajectory. Browser journal history and catch-up after a suspended tab are
bounded. Cursor reads skip already-consumed records, and throughput scans only
the trailing three seconds rather than the full retained journal.

The browser-local adapter replaces the Rust service, not the frontend control,
rendering, or analysis code. This demo does not validate binary MAVLink transport
or the Rust service. Integration tests exercise the production `LinkHubApi`,
`TelemetryStore`, `CommandRunner`, `PassiveController` reconciliation,
`YawRecorder`, and `WobbleRecorder`.
Chart polling releases each completed delay's abort listener, keeping cancellation
cost bounded across replay loops and long-running live sessions.

To generate a replay fixture from selected run artifacts:

```powershell
uv run python -m analysis.export_demo_cycle simulation\logs\<test_name> --journal simulation\logs\<test_name>\linkhub\<run_id> --linkhub linkhub\target\debug\linkhub.exe --cycle <cycle>
```

The exporter uses `linkhub query ... show --json`.
Only the compact `src/demo/pumping-cycle.json` is shipped; raw logs and physics
CSV files are not published. Its metadata records the source run, channel
provenance, selected timestamps, and source telemetry SHA-256. Deployment builds
consume that checked-in recording and do not need a local SITL run or exporter.

To publish, select **Settings > Pages > Build and deployment > GitHub Actions**
on the repository. `.github/workflows/pages.yml` tests and builds the demo, then
deploys it after relevant pushes to `main` or a manual workflow dispatch.
The default project-site URL is `https://mcroomp.github.io/windpower/`.
No live service or credentials are required.

### Live UI

From the repository root, launch a previously built UI plus LinkHub:

```powershell
.\run_linkhub_ui.cmd auto
```

For SITL:

```powershell
.\run_linkhub_ui.cmd tcp:127.0.0.1:5760
```

Add `--build` to run the complete project setup before launching
(Bluetooth support is always included):

```powershell
.\run_linkhub_ui.cmd --build auto
```

`setup.cmd` owns tool installation, uv-managed Python, dependencies, and both
builds; see the [root setup instructions](../README.md#running-tests).
The launcher also runs setup automatically when either build artifact is
missing. With both artifacts present, a normal launch needs no Python or uv.

`run_linkhub_ui.cmd` also:

- accepts `LINKHUB_CONNECTION`, `LINKHUB_BAUD`, and `LINKHUB_PORT`;
- leaves baud unrestricted by default so LinkHub probes
  `115200,57600,38400,19200,9600` and can move between native USB and a
  telemetry radio; set a numeric baud only when intentionally restricting the
  scan;
- defaults the HTTP port to `8999`;
- serves `linkhub-ui\dist` through LinkHub's `--static-dir`;
- enables the optional external motor backend with `--motor-name-prefix BLDC`,
  initially disconnected;
- adds `--no-cache` so a browser refresh picks up a new build immediately.

## Bluetooth motor panel

Click **Motor**, then **Scan**, select the intended device by name and ID, and
click **Connect**. Scanning uses the **LinkHub server's Bluetooth adapter**,
not Web Bluetooth on the browser machine. Devices must advertise a name
starting with the server's configured prefix and expose Nordic UART.
Scanning takes the configured scan timeout (10 s by default).
For a manually launched LinkHub, pass `--motor-name-prefix BLDC` to enable it.
No adapter or device is needed until scanning/connecting.

Connecting leaves the motor off. Set the **Speed** slider and click **Turn on**.
The slider commands speed percentage, not closed-loop RPM. While running,
changes update the command; zero speed and **Turn off / stop** stop it.
Direction is preserved from the server's current motor state.
The panel can remain open alongside the flight diagnostic/calibration panels.

Only an explicit **Turn on** starts browser lease renewal. Closing this panel,
hiding the tab, Escape, or a failed request stops renewal and attempts safe-stop.
After a browser crash/network loss the server's 2 s command lease expires;
the heartbeat supervisor checks expiry every 500 ms by default. There is no
automatic restart after expiry, reconnect, or reopening the panel.
Another client's running motor is displayed but never adopted for renewal.
The offline demo shows the backend as unavailable and never accesses Bluetooth.
Keep clear of the mechanism; a transport-level stop is not physical stop feedback.
See [LinkHub motor semantics](../design/linkhub.md#bluetooth-bench-motor).

## `npm` scripts

`linkhub-ui\package.json` defines the following scripts:

| Command | What it runs | Notes |
| --- | --- | --- |
| `npm run build` | `tsc --noEmit && vite build` | Type-checks, then writes static assets to `dist\`. |
| `npm run build:demo` | TypeScript check + standalone Vite build | Writes the offline demo to `dist-demo\`. |
| `npm run dev:demo` | Demo Vite dev server | Runs the full UI against the recorded SITL replay. |
| `npm run cli -- <command>` | esbuild runner + Node | Bundles the CLI quickly under `tmp\linkhub-cli\`, then runs it against LinkHub; defaults to `http://127.0.0.1:8999`. |
| `npm run cli:debug -- <command>` | Node inspector + esbuild runner | Runs the same CLI paused on startup for a VS Code/Chrome debugger. |
| `npm run dev` | `vite --host 127.0.0.1` | Frontend-only Vite dev server bound to loopback. The command does not set a port, so Vite chooses its default port. |
| `npm run test` | `vitest run` | One-shot test run. |

There is no checked-in Vite proxy configuration and the frontend uses relative
`/v1/...` fetches, so `npm run dev` is only a frontend server. If you need the
live LinkHub API, either serve the built UI through LinkHub or put your own
reverse proxy in front of the dev server.

## Node CLI

The npm CLI and browser terminal share `CommandRunner`, `PassiveController`,
`TelemetryStore`, `LinkHubApi`, parameter parsing/merging, and generated
protocol code. The CLI is only a process/input-output and file-loading adapter;
it does not maintain a second passive-start implementation. The CLI uses a
small esbuild runner and Node filesystem reads for fast iteration, while the
browser keeps its Vite raw-file imports and full Three.js build. Every CLI
invocation checks the CLI TypeScript dependency tree and rebuilds its
source-mapped bundle only when those sources changed; it never rebuilds the
browser or Three.js bundle. Pass `--rebuild` before the command to force it:

```powershell
.\rawes --rebuild help
```

From the repository root:

```powershell
.\rawes status
.\rawes hub status
.\rawes journal cursor
.\rawes journal read v1:0 --messages HEARTBEAT,STATUSTEXT --direction rx
.\rawes param get RAWES_MODE
.\rawes param set RAWES_MODE 0
.\rawes rates set ATTITUDE=0 RPM=5 BATTERY_STATUS=default
.\rawes config check
.\rawes run passive --duration 30 --trim thr=0.342
.\rawes run collective --duration 30 --trim thr=0.5
.\rawes stop
```

The npm form remains available for automation:

```powershell
npm --prefix linkhub-ui run cli -- status
```

Use a non-default LinkHub endpoint with either form:

```powershell
.\rawes --server http://127.0.0.1:8999 status
```

`run passive` and `run collective` keep the Node process alive until the duration expires or it
receives Ctrl+C. Both paths call the shared canonical safe-off implementation.
Passive entry/capture uses acknowledged Lua commands; ongoing thrust/offset
and setup inputs remain named values. The
[flight-stack wire contract](../design/flight_stack.md#command-setpoint-and-telemetry-roles)
owns the exact split.
The standalone `stop` command and its `disarm` alias apply that same safe-off
sequence, so either can recover a run started by another CLI or browser
process. `arm [--force]` sends and confirms a direct arm request; `--force`
explicitly bypasses ArduPilot pre-arm checks and is only appropriate for
secured bench hardware.

The shared `LinkHubApi` also supports service and vehicle status, current
journal cursor and flush, finite filtered message reads with generation-change
protection, decoded message send, arbitrary `COMMAND_LONG`, typed single and
batch parameter operations, confirmed mode/arm state transitions, and message
rates/intervals. These methods are used by both the browser and Node runtimes.
`getParameters(names, timeoutMs, retries)` reads a selected group through one
pipelined batch operation, returning a map of typed parameter records.
The yaw chart uses it for the output-9 range and yaw slope, avoiding competing
single reads with overlapping timeout budgets. API behavior is owned by
[LinkHub parameter semantics](../design/linkhub.md#parameter-values).
Core CLI commands print API records as JSON so they can be used in scripts:

```text
hub status
journal cursor
journal flush
journal read AFTER [--messages A,B] [--direction rx|tx] [--wait S]
                   [--limit N] [--collapse]
                   [--expected-generation TOKEN]
message send NAME COMPACT_JSON_FIELDS
command send MAV_CMD_NAME [PARAM ...] [--timeout S]
param get NAME
param list
param set NAME VALUE [--type MAV_PARAM_TYPE_NAME]
mode set CUSTOM_MODE
arm [--force]
disarm
rates set MESSAGE=HZ|default [MESSAGE=HZ|default ...]
```

`message send` accepts one compact JSON object argument, for example:

```powershell
.\rawes message send NAMED_VALUE_FLOAT '{"time_boot_ms":0,"name":"TEST","value":1}'
```

For an inspector session:

```powershell
npm --prefix linkhub-ui run cli:debug -- --server http://127.0.0.1:8999 status
```

## Browser interface

Supported terminal commands are dispatched by the shared `src\commands.ts`;
`src\terminal.ts` owns browser input/output:

```text
status
hub status
journal cursor|flush
journal read AFTER [--messages A,B] [--direction rx|tx] [--wait S]
message send NAME COMPACT_JSON_FIELDS
command send MAV_CMD_NAME [PARAM ...] [--timeout S]
param get NAME
param list
param set NAME VALUE [--type MAV_PARAM_TYPE_NAME]
mode set CUSTOM_MODE
arm [--force]
disarm
rates set MESSAGE=HZ|default [...]
config check [--all]
config apply [--all]
run passive [--force] [--duration S] [--trim thr=0.342]
            [--roll DEG] [--pitch DEG] [--yaw DEG]
run collective [--duration S] [--trim thr=0.5]
compass calibrate [--mask N] [--retry] [--autosave] [--delay S]
compass accept
compass cancel
reboot
stop
help
```

`reboot` sends `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN` (param1 = 1, a normal
autopilot reboot). It refuses while the vehicle is armed (ArduPilot refuses too),
during a passive run, and during a compass calibration. The link reconnects by
itself. The vehicle comes back in its `INITIAL_MODE`, which the hardware defaults
set to ACRO (`config apply` writes it); run `stop` afterwards to verify the
canonical safe-off state.

`compass calibrate` sends `MAV_CMD_DO_START_MAG_CAL` and streams
`MAG_CAL_PROGRESS` and `MAG_CAL_REPORT` until every compass reports a final
result (Esc, Ctrl-C in the CLI, or `compass cancel` aborts):

- it refuses while armed or during a passive run, and the vehicle only
  calibrates compasses that are healthy with `COMPASS_USE`/`USE2`/`USE3`
  enabled, so a bare `FAILED` usually means one of those;
- `--mask` selects compasses (bitmask 0..7, default 0 = all), `--retry` lets the
  vehicle restart a failed fit, `--delay` waits S seconds before starting, and
  `--autosave` saves on success;
- without `--autosave` a successful result is held on the vehicle until
  `compass accept`; offsets apply after a reboot, which the UI never triggers;
- ArduPilot only streams these two messages when asked (they belong to its
  EXTRA3 stream), so the command requests them at 2 Hz and returns them to the
  vehicle's default rate when it ends. Reports repeat while a result is held,
  so the UI prints each distinct result once and ignores anything the vehicle
  sent before it accepted the start command.

### Swash calibration panel

Click **Swash** beside **Yaw** and **Wobble** for collective/cyclic calibration.
Opening the drawer only reads limits and live S1/S2/S3/S9 PWM telemetry; it does
not enable movement. Confirm secured hardware, then **Enable movement**.

- Drag the cyclic pad or choose a cardinal-limit or center button.
- Use the collective slider from 0 to 100% of configured collective travel.
- Edit `H_COL_MIN`, `H_COL_MAX` (microseconds), and `H_CYC_MAX` (centidegrees)
  while stopped, then explicitly **Apply limits**. This also sets
  `SERVO1_TRIM`, `SERVO2_TRIM`, and `SERVO3_TRIM` to the collective midpoint,
  rounded down (1200..1557 gives 1378 us). It can move the swash even while
  disarmed; confirm secured hardware and clearance first. All six values use
  one batch write, with disarmed checks before submission and after verification.
  The batch is not atomic and does not guarantee endpoint-write order.
  Its response and independent readbacks are checked, including equal midpoint trims and valid mixer
  calibration. Partial writes or inconsistent readbacks are errors, not
  success; reload and verify before continuing. **Reload limits** refreshes
  the limits and mixer calibration, including trims. Settings are not
  automatically restored on Stop.
- The neutral readout shows the draft midpoint and all three loaded trims,
  explicitly indicating whether they match. Matching numeric settings do not
  establish zero blade pitch or mechanical clearance.
- The read-only calculated S1/S2/S3 min/max updates as the draft limits change,
  without writing parameters. It predicts the full configured cyclic envelope,
  not just the current slider position; see the
  [mixer assumptions and clipping](../design/hardware.md#swashplate-and-servos).
- For an automatic single-circle sweep, center cyclic, open **Automatic cyclic
  sweep...**, choose radius/duration in its modal dialog, confirm clearance at
  the current collective, then **Sweep one circle**. The dialog shows a live
  commanded-position graphic, roll/pitch/collective values, and servo PWM.
  **Stop / close** or Escape
  closes the modal and applies safe-off. **Stop / safe-off** exits setup;
  closing or switching drawers also stops movement started by the panel.

This uses the existing collective mode 6, extended to accept fresh cyclic
commands. Deploy the updated Lua before use; a browser rebuild alone does not
update the vehicle. Stale servo telemetry inhibits/aborts panel movement.
The [arming owner](../design/arming.md#disarmed-collective-setup) defines the
watchdog, limit-write, and physical-clearance safety contract.

`run collective` is disarmed-only, collective-only command-line setup (default 50%).
The browser displays **servos live**; minus/equals adjusts collective by 5
percentage points, and Escape or `stop` returns to neutral safe-off. The CLI
uses `--trim thr=...` to select the value and Ctrl-C or `--duration` to stop.
It does not arm, and passive attitude controls are inactive. Secure the
mechanism before use. The vehicle watchdog and full safety contract are owned
by [Disarmed collective setup](../design/arming.md#disarmed-collective-setup).
The updated Lua script must be deployed before using this command; rebuilding
the browser alone does not update the vehicle.

During a passive run, the browser hotkeys are:

```text
Arrows = roll/pitch
Minus or equals = thrust
Comma or period = yaw
Space = recapture the current direction and clear roll/pitch/yaw offsets
Esc = stop
```

`config check` compares the live vehicle against:

- `tests\sitl\rawes_common_defaults.parm`;
- `hardware\rawes_hardware_defaults.parm`;
- and, with `--all`, `tests\sitl\copter-heli.parm`.

Calibration-only sensor values are excluded. `config apply` writes only
differing parameters in one batch, verifies them, and refuses while armed or
while a passive run is active.

### Yaw-trim bench sweep

The standalone `scripts\yaw-trim-sweep.ts` records a bounded trim ladder through
the shared LinkHub HTTP client. Follow the hardware prerequisites and safe-off
procedure in [the arming owner](../design/arming.md) before operating it.
From `linkhub-ui`, inspect its options without connecting to hardware:

```powershell
npm exec -- vite-node --script scripts\yaw-trim-sweep.ts --help
```

`--check` only checks the existing connection and required telemetry; it never
arms. An actual sweep requires an explicit operator invocation. On exit the
script calls the production passive controller's canonical cleanup, restores
the saved yaw gains, and requires fresh output-9 telemetry to verify safe-off.
Missing telemetry or cleanup failures produce a nonzero exit status. Recordings
and summaries go under `simulation\logs\calibrate\yaw_trim_sweep_<timestamp>`.

## Wire contract

LinkHub serializes MAVLink enumerations as `{"type": "MAV_X_NAME"}` objects and
bitmasks as ` | `-joined name strings (`""` when empty); `custom_mode`, ids and
counts stay integers. `src\generated\protocol.ts` (generated by `linkhub-clientgen`,
never hand-edited) holds the typed enums and field interfaces; `src\mav.ts` holds
the hand-written `enumName`/`enumIs`/`parseFlags`/`isArmed` helpers. Names outside
the generated enums are tolerated, and commands and parameter types are sent as
`{type: ...}` objects.

## Telemetry behavior

The selector beside the connection state is populated from
`src\telemetry-profiles.json` and applies the selected display telemetry profile
through `PUT /v1/mavlink/message-rates`. A profile may declare a
`connection_prefix`; a connection starting with it (for example `ble:`) uses
that profile. Otherwise each profile declares its minimum serial baud
(`minbaud`, `null` for connection-selected profiles). On each connection
generation, the UI selects the highest-rate
profile whose minimum fits the detected baud; below every minimum it uses the
lowest-rate profile. Connections without a baud, such as Wi-Fi/TCP, use the
highest-rate profile. A manual selection remains active until the connection
generation changes.

| Message | Wi-Fi | BLE | Radio |
| --- | ---: | ---: | ---: |
| `ATTITUDE` | off | off | off |
| `ATTITUDE_QUATERNION` | 300 Hz | 50 Hz | 10 Hz |
| `ATTITUDE_TARGET` | 300 Hz | 50 Hz | 10 Hz |
| `SERVO_OUTPUT_RAW` | 300 Hz | 50 Hz | 10 Hz |
| `PID_TUNING` (yaw) | 200 Hz | 50 Hz | 4 Hz |
| `LOCAL_POSITION_NED` | 10 Hz | 10 Hz | 5 Hz |
| `BATTERY_STATUS` | 2 Hz | 2 Hz | 1 Hz |
| `RC_CHANNELS` | 1 Hz | 1 Hz | 1 Hz |
| `RPM` | 100 Hz | 20 Hz | 5 Hz |
| `EKF_STATUS_REPORT` | 2 Hz | 2 Hz | 2 Hz |
| `VIBRATION` | 2 Hz | 2 Hz | 2 Hz |
| `ESC_TELEMETRY_1_TO_4` (see note) | 10 Hz | 5 Hz | 1 Hz |
| `RAWES_TEL_HZ` (parameter, `DEBUG_FLOAT_ARRAY` rate) | 20 Hz | 10 Hz | 2 Hz |

The BLE profile requests lower rates than Wi-Fi to leave transport capacity
for command acknowledgements and parameter replies.

Each profile in `src\telemetry-profiles.json` has `label`, `minbaud`, `rates`
(message rates in Hz, `0` = off) and `params` (vehicle parameters written as
floats after the rates). Rates and parameters persist on the vehicle, so every
profile must set exactly the same keys; the loader throws otherwise, which
guarantees that switching profiles never leaves a stale value behind. A rate
the vehicle rejects fails the profile (later entries in the file are not
applied, so keep optional messages last); a parameter that cannot be written,
for example `RAWES_TEL_HZ` while `rawes.lua` is not loaded, is reported as a
warning and the rates stay in effect. Parameters owned by a profile are
excluded from `config check`/`config apply`, otherwise `config apply` would
reset them to the `.parm` default.

`ESC_TELEMETRY_1_TO_4` is the only ESC block ArduPilot accepts an interval for
(`5_TO_8` and `9_TO_12` return `MAV_RESULT_DENIED`, which would fail the whole
profile). The request makes the autopilot emit whichever blocks have data, so
the yaw motor on output 9 arrives as `ESC_TELEMETRY_9_TO_12`. While disarmed
ArduPilot holds the DShot output off, so no ESC frames arrive until the motor
is armed.

`ATTITUDE` is requested at rate 0, which disables it on the vehicle: it is
redundant with `ATTITUDE_QUATERNION`, which carries the same body rates, and
the overlay's roll/pitch/yaw are computed from the quaternion
(`src\attitude.ts`, ZYX convention). Other tools that need `ATTITUDE` (for
example `calibrate`) request their own stream when they run.

Changing the selector applies the new profile immediately. A manual choice is
kept until the connection generation changes, then the connection/baud-based selection
runs again.

The Wi-Fi profile has `minbaud=1500000`; slower serial links get the Radio
profile. High-rate display streams share capacity with file/log transfers.
`PID_TUNING` is sent by
the autopilot only for axes in `GCS_PID_MASK` (hardware defaults set it to 4,
yaw). Rates persist on the vehicle, so every profile sets `PID_TUNING`
explicitly.

The UI also consumes other messages when present:

- `HEARTBEAT` for mode/armed state;
- the RAWES diagnostic `DEBUG_FLOAT_ARRAY` (`array_id` 1) for the overlay's
  `YFF_U` yaw-motor command and `YFF_T` yaw trim (`H_YAW_TRIM`, the Lua
  observer output), both shown as inactive while disarmed; the key order is in
  `src\diag-array.ts` and owned by
  [flight stack](../design/flight_stack.md);
- `EKF_STATUS_REPORT` for status text (`status` prints the set flag names);
- `RPM.rpm1` for measured yaw-motor speed. The 3D view divides motor RPM by
  the 10:1 gearbox ratio and spins the rotor at that measured output speed.

Reads use `GET /v1/mavlink/messages` with `collapse=true` and
`max_lag_ms=1000`, so the browser does not replay stale backlog beyond one
second. A generation change clears reconstructed state and causes the display
message-rate configuration to be requested again.

The instrument-bay antenna and black arrow extend outward from the hub,
perpendicular to the axle on the FRD -X side, and follow the device's measured attitude.

The airframe model uses +X forward, +Y up toward the rotor, and +Z right.
Its model-to-FRD mapping is `(x, z, -y)`: model +Y is body -Z, while the
short tether stub extends along body +Z. A level attitude therefore puts the
rotor above the instrument bay and the tether below it. Apply the measured
body-to-NED attitude before mapping NED into the scene as `(east, -down, -north)`.
The stub is an airframe detail, not a reconstruction of the full tether to
the ground anchor.

The yellow reference arrow is driven by `ATTITUDE_TARGET`. Its quaternion `q`
is interpreted as `[w, x, y, z]`, and the arrow points along the target
upper-axis direction (opposite FRD body-down), not the current rotor axle. It
is hidden until a target arrives and cleared on disconnect or generation
change.

The target-error bullseye next to the swash instruments compares the measured
and target upper rotor-axis directions. Center means the directions coincide;
the dot shows forward/right error in the current hub frame and the number is
the total axis-angle error. It intentionally ignores yaw about the rotor axis.

Position, attitude, target, and RPM are display-only smoothed. The smoothing
time constant is 80 ms. The rotor holds the last valid RPM through telemetry
gaps; disconnects and generation changes clear reconstructed state.

## Yaw panel

The **Yaw** button beside the telemetry selector opens a drawer under the 3D
scene for diagnosing the anti-rotation loop. It needs the Wi-Fi telemetry
profile for full detail; over a slow link it still draws whatever arrives.

Four charts share one time axis (10, 30 or 60 s). Dashed lines are setpoints,
solid lines are measurements, and red shading is the error between them:

| Chart | Setpoint | Measured | Source |
| --- | --- | --- | --- |
| Heading (deg) | heading the target implies | actual heading | `ATTITUDE_TARGET`, `ATTITUDE_QUATERNION` |
| Yaw rate (rad/s) | `PID_TUNING.desired`, attitude-target rate, Lua `OL_YSP` | `PID_TUNING.achieved` | `PID_TUNING` (yaw axis), `ATTITUDE_TARGET`, `DEBUG_FLOAT_ARRAY` |
| Motor command (0-1) | stacked trim + FF + P + I + D | applied throttle (`servo9_raw`), observer target | `PID_TUNING`, `SERVO_OUTPUT_RAW`, `DEBUG_FLOAT_ARRAY` |
| Motor speed (RPM) | | measured RPM, applied throttle on its own 0-1 scale | `RPM`, `SERVO_OUTPUT_RAW` |

Details that matter when reading it:

- **Heading error is a quaternion twist about the rotor axis**, not a
  difference of Euler yaw angles, so it does not wrap at 180 degrees and stays
  defined with the rotor axis horizontal. The dashed heading line is the actual
  heading minus that error.
- **The motor command stack is exact for this vehicle.** ArduPilot sums
  `P+I+D+FF` and adds `H_YAW_TRIM` (the heli yaw output plus the DDFP trim), then clamps to 0-1. With
  `H_DDFP_SPIN_MIN=0`, `SPIN_MAX=1`, `THST_EXPO=0` and `H_COL2YAW=0` the applied
  throttle equals that clamped sum. Changing those parameters breaks the
  equality; the applied line still shows the truth. The trim comes from the Lua
  `YFF_T` (`H_YAW_TRIM`), which is only reported while armed, so it reads zero
  while disarmed. The dotted red lines mark the 0 and 1 limits and red ticks
  mark armed samples at a limit.
- **The observer line** is `YFF_U - YFF_GZ / YFF_A`, the throttle the Lua trim
  observer is converging to, using `RAWES_YAW_SLP` (read when the panel opens;
  `0` selects the script fallback slope 0.504 RPM/us).
- **Time alignment.** `ATTITUDE_*`, `SERVO_OUTPUT_RAW` and the Lua array carry
  the vehicle clock and are mapped to the PC clock by the smallest observed
  transit delay. `PID_TUNING`, `RPM` and ESC frames have no timestamp, so their
  arrival times are de-batched to handle bursty delivery by fitting the tightest
  line under the arrivals in blocks
  of about two seconds; a pause over one second starts a new segment. Their
  alignment with attitude is therefore approximate, to about 50 ms.
- **Gaps are real.** Lines break where frames were not received, and a link
  reconnect clears the chart.
- The readout strip shows rate-error RMS, heading-error RMS, the I term's share
  of the output, the share of armed samples at a limit, and the command-to-RPM
  delay (cross-correlation over the last 10 s, only shown above 0.5
  correlation). All use the last 10 s and only armed samples where noted.
- Hover reads every chart at the cursor. **Pause** freezes the view and lets
  you drag to scroll back up to a minute. Markers on the bottom axis show arm
  and disarm, mode changes and important status text.

The panel has its own journal reader (`src\yaw-recorder.ts`) because the
shared telemetry store reads collapsed frames, which would drop almost every
sample of a 300 Hz stream. It starts at the live end of the journal when the
panel opens and stops when it closes. The data model and maths
(`src\yaw-model.ts`, `src\yaw-math.ts`) are DOM-free and unit tested; drawing is
in `src\yaw-panel.ts`.

## Wobble panel

The **Wobble** button beside the telemetry selector opens a live Pixhawk
body-rate frequency analyzer. Like the yaw panel, its uncollapsed journal reader
only runs while the drawer is open. Opening either drawer closes the other.

The panel uses `ATTITUDE_QUATERNION.rollspeed`, `pitchspeed`, and `yawspeed` from
the Wi-Fi profile's 300 Hz stream. For each axis it resamples the latest
8-second window, removes the mean, applies a Hann window, and displays
the FFT spectrum and three strongest component frequencies. The selectable
frequency range changes only the displayed spectrum and peak list. All three
plots use one selectable absolute amplitude ceiling (default 0.25 rad/s), so
levels remain comparable across axes and over time. Peaks above the ceiling are
clamped and marked at the top edge; reported component amplitudes remain
unclamped in rad/s.

Occasional gaps do not restart the analysis window. Intervals longer than four
times the median sample interval have zero weight in the FFT rather than being
interpolated across. Amplitudes are normalized by the Hann weights of the
available grid samples, and all axes use the same time grid and missing-data
mask. The grid size follows the nominal sample interval, not the received sample
count, so a gap does not halve the frequency range. The status reports the window
duration, the percentage of usable grid samples, and the FFT grid rate. Missing
data can still introduce spectral leakage; coverage is not a guarantee of
spectral accuracy.

If fewer than 32 usable samples remain, all plots and peak lists retain the last
valid analysis and the status says "Rebuilding after telemetry gap - showing last
valid spectrum". They resume live analysis once enough samples are available.
If attitude telemetry stops for more than two seconds, the status instead says
"No recent attitude telemetry" and identifies any displayed spectrum as held.
Closing and reopening the panel, or a vehicle/journal generation reset, clears
the held analysis.

The fourth **Dominant mode** plot combines the complex FFT values from all three
body-rate axes at the strongest shared frequency. It draws the principal body
axis and reports roll/pitch/yaw axis shares. Its linearity score is derived from
the two axes of the polarization ellipse: a high score indicates oscillation
about one stable axis, while a low score is labeled circular/multi-axis rather
than presenting an unreliable single axis.

## Throughput and rate-policy status

Implemented today:

- explicit message-rate requests from the browser;
- display of LinkHub-reported `rx_bps` / `tx_bps`;
- a per-message-type RX/TX kbps table, shown by clicking the RX/TX readout
  beside the connection state (click again, click elsewhere, or press Esc to
  close it), built from LinkHub's `rx_bps_by_message` / `tx_bps_by_message`
  and sorted busiest first;
- LinkHub-side 1-second sampling and 3-second averaging;
- `null` throughput while disconnected.

Proposed, not implemented:

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
