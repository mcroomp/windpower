# mavlink-protocol

The MAVLink `ardupilotmega` dialect LinkHub speaks, generated at build time
with `mavlink-bindgen` on top of `mavlink-core`, the same pair the `mavlink`
crate is made of. The difference is the XML: instead of the snapshot bundled in
the `mavlink` crate, it is [ArduPilot's own](https://github.com/ArduPilot/mavlink)
definitions, vendored in [definitions/](definitions) at the exact commit that the
ArduPilot release under test builds from. LinkHub's message and enumeration set
therefore matches the firmware's instead of an unrelated upstream snapshot.

The default `codec` feature exposes `mavlink_protocol::codec`, which owns binary
framing, typed-message JSON conversion, and concrete enum/bitmask JSON adapters.
Callers may construct and match generated types, but must route MAVLink
serialization through the codec so its code generation is reused across
application rebuilds. See the [architecture owner](../../design/linkhub.md#mavlink-value-representation)
for the boundary and wire representation.

The `client` module exposes an async `Protocol` trait. `client::new_protocol`
accepts an `Arc<dyn transport::Transport>` supplied by the host and returns
`Arc<dyn client::Protocol>` backed by a private implementation. `#[async_trait]`
generates boxed `Send` futures so callers can `.await` typed results without
compiling the concrete protocol futures. It owns command, parameter and message
transactions plus MAVFTP and DataFlash protocol drivers. The `mavftp`,
`ftp_read`, `dataflash` and `stats` modules provide packet/state-machine types
and transfer accounting. LinkHub's host adapter retains actual transmission,
connection and journal ownership; application transfer jobs retain progress,
cancellation and content storage integration. See the architecture owner above
for the complete ownership boundary.

[ARDUPILOT_VERSION](ARDUPILOT_VERSION) records the pin and a SHA-256 per file:

```text
tag=Copter-4.7.1
mavlink_commit=288b907c384a892c8519bfe271682424b1e1a3a0
sha256.ardupilotmega.xml=...
```

`tag` must equal `ARDUPILOT_TAG` in [simulation/Dockerfile](../../simulation/Dockerfile),
and the files must be unmodified; `cargo test -p mavlink-protocol` fails when
either drifts. The files are exact copies (marked `-text` in `.gitattributes`):
never edit them by hand.

## Moving to a new ArduPilot release

1. Change `ARDUPILOT_TAG` in `simulation/Dockerfile` (see its notes).
2. Re-vendor the definitions:

   ```powershell
   uv run python scripts/update_mavlink_definitions.py
   ```

   It reads the tag from the Dockerfile, resolves the `ArduPilot/mavlink` commit
   that ArduPilot tag pins as `modules/mavlink` through the GitHub API
   (`GITHUB_TOKEN` avoids the unauthenticated rate limit), downloads
   `ardupilotmega.xml` and everything it includes at that commit, and rewrites
   `ARDUPILOT_VERSION`. Pass `--tag <tag>` to override the Dockerfile.
3. Build and test LinkHub, regenerate the checked protocol artifacts
   (`cargo run -p linkhub-clientgen`, see [../README.md](../README.md)), and fix
   any Python, TypeScript or Rust caller of a message or enumeration member the
   new definitions renamed or removed.

`uv run python scripts/update_mavlink_definitions.py --check` verifies the vendored
files against GitHub without changing anything.

## Build dependencies

`mavlink-core` and `mavlink-bindgen` come from
[mavlink/rust-mavlink](https://github.com/mavlink/rust-mavlink) through crates.io
at the same release. Version 0.19.0 is the minimum usable release because older
versions of `mavlink-bindgen` reject ArduPilot's definitions, where a 16-bit and
a 32-bit field share `GIMBAL_DEVICE_CAP_FLAGS`
([rust-mavlink#495](https://github.com/mavlink/rust-mavlink/issues/495), fixed by
[#500](https://github.com/mavlink/rust-mavlink/pull/500)).
