//! Generates the `ardupilotmega` dialect from ArduPilot's own MAVLink
//! definitions, vendored in `definitions/` at the `ArduPilot/mavlink` commit that
//! the ArduPilot release under test builds from (see `ARDUPILOT_VERSION` and
//! `scripts/update_mavlink_definitions.py`).

use std::{env, path::PathBuf, process::ExitCode};

use mavlink_bindgen::XmlDefinitions;

fn main() -> ExitCode {
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").expect("manifest dir"));
    let source = manifest_dir.join("definitions");
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed={}", source.display());

    let definition = source.join("ardupilotmega.xml");
    if !definition.is_file() {
        eprintln!(
            "MAVLink definitions not found in {}.\n\
             Vendor them with: uv run python scripts/update_mavlink_definitions.py",
            source.display()
        );
        return ExitCode::FAILURE;
    }

    let out_dir = env::var("OUT_DIR").expect("out dir");
    match mavlink_bindgen::generate(XmlDefinitions::Files(vec![definition]), out_dir) {
        Ok(generated) => {
            mavlink_bindgen::emit_cargo_build_messages(&generated);
            ExitCode::SUCCESS
        }
        Err(error) => {
            eprintln!("{error}");
            ExitCode::FAILURE
        }
    }
}
