//! The MAVLink `ardupilotmega` dialect generated from ArduPilot's own
//! definitions.
//!
//! This mirrors the `mavlink` crate (generated dialects plus `mavlink-core`),
//! but its XML is vendored from the `ArduPilot/mavlink` commit that the ArduPilot
//! release under test builds from, so LinkHub's message set matches the
//! firmware's. See `README.md` for how to move to a new ArduPilot release.

#[allow(clippy::all, clippy::pedantic, deprecated)]
pub mod dialects {
    include!(concat!(env!("OUT_DIR"), "/mod.rs"));
}

// The generated code addresses `mavlink-core` items through `crate::`.
pub use mavlink_core::*;

#[cfg(test)]
mod tests {
    use std::{collections::BTreeMap, fs, path::Path};

    use sha2::{Digest, Sha256};

    const MANIFEST_DIR: &str = env!("CARGO_MANIFEST_DIR");

    fn recorded() -> BTreeMap<String, String> {
        let text = fs::read_to_string(Path::new(MANIFEST_DIR).join("ARDUPILOT_VERSION"))
            .expect("ARDUPILOT_VERSION");
        text.lines()
            .filter_map(|line| line.split_once('='))
            .map(|(key, value)| (key.trim().to_owned(), value.trim().to_owned()))
            .collect()
    }

    #[test]
    fn dialect_is_pinned_to_the_simulated_ardupilot_release() {
        let dockerfile =
            fs::read_to_string(Path::new(MANIFEST_DIR).join("../../simulation/Dockerfile"))
                .expect("simulation/Dockerfile");
        let docker_tag = dockerfile
            .lines()
            .find_map(|line| line.strip_prefix("ARG ARDUPILOT_TAG="))
            .expect("Dockerfile ARDUPILOT_TAG")
            .trim();

        assert_eq!(
            recorded()["tag"],
            docker_tag,
            "linkhub/dialect/ARDUPILOT_VERSION and simulation/Dockerfile ARDUPILOT_TAG must name \
             the same ArduPilot release; run scripts/update_mavlink_definitions.py"
        );
    }

    #[test]
    fn vendored_definitions_are_unmodified_from_the_recorded_commit() {
        let recorded = recorded();
        let definitions = Path::new(MANIFEST_DIR).join("definitions");
        let mut on_disk: Vec<String> = fs::read_dir(&definitions)
            .expect("definitions directory")
            .map(|entry| {
                entry
                    .expect("entry")
                    .file_name()
                    .to_string_lossy()
                    .into_owned()
            })
            .filter(|name| name.ends_with(".xml"))
            .collect();
        on_disk.sort();
        let expected: Vec<String> = recorded
            .keys()
            .filter_map(|key| key.strip_prefix("sha256.").map(str::to_owned))
            .collect();

        assert_eq!(
            on_disk, expected,
            "definitions/ and ARDUPILOT_VERSION list different files"
        );
        for name in expected {
            let digest = Sha256::digest(fs::read(definitions.join(&name)).expect("definition"));
            assert_eq!(
                format!("{digest:x}"),
                recorded[&format!("sha256.{name}")],
                "{name} differs from ArduPilot/mavlink {}; run scripts/update_mavlink_definitions.py",
                recorded["mavlink_commit"]
            );
        }
    }

    #[test]
    fn dialect_contains_messages_arducopter_sends_that_the_crates_io_snapshot_lacks() {
        use crate::{Message, dialects::ardupilotmega::MavMessage};

        assert_eq!(MavMessage::message_id_from_name("RELAY_STATUS"), Some(376));
        assert_eq!(MavMessage::message_id_from_name("HEARTBEAT"), Some(0));
    }
}
