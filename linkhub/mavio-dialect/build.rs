use std::{env::var, fmt::Write, fs, path::Path};

use mavinspect::protocol::Filter;
use mavspec::rust::r#gen::BuildHelper;

const MESSAGES: &[&str] = &[
    "ATTITUDE",
    "ATTITUDE_QUATERNION",
    "ATTITUDE_TARGET",
    "AUTOPILOT_VERSION",
    "BATTERY_STATUS",
    "COMMAND_ACK",
    "COMMAND_LONG",
    "EKF_STATUS_REPORT",
    "EXTENDED_SYS_STATE",
    "FILE_TRANSFER_PROTOCOL",
    "GLOBAL_POSITION_INT",
    "HEARTBEAT",
    "LOCAL_POSITION_NED",
    "LOG_DATA",
    "LOG_ENTRY",
    "LOG_REQUEST_DATA",
    "LOG_REQUEST_END",
    "LOG_REQUEST_LIST",
    "MESSAGE_INTERVAL",
    "NAMED_VALUE_FLOAT",
    "NAMED_VALUE_INT",
    "PARAM_REQUEST_LIST",
    "PARAM_REQUEST_READ",
    "PARAM_SET",
    "PARAM_VALUE",
    "PID_TUNING",
    "RC_CHANNELS",
    "REQUEST_DATA_STREAM",
    "SERVO_OUTPUT_RAW",
    "SET_ATTITUDE_TARGET",
    "STATUSTEXT",
    "SYS_STATUS",
];

fn main() {
    println!("cargo:rerun-if-changed=build.rs");

    let out_dir = var("OUT_DIR").expect("OUT_DIR is set");
    let destination = Path::new(&out_dir).join("mavlink");
    let protocol = mavlink_message_definitions::protocol()
        .clone()
        .with_dialects_included(&["ardupilotmega"], true, None);
    let filtered = protocol.filtered(&Filter::new().with_messages(MESSAGES));

    BuildHelper::builder(destination)
        .set_protocol(filtered)
        .set_serde(true)
        .generate()
        .expect("generate filtered MAVLink dialect");

    let dialect = protocol
        .get_dialect_by_name("ardupilotmega")
        .expect("ArduPilotMega dialect is available");
    let mut entries = dialect.messages().into_iter().collect::<Vec<_>>();
    entries.sort_by_key(|message| message.id());
    let mut registry = String::from(
        "pub const fn message_info(message_id: u32) -> Option<(&'static str, u8)> {\n    Some(match message_id {\n",
    );
    for message in entries {
        writeln!(
            registry,
            "        {} => ({:?}, {}),",
            message.id(),
            message.name(),
            message.crc_extra()
        )
        .expect("write registry entry");
    }
    registry.push_str("        _ => return None,\n    })\n}\n");
    fs::write(Path::new(&out_dir).join("mavlink_registry.rs"), registry)
        .expect("write MAVLink metadata registry");
}
