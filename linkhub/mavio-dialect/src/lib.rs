pub mod generated {
    include!(concat!(env!("OUT_DIR"), "/mavlink/mod.rs"));
}

pub use generated::dialects;

include!(concat!(env!("OUT_DIR"), "/mavlink_registry.rs"));
