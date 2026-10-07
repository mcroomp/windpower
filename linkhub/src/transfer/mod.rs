//! Transfers over the MAVLink link: a loss-tolerant file reader, per-transfer
//! statistics, and a registry that makes every transfer observable.

pub mod ftp_read;
mod registry;
mod stats;

pub use ftp_read::{FtpReadError, FtpReader, Timing};
pub use registry::{
    CancelResult, ContentError, Outcome, TransferHandle, TransferInfo, TransferKind,
    TransferRegistry, TransferState,
};
pub use stats::TransferStats;
