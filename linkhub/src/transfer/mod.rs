//! LinkHub's transfer registry, cancellation and progress journal.

mod registry;

pub use registry::{
    CancelResult, ContentError, Outcome, TransferHandle, TransferInfo, TransferKind,
    TransferRegistry, TransferState,
};
