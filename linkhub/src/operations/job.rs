//! What the transfer workers share: how a job ends and how it waits its turn.

use super::OperationError;
use crate::transfer::{Outcome, TransferHandle};

/// Why a transfer stopped before delivering a verified file.
pub(super) enum Failure {
    /// The caller cancelled it.
    Cancelled,
    Error(OperationError),
}

impl<E: Into<OperationError>> From<E> for Failure {
    fn from(error: E) -> Self {
        Self::Error(error.into())
    }
}

impl Failure {
    pub(super) fn into_outcome(self) -> Outcome {
        match self {
            Self::Cancelled => Outcome::Cancelled,
            Self::Error(error) => Outcome::Failed(error.to_string()),
        }
    }
}

/// Waits for `lock` while staying cancellable, so a queued job can be dropped.
pub(super) async fn acquire<T: ?Sized>(
    lock: std::sync::Arc<tokio::sync::Mutex<T>>,
    handle: &mut TransferHandle,
) -> Result<tokio::sync::OwnedMutexGuard<T>, Failure> {
    tokio::select! {
        guard = lock.lock_owned() => Ok(guard),
        () = handle.cancelled() => Err(Failure::Cancelled),
    }
}
