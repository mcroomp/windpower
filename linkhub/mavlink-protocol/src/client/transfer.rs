//! Progress and cancellation hooks; scheduling and job retention belong to the host.

use crate::{stats::TransferStats, transport::BoxFuture};

use super::OperationError;

pub trait TransferObserver: Send + Sync {
    fn is_cancelled(&self) -> bool;
    fn cancelled(&self) -> BoxFuture<'_, ()>;
    fn start(&self, total_bytes: u64) -> BoxFuture<'_, ()>;
    fn progress(&self, done_bytes: u64, received_bytes: u64, stats: TransferStats);
}

#[derive(Default)]
pub struct TransferSummary {
    pub done: u64,
    pub received: u64,
    pub stats: TransferStats,
}

#[derive(Debug, thiserror::Error)]
pub enum TransferError {
    #[error("transfer cancelled")]
    Cancelled,
    #[error(transparent)]
    Error(OperationError),
    #[error("transfer cancelled; cleanup failed: {0}")]
    CancelledCleanup(OperationError),
}

pub(super) fn merge_cleanup<T>(
    result: Result<T, TransferError>,
    cleanup: Result<(), OperationError>,
) -> Result<T, TransferError> {
    match (result, cleanup) {
        (result, Ok(())) => result,
        (Ok(_), Err(cleanup)) => Err(cleanup.into()),
        (Err(TransferError::Cancelled), Err(cleanup)) => {
            Err(TransferError::CancelledCleanup(cleanup))
        }
        (Err(TransferError::Error(primary)), Err(cleanup)) => Err(OperationError::Cleanup {
            primary: Box::new(primary),
            cleanup: Box::new(cleanup),
        }
        .into()),
        (Err(TransferError::CancelledCleanup(primary)), Err(cleanup)) => {
            Err(TransferError::CancelledCleanup(OperationError::Cleanup {
                primary: Box::new(primary),
                cleanup: Box::new(cleanup),
            }))
        }
    }
}

impl<E: Into<OperationError>> From<E> for TransferError {
    fn from(error: E) -> Self {
        Self::Error(error.into())
    }
}

pub(super) async fn acquire<T: Send + ?Sized>(
    lock: std::sync::Arc<tokio::sync::Mutex<T>>,
    observer: &dyn TransferObserver,
) -> Result<tokio::sync::OwnedMutexGuard<T>, TransferError> {
    if observer.is_cancelled() {
        return Err(TransferError::Cancelled);
    }
    tokio::select! {
        guard = lock.lock_owned() => Ok(guard),
        () = observer.cancelled() => Err(TransferError::Cancelled),
    }
}
