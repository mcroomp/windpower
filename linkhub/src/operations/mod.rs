//! LinkHub's transfer jobs. MAVLink transaction and transfer mechanics live in
//! `mavlink-protocol`; this module only schedules and records their results.

use std::{ops::Deref, sync::Arc, time::Duration};

use mavlink_protocol::client::{Protocol, TransferError, TransferSummary, new_protocol};

use crate::{
    mavlink::MavlinkLinkHandle,
    records::wall_time_ns,
    transfer::{Outcome, TransferHandle, TransferInfo, TransferKind, TransferRegistry},
};

#[derive(Clone)]
pub struct MavlinkOperations {
    protocol: Arc<dyn Protocol>,
    transfers: TransferRegistry,
}

impl Deref for MavlinkOperations {
    type Target = dyn Protocol;

    fn deref(&self) -> &Self::Target {
        self.protocol.as_ref()
    }
}

impl MavlinkOperations {
    #[must_use]
    pub fn new(link: MavlinkLinkHandle, transfers: TransferRegistry) -> Self {
        Self {
            protocol: new_protocol(Arc::new(link), wall_time_ns() as u16),
            transfers,
        }
    }

    #[must_use]
    pub fn transfers(&self) -> &TransferRegistry {
        &self.transfers
    }

    #[must_use]
    pub fn start_file_download(
        &self,
        path: &str,
        verify_crc: bool,
        stall_timeout: Duration,
    ) -> TransferInfo {
        let handle = self.transfers.submit(TransferKind::FtpDownload, path);
        let info = self.transfers.get(handle.id()).expect("submitted transfer");
        let client = self.protocol.clone();
        let path = path.to_owned();
        tokio::spawn(async move {
            let mut summary = TransferSummary::default();
            let result = client
                .download_file(&handle, &path, verify_crc, stall_timeout, &mut summary)
                .await;
            finish(handle, result.map(Outcome::Complete), summary).await;
        });
        info
    }

    #[must_use]
    pub fn start_file_upload(
        &self,
        path: &str,
        content: Vec<u8>,
        stall_timeout: Duration,
    ) -> TransferInfo {
        let handle = self.transfers.submit(TransferKind::FtpUpload, path);
        let info = self.transfers.get(handle.id()).expect("submitted transfer");
        let client = self.protocol.clone();
        let path = path.to_owned();
        tokio::spawn(async move {
            let mut summary = TransferSummary::default();
            let result = client
                .upload_file(&handle, &path, content, stall_timeout, &mut summary)
                .await;
            finish(handle, result.map(|()| Outcome::CompleteNoContent), summary).await;
        });
        info
    }

    #[must_use]
    pub fn start_log_download(
        &self,
        log_id: u16,
        packet_timeout: Duration,
        max_retries: u32,
    ) -> TransferInfo {
        let handle = self
            .transfers
            .submit(TransferKind::LogDownload, format!("log {log_id}"));
        let info = self.transfers.get(handle.id()).expect("submitted transfer");
        let client = self.protocol.clone();
        tokio::spawn(async move {
            let mut summary = TransferSummary::default();
            let result = client
                .download_log(&handle, log_id, packet_timeout, max_retries, &mut summary)
                .await;
            finish(handle, result.map(Outcome::Complete), summary).await;
        });
        info
    }
}

async fn finish(
    handle: TransferHandle,
    result: Result<Outcome, TransferError>,
    summary: TransferSummary,
) {
    let outcome = match result {
        Ok(outcome) => outcome,
        Err(TransferError::Cancelled) => Outcome::Cancelled,
        Err(error) => Outcome::Failed(error.to_string()),
    };
    handle
        .finish(outcome, summary.done, summary.received, summary.stats)
        .await;
}
