use std::collections::{BTreeMap, HashMap};

use thiserror::Error;

pub const LOG_DATA_LEN: usize = 90;
pub const REQUEST_BLOCK_LEN: u32 = (LOG_DATA_LEN * 128) as u32;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Target {
    pub system: u8,
    pub component: u8,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ListRequest {
    pub target: Target,
    pub start: u16,
    pub end: u16,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct LogEntry {
    pub id: u16,
    pub size: u32,
    pub time_utc: u32,
    pub num_logs: u16,
    pub last_log_num: u16,
}

#[derive(Debug)]
pub struct LogList {
    request: ListRequest,
    entries: HashMap<u16, LogEntry>,
    expected: Option<usize>,
}

impl LogList {
    pub fn new(target: Target, start: u16, end: u16) -> Result<Self, DataFlashError> {
        if start > end {
            return Err(DataFlashError::InvalidRange);
        }
        Ok(Self {
            request: ListRequest { target, start, end },
            entries: HashMap::new(),
            expected: None,
        })
    }

    pub fn request(&self) -> ListRequest {
        self.request
    }

    /// Accepts duplicate and out-of-order LOG_ENTRY messages.
    pub fn receive(&mut self, entry: LogEntry) -> Result<(), DataFlashError> {
        if entry.id < self.request.start || entry.id > self.request.end {
            return Err(DataFlashError::UnexpectedLog(entry.id));
        }
        let expected = usize::from(entry.num_logs);
        if let Some(previous) = self.expected
            && previous != expected
        {
            return Err(DataFlashError::InconsistentLogCount {
                expected: previous,
                actual: expected,
            });
        }
        self.expected = Some(expected);
        self.entries.insert(entry.id, entry);
        Ok(())
    }

    pub fn is_complete(&self) -> bool {
        match self.expected {
            Some(0) => true,
            Some(_) if self.request.start == self.request.end => !self.entries.is_empty(),
            Some(expected) => self.entries.len() >= expected,
            None => false,
        }
    }

    pub fn entries(&self) -> Vec<LogEntry> {
        let mut entries: Vec<_> = self.entries.values().cloned().collect();
        entries.sort_by_key(|entry| entry.id);
        entries
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct DataRequest {
    pub target: Target,
    pub id: u16,
    pub offset: u32,
    pub count: u32,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DataPacket {
    pub id: u16,
    pub offset: u32,
    pub count: u8,
    pub data: [u8; LOG_DATA_LEN],
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct EndRequest {
    pub target: Target,
}

#[derive(Debug, Error, Eq, PartialEq)]
pub enum DataFlashError {
    #[error("DataFlash list range start must not exceed end")]
    InvalidRange,
    #[error("unexpected DataFlash log id {0}")]
    UnexpectedLog(u16),
    #[error("LOG_ENTRY num_logs changed from {expected} to {actual}")]
    InconsistentLogCount { expected: usize, actual: usize },
    #[error("DataFlash packet has invalid count {0}")]
    InvalidCount(u8),
    #[error("DataFlash log ended early at {offset}/{size} bytes")]
    EarlyEnd { offset: u32, size: u32 },
    #[error("DataFlash packet at {offset} extends beyond requested block ending at {block_end}")]
    BeyondRequestedRange { offset: u32, block_end: u32 },
    #[error("DataFlash packet overlaps assembled data at {0}")]
    OverlappingPacket(u32),
    #[error("DataFlash log {id} stalled at {offset}/{size} bytes after {retries} retries")]
    Stalled {
        id: u16,
        offset: u32,
        size: u32,
        retries: u32,
    },
}

/// Reassembles LOG_DATA by offset. Future packets are retained while a missing
/// packet is re-requested, so reversed and otherwise out-of-order delivery is
/// lossless.
#[derive(Debug)]
pub struct LogDownload {
    target: Target,
    id: u16,
    size: u32,
    max_retries: u32,
    retries: u32,
    offset: u32,
    block_end: u32,
    request_due: bool,
    pending: BTreeMap<u32, Vec<u8>>,
    content: Vec<u8>,
}

impl LogDownload {
    pub fn new(target: Target, id: u16, size: u32, max_retries: u32) -> Self {
        Self {
            target,
            id,
            size,
            max_retries,
            retries: 0,
            offset: 0,
            block_end: size.min(REQUEST_BLOCK_LEN),
            request_due: size > 0,
            pending: BTreeMap::new(),
            content: Vec::with_capacity(size as usize),
        }
    }

    pub fn next_request(&mut self) -> Option<DataRequest> {
        if !self.request_due || self.is_complete() {
            return None;
        }
        self.request_due = false;
        Some(DataRequest {
            target: self.target,
            id: self.id,
            offset: self.offset,
            count: self.block_end - self.offset,
        })
    }

    pub fn receive(&mut self, packet: DataPacket) -> Result<(), DataFlashError> {
        if packet.id != self.id {
            return Err(DataFlashError::UnexpectedLog(packet.id));
        }
        if packet.count == 0 {
            return Err(DataFlashError::EarlyEnd {
                offset: packet.offset,
                size: self.size,
            });
        }
        if usize::from(packet.count) > LOG_DATA_LEN {
            return Err(DataFlashError::InvalidCount(packet.count));
        }
        let packet_end = packet.offset + u32::from(packet.count);
        if packet.offset >= self.block_end || packet_end > self.block_end {
            return Err(DataFlashError::BeyondRequestedRange {
                offset: packet.offset,
                block_end: self.block_end,
            });
        }
        if packet_end <= self.offset {
            return Ok(());
        }
        if packet.offset < self.offset {
            return Err(DataFlashError::OverlappingPacket(packet.offset));
        }
        self.pending
            .entry(packet.offset)
            .or_insert_with(|| packet.data[..usize::from(packet.count)].to_vec());
        self.drain_contiguous();
        Ok(())
    }

    fn drain_contiguous(&mut self) {
        let mut progressed = false;
        while let Some(chunk) = self.pending.remove(&self.offset) {
            self.offset += chunk.len() as u32;
            self.content.extend_from_slice(&chunk);
            progressed = true;
        }
        if progressed {
            self.retries = 0;
        }
        if self.offset == self.block_end && self.offset < self.size {
            self.block_end = self.size.min(self.offset + REQUEST_BLOCK_LEN);
            self.request_due = true;
        }
    }

    /// Marks the current receive wait as timed out and schedules a request for
    /// the missing suffix. Already buffered future packets remain available.
    pub fn on_timeout(&mut self) -> Result<(), DataFlashError> {
        if self.is_complete() {
            return Ok(());
        }
        if self.retries >= self.max_retries {
            return Err(DataFlashError::Stalled {
                id: self.id,
                offset: self.offset,
                size: self.size,
                retries: self.max_retries,
            });
        }
        self.retries += 1;
        self.request_due = true;
        Ok(())
    }

    pub fn is_complete(&self) -> bool {
        self.offset == self.size
    }

    pub fn bytes(&self) -> Option<&[u8]> {
        self.is_complete().then_some(&self.content)
    }

    pub fn into_bytes(self) -> Option<Vec<u8>> {
        self.is_complete().then_some(self.content)
    }

    pub fn end_request(&self) -> EndRequest {
        EndRequest {
            target: self.target,
        }
    }

    pub fn offset(&self) -> u32 {
        self.offset
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const TARGET: Target = Target {
        system: 1,
        component: 0,
    };

    fn packet(id: u16, offset: u32, content: &[u8]) -> DataPacket {
        let mut data = [0; LOG_DATA_LEN];
        data[..content.len()].copy_from_slice(content);
        DataPacket {
            id,
            offset,
            count: content.len() as u8,
            data,
        }
    }

    #[test]
    fn list_deduplicates_and_orders_entries() {
        let mut list = LogList::new(TARGET, 0, u16::MAX).unwrap();
        for id in [8, 7, 8] {
            list.receive(LogEntry {
                id,
                size: 12,
                time_utc: 100,
                num_logs: 2,
                last_log_num: 8,
            })
            .unwrap();
        }
        assert!(list.is_complete());
        assert_eq!(
            list.entries()
                .iter()
                .map(|entry| entry.id)
                .collect::<Vec<_>>(),
            [7, 8]
        );
    }

    #[test]
    fn download_reorders_packets_and_retries_only_missing_suffix() {
        let content: Vec<_> = (0..311).map(|index| (index % 251) as u8).collect();
        let mut download = LogDownload::new(TARGET, 7, content.len() as u32, 2);
        assert_eq!(download.next_request().unwrap().offset, 0);

        download
            .receive(packet(7, 180, &content[180..270]))
            .unwrap();
        download.receive(packet(7, 90, &content[90..180])).unwrap();
        assert_eq!(download.offset(), 0);
        download.on_timeout().unwrap();
        let retry = download.next_request().unwrap();
        assert_eq!(retry.offset, 0);
        assert_eq!(retry.count, content.len() as u32);

        download.receive(packet(7, 0, &content[..90])).unwrap();
        assert_eq!(download.offset(), 270);
        download.receive(packet(7, 270, &content[270..])).unwrap();
        assert_eq!(download.bytes().unwrap(), content);
        assert_eq!(download.end_request(), EndRequest { target: TARGET });
    }

    #[test]
    fn timeout_limit_reports_missing_offset() {
        let mut download = LogDownload::new(TARGET, 3, 100, 1);
        download.next_request();
        download.on_timeout().unwrap();
        download.next_request();
        assert_eq!(
            download.on_timeout(),
            Err(DataFlashError::Stalled {
                id: 3,
                offset: 0,
                size: 100,
                retries: 1,
            })
        );
    }

    #[test]
    fn rejects_zero_and_out_of_range_packets() {
        let mut download = LogDownload::new(TARGET, 2, 100, 1);
        let mut zero = packet(2, 0, &[]);
        zero.count = 0;
        assert!(matches!(
            download.receive(zero),
            Err(DataFlashError::EarlyEnd { .. })
        ));
        download.on_timeout().unwrap();
        assert_eq!(
            download.next_request(),
            Some(DataRequest {
                target: TARGET,
                id: 2,
                offset: 0,
                count: 100,
            })
        );
        assert!(matches!(
            download.receive(packet(2, 90, &[1; 20])),
            Err(DataFlashError::BeyondRequestedRange { .. })
        ));
    }
}
