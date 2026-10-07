use std::collections::{BTreeMap, HashMap};

use thiserror::Error;

use crate::transfer::TransferStats;

pub const LOG_DATA_LEN: usize = 90;
/// Bytes requested per LOG_REQUEST_DATA window. A lost packet only costs a
/// one-hole repair, so a large window keeps the autopilot streaming.
pub const REQUEST_BLOCK_LEN: u32 = (LOG_DATA_LEN * 512) as u32;

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
/// lossless. Only the first hole is re-requested, as soon as the end of the
/// requested window has arrived (or after a silent timeout), so a lost packet
/// neither stalls the transfer nor resends data that already arrived.
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
    /// The last packet of the current window has arrived.
    tail_seen: bool,
    /// End of the hole the outstanding repair request covers, if one is in flight.
    repair_end: Option<u32>,
    /// The next request repeats one that drew no data.
    resend: bool,
    /// End of the furthest packet seen, to recognise a packet that skips ahead.
    highest_end: u32,
    stats: TransferStats,
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
            tail_seen: false,
            repair_end: None,
            resend: false,
            highest_end: 0,
            stats: TransferStats::default(),
            pending: BTreeMap::new(),
            content: Vec::with_capacity(size as usize),
        }
    }

    pub fn next_request(&mut self) -> Option<DataRequest> {
        if !self.request_due || self.is_complete() {
            return None;
        }
        self.request_due = false;
        // Buffered packets beyond the offset bound the first hole.
        let hole_end = self
            .pending
            .keys()
            .next()
            .map_or(self.block_end, |first| (*first).min(self.block_end));
        self.repair_end = (hole_end < self.block_end).then_some(hole_end);
        if self.resend {
            self.resend = false;
            self.stats.retransmits += 1;
        } else {
            self.stats.requests_sent += 1;
            if self.repair_end.is_some() {
                self.stats.repair_requests += 1;
            } else {
                self.stats.bursts += 1;
            }
        }
        Some(DataRequest {
            target: self.target,
            id: self.id,
            offset: self.offset,
            count: hole_end - self.offset,
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
        self.stats.packets_received += 1;
        if packet_end <= self.offset {
            self.stats.duplicate_packets += 1;
            return Ok(());
        }
        if packet.offset < self.offset {
            return Err(DataFlashError::OverlappingPacket(packet.offset));
        }
        if self.pending.contains_key(&packet.offset) {
            self.stats.duplicate_packets += 1;
        }
        if packet.offset > self.highest_end.max(self.offset) {
            self.stats.gaps += 1;
        }
        self.highest_end = self.highest_end.max(packet_end);
        let block_end = self.block_end;
        let is_tail = packet_end == block_end;
        self.pending
            .entry(packet.offset)
            .or_insert_with(|| packet.data[..usize::from(packet.count)].to_vec());
        self.drain_contiguous();
        if is_tail && self.block_end == block_end {
            self.tail_seen = true;
        }
        self.schedule_repair();
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
            self.tail_seen = false;
            self.repair_end = None;
        }
    }

    /// Re-requests the first hole without waiting for a timeout once the
    /// stream that should have filled it has demonstrably finished.
    fn schedule_repair(&mut self) {
        if self.request_due || self.is_complete() {
            return;
        }
        let has_hole = !self.pending.is_empty();
        match self.repair_end {
            Some(end) if self.offset < end => {}
            Some(_) => {
                self.repair_end = None;
                self.request_due = has_hole;
            }
            None => self.request_due = has_hole && self.tail_seen,
        }
    }

    /// Marks the current receive wait as timed out and schedules a request for
    /// the first missing range. Already buffered future packets remain available.
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
        self.stats.timeouts += 1;
        self.resend = true;
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

    #[must_use]
    pub fn stats(&self) -> TransferStats {
        self.stats
    }

    /// Bytes that have arrived anywhere, including beyond a hole.
    #[must_use]
    pub fn received_bytes(&self) -> u64 {
        self.offset as u64
            + self
                .pending
                .values()
                .map(|chunk| chunk.len() as u64)
                .sum::<u64>()
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
    fn download_rerequests_only_the_first_hole_after_a_timeout() {
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
        assert_eq!(retry.count, 90, "only the hole, not the buffered suffix");

        download.receive(packet(7, 0, &content[..90])).unwrap();
        assert_eq!(download.offset(), 270);
        download.receive(packet(7, 270, &content[270..])).unwrap();
        assert_eq!(download.bytes().unwrap(), content);
        assert_eq!(download.end_request(), EndRequest { target: TARGET });
    }

    #[test]
    fn download_repairs_holes_as_soon_as_the_window_tail_arrives() {
        // Three 90-byte packets and a short fourth; the first and third are lost.
        let content: Vec<_> = (0..300).map(|index| (index % 251) as u8).collect();
        let mut download = LogDownload::new(TARGET, 9, content.len() as u32, 3);
        let first = download.next_request().unwrap();
        assert_eq!((first.offset, first.count), (0, 300));

        download.receive(packet(9, 90, &content[90..180])).unwrap();
        assert!(
            download.next_request().is_none(),
            "a gap alone is not evidence of loss until the window ends"
        );
        download.receive(packet(9, 270, &content[270..])).unwrap();
        // The tail arrived with holes behind it: repair the first hole at once.
        let repair = download.next_request().unwrap();
        assert_eq!((repair.offset, repair.count), (0, 90));

        download.receive(packet(9, 0, &content[..90])).unwrap();
        assert_eq!(download.offset(), 180);
        // Hole one is filled; the next hole (180..270) is requested immediately.
        let repair = download.next_request().unwrap();
        assert_eq!((repair.offset, repair.count), (180, 90));
        download
            .receive(packet(9, 180, &content[180..270]))
            .unwrap();
        assert!(download.is_complete());
        assert_eq!(download.bytes().unwrap(), content);
    }

    #[test]
    fn download_ignores_duplicates_from_overlapping_streams() {
        let content: Vec<_> = (0..180).map(|index| index as u8).collect();
        let mut download = LogDownload::new(TARGET, 4, content.len() as u32, 2);
        download.next_request();
        download.receive(packet(4, 0, &content[..90])).unwrap();
        download.receive(packet(4, 0, &content[..90])).unwrap();
        download.receive(packet(4, 90, &content[90..])).unwrap();
        download.receive(packet(4, 90, &content[90..])).unwrap();
        assert_eq!(download.bytes().unwrap(), content);
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
