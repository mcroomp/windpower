use serde::Serialize;

/// Counters describing how a transfer coped with the link. They are journaled
/// when a transfer ends and served live from `GET /v1/mavlink/transfers`, so a
/// slow or lossy transfer can be diagnosed from the record alone.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq, Serialize)]
pub struct TransferStats {
    /// Data packets received, including duplicates.
    pub packets_received: u64,
    /// Data packets for bytes that had already arrived (overlapping streams or
    /// retransmissions that crossed an in-flight reply).
    pub duplicate_packets: u64,
    /// Times a packet arrived beyond the next expected offset, i.e. a hole opened.
    pub gaps: u64,
    /// Requests sent for the first time (bursts, windows and repair reads).
    pub requests_sent: u64,
    /// Burst or window requests among `requests_sent`.
    pub bursts: u64,
    /// Requests sent only to fill a hole.
    pub repair_requests: u64,
    /// Requests sent again after their reply never arrived.
    pub retransmits: u64,
    /// Waits that ended without data.
    pub timeouts: u64,
    /// NAK replies received.
    pub naks: u64,
}

impl TransferStats {
    /// Fraction of received packets that carried nothing new.
    #[must_use]
    pub fn duplicate_ratio(&self) -> f64 {
        if self.packets_received == 0 {
            0.0
        } else {
            self.duplicate_packets as f64 / self.packets_received as f64
        }
    }
}
