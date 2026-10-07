//! A loss-tolerant MAVFTP file reader.
//!
//! ArduPilot answers one `BurstReadFile` with up to 2000 consecutive
//! 239-byte `ACK`s, so a read costs one round trip per burst instead of one per
//! packet. On a lossy link some of those packets never arrive; the reader keeps
//! everything that did, then re-reads only the missing chunks with a small
//! pipeline of `ReadFile` requests before starting the next burst. Requests are
//! always chunk aligned, so each reply names its chunk by offset and arrival
//! order does not matter.
//!
//! This is a pure state machine: callers send what [`FtpReader::poll`] returns,
//! feed replies to [`FtpReader::on_reply`], and call [`FtpReader::on_tick`] when
//! [`FtpReader::next_deadline`] passes.

use std::time::{Duration, Instant};

use thiserror::Error;

use super::TransferStats;
use crate::mavftp::{DATA_LEN, MavFtpError, NakCode, Opcode, Packet};

/// `BurstReadFile` replies ArduPilot sends per request (`GCS_FTP.cpp`).
pub const BURST_PACKETS: u32 = 2000;
/// ArduPilot queues five FTP requests per channel; a burst or retransmission
/// waiting in that queue must not be displaced by a deeper repair pipeline.
pub const REPAIR_WINDOW: usize = 4;

const CHUNK: u32 = DATA_LEN as u32;

#[derive(Clone, Copy, Debug)]
pub struct Timing {
    /// Silence after which a burst is considered finished (or never started).
    pub burst_idle: Duration,
    pub initial_rto: Duration,
    pub min_rto: Duration,
    pub max_rto: Duration,
    /// Attempts at one request before the transfer is declared stalled.
    pub max_retries: u32,
}

impl Default for Timing {
    fn default() -> Self {
        Self {
            burst_idle: Duration::from_millis(1_500),
            initial_rto: Duration::from_millis(750),
            min_rto: Duration::from_millis(300),
            max_rto: Duration::from_secs(4),
            max_retries: 8,
        }
    }
}

#[derive(Debug, Error, Eq, PartialEq)]
pub enum FtpReadError {
    #[error(transparent)]
    Packet(#[from] MavFtpError),
    #[error("MAVFTP read refused (NAK code {code}) at offset {offset}")]
    Nak { code: u8, offset: u32 },
    #[error("MAVFTP read reply is invalid: {0}")]
    BadReply(&'static str),
    #[error("MAVFTP read stalled at offset {offset} of {size} after {retries} retries")]
    Stalled {
        offset: u32,
        size: u32,
        retries: u32,
    },
}

#[derive(Debug)]
struct Pending {
    chunk: u32,
    packet: Packet,
    sent_at: Instant,
    attempts: u32,
    resend: bool,
}

#[derive(Debug)]
enum Phase {
    /// Decide the next step from what has arrived.
    Select,
    Burst {
        request: Packet,
        sent: bool,
        got: u32,
        retries: u32,
    },
    Repair {
        outstanding: Vec<Pending>,
    },
    Complete,
}

#[derive(Debug)]
pub struct FtpReader {
    session: u8,
    size: u32,
    chunk_count: u32,
    data: Vec<u8>,
    have: Vec<bool>,
    /// Chunks `[0, contiguous)` have all arrived.
    contiguous: u32,
    highest: Option<u32>,
    received_chunks: u32,
    next_seq: u16,
    phase: Phase,
    timing: Timing,
    srtt: Option<Duration>,
    last_activity: Instant,
    stats: TransferStats,
}

impl FtpReader {
    /// A reader for the open file `session` of `size` bytes. `first_sequence`
    /// must not collide with a still-cached reply on the autopilot, so callers
    /// pass the sequence after the one the open used.
    pub fn new(session: u8, size: u32, first_sequence: u16, timing: Timing, now: Instant) -> Self {
        let chunk_count = size.div_ceil(CHUNK);
        Self {
            session,
            size,
            chunk_count,
            data: vec![0; size as usize],
            have: vec![false; chunk_count as usize],
            contiguous: 0,
            highest: None,
            received_chunks: 0,
            next_seq: first_sequence,
            phase: if size == 0 {
                Phase::Complete
            } else {
                Phase::Select
            },
            timing,
            srtt: None,
            last_activity: now,
            stats: TransferStats::default(),
        }
    }

    #[must_use]
    pub fn is_complete(&self) -> bool {
        matches!(self.phase, Phase::Complete)
    }

    #[must_use]
    pub fn stats(&self) -> TransferStats {
        self.stats
    }

    #[must_use]
    pub fn size(&self) -> u32 {
        self.size
    }

    /// Bytes from the start of the file that have all arrived.
    #[must_use]
    pub fn contiguous_bytes(&self) -> u64 {
        u64::from((self.contiguous * CHUNK).min(self.size))
    }

    /// Bytes that have arrived anywhere in the file.
    #[must_use]
    pub fn received_bytes(&self) -> u64 {
        let full = u64::from(self.received_chunks) * u64::from(CHUNK);
        if self.have.last().copied().unwrap_or(false) {
            // The final chunk may be short.
            full - u64::from(self.chunk_count * CHUNK - self.size)
        } else {
            full
        }
    }

    /// The whole file once [`is_complete`](Self::is_complete).
    #[must_use]
    pub fn into_data(self) -> Vec<u8> {
        self.data
    }

    #[must_use]
    pub fn data(&self) -> &[u8] {
        &self.data
    }

    /// The sequence number the next request will use.
    #[must_use]
    pub fn next_sequence(&self) -> u16 {
        self.next_seq
    }

    fn alloc_seq(&mut self) -> u16 {
        let sequence = self.next_seq;
        self.next_seq = sequence.wrapping_add(1);
        sequence
    }

    fn rto(&self) -> Duration {
        self.srtt.map_or(self.timing.initial_rto, |srtt| {
            (srtt * 3).clamp(self.timing.min_rto, self.timing.max_rto)
        })
    }

    fn has_hole(&self) -> bool {
        self.highest
            .is_some_and(|highest| highest > self.contiguous)
    }

    /// Requests to transmit now. A repair read that has to be sent again gets
    /// a fresh sequence number, so a stale cached reply on the autopilot (which
    /// answers a repeated sequence number from its cache) can never swallow it.
    /// A burst request that drew no reply at all is repeated unchanged.
    pub fn poll(&mut self, now: Instant) -> Vec<Packet> {
        loop {
            match &mut self.phase {
                Phase::Complete => return Vec::new(),
                Phase::Select => self.select(),
                Phase::Burst {
                    request,
                    sent,
                    retries,
                    ..
                } => {
                    if *sent {
                        return Vec::new();
                    }
                    *sent = true;
                    let first = *retries == 0;
                    let packet = request.clone();
                    if first {
                        self.stats.requests_sent += 1;
                        self.stats.bursts += 1;
                    } else {
                        self.stats.retransmits += 1;
                    }
                    self.last_activity = now;
                    return vec![packet];
                }
                Phase::Repair { .. } => {
                    let sent = self.top_up(now);
                    if sent.is_empty() && self.repair_idle() {
                        self.phase = Phase::Select;
                        continue;
                    }
                    return sent;
                }
            }
        }
    }

    fn repair_idle(&self) -> bool {
        matches!(&self.phase, Phase::Repair { outstanding } if outstanding.is_empty())
    }

    fn select(&mut self) {
        if self.contiguous >= self.chunk_count {
            self.phase = Phase::Complete;
        } else if self.has_hole() {
            self.phase = Phase::Repair {
                outstanding: Vec::new(),
            };
        } else {
            let sequence = self.alloc_seq();
            let request = Packet::request(
                sequence,
                self.session,
                Opcode::BurstReadFile,
                self.contiguous * CHUNK,
                &[],
            )
            .expect("an empty read request is valid");
            // The burst's replies number from `sequence + 1`; skip past all of
            // them so no later request can collide with the autopilot's cache.
            self.next_seq = sequence.wrapping_add(BURST_PACKETS as u16 + 2);
            self.phase = Phase::Burst {
                request,
                sent: false,
                got: 0,
                retries: 0,
            };
        }
    }

    fn top_up(&mut self, now: Instant) -> Vec<Packet> {
        let Phase::Repair { outstanding } = &mut self.phase else {
            return Vec::new();
        };
        let mut sent = Vec::new();
        for pending in outstanding.iter_mut().filter(|pending| pending.resend) {
            self.stats.retransmits += 1;
            pending.resend = false;
            pending.sent_at = now;
            let sequence = self.next_seq;
            self.next_seq = sequence.wrapping_add(1);
            pending.packet.sequence = sequence;
            sent.push(pending.packet.clone());
        }
        let highest = self.highest.unwrap_or(0);
        let mut chunk = self.contiguous;
        while outstanding.len() < REPAIR_WINDOW && chunk <= highest {
            if !self.have[chunk as usize] && outstanding.iter().all(|item| item.chunk != chunk) {
                let sequence = self.next_seq;
                self.next_seq = sequence.wrapping_add(1);
                let packet =
                    Packet::request(sequence, self.session, Opcode::ReadFile, chunk * CHUNK, &[])
                        .expect("an empty read request is valid");
                self.stats.requests_sent += 1;
                self.stats.repair_requests += 1;
                sent.push(packet.clone());
                outstanding.push(Pending {
                    chunk,
                    packet,
                    sent_at: now,
                    attempts: 1,
                    resend: false,
                });
            }
            chunk += 1;
        }
        sent
    }

    /// Feeds a reply. Replies that are not for this read are ignored.
    pub fn on_reply(&mut self, now: Instant, reply: &Packet) -> Result<(), FtpReadError> {
        let ours = reply.request_opcode == Opcode::ReadFile as u8
            || reply.request_opcode == Opcode::BurstReadFile as u8;
        if !ours || reply.session != self.session {
            return Ok(());
        }
        match reply.opcode {
            Opcode::Ack => self.store(now, reply),
            Opcode::Nak => {
                self.stats.naks += 1;
                let code = reply.data.first().copied().unwrap_or(0);
                if code == NakCode::EndOfFile as u8 && matches!(self.phase, Phase::Burst { .. }) {
                    // A file that is a whole number of chunks ends with EOF.
                    self.last_activity = now;
                    self.phase = Phase::Select;
                    Ok(())
                } else {
                    Err(FtpReadError::Nak {
                        code,
                        offset: reply.offset,
                    })
                }
            }
            _ => Ok(()),
        }
    }

    fn store(&mut self, now: Instant, reply: &Packet) -> Result<(), FtpReadError> {
        if !reply.offset.is_multiple_of(CHUNK) {
            return Err(FtpReadError::BadReply("offset is not chunk aligned"));
        }
        let chunk = reply.offset / CHUNK;
        if chunk >= self.chunk_count {
            return Err(FtpReadError::BadReply("data beyond the end of the file"));
        }
        let start = (chunk * CHUNK) as usize;
        let expected = (self.size as usize - start).min(DATA_LEN);
        if reply.data.len() != expected {
            return Err(FtpReadError::BadReply("chunk has an unexpected length"));
        }
        self.last_activity = now;
        self.stats.packets_received += 1;
        if self.have[chunk as usize] {
            self.stats.duplicate_packets += 1;
        } else {
            if self.highest.is_some_and(|highest| chunk > highest + 1)
                || (self.highest.is_none() && chunk > self.contiguous)
            {
                self.stats.gaps += 1;
            }
            self.data[start..start + expected].copy_from_slice(&reply.data);
            self.have[chunk as usize] = true;
            self.received_chunks += 1;
            self.highest = Some(self.highest.map_or(chunk, |highest| highest.max(chunk)));
            while self.contiguous < self.chunk_count && self.have[self.contiguous as usize] {
                self.contiguous += 1;
            }
        }
        match &mut self.phase {
            Phase::Burst { got, retries, .. } => {
                *got += 1;
                *retries = 0;
                if reply.burst_complete != 0 {
                    self.phase = Phase::Select;
                }
            }
            Phase::Repair { outstanding } => {
                if let Some(index) = outstanding.iter().position(|item| item.chunk == chunk) {
                    let item = outstanding.swap_remove(index);
                    if item.attempts == 1 {
                        let sample = now.saturating_duration_since(item.sent_at);
                        self.srtt = Some(self.srtt.map_or(sample, |srtt| (srtt * 7 + sample) / 8));
                    }
                }
            }
            Phase::Select | Phase::Complete => {}
        }
        if self.contiguous >= self.chunk_count {
            self.phase = Phase::Complete;
        }
        Ok(())
    }

    /// When the caller must next call [`on_tick`](Self::on_tick) if nothing
    /// arrives; `None` once complete.
    #[must_use]
    pub fn next_deadline(&self, now: Instant) -> Option<Instant> {
        match &self.phase {
            Phase::Complete => None,
            Phase::Select => Some(now),
            Phase::Burst { sent, .. } => Some(if *sent {
                self.last_activity + self.timing.burst_idle
            } else {
                now
            }),
            Phase::Repair { outstanding } => {
                let rto = self.rto();
                outstanding
                    .iter()
                    .map(|item| if item.resend { now } else { item.sent_at + rto })
                    .min()
                    .or(Some(now))
            }
        }
    }

    /// Handles silence: retransmits lost requests and decides when a burst has
    /// stopped delivering.
    pub fn on_tick(&mut self, now: Instant) -> Result<(), FtpReadError> {
        let rto = self.rto();
        let idle = now.saturating_duration_since(self.last_activity) >= self.timing.burst_idle;
        let (offset, size, max_retries) =
            (self.contiguous * CHUNK, self.size, self.timing.max_retries);
        match &mut self.phase {
            Phase::Burst {
                sent, got, retries, ..
            } if *sent && idle => {
                self.stats.timeouts += 1;
                if *got == 0 {
                    *retries += 1;
                    if *retries > max_retries {
                        return Err(FtpReadError::Stalled {
                            offset,
                            size,
                            retries: max_retries,
                        });
                    }
                    *sent = false;
                } else {
                    // The stream went quiet after delivering: mend what is
                    // missing, then start the next burst.
                    self.phase = Phase::Select;
                }
            }
            Phase::Repair { outstanding } => {
                for item in outstanding.iter_mut() {
                    if !item.resend && now.saturating_duration_since(item.sent_at) >= rto {
                        self.stats.timeouts += 1;
                        item.attempts += 1;
                        if item.attempts > max_retries {
                            return Err(FtpReadError::Stalled {
                                offset,
                                size,
                                retries: max_retries,
                            });
                        }
                        item.resend = true;
                    }
                }
            }
            _ => {}
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests;
