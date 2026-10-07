use std::{
    collections::{HashSet, VecDeque},
    time::{Duration, Instant},
};

use super::*;

const SESSION: u8 = 3;

/// Deterministic xorshift so a failing loss pattern reproduces exactly.
struct Lcg(u64);

impl Lcg {
    fn chance(&mut self, per_mille: u64) -> bool {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0 % 1000 < per_mille
    }
}

/// An autopilot serving one file the way `GCS_FTP.cpp` does: a burst request
/// yields consecutive replies numbered from `request.sequence + 1`, a read
/// yields one, and the burst ends with `burst_complete` or an EOF NAK.
struct Autopilot {
    file: Vec<u8>,
    loss: Lcg,
    per_mille: u64,
    drop_next_requests: usize,
    requests: Vec<Packet>,
}

impl Autopilot {
    fn new(len: usize, per_mille: u64) -> Self {
        Self {
            file: (0..len).map(|i| (i * 31 % 251) as u8).collect(),
            loss: Lcg(0x9e37_79b9_7f4a_7c15),
            per_mille,
            drop_next_requests: 0,
            requests: Vec::new(),
        }
    }

    fn reply(
        &self,
        request: &Packet,
        sequence: u16,
        offset: u32,
        complete: bool,
    ) -> Option<Packet> {
        let start = offset as usize;
        if start >= self.file.len() {
            return Some(Packet {
                sequence,
                session: request.session,
                opcode: Opcode::Nak,
                size: 1,
                request_opcode: request.opcode as u8,
                burst_complete: 0,
                offset,
                data: vec![NakCode::EndOfFile as u8],
            });
        }
        let end = (start + DATA_LEN).min(self.file.len());
        Some(Packet {
            sequence,
            session: request.session,
            opcode: Opcode::Ack,
            size: (end - start) as u8,
            request_opcode: request.opcode as u8,
            burst_complete: u8::from(complete || end - start < DATA_LEN),
            offset,
            data: self.file[start..end].to_vec(),
        })
    }

    fn respond(&mut self, request: &Packet) -> Vec<Packet> {
        self.requests.push(request.clone());
        if self.drop_next_requests > 0 {
            self.drop_next_requests -= 1;
            return Vec::new();
        }
        let mut replies = Vec::new();
        match request.opcode {
            Opcode::BurstReadFile => {
                for i in 0..BURST_PACKETS {
                    let offset = request.offset + i * DATA_LEN as u32;
                    let complete = i == BURST_PACKETS - 1;
                    let Some(reply) = self.reply(
                        request,
                        request.sequence.wrapping_add(1 + i as u16),
                        offset,
                        complete,
                    ) else {
                        break;
                    };
                    let last = reply.opcode == Opcode::Nak || reply.burst_complete != 0;
                    replies.push(reply);
                    if last {
                        break;
                    }
                }
            }
            Opcode::ReadFile => replies.extend(self.reply(
                request,
                request.sequence.wrapping_add(1),
                request.offset,
                false,
            )),
            other => panic!("unexpected request {other:?}"),
        }
        let per_mille = self.per_mille;
        replies
            .into_iter()
            .filter(|_| !self.loss.chance(per_mille))
            .collect()
    }
}

/// Runs the reader against the autopilot on a virtual clock; the link takes
/// `rtt` per exchange and nothing happens in real time.
fn run(autopilot: &mut Autopilot, timing: Timing) -> Result<FtpReader, FtpReadError> {
    let start = Instant::now();
    let mut now = start;
    let mut reader = FtpReader::new(SESSION, autopilot.file.len() as u32, 100, timing, now);
    let mut inbox: VecDeque<Packet> = VecDeque::new();
    for _ in 0..2_000_000 {
        for request in reader.poll(now) {
            inbox.extend(autopilot.respond(&request));
        }
        if reader.is_complete() {
            return Ok(reader);
        }
        if let Some(reply) = inbox.pop_front() {
            now += Duration::from_micros(500);
            reader.on_reply(now, &reply)?;
        } else {
            let deadline = reader.next_deadline(now).expect("not complete");
            now = deadline.max(now) + Duration::from_millis(1);
            reader.on_tick(now)?;
        }
    }
    panic!("reader did not finish");
}

#[test]
fn a_clean_link_reads_the_file_in_bursts_without_repairs() {
    let mut autopilot = Autopilot::new(1_200_000, 0);
    let reader = run(&mut autopilot, Timing::default()).unwrap();
    let stats = reader.stats();
    assert_eq!(reader.into_data(), autopilot.file);
    assert_eq!(stats.bursts, 3, "1.2 MB is three 2000-packet bursts");
    assert_eq!(stats.repair_requests, 0);
    assert_eq!(stats.duplicate_packets, 0);
    assert_eq!(stats.retransmits, 0);
}

#[test]
fn a_lossy_link_repairs_only_the_holes() {
    let mut autopilot = Autopilot::new(1_200_000, 40);
    let reader = run(&mut autopilot, Timing::default()).unwrap();
    let stats = reader.stats();
    let chunks = 1_200_000_u64.div_ceil(DATA_LEN as u64);
    assert_eq!(reader.into_data(), autopilot.file);
    assert!(stats.repair_requests > 0, "4% loss must open holes");
    assert!(stats.gaps > 0);
    // Repairs are single reads, so the total traffic stays near the file size
    // instead of re-sending whole windows after each loss.
    assert!(
        stats.packets_received < chunks * 11 / 10,
        "received {} packets for {chunks} chunks",
        stats.packets_received
    );
    assert!(stats.duplicate_ratio() < 0.05, "{stats:?}");
}

#[test]
fn a_heavily_lossy_link_still_converges() {
    let mut autopilot = Autopilot::new(300_000, 300);
    let timing = Timing {
        max_retries: 24,
        ..Timing::default()
    };
    let reader = run(&mut autopilot, timing).unwrap();
    assert_eq!(reader.into_data(), autopilot.file);
}

#[test]
fn a_whole_number_of_chunks_ends_with_an_eof_nak() {
    let mut autopilot = Autopilot::new(DATA_LEN * 7, 0);
    let reader = run(&mut autopilot, Timing::default()).unwrap();
    assert_eq!(reader.into_data(), autopilot.file);
}

#[test]
fn a_lost_burst_request_is_sent_again() {
    let mut autopilot = Autopilot::new(50_000, 0);
    autopilot.drop_next_requests = 2;
    let reader = run(&mut autopilot, Timing::default()).unwrap();
    let stats = reader.stats();
    assert_eq!(reader.into_data(), autopilot.file);
    assert_eq!(stats.retransmits, 2);
    assert!(stats.timeouts >= 2);
}

#[test]
fn a_silent_autopilot_stalls_instead_of_hanging() {
    let mut autopilot = Autopilot::new(50_000, 0);
    autopilot.drop_next_requests = usize::MAX;
    let timing = Timing {
        max_retries: 3,
        ..Timing::default()
    };
    let error = run(&mut autopilot, timing).unwrap_err();
    assert!(
        matches!(error, FtpReadError::Stalled { retries: 3, .. }),
        "{error}"
    );
}

#[test]
fn no_request_reuses_a_sequence_inside_an_earlier_burst_reply_range() {
    let mut autopilot = Autopilot::new(600_000, 30);
    run(&mut autopilot, Timing::default()).unwrap();
    // The autopilot answers a request numbered like the last packet of a burst
    // from its reply cache instead of executing it, so later requests must
    // stay clear of every earlier burst's reply numbers.
    let mut burst_starts: Vec<u16> = Vec::new();
    let mut read_sequences = HashSet::new();
    for request in &autopilot.requests {
        for start in &burst_starts {
            let distance = request.sequence.wrapping_sub(*start);
            assert!(
                !(1..=BURST_PACKETS as u16).contains(&distance),
                "request {} falls inside the reply numbers of burst {start}",
                request.sequence
            );
        }
        match request.opcode {
            Opcode::BurstReadFile => {
                if !burst_starts.contains(&request.sequence) {
                    burst_starts.push(request.sequence);
                }
            }
            _ => assert!(
                read_sequences.insert(request.sequence),
                "read sequence {} was sent twice",
                request.sequence
            ),
        }
    }
    assert!(burst_starts.len() >= 2);
}

#[test]
fn malformed_replies_are_rejected() {
    let now = Instant::now();
    let mut reader = FtpReader::new(SESSION, 1000, 1, Timing::default(), now);
    let ack = |offset: u32, len: usize| Packet {
        sequence: 2,
        session: SESSION,
        opcode: Opcode::Ack,
        size: len as u8,
        request_opcode: Opcode::BurstReadFile as u8,
        burst_complete: 0,
        offset,
        data: vec![0; len],
    };
    assert!(matches!(
        reader.on_reply(now, &ack(10, DATA_LEN)),
        Err(FtpReadError::BadReply(_))
    ));
    assert!(matches!(
        reader.on_reply(now, &ack(0, 100)),
        Err(FtpReadError::BadReply(_))
    ));
    assert!(matches!(
        reader.on_reply(now, &ack(5 * DATA_LEN as u32, DATA_LEN)),
        Err(FtpReadError::BadReply(_))
    ));
}

#[test]
fn replies_for_other_requests_or_sessions_are_ignored() {
    let now = Instant::now();
    let mut reader = FtpReader::new(SESSION, 1000, 1, Timing::default(), now);
    let foreign = Packet {
        sequence: 2,
        session: SESSION + 1,
        opcode: Opcode::Ack,
        size: 0,
        request_opcode: Opcode::BurstReadFile as u8,
        burst_complete: 0,
        offset: 0,
        data: Vec::new(),
    };
    reader.on_reply(now, &foreign).unwrap();
    let other_opcode = Packet {
        session: SESSION,
        request_opcode: Opcode::ListDirectory as u8,
        ..foreign
    };
    reader.on_reply(now, &other_opcode).unwrap();
    assert_eq!(reader.stats().packets_received, 0);
}

#[test]
fn contiguous_bytes_only_advance_when_the_prefix_is_whole() {
    let mut autopilot = Autopilot::new(DATA_LEN * 4, 0);
    let now = Instant::now();
    let mut reader = FtpReader::new(
        SESSION,
        autopilot.file.len() as u32,
        1,
        Timing::default(),
        now,
    );
    let request = reader.poll(now).remove(0);
    let replies = autopilot.respond(&request);
    // Chunk 1 arrives before chunk 0: nothing is contiguous yet.
    reader.on_reply(now, &replies[1]).unwrap();
    assert_eq!(reader.contiguous_bytes(), 0);
    assert_eq!(reader.received_bytes(), DATA_LEN as u64);
    reader.on_reply(now, &replies[0]).unwrap();
    assert_eq!(reader.contiguous_bytes(), 2 * DATA_LEN as u64);
    reader.on_reply(now, &replies[3]).unwrap();
    assert_eq!(reader.contiguous_bytes(), 2 * DATA_LEN as u64);
    assert_eq!(reader.received_bytes(), 3 * DATA_LEN as u64);
}
