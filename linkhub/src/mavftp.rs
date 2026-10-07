use thiserror::Error;

pub const DATA_LEN: usize = 239;
pub const PAYLOAD_LEN: usize = 251;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum Opcode {
    TerminateSession = 1,
    ResetSessions = 2,
    ListDirectory = 3,
    OpenFileReadOnly = 4,
    ReadFile = 5,
    CreateFile = 6,
    WriteFile = 7,
    RemoveFile = 8,
    CreateDirectory = 9,
    CalculateFileCrc32 = 14,
    BurstReadFile = 15,
    Ack = 128,
    Nak = 129,
}

impl Opcode {
    fn from_u8(value: u8) -> Result<Self, MavFtpError> {
        match value {
            1 => Ok(Self::TerminateSession),
            2 => Ok(Self::ResetSessions),
            3 => Ok(Self::ListDirectory),
            4 => Ok(Self::OpenFileReadOnly),
            5 => Ok(Self::ReadFile),
            6 => Ok(Self::CreateFile),
            7 => Ok(Self::WriteFile),
            8 => Ok(Self::RemoveFile),
            9 => Ok(Self::CreateDirectory),
            14 => Ok(Self::CalculateFileCrc32),
            15 => Ok(Self::BurstReadFile),
            128 => Ok(Self::Ack),
            129 => Ok(Self::Nak),
            _ => Err(MavFtpError::UnknownOpcode(value)),
        }
    }

    /// Reads carry their chunk size in `size` while their `data` stays empty.
    fn is_read_request(self) -> bool {
        matches!(self, Self::ReadFile | Self::BurstReadFile)
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Packet {
    pub sequence: u16,
    pub session: u8,
    pub opcode: Opcode,
    pub size: u8,
    pub request_opcode: u8,
    pub burst_complete: u8,
    pub offset: u32,
    pub data: Vec<u8>,
}

impl Packet {
    pub fn request(
        sequence: u16,
        session: u8,
        opcode: Opcode,
        offset: u32,
        data: &[u8],
    ) -> Result<Self, MavFtpError> {
        if data.len() > DATA_LEN {
            return Err(MavFtpError::PayloadTooLarge(data.len()));
        }
        Ok(Self {
            sequence,
            session,
            opcode,
            size: if opcode.is_read_request() && data.is_empty() {
                DATA_LEN as u8
            } else {
                data.len() as u8
            },
            request_opcode: 0,
            burst_complete: 0,
            offset,
            data: data.to_vec(),
        })
    }

    pub fn encode(&self) -> Result<[u8; PAYLOAD_LEN], MavFtpError> {
        let read_request = self.opcode.is_read_request() && self.data.is_empty();
        if (!read_request && self.data.len() != usize::from(self.size))
            || self.data.len() > DATA_LEN
        {
            return Err(MavFtpError::InvalidPacketSize {
                declared: self.size,
                actual: self.data.len(),
            });
        }
        let mut bytes = [0_u8; PAYLOAD_LEN];
        bytes[0..2].copy_from_slice(&self.sequence.to_le_bytes());
        bytes[2] = self.session;
        bytes[3] = self.opcode as u8;
        bytes[4] = self.size;
        bytes[5] = self.request_opcode;
        bytes[6] = self.burst_complete;
        bytes[8..12].copy_from_slice(&self.offset.to_le_bytes());
        bytes[12..12 + self.data.len()].copy_from_slice(&self.data);
        Ok(bytes)
    }

    pub fn decode(bytes: &[u8]) -> Result<Self, MavFtpError> {
        if bytes.len() != PAYLOAD_LEN {
            return Err(MavFtpError::InvalidPayloadLength(bytes.len()));
        }
        let size = bytes[4];
        if usize::from(size) > DATA_LEN {
            return Err(MavFtpError::InvalidPacketSize {
                declared: size,
                actual: DATA_LEN,
            });
        }
        Ok(Self {
            sequence: u16::from_le_bytes([bytes[0], bytes[1]]),
            session: bytes[2],
            opcode: Opcode::from_u8(bytes[3])?,
            size,
            request_opcode: bytes[5],
            burst_complete: bytes[6],
            offset: u32::from_le_bytes(bytes[8..12].try_into().expect("fixed range")),
            data: bytes[12..12 + usize::from(size)].to_vec(),
        })
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum NakCode {
    Fail = 1,
    FailErrno = 2,
    InvalidDataSize = 3,
    InvalidSession = 4,
    NoSessionsAvailable = 5,
    EndOfFile = 6,
    UnknownCommand = 7,
    FileExists = 8,
    FileProtected = 9,
    FileNotFound = 10,
}

#[derive(Debug, Error, Eq, PartialEq)]
pub enum MavFtpError {
    #[error("MAVFTP payload must be {PAYLOAD_LEN} bytes, got {0}")]
    InvalidPayloadLength(usize),
    #[error("MAVFTP data is too large: {0} bytes")]
    PayloadTooLarge(usize),
    #[error("MAVFTP packet declares {declared} bytes but contains {actual}")]
    InvalidPacketSize { declared: u8, actual: usize },
    #[error("unknown MAVFTP opcode {0}")]
    UnknownOpcode(u8),
    #[error("unexpected MAVFTP reply sequence {actual}, expected {expected}")]
    UnexpectedSequence { expected: u16, actual: u16 },
    #[error("MAVFTP reply is for opcode {actual}, expected {expected}")]
    UnexpectedRequestOpcode { expected: u8, actual: u8 },
    #[error("MAVFTP operation failed with code {code} (errno {system_error:?})")]
    Nak { code: u8, system_error: Option<u8> },
    #[error("malformed MAVFTP reply: {0}")]
    MalformedReply(&'static str),
    #[error("MAVFTP CRC mismatch: remote={remote:08x}, local={local:08x}")]
    CrcMismatch { remote: u32, local: u32 },
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DirectoryEntry {
    pub name: String,
    pub is_dir: bool,
    pub size: u64,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum OperationResult {
    Files(Vec<DirectoryEntry>),
    /// A file opened for reading: its session and size in bytes.
    Opened {
        session: u8,
        size: u32,
    },
    /// A session was terminated (a refused close is not an error).
    Closed,
    Uploaded(usize),
    Removed,
    DirectoryCreated(bool),
    Crc32(u32),
}

#[derive(Debug)]
enum State {
    List {
        path: Vec<u8>,
        entries: Vec<DirectoryEntry>,
        offset: u32,
    },
    /// Clears sessions a cancelled or failed earlier transfer left open.
    OpenReset {
        path: Vec<u8>,
    },
    Open {
        path: Vec<u8>,
    },
    Terminate {
        session: u8,
    },
    UploadReset {
        path: Vec<u8>,
        content: Vec<u8>,
    },
    UploadCreate {
        path: Vec<u8>,
        content: Vec<u8>,
    },
    UploadWrite {
        session: u8,
        content: Vec<u8>,
        written: usize,
    },
    UploadTerminate {
        session: u8,
        written: usize,
    },
    Simple {
        opcode: Opcode,
        path: Vec<u8>,
        result: OperationResult,
        file_exists_is_success: bool,
    },
    Crc(Vec<u8>),
    Complete(OperationResult),
}

/// A stop-and-wait MAVFTP transaction. Call [`next_request`](Self::next_request),
/// transmit it in FILE_TRANSFER_PROTOCOL, then pass the decoded reply to
/// [`handle_reply`](Self::handle_reply).
#[derive(Debug)]
pub struct Operation {
    next_sequence: u16,
    outstanding: Option<Packet>,
    state: State,
}

impl Operation {
    pub fn list(sequence: u16, path: &str) -> Self {
        Self::new(
            sequence,
            State::List {
                path: path.as_bytes().to_vec(),
                entries: Vec::new(),
                offset: 0,
            },
        )
    }

    /// Opens `path` for reading, first clearing sessions a cancelled earlier
    /// transfer may have left open. The bulk read is done by
    /// [`FtpReader`](crate::transfer::FtpReader), not by this operation.
    pub fn open_read(sequence: u16, path: &str) -> Self {
        Self::new(
            sequence,
            State::OpenReset {
                path: path.as_bytes().to_vec(),
            },
        )
    }

    /// Releases the file `session` holds open.
    pub fn terminate(sequence: u16, session: u8) -> Self {
        Self::new(sequence, State::Terminate { session })
    }

    pub fn upload(sequence: u16, path: &str, content: Vec<u8>) -> Self {
        Self::new(
            sequence,
            State::UploadReset {
                path: path.as_bytes().to_vec(),
                content,
            },
        )
    }

    pub fn remove(sequence: u16, path: &str) -> Self {
        Self::simple(
            sequence,
            Opcode::RemoveFile,
            path,
            OperationResult::Removed,
            false,
        )
    }

    pub fn mkdir(sequence: u16, path: &str) -> Self {
        Self::simple(
            sequence,
            Opcode::CreateDirectory,
            path,
            OperationResult::DirectoryCreated(true),
            true,
        )
    }

    pub fn crc32(sequence: u16, path: &str) -> Self {
        Self::new(sequence, State::Crc(path.as_bytes().to_vec()))
    }

    fn simple(
        sequence: u16,
        opcode: Opcode,
        path: &str,
        result: OperationResult,
        file_exists_is_success: bool,
    ) -> Self {
        Self::new(
            sequence,
            State::Simple {
                opcode,
                path: path.as_bytes().to_vec(),
                result,
                file_exists_is_success,
            },
        )
    }

    fn new(sequence: u16, state: State) -> Self {
        Self {
            next_sequence: sequence,
            outstanding: None,
            state,
        }
    }

    pub fn next_request(&mut self) -> Result<Option<Packet>, MavFtpError> {
        if let Some(packet) = &self.outstanding {
            return Ok(Some(packet.clone()));
        }
        let (session, opcode, offset, data) = match &self.state {
            State::List { path, offset, .. } => (0, Opcode::ListDirectory, *offset, path.clone()),
            State::OpenReset { .. } | State::UploadReset { .. } => {
                (0, Opcode::ResetSessions, 0, Vec::new())
            }
            State::Open { path } => (0, Opcode::OpenFileReadOnly, 0, path.clone()),
            State::Terminate { session } => (*session, Opcode::TerminateSession, 0, Vec::new()),
            State::UploadCreate { path, .. } => (0, Opcode::CreateFile, 0, path.clone()),
            State::UploadWrite {
                session,
                content,
                written,
            } => {
                let end = (*written + DATA_LEN).min(content.len());
                if *written == end {
                    (*session, Opcode::TerminateSession, 0, Vec::new())
                } else {
                    (
                        *session,
                        Opcode::WriteFile,
                        *written as u32,
                        content[*written..end].to_vec(),
                    )
                }
            }
            State::UploadTerminate { session, .. } => {
                (*session, Opcode::TerminateSession, 0, Vec::new())
            }
            State::Simple { opcode, path, .. } => (0, *opcode, 0, path.clone()),
            State::Crc(path) => (0, Opcode::CalculateFileCrc32, 0, path.clone()),
            State::Complete(_) => return Ok(None),
        };
        let packet = Packet::request(self.next_sequence, session, opcode, offset, &data)?;
        self.next_sequence = self.next_sequence.wrapping_add(1);
        self.outstanding = Some(packet.clone());
        Ok(Some(packet))
    }

    pub fn handle_reply(&mut self, reply: Packet) -> Result<Option<&OperationResult>, MavFtpError> {
        let request = self.outstanding.take().ok_or(MavFtpError::MalformedReply(
            "reply without an outstanding request",
        ))?;
        let expected_sequence = request.sequence.wrapping_add(1);
        if reply.sequence != expected_sequence {
            return Err(MavFtpError::UnexpectedSequence {
                expected: expected_sequence,
                actual: reply.sequence,
            });
        }
        if reply.request_opcode != request.opcode as u8 {
            return Err(MavFtpError::UnexpectedRequestOpcode {
                expected: request.opcode as u8,
                actual: reply.request_opcode,
            });
        }
        self.next_sequence = reply.sequence.wrapping_add(1);
        let nak = if reply.opcode == Opcode::Nak {
            Some(nak_details(&reply)?)
        } else {
            if reply.opcode != Opcode::Ack {
                return Err(MavFtpError::MalformedReply("reply is neither ACK nor NAK"));
            }
            None
        };

        let old = std::mem::replace(&mut self.state, State::Complete(OperationResult::Removed));
        self.state = match old {
            State::List {
                path,
                mut entries,
                offset,
            } => {
                if let Some((code, system_error)) = nak {
                    if code == NakCode::EndOfFile as u8 {
                        State::Complete(OperationResult::Files(entries))
                    } else {
                        return Err(MavFtpError::Nak { code, system_error });
                    }
                } else {
                    let added = parse_directory_entries(&reply.data, &mut entries)?;
                    if added == 0 {
                        State::Complete(OperationResult::Files(entries))
                    } else {
                        State::List {
                            path,
                            entries,
                            offset: offset + added as u32,
                        }
                    }
                }
            }
            State::OpenReset { path } => {
                // Advisory: firmware without ResetSessions NAKs and is still usable.
                State::Open { path }
            }
            State::UploadReset { path, content } => State::UploadCreate { path, content },
            State::Open { .. } => {
                reject_nak(nak)?;
                let size = reply
                    .data
                    .get(..4)
                    .ok_or(MavFtpError::MalformedReply("open reply has no file size"))?;
                State::Complete(OperationResult::Opened {
                    session: reply.session,
                    size: u32::from_le_bytes(size.try_into().expect("checked")),
                })
            }
            // The data is already complete; a refused close is not a failure.
            State::Terminate { .. } => State::Complete(OperationResult::Closed),
            State::UploadCreate { content, .. } => {
                reject_nak(nak)?;
                State::UploadWrite {
                    session: reply.session,
                    content,
                    written: 0,
                }
            }
            State::UploadWrite {
                session,
                content,
                mut written,
            } => {
                reject_nak(nak)?;
                if request.opcode == Opcode::TerminateSession {
                    State::Complete(OperationResult::Uploaded(written))
                } else {
                    written += request.data.len();
                    if written == content.len() {
                        State::UploadTerminate { session, written }
                    } else {
                        State::UploadWrite {
                            session,
                            content,
                            written,
                        }
                    }
                }
            }
            State::UploadTerminate { written, .. } => {
                reject_nak(nak)?;
                State::Complete(OperationResult::Uploaded(written))
            }
            State::Simple {
                result,
                file_exists_is_success,
                ..
            } => {
                if file_exists_is_success
                    && nak.is_some_and(|(code, _)| code == NakCode::FileExists as u8)
                {
                    State::Complete(OperationResult::DirectoryCreated(false))
                } else {
                    reject_nak(nak)?;
                    State::Complete(result)
                }
            }
            State::Crc(_) => {
                reject_nak(nak)?;
                State::Complete(OperationResult::Crc32(read_crc(&reply.data)?))
            }
            State::Complete(result) => State::Complete(result),
        };
        Ok(self.result())
    }

    pub fn result(&self) -> Option<&OperationResult> {
        match &self.state {
            State::Complete(result) => Some(result),
            _ => None,
        }
    }

    pub fn next_sequence(&self) -> u16 {
        self.next_sequence
    }
}

fn nak_details(reply: &Packet) -> Result<(u8, Option<u8>), MavFtpError> {
    let code = *reply
        .data
        .first()
        .ok_or(MavFtpError::MalformedReply("NAK has no error code"))?;
    let system_error = (code == NakCode::FailErrno as u8)
        .then(|| reply.data.get(1).copied())
        .flatten();
    Ok((code, system_error))
}

fn reject_nak(nak: Option<(u8, Option<u8>)>) -> Result<(), MavFtpError> {
    match nak {
        Some((code, system_error)) => Err(MavFtpError::Nak { code, system_error }),
        None => Ok(()),
    }
}

fn read_crc(data: &[u8]) -> Result<u32, MavFtpError> {
    let bytes = data.get(..4).ok_or(MavFtpError::MalformedReply(
        "CRC reply is shorter than four bytes",
    ))?;
    Ok(u32::from_le_bytes(bytes.try_into().expect("checked")))
}

fn parse_directory_entries(
    data: &[u8],
    output: &mut Vec<DirectoryEntry>,
) -> Result<usize, MavFtpError> {
    let mut count = 0;
    for raw in data
        .split(|byte| *byte == 0)
        .filter(|entry| !entry.is_empty())
    {
        count += 1;
        let kind = raw[0];
        if kind == b'S' {
            continue;
        }
        let text = std::str::from_utf8(&raw[1..])
            .map_err(|_| MavFtpError::MalformedReply("directory name is not UTF-8"))?;
        let (name, size) = if kind == b'F' {
            let (name, size) = text
                .rsplit_once('\t')
                .ok_or(MavFtpError::MalformedReply("file entry has no size"))?;
            let size = size
                .parse()
                .map_err(|_| MavFtpError::MalformedReply("invalid file size"))?;
            (name, size)
        } else if kind == b'D' {
            (text, 0)
        } else {
            return Err(MavFtpError::MalformedReply("unknown directory entry kind"));
        };
        output.push(DirectoryEntry {
            name: name.to_owned(),
            is_dir: kind == b'D',
            size,
        });
    }
    Ok(count)
}

/// ArduPilot's MAVFTP CRC convention: reflected CRC-32 update with an initial
/// raw register of zero and no final complement.
pub fn mavftp_crc32(content: &[u8]) -> u32 {
    let mut crc = 0_u32;
    for byte in content {
        crc ^= u32::from(*byte);
        for _ in 0..8 {
            crc = (crc >> 1) ^ (0xedb8_8320 & 0_u32.wrapping_sub(crc & 1));
        }
    }
    crc
}

#[cfg(test)]
mod tests {
    use super::*;

    fn reply(request: &Packet, opcode: Opcode, data: &[u8], offset: u32) -> Packet {
        Packet {
            sequence: request.sequence.wrapping_add(1),
            session: 1,
            opcode,
            size: data.len() as u8,
            request_opcode: request.opcode as u8,
            burst_complete: 0,
            offset,
            data: data.to_vec(),
        }
    }

    #[test]
    fn packet_round_trip_and_crc_match_ardupilot() {
        let packet = Packet::request(0x1234, 7, Opcode::WriteFile, 42, b"abc").unwrap();
        assert_eq!(Packet::decode(&packet.encode().unwrap()).unwrap(), packet);
        let content: Vec<_> = (0..4096).map(|index| (index % 251) as u8).collect();
        assert_eq!(mavftp_crc32(&content), 0x1379_f916);
    }

    #[test]
    fn lists_paginated_directory_until_eof() {
        let mut operation = Operation::list(10, "/APM");
        let first = operation.next_request().unwrap().unwrap();
        operation
            .handle_reply(reply(&first, Opcode::Ack, b"Done\0Flog.bin\t123\0", 0))
            .unwrap();
        let second = operation.next_request().unwrap().unwrap();
        assert_eq!(second.offset, 2);
        operation
            .handle_reply(reply(&second, Opcode::Nak, &[NakCode::EndOfFile as u8], 0))
            .unwrap();
        assert_eq!(
            operation.result(),
            Some(&OperationResult::Files(vec![
                DirectoryEntry {
                    name: "one".into(),
                    is_dir: true,
                    size: 0,
                },
                DirectoryEntry {
                    name: "log.bin".into(),
                    is_dir: false,
                    size: 123,
                },
            ]))
        );
    }

    #[test]
    fn open_read_clears_stale_sessions_then_returns_the_session_and_size() {
        let mut operation = Operation::open_read(1, "/file");
        let reset = operation.next_request().unwrap().unwrap();
        assert_eq!(reset.opcode, Opcode::ResetSessions);
        // Firmware without ResetSessions refuses it; the open still proceeds.
        operation
            .handle_reply(reply(
                &reset,
                Opcode::Nak,
                &[NakCode::UnknownCommand as u8],
                0,
            ))
            .unwrap();
        let open = operation.next_request().unwrap().unwrap();
        assert_eq!(open.opcode, Opcode::OpenFileReadOnly);
        operation
            .handle_reply(reply(&open, Opcode::Ack, &300_u32.to_le_bytes(), 0))
            .unwrap();
        assert_eq!(
            operation.result(),
            Some(&OperationResult::Opened {
                session: 1,
                size: 300
            })
        );
    }

    #[test]
    fn open_read_fails_when_the_file_cannot_be_opened() {
        let mut operation = Operation::open_read(1, "/missing");
        let reset = operation.next_request().unwrap().unwrap();
        operation
            .handle_reply(reply(&reset, Opcode::Ack, &[], 0))
            .unwrap();
        let open = operation.next_request().unwrap().unwrap();
        let error = operation
            .handle_reply(reply(&open, Opcode::Nak, &[NakCode::FileNotFound as u8], 0))
            .unwrap_err();
        assert!(matches!(
            error,
            MavFtpError::Nak {
                code: 10,
                system_error: None
            }
        ));
    }

    #[test]
    fn terminate_names_the_session_and_tolerates_a_refusal() {
        let mut operation = Operation::terminate(5, 9);
        let request = operation.next_request().unwrap().unwrap();
        assert_eq!(request.opcode, Opcode::TerminateSession);
        assert_eq!(request.session, 9);
        operation
            .handle_reply(reply(&request, Opcode::Nak, &[NakCode::Fail as u8], 0))
            .unwrap();
        assert_eq!(operation.result(), Some(&OperationResult::Closed));
    }

    #[test]
    fn crc_request_returns_the_remote_checksum() {
        let mut operation = Operation::crc32(7, "/file");
        let request = operation.next_request().unwrap().unwrap();
        assert_eq!(request.opcode, Opcode::CalculateFileCrc32);
        operation
            .handle_reply(reply(
                &request,
                Opcode::Ack,
                &0xdead_beef_u32.to_le_bytes(),
                0,
            ))
            .unwrap();
        assert_eq!(
            operation.result(),
            Some(&OperationResult::Crc32(0xdead_beef))
        );
    }

    #[test]
    fn upload_writes_all_chunks_and_terminates_session() {
        let content = vec![7; DATA_LEN + 2];
        let mut operation = Operation::upload(20, "/new", content.clone());
        loop {
            let request = operation.next_request().unwrap().unwrap();
            let response = reply(&request, Opcode::Ack, &[], request.offset);
            operation.handle_reply(response).unwrap();
            if operation.result().is_some() {
                break;
            }
        }
        assert_eq!(
            operation.result(),
            Some(&OperationResult::Uploaded(content.len()))
        );
    }

    #[test]
    fn mkdir_maps_file_exists_to_not_created() {
        let mut operation = Operation::mkdir(1, "/existing");
        let request = operation.next_request().unwrap().unwrap();
        operation
            .handle_reply(reply(
                &request,
                Opcode::Nak,
                &[NakCode::FileExists as u8],
                0,
            ))
            .unwrap();
        assert_eq!(
            operation.result(),
            Some(&OperationResult::DirectoryCreated(false))
        );
    }
}
