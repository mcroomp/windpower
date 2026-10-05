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
            128 => Ok(Self::Ack),
            129 => Ok(Self::Nak),
            _ => Err(MavFtpError::UnknownOpcode(value)),
        }
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
            size: if opcode == Opcode::ReadFile && data.is_empty() {
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
        let read_request = self.opcode == Opcode::ReadFile && self.data.is_empty();
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
    Download { content: Vec<u8>, crc32: u32 },
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
    DownloadOpen {
        path: Vec<u8>,
        verify_crc: bool,
    },
    DownloadRead {
        path: Vec<u8>,
        session: u8,
        content: Vec<u8>,
        expected_size: Option<usize>,
        verify_crc: bool,
    },
    DownloadCrc {
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

    pub fn download(sequence: u16, path: &str, verify_crc: bool) -> Self {
        Self::new(
            sequence,
            State::DownloadOpen {
                path: path.as_bytes().to_vec(),
                verify_crc,
            },
        )
    }

    pub fn upload(sequence: u16, path: &str, content: Vec<u8>) -> Self {
        Self::new(
            sequence,
            State::UploadCreate {
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
            State::DownloadOpen { path, .. } => (0, Opcode::OpenFileReadOnly, 0, path.clone()),
            State::DownloadRead {
                session, content, ..
            } => {
                let offset = u32::try_from(content.len())
                    .map_err(|_| MavFtpError::MalformedReply("file is larger than 4 GiB"))?;
                (*session, Opcode::ReadFile, offset, Vec::new())
            }
            State::DownloadCrc { .. } => {
                return Err(MavFtpError::MalformedReply("CRC path was not retained"));
            }
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
            State::DownloadOpen { path, verify_crc } => {
                reject_nak(nak)?;
                let expected_size = if reply.data.len() >= 4 {
                    Some(u32::from_le_bytes(reply.data[..4].try_into().expect("checked")) as usize)
                } else {
                    None
                };
                State::DownloadRead {
                    path,
                    session: reply.session,
                    content: Vec::with_capacity(expected_size.unwrap_or(0)),
                    expected_size,
                    verify_crc,
                }
            }
            State::DownloadRead {
                path,
                session,
                mut content,
                expected_size,
                verify_crc,
            } => {
                if let Some((code, system_error)) = nak {
                    if code != NakCode::EndOfFile as u8 {
                        return Err(MavFtpError::Nak { code, system_error });
                    }
                } else {
                    if reply.offset != content.len() as u32 {
                        return Err(MavFtpError::MalformedReply("non-contiguous read reply"));
                    }
                    content.extend_from_slice(&reply.data);
                }
                let done = nak.is_some()
                    || expected_size.is_some_and(|size| content.len() >= size)
                    || reply.data.len() < DATA_LEN;
                if done {
                    if expected_size.is_some_and(|size| size != content.len()) {
                        return Err(MavFtpError::MalformedReply("download size mismatch"));
                    }
                    if verify_crc {
                        let packet = Packet::request(
                            self.next_sequence,
                            0,
                            Opcode::CalculateFileCrc32,
                            0,
                            &path,
                        )?;
                        self.next_sequence = self.next_sequence.wrapping_add(1);
                        self.outstanding = Some(packet);
                        State::DownloadCrc { content }
                    } else {
                        let crc32 = mavftp_crc32(&content);
                        State::Complete(OperationResult::Download { content, crc32 })
                    }
                } else {
                    State::DownloadRead {
                        path,
                        session,
                        content,
                        expected_size,
                        verify_crc,
                    }
                }
            }
            State::DownloadCrc { content } => {
                reject_nak(nak)?;
                let remote = read_crc(&reply.data)?;
                let local = mavftp_crc32(&content);
                if remote != local {
                    return Err(MavFtpError::CrcMismatch { remote, local });
                }
                State::Complete(OperationResult::Download {
                    content,
                    crc32: local,
                })
            }
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
    fn download_reads_chunks_and_verifies_crc() {
        let content: Vec<_> = (0..300).map(|value| value as u8).collect();
        let mut operation = Operation::download(1, "/file", true);
        let open = operation.next_request().unwrap().unwrap();
        operation
            .handle_reply(reply(
                &open,
                Opcode::Ack,
                &(content.len() as u32).to_le_bytes(),
                0,
            ))
            .unwrap();
        let read = operation.next_request().unwrap().unwrap();
        assert_eq!(read.size, DATA_LEN as u8);
        operation
            .handle_reply(reply(&read, Opcode::Ack, &content[..DATA_LEN], 0))
            .unwrap();
        let read = operation.next_request().unwrap().unwrap();
        operation
            .handle_reply(reply(
                &read,
                Opcode::Ack,
                &content[DATA_LEN..],
                DATA_LEN as u32,
            ))
            .unwrap();
        let crc = operation.next_request().unwrap().unwrap();
        assert_eq!(crc.opcode, Opcode::CalculateFileCrc32);
        operation
            .handle_reply(reply(
                &crc,
                Opcode::Ack,
                &mavftp_crc32(&content).to_le_bytes(),
                0,
            ))
            .unwrap();
        assert_eq!(
            operation.result(),
            Some(&OperationResult::Download {
                content: content.clone(),
                crc32: mavftp_crc32(&content),
            })
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
