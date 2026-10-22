use std::sync::{
    Mutex as StdMutex,
    atomic::{AtomicU64, Ordering},
};

use tokio::sync::watch;

use super::*;
use crate::{
    Message,
    codec::{
        dialect::{FILE_TRANSFER_PROTOCOL_DATA, LOG_DATA_DATA, LOG_ENTRY_DATA},
        message_fields,
    },
    mavftp::{Opcode, Packet},
    stats::TransferStats,
    transport::BoxFuture,
};

#[derive(Debug, thiserror::Error)]
#[error("test transport failed")]
struct SendFailure;

enum Response {
    Messages(Vec<MavMessage>),
    Close,
    Fail,
}

type Respond = dyn FnMut(&MavMessage, usize) -> Response + Send;
type SentMessage = (MavMessage, Option<u8>, Option<u8>);

#[test]
fn public_operations_return_boxed_send_futures_and_stay_lazy() {
    fn assert_boxed<T>(future: BoxFuture<'_, T>) {
        drop(future);
    }

    let transport = MockTransport::new(|_, _| panic!("unpolled operation sent a message"));
    let client = new_protocol(transport.clone(), 0);
    let timeout = Duration::from_secs(1);
    let name = String::from("RAWES_MODE");
    let message = String::from("HEARTBEAT");
    let path = String::from("/logs");
    let params = vec![1.0];
    let settings = vec![ParameterSetting {
        name: name.clone(),
        value: ParameterValue::Real32(0.0),
    }];
    let rates = Map::from_iter([(message.clone(), Value::from(1.0))]);
    let observer = Observer::new(None);
    let mut summary = TransferSummary::default();

    assert_boxed(client.command(
        MavCmd::MAV_CMD_REQUEST_MESSAGE,
        &params,
        None,
        None,
        timeout,
    ));
    assert_boxed(client.get_parameter(&name, timeout));
    assert_boxed(client.set_parameter(&name, ParameterValue::Real32(0.0), timeout));
    assert_boxed(client.list_parameters(timeout));
    assert_boxed(client.set_parameters(&settings, timeout, 2));
    assert_boxed(client.request_message(&message, None, None, timeout));
    assert_boxed(client.autopilot_version(None, None, timeout));
    assert_boxed(client.capabilities(timeout));
    assert_boxed(client.message_interval(&message, timeout));
    assert_boxed(client.set_message_rates(&rates, timeout));
    assert_boxed(client.list_files(&path, timeout));
    assert_boxed(client.remove_file(&path, timeout));
    assert_boxed(client.create_directory(&path, timeout));
    assert_boxed(client.download_file(&observer, &path, true, timeout, &mut summary));
    assert_boxed(client.upload_file(&observer, &path, vec![1, 2, 3], timeout, &mut summary));
    assert_boxed(client.list_logs(timeout));
    assert_boxed(client.download_log(&observer, 1, timeout, 2, &mut summary));

    assert!(transport.requests().is_empty());
    assert_eq!(
        transport
            .sender
            .lock()
            .unwrap()
            .as_ref()
            .unwrap()
            .receiver_count(),
        0
    );
    assert!(observer.totals.lock().unwrap().is_empty());
    assert!(observer.progress.lock().unwrap().is_empty());
    assert_eq!(summary.done, 0);
    assert_eq!(summary.received, 0);
}

struct MockTransport {
    sender: StdMutex<Option<broadcast::Sender<Arc<ReceivedMessage>>>>,
    sent: StdMutex<Vec<SentMessage>>,
    respond: StdMutex<Box<Respond>>,
    sequence: AtomicU64,
}

impl MockTransport {
    fn new(respond: impl FnMut(&MavMessage, usize) -> Response + Send + 'static) -> Arc<Self> {
        Arc::new(Self {
            sender: StdMutex::new(Some(broadcast::channel(2).0)),
            sent: StdMutex::new(Vec::new()),
            respond: StdMutex::new(Box::new(respond)),
            sequence: AtomicU64::new(40),
        })
    }

    fn requests(&self) -> Vec<MavMessage> {
        self.sent
            .lock()
            .unwrap()
            .iter()
            .map(|(message, _, _)| message.clone())
            .collect()
    }
}

impl Transport for MockTransport {
    fn target(&self) -> (u8, u8) {
        (1, 1)
    }

    fn subscribe_messages(&self) -> broadcast::Receiver<Arc<ReceivedMessage>> {
        self.sender
            .lock()
            .unwrap()
            .as_ref()
            .expect("open mock")
            .subscribe()
    }

    fn send_message(
        &self,
        message: MavMessage,
        source_system: Option<u8>,
        source_component: Option<u8>,
    ) -> BoxFuture<'_, Result<(), TransportError>> {
        Box::pin(async move {
            if !matches!(message, MavMessage::LOG_REQUEST_END(_)) {
                assert!(
                    self.sender
                        .lock()
                        .unwrap()
                        .as_ref()
                        .unwrap()
                        .receiver_count()
                        > 0,
                    "protocol must subscribe before sending a request"
                );
            }
            let index = {
                let mut sent = self.sent.lock().unwrap();
                let index = sent.len();
                sent.push((message.clone(), source_system, source_component));
                index
            };
            let response = (self.respond.lock().unwrap())(&message, index);
            match response {
                Response::Fail => return Err(TransportError(Box::new(SendFailure))),
                Response::Close => {
                    self.sender.lock().unwrap().take();
                }
                Response::Messages(messages) => {
                    let sender = self.sender.lock().unwrap();
                    for message in messages {
                        let journal_sequence = self.sequence.fetch_add(1, Ordering::Relaxed);
                        sender
                            .as_ref()
                            .unwrap()
                            .send(Arc::new(ReceivedMessage {
                                cursor: format!("v1:{journal_sequence}"),
                                system_id: 1,
                                component_id: 1,
                                name: message.message_name().into(),
                                fields: message_fields(&message).unwrap(),
                                message,
                            }))
                            .unwrap();
                    }
                }
            }
            Ok(())
        })
    }
}

struct Observer {
    cancel: watch::Sender<bool>,
    cancel_after: Option<u64>,
    progress: StdMutex<Vec<(u64, u64, TransferStats)>>,
    totals: StdMutex<Vec<u64>>,
}

impl Observer {
    fn new(cancel_after: Option<u64>) -> Self {
        Self {
            cancel: watch::channel(false).0,
            cancel_after,
            progress: StdMutex::new(Vec::new()),
            totals: StdMutex::new(Vec::new()),
        }
    }
}

impl TransferObserver for Observer {
    fn is_cancelled(&self) -> bool {
        *self.cancel.borrow()
    }

    fn cancelled(&self) -> BoxFuture<'_, ()> {
        let mut receiver = self.cancel.subscribe();
        Box::pin(async move {
            while !*receiver.borrow_and_update() {
                receiver.changed().await.unwrap();
            }
        })
    }

    fn start(&self, total_bytes: u64) -> BoxFuture<'_, ()> {
        Box::pin(async move {
            self.totals.lock().unwrap().push(total_bytes);
        })
    }

    fn progress(&self, done: u64, received: u64, stats: TransferStats) {
        self.progress.lock().unwrap().push((done, received, stats));
        if self.cancel_after.is_some_and(|threshold| done >= threshold) {
            self.cancel.send_replace(true);
        }
    }
}

fn ack(command: MavCmd) -> MavMessage {
    MavMessage::COMMAND_ACK(COMMAND_ACK_DATA {
        command,
        result: MavResult::MAV_RESULT_ACCEPTED,
        ..Default::default()
    })
}

fn ftp_packet(message: &MavMessage) -> Packet {
    let MavMessage::FILE_TRANSFER_PROTOCOL(ftp) = message else {
        panic!("FTP request")
    };
    Packet::decode(&ftp.payload).unwrap()
}

fn ftp_reply(request: &Packet, data: &[u8]) -> MavMessage {
    let packet = Packet {
        sequence: request.sequence.wrapping_add(1),
        session: 7,
        opcode: Opcode::Ack,
        size: data.len() as u8,
        request_opcode: request.opcode as u8,
        burst_complete: 0,
        offset: request.offset,
        data: data.to_vec(),
    };
    MavMessage::FILE_TRANSFER_PROTOCOL(FILE_TRANSFER_PROTOCOL_DATA {
        target_component: 190,
        payload: packet.encode().unwrap(),
        ..Default::default()
    })
}

#[tokio::test(start_paused = true)]
async fn command_subscribes_first_retries_with_confirmation_and_keeps_cursor() {
    let transport = MockTransport::new(|message, index| {
        let MavMessage::COMMAND_LONG(command) = message else {
            panic!("command")
        };
        assert_eq!(command.confirmation, index as u8);
        Response::Messages(if index == 0 {
            vec![]
        } else {
            vec![ack(command.command)]
        })
    });
    let client = new_protocol(transport.clone(), 0);
    let result = client
        .command(
            MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
            &[1.0],
            None,
            None,
            Duration::from_secs(4),
        )
        .await
        .unwrap();
    assert_eq!(result.after_cursor, "v1:40");
    assert_eq!(transport.requests().len(), 2);
}

#[tokio::test(start_paused = true)]
async fn command_timeout_and_receive_failures_are_not_swallowed() {
    for (response, expected) in [(0, "timed out"), (1, "closed"), (2, "lagged")] {
        let transport = MockTransport::new(move |_, _| match response {
            0 => Response::Messages(vec![]),
            1 => Response::Close,
            _ => Response::Messages(vec![MavMessage::HEARTBEAT(Default::default()); 4]),
        });
        let client = new_protocol(transport, 0);
        let error = client
            .command(
                MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
                &[],
                None,
                None,
                Duration::from_secs(2),
            )
            .await
            .unwrap_err();
        assert!(error.to_string().contains(expected), "{error}");
    }
}

#[tokio::test]
async fn transport_error_retains_concrete_type_and_display() {
    let client = new_protocol(MockTransport::new(|_, _| Response::Fail), 0);
    let error = client
        .get_parameter("TEST", Duration::from_secs(1))
        .await
        .unwrap_err();
    assert_eq!(error.to_string(), "test transport failed");
    let OperationError::Link(error) = error else {
        panic!("typed transport error")
    };
    assert!(error.0.downcast_ref::<SendFailure>().is_some());
}

#[tokio::test(start_paused = true)]
async fn cloned_clients_share_command_and_parameter_locks() {
    let transport = MockTransport::new(|_, _| Response::Messages(vec![]));
    let client = Arc::new(ProtocolClient::new(transport.clone(), 0));
    let clone: Arc<dyn Protocol> = client.clone();
    let command_guard = client.command_lock.lock().await;
    let parameter_guard = client.parameter_lock.lock().await;
    let (command, parameter) = tokio::join!(
        clone.command(
            MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
            &[],
            None,
            None,
            Duration::from_secs(1)
        ),
        clone.get_parameter("TEST", Duration::from_secs(1)),
    );
    assert!(matches!(command, Err(OperationError::Timeout)));
    assert!(matches!(parameter, Err(OperationError::Timeout)));
    assert!(transport.requests().is_empty());
    drop((command_guard, parameter_guard));
}

#[tokio::test(start_paused = true)]
async fn ftp_retries_identical_packet_and_advances_wrapping_sequence() {
    let transport = MockTransport::new(|message, index| {
        let packet = ftp_packet(message);
        if index < 2 {
            assert_eq!(packet.sequence, u16::MAX);
            if index == 0 {
                return Response::Messages(vec![]);
            }
        } else {
            assert_eq!(packet.sequence, 1);
        }
        Response::Messages(vec![ftp_reply(&packet, &[])])
    });
    let client = new_protocol(transport.clone(), u16::MAX);
    client
        .create_directory("/test", Duration::from_secs(5))
        .await
        .unwrap();
    client
        .remove_file("/test", Duration::from_secs(5))
        .await
        .unwrap();
    assert_eq!(transport.requests().len(), 3);
    assert!(
        transport
            .sent
            .lock()
            .unwrap()
            .iter()
            .all(|(_, system, component)| { system.is_none() && *component == Some(190) })
    );
}

#[tokio::test]
async fn file_cancel_reports_progress_and_terminates_session_before_unlocking() {
    let transport = MockTransport::new(|message, _| {
        let packet = ftp_packet(message);
        let data = match packet.opcode {
            Opcode::OpenFileReadOnly => 240_u32.to_le_bytes().to_vec(),
            Opcode::BurstReadFile => vec![1; 239],
            _ => vec![],
        };
        Response::Messages(vec![ftp_reply(&packet, &data)])
    });
    let concrete = Arc::new(ProtocolClient::new(transport.clone(), 10));
    let client: Arc<dyn Protocol> = concrete.clone();
    let observer = Observer::new(Some(239));
    let mut summary = TransferSummary::default();
    let result = client
        .download_file(
            &observer,
            "/test",
            false,
            Duration::from_secs(3),
            &mut summary,
        )
        .await;
    assert!(matches!(result, Err(TransferError::Cancelled)));
    assert_eq!(summary.done, 239);
    assert_eq!(*observer.totals.lock().unwrap(), [240]);
    assert_eq!(
        ftp_packet(transport.requests().last().unwrap()).opcode,
        Opcode::TerminateSession
    );
    assert!(concrete.ftp_sequence.try_lock().is_ok());
}

#[tokio::test]
async fn upload_cancel_releases_open_session_and_preserves_statistics() {
    let transport = MockTransport::new(|message, _| {
        let packet = ftp_packet(message);
        Response::Messages(vec![ftp_reply(&packet, &[])])
    });
    let client = new_protocol(transport.clone(), 0);
    let observer = Observer::new(Some(239));
    let mut summary = TransferSummary::default();
    let result = client
        .upload_file(
            &observer,
            "/test",
            vec![0; 240],
            Duration::from_secs(3),
            &mut summary,
        )
        .await;
    assert!(matches!(result, Err(TransferError::Cancelled)));
    assert_eq!(summary.done, 239);
    assert_eq!(summary.stats.packets_received, 3);
    assert_eq!(
        ftp_packet(transport.requests().last().unwrap()).opcode,
        Opcode::TerminateSession
    );
}

#[tokio::test]
async fn failed_file_transfer_surfaces_cleanup_error_instead_of_hiding_it() {
    let transport = MockTransport::new(|message, _| {
        let packet = ftp_packet(message);
        match packet.opcode {
            Opcode::OpenFileReadOnly => {
                Response::Messages(vec![ftp_reply(&packet, &1_u32.to_le_bytes())])
            }
            Opcode::BurstReadFile | Opcode::TerminateSession => Response::Fail,
            _ => Response::Messages(vec![ftp_reply(&packet, &[])]),
        }
    });
    let client = new_protocol(transport.clone(), 0);
    let mut summary = TransferSummary::default();
    let error = client
        .download_file(
            &Observer::new(None),
            "/test",
            false,
            Duration::from_secs(3),
            &mut summary,
        )
        .await
        .unwrap_err();
    assert!(matches!(
        error,
        TransferError::Error(OperationError::Cleanup { .. })
    ));
    assert_eq!(
        ftp_packet(transport.requests().last().unwrap()).opcode,
        Opcode::TerminateSession
    );
}

#[tokio::test]
async fn log_cancel_ends_stream_even_while_waiting_for_metadata() {
    let observer = Arc::new(Observer::new(None));
    let cancel = observer.clone();
    let transport = MockTransport::new(move |message, _| {
        if matches!(message, MavMessage::LOG_REQUEST_LIST(_)) {
            cancel.cancel.send_replace(true);
        }
        Response::Messages(vec![])
    });
    let client = new_protocol(transport.clone(), 0);
    let mut summary = TransferSummary::default();
    assert!(matches!(
        client
            .download_log(
                observer.as_ref(),
                1,
                Duration::from_secs(3),
                1,
                &mut summary
            )
            .await,
        Err(TransferError::Cancelled)
    ));
    assert!(matches!(
        transport.requests().last(),
        Some(MavMessage::LOG_REQUEST_END(_))
    ));
}

#[tokio::test(start_paused = true)]
async fn log_stall_ends_stream_and_releases_shared_lock() {
    let transport = MockTransport::new(|message, _| {
        Response::Messages(match message {
            MavMessage::LOG_REQUEST_LIST(_) => vec![MavMessage::LOG_ENTRY(LOG_ENTRY_DATA {
                id: 1,
                num_logs: 1,
                last_log_num: 1,
                size: 90,
                ..Default::default()
            })],
            _ => vec![],
        })
    });
    let concrete = Arc::new(ProtocolClient::new(transport.clone(), 0));
    let client: Arc<dyn Protocol> = concrete.clone();
    let mut summary = TransferSummary::default();
    assert!(
        client
            .download_log(
                &Observer::new(None),
                1,
                Duration::from_secs(1),
                1,
                &mut summary
            )
            .await
            .is_err()
    );
    assert!(matches!(
        transport.requests().last(),
        Some(MavMessage::LOG_REQUEST_END(_))
    ));
    assert!(summary.stats.timeouts > 0);
    assert!(concrete.log_lock.try_lock().is_ok());
}

#[tokio::test]
async fn file_success_terminates_before_crc_verification_and_keeps_progress() {
    let transport = MockTransport::new(|message, index| {
        let packet = ftp_packet(message);
        let data = match packet.opcode {
            Opcode::OpenFileReadOnly => 1_u32.to_le_bytes().to_vec(),
            Opcode::BurstReadFile => vec![42],
            Opcode::CalculateFileCrc32 => {
                assert_eq!(index, 4, "reset, open, burst, terminate must precede CRC");
                crate::mavftp::mavftp_crc32(&[42]).to_le_bytes().to_vec()
            }
            _ => vec![],
        };
        Response::Messages(vec![ftp_reply(&packet, &data)])
    });
    let client = new_protocol(transport, 0);
    let observer = Observer::new(None);
    let mut summary = TransferSummary::default();
    assert_eq!(
        client
            .download_file(
                &observer,
                "/test",
                true,
                Duration::from_secs(3),
                &mut summary
            )
            .await
            .unwrap(),
        [42]
    );
    assert_eq!(summary.done, 1);
    assert_eq!(summary.received, 1);
    assert_eq!(summary.stats.bursts, 1);
}

#[tokio::test]
async fn log_cancel_after_progress_surfaces_failed_end_request() {
    let transport = MockTransport::new(|message, _| match message {
        MavMessage::LOG_REQUEST_LIST(_) => {
            Response::Messages(vec![MavMessage::LOG_ENTRY(LOG_ENTRY_DATA {
                id: 1,
                num_logs: 1,
                last_log_num: 1,
                size: 180,
                ..Default::default()
            })])
        }
        MavMessage::LOG_REQUEST_DATA(_) => {
            Response::Messages(vec![MavMessage::LOG_DATA(LOG_DATA_DATA {
                id: 1,
                ofs: 0,
                count: 90,
                data: [42; 90],
            })])
        }
        MavMessage::LOG_REQUEST_END(_) => Response::Fail,
        _ => panic!("log request"),
    });
    let client = new_protocol(transport.clone(), 0);
    let observer = Observer::new(Some(90));
    let mut summary = TransferSummary::default();
    let result = client
        .download_log(&observer, 1, Duration::from_secs(3), 1, &mut summary)
        .await;
    assert!(matches!(
        result,
        Err(TransferError::CancelledCleanup(OperationError::Link(_)))
    ));
    assert_eq!(summary.done, 90);
    assert_eq!(summary.received, 90);
    assert_eq!(*observer.totals.lock().unwrap(), [180]);
    assert!(matches!(
        transport.requests().last(),
        Some(MavMessage::LOG_REQUEST_END(_))
    ));
}

#[tokio::test]
async fn queued_transfers_cancel_without_sending_or_waiting_for_shared_locks() {
    let transport = MockTransport::new(|_, _| panic!("queued transfer must not send"));
    let client = Arc::new(ProtocolClient::new(transport.clone(), 0));
    let clone: Arc<dyn Protocol> = client.clone();
    let _ftp = client.ftp_sequence.lock().await;
    let _log = client.log_lock.lock().await;
    let observer = Observer::new(None);
    observer.cancel.send_replace(true);
    let mut file_summary = TransferSummary::default();
    let mut log_summary = TransferSummary::default();
    let (file, log) = tokio::join!(
        clone.download_file(
            &observer,
            "/test",
            false,
            Duration::from_secs(1),
            &mut file_summary
        ),
        clone.download_log(&observer, 1, Duration::from_secs(1), 1, &mut log_summary),
    );
    assert!(matches!(file, Err(TransferError::Cancelled)));
    assert!(matches!(log, Err(TransferError::Cancelled)));
    assert!(transport.requests().is_empty());
}

#[tokio::test]
async fn parameter_message_and_rate_transactions_use_the_transport_boundary() {
    let transport = MockTransport::new(|message, _| {
        Response::Messages(match message {
            MavMessage::PARAM_REQUEST_READ(request) => {
                vec![MavMessage::PARAM_VALUE(PARAM_VALUE_DATA {
                    param_id: request.param_id,
                    param_value: 4.5,
                    param_type: MavParamType::MAV_PARAM_TYPE_REAL32,
                    param_count: 1,
                    param_index: 0,
                })]
            }
            MavMessage::PARAM_SET(request) => vec![MavMessage::PARAM_VALUE(PARAM_VALUE_DATA {
                param_id: request.param_id,
                param_value: request.param_value,
                param_type: request.param_type,
                param_count: 1,
                param_index: 0,
            })],
            MavMessage::COMMAND_LONG(request)
                if request.command == MavCmd::MAV_CMD_REQUEST_MESSAGE =>
            {
                vec![
                    MavMessage::AUTOPILOT_VERSION(Default::default()),
                    ack(request.command),
                ]
            }
            MavMessage::COMMAND_LONG(request) => vec![ack(request.command)],
            _ => panic!("unexpected transaction"),
        })
    });
    let client = new_protocol(transport, 0);
    let timeout = Duration::from_secs(1);
    let read = client.get_parameter(" test ", timeout).await.unwrap();
    assert_eq!(
        (read.name.as_str(), read.value, read.after_cursor.as_str()),
        ("TEST", ParameterValue::Real32(4.5), "v1:40")
    );
    let write = client
        .set_parameter("TEST", ParameterValue::Real32(7.0), timeout)
        .await
        .unwrap();
    assert_eq!(write.value, ParameterValue::Real32(7.0));
    assert_eq!(write.after_cursor, "v1:41");
    let response = client
        .request_message("autopilot_version", None, None, timeout)
        .await
        .unwrap();
    assert_eq!(response["after_cursor"], "v1:42");
    assert_eq!(response["ack_cursor"], "v1:43");
    let rates = client
        .set_message_rates(json!({"ATTITUDE": 10.0}).as_object().unwrap(), timeout)
        .await
        .unwrap();
    assert_eq!(rates[0]["interval_us"], 100_000);
    assert_eq!(rates[0]["after_cursor"], "v1:44");
}

#[tokio::test]
async fn parameter_transactions_preserve_integer_and_float_types() {
    for value in [
        ParameterValue::Int8(-12),
        ParameterValue::Int16(30000),
        ParameterValue::Int32(16_777_218),
        ParameterValue::Real32(4.5),
    ] {
        let transport = MockTransport::new(move |message, _| {
            if let MavMessage::PARAM_SET(request) = message {
                assert_eq!(request.param_type, value.param_type());
                assert_eq!(request.param_value, value.encode_classic().unwrap());
            }
            Response::Messages(vec![MavMessage::PARAM_VALUE(PARAM_VALUE_DATA {
                param_id: CharArray::from("TEST"),
                param_value: value.encode_classic().unwrap(),
                param_type: value.param_type(),
                param_count: 1,
                param_index: 0,
            })])
        });
        let client = new_protocol(transport, 0);
        let timeout = Duration::from_secs(1);
        assert_eq!(
            client.get_parameter("TEST", timeout).await.unwrap().value,
            value
        );
        assert_eq!(
            client.list_parameters(timeout).await.unwrap()[0].value,
            value
        );
        assert_eq!(
            client
                .set_parameter("TEST", value, timeout)
                .await
                .unwrap()
                .value,
            value
        );
        let settings = [ParameterSetting {
            name: "TEST".to_owned(),
            value,
        }];
        assert_eq!(
            client.set_parameters(&settings, timeout, 0).await.unwrap()[0].value,
            value
        );
    }
}

#[tokio::test(start_paused = true)]
async fn parameter_reads_retry_after_an_unrelated_reply() {
    let transport = MockTransport::new(|message, index| {
        let MavMessage::PARAM_REQUEST_READ(request) = message else {
            panic!("parameter read")
        };
        Response::Messages(vec![MavMessage::PARAM_VALUE(PARAM_VALUE_DATA {
            param_id: if index == 0 {
                CharArray::from("OTHER")
            } else {
                request.param_id
            },
            param_value: 0.0,
            param_type: MavParamType::MAV_PARAM_TYPE_REAL32,
            param_count: 1,
            param_index: 0,
        })])
    });
    let client = new_protocol(transport.clone(), 0);
    let start = time::Instant::now();
    let result = client
        .get_parameter("RAWES_MODE", Duration::from_secs(3))
        .await
        .unwrap();
    assert_eq!(result.name, "RAWES_MODE");
    assert_eq!(result.after_cursor, "v1:41");
    assert_eq!(transport.requests().len(), 2);
    assert_eq!(start.elapsed(), Duration::from_secs(1));
}

#[tokio::test(start_paused = true)]
async fn parameter_writes_retry_the_identical_request_after_a_lost_reply() {
    let transport = MockTransport::new(|message, index| {
        let MavMessage::PARAM_SET(request) = message else {
            panic!("parameter write")
        };
        assert_eq!(request.param_type, MavParamType::MAV_PARAM_TYPE_REAL32);
        assert_eq!(request.param_value, 0.0);
        Response::Messages(if index == 0 {
            vec![]
        } else {
            vec![MavMessage::PARAM_VALUE(PARAM_VALUE_DATA {
                param_id: request.param_id,
                param_value: request.param_value,
                param_type: request.param_type,
                param_count: 1,
                param_index: 0,
            })]
        })
    });
    let client = new_protocol(transport.clone(), 0);
    let result = client
        .set_parameter(
            "H_YAW_TRIM",
            ParameterValue::Real32(0.0),
            Duration::from_secs(3),
        )
        .await
        .unwrap();
    assert_eq!(result.value, ParameterValue::Real32(0.0));
    let requests = transport.requests();
    assert_eq!(requests.len(), 2);
    assert_eq!(
        message_fields(&requests[0]).unwrap(),
        message_fields(&requests[1]).unwrap()
    );
}

#[tokio::test(start_paused = true)]
async fn parameter_retries_keep_the_deadline_and_surface_transport_failures() {
    for write in [false, true] {
        for (response, expected) in [
            (0, "timed out"),
            (1, "closed"),
            (2, "lagged"),
            (3, "test transport failed"),
        ] {
            let transport = MockTransport::new(move |_, _| match response {
                0 => Response::Messages(vec![]),
                1 => Response::Close,
                2 => Response::Messages(vec![MavMessage::HEARTBEAT(Default::default()); 4]),
                _ => Response::Fail,
            });
            let client = new_protocol(transport.clone(), 0);
            let start = time::Instant::now();
            let timeout = Duration::from_millis(2500);
            let result = if write {
                client
                    .set_parameter("TEST", ParameterValue::Real32(0.0), timeout)
                    .await
            } else {
                client.get_parameter("TEST", timeout).await
            };
            let error = result.unwrap_err();
            assert!(error.to_string().contains(expected), "{error}");
            if response == 0 {
                assert_eq!(start.elapsed(), timeout);
                assert_eq!(transport.requests().len(), 3);
            } else {
                assert_eq!(transport.requests().len(), 1);
            }
        }
    }
}

#[tokio::test]
async fn invalid_parameter_writes_send_nothing_even_in_a_batch() {
    let transport = MockTransport::new(|_, _| panic!("invalid parameter write must not send"));
    let client = new_protocol(transport.clone(), 0);
    let timeout = Duration::from_secs(1);
    for value in [
        ParameterValue::Int32(16_777_217),
        ParameterValue::Real32(f32::NAN),
    ] {
        assert!(matches!(
            client.set_parameter("TEST", value, timeout).await,
            Err(OperationError::Invalid(_))
        ));
        let settings = [
            ParameterSetting {
                name: "VALID".to_owned(),
                value: ParameterValue::Int8(1),
            },
            ParameterSetting {
                name: "INVALID".to_owned(),
                value,
            },
        ];
        assert!(matches!(
            client.set_parameters(&settings, timeout, 0).await,
            Err(OperationError::Invalid(_))
        ));
    }
    assert!(transport.requests().is_empty());
}

#[tokio::test]
async fn unsupported_parameter_replies_surface_an_error() {
    let transport = MockTransport::new(|_, _| {
        Response::Messages(vec![MavMessage::PARAM_VALUE(PARAM_VALUE_DATA {
            param_id: CharArray::from("TEST"),
            param_value: 1.0,
            param_type: MavParamType::MAV_PARAM_TYPE_UINT64,
            param_count: 1,
            param_index: 0,
        })])
    });
    let client = new_protocol(transport, 0);
    let timeout = Duration::from_secs(1);
    assert!(matches!(
        client.get_parameter("TEST", timeout).await,
        Err(OperationError::Invalid(_))
    ));
    assert!(matches!(
        client.list_parameters(timeout).await,
        Err(OperationError::Invalid(_))
    ));
}
