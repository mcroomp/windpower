use std::{net::SocketAddr, path::PathBuf, time::Duration};

use clap::{Parser, Subcommand};
use linkhub::{
    discovery::{self, DiscoveryConfig},
    dronebridge::DroneBridge,
    http,
    journal::{JournalConfig, JournalHandle},
    mavlink::{MavlinkLinkConfig, MavlinkTransport, start_link},
    motor::MotorHandle,
    query::QueryArgs,
    records::{DiagnosticEvent, DiagnosticLevel, wall_time_ns},
};
use mavlink_protocol::codec::{FrameReader, MavHeader, serialize_message};
use serde_json::{Map, json};
use tokio::{
    fs,
    io::{AsyncRead, AsyncWriteExt},
    net::{TcpListener, TcpStream},
    time,
};
use tracing_subscriber::EnvFilter;
use uuid::Uuid;

#[derive(Debug, Parser)]
#[command(name = "linkhub", about = "Transport and diagnostic gateway")]
struct Args {
    #[command(subcommand)]
    command: Command,
}

#[derive(Debug, Subcommand)]
enum Command {
    Query(QueryArgs),
    TestConnection {
        /// MAVLink endpoint: `tcp:IP:PORT`, `udp:IP:PORT` (remote peer),
        /// `udpin:IP:PORT` (local listener), `ble:DEVICE_NAME_OR_ADDRESS`, or `auto`
        /// to discover a serial port by scanning for a MAVLink heartbeat.
        #[arg(long, env = "LINKHUB_CONNECTION", default_value = "auto")]
        connection: String,
        /// Exclusive DroneBridge HTTP origin. Clear ALL UDP clients, including the
        /// saved target, after this check. Requires a udp: remote-peer connection.
        #[arg(long, env = "LINKHUB_DRONEBRIDGE_URL")]
        dronebridge_url: Option<String>,
        /// Restrict serial discovery to one baud rate.
        #[arg(long)]
        baud: Option<u32>,
        /// Baud rates tried when `--baud` is not supplied.
        #[arg(long, default_value = "115200,57600,38400,19200,9600")]
        discovery_bauds: String,
        /// Maximum network check time and BLE scan/heartbeat or serial probe time.
        #[arg(long, default_value_t = 10_000)]
        timeout_ms: u64,
    },
    Serve {
        /// MAVLink endpoint: `tcp:IP:PORT`, `udp:IP:PORT` (remote peer),
        /// `udpin:IP:PORT` (local listener), `ble:DEVICE_NAME_OR_ADDRESS` for
        /// DroneBridge BLE, or `auto` (the default) to discover a serial port
        /// by scanning for a MAVLink heartbeat. Any other value is treated as
        /// a serial port name that *restricts* the scan (e.g. `COM7`).
        #[arg(long, env = "LINKHUB_CONNECTION", default_value = "auto")]
        connection: String,
        /// Exclusive DroneBridge HTTP origin. Clear ALL UDP clients, including the
        /// saved target, on graceful stop. Requires a udp: remote-peer connection.
        #[arg(long, env = "LINKHUB_DRONEBRIDGE_URL")]
        dronebridge_url: Option<String>,
        #[arg(long, default_value = "127.0.0.1")]
        listen: String,
        #[arg(long, default_value_t = 8999)]
        port: u16,
        #[arg(long, default_value = "linkhub-data")]
        data_dir: PathBuf,
        #[arg(long, env = "LINKHUB_STATIC_DIR")]
        static_dir: Option<PathBuf>,
        /// Send `Cache-Control: no-store` on every response so browsers
        /// always load the latest rebuilt UI.
        #[arg(long)]
        no_cache: bool,
        #[arg(long)]
        run_id: Option<Uuid>,
        #[arg(long, default_value_t = 60_000)]
        flush_ms: u64,
        #[arg(long, default_value_t = 4)]
        chunk_mb: usize,
        #[arg(long, default_value_t = 255)]
        source_system: u8,
        #[arg(long, default_value_t = 0)]
        source_component: u8,
        /// Restrict discovery to a single baud rate instead of scanning
        /// `--discovery-bauds`.
        #[arg(long)]
        baud: Option<u32>,
        /// Baud rates probed, in order, while discovery scans a serial port.
        /// Ignored when `--baud` is set.
        #[arg(long, default_value = "115200,57600,38400,19200,9600")]
        discovery_bauds: String,
        /// Time each port/baud candidate is given to deliver a heartbeat.
        #[arg(long, default_value_t = 3_000)]
        discovery_timeout_ms: u64,
        #[arg(long)]
        motor_name_prefix: Option<String>,
        #[arg(long, default_value_t = 10_000)]
        motor_scan_timeout_ms: u64,
        #[arg(long, default_value_t = 500)]
        motor_heartbeat_ms: u64,
        #[arg(long, default_value_t = 10_000)]
        motor_max_command_timeout_ms: u64,
    },
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    tracing_subscriber::fmt()
        .with_env_filter(
            EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info")),
        )
        .init();

    match Args::parse().command {
        Command::Query(args) => linkhub::query::execute(args).await?,
        Command::TestConnection {
            connection,
            dronebridge_url,
            baud,
            discovery_bauds,
            timeout_ms,
        } => {
            let timeout = Duration::from_millis(timeout_ms);
            let config = link_config(&connection, baud, &discovery_bauds, timeout)?;
            let bridge =
                configure_dronebridge(dronebridge_url.as_deref(), &config.transport).await?;
            let result = test_connection_with_cleanup(config, timeout, bridge.as_ref()).await?;
            println!("{result}");
        }
        Command::Serve {
            connection,
            dronebridge_url,
            listen,
            port,
            data_dir,
            static_dir,
            no_cache,
            run_id,
            flush_ms,
            chunk_mb,
            source_system,
            source_component,
            baud,
            discovery_bauds,
            discovery_timeout_ms,
            motor_name_prefix,
            motor_scan_timeout_ms,
            motor_heartbeat_ms,
            motor_max_command_timeout_ms,
        } => {
            let mut link_config = link_config(
                &connection,
                baud,
                &discovery_bauds,
                Duration::from_millis(discovery_timeout_ms),
            )?;
            link_config.source_system = source_system;
            link_config.source_component = source_component;
            let bridge =
                configure_dronebridge(dronebridge_url.as_deref(), &link_config.transport).await?;
            if let Some(directory) = &static_dir {
                let metadata = fs::metadata(directory).await.map_err(|error| {
                    format!(
                        "could not read static directory {}: {error}",
                        directory.display()
                    )
                })?;
                if !metadata.is_dir() {
                    return Err(
                        format!("static path is not a directory: {}", directory.display()).into(),
                    );
                }
            }
            let run_id = run_id.unwrap_or_else(Uuid::new_v4);
            let run_dir = data_dir.join(run_id.to_string());
            fs::create_dir_all(&run_dir).await?;
            fs::write(
                run_dir.join("run.json"),
                serde_json::to_vec_pretty(&json!({
                    "schema_version": 1,
                    "service": "linkhub",
                    "run_id": run_id,
                    "started_at_ns": wall_time_ns(),
                    "connection": connection,
                }))?,
            )
            .await?;

            let mut journal_config = JournalConfig::for_directory(run_dir.join("journal"), run_id);
            journal_config.flush_interval = Duration::from_millis(flush_ms);
            journal_config.max_chunk_bytes = chunk_mb * 1024 * 1024;
            let (journal, journal_task) = JournalHandle::start(journal_config).await?;

            journal.append_diagnostic(startup_event(run_id)).await?;

            let bind: SocketAddr = format!("{listen}:{port}").parse()?;
            let listener = TcpListener::bind(bind).await?;
            let motor = configure_motor(
                motor_name_prefix,
                motor_scan_timeout_ms,
                motor_heartbeat_ms,
                motor_max_command_timeout_ms,
            )
            .await?;

            let (link, link_task) = start_link(link_config, journal.clone());
            tracing::info!(%bind, %run_id, "LinkHub listening");
            let server_result = axum::serve(
                listener,
                http::router_with_motor_and_static(
                    journal.clone(),
                    Some(link),
                    motor.clone(),
                    static_dir,
                    no_cache,
                ),
            )
            .with_graceful_shutdown(shutdown_signal())
            .await;

            link_task.abort();
            let link_result = link_task.await;
            let motor_result = match motor {
                Some(motor) => motor.close().await,
                None => Ok(()),
            };
            let cleanup_result = cleanup_dronebridge(bridge.as_ref()).await;
            let journal_result = journal.shutdown().await;
            let journal_task_result = journal_task.await;
            server_result?;
            if let Err(error) = link_result {
                if !error.is_cancelled() {
                    return Err(error.into());
                }
            }
            motor_result?;
            cleanup_result?;
            journal_result?;
            journal_task_result?;
        }
    }
    Ok(())
}

async fn configure_dronebridge(
    url: Option<&str>,
    transport: &MavlinkTransport,
) -> Result<Option<DroneBridge>, String> {
    let Some(url) = url else { return Ok(None) };
    if !matches!(transport, MavlinkTransport::Udp(_)) {
        return Err("--dronebridge-url requires a udp: remote-peer connection".to_owned());
    }
    let bridge = DroneBridge::new(url)?;
    let clients = bridge.clients().await?;
    tracing::warn!(%url, ?clients, "Exclusive DroneBridge UDP ownership: cleanup will clear ALL clients and the saved target");
    Ok(Some(bridge))
}

async fn cleanup_dronebridge(bridge: Option<&DroneBridge>) -> Result<(), String> {
    if let Some(bridge) = bridge {
        bridge.clear_clients().await.inspect_err(|error| {
            tracing::error!(%error, "DroneBridge UDP cleanup could not be verified");
        })?;
    }
    Ok(())
}

async fn test_connection_with_cleanup(
    config: MavlinkLinkConfig,
    timeout: Duration,
    bridge: Option<&DroneBridge>,
) -> Result<String, String> {
    let result = tokio::select! {
        result = test_connection(config, timeout) => result,
        () = shutdown_signal() => Err("connection check interrupted".to_owned()),
    };
    let cleanup = cleanup_dronebridge(bridge).await;
    let result = result?;
    cleanup?;
    Ok(result)
}

async fn test_connection(config: MavlinkLinkConfig, timeout: Duration) -> Result<String, String> {
    let target = config.transport.to_string();
    match config.transport {
        MavlinkTransport::Udp(address) | MavlinkTransport::UdpIn(address) => {
            let listen = target.starts_with("udpin:");
            let result = time::timeout(timeout, async {
                let mut stream = linkhub::udp::UdpStream::open(address, listen)
                    .await
                    .map_err(|error| format!("could not open {target}: {error}"))?;
                if !listen {
                    let heartbeat = serialize_message(
                        &linkhub::mavlink::heartbeat_message(),
                        MavHeader {
                            system_id: config.source_system,
                            component_id: config.source_component,
                            sequence: 0,
                        },
                    );
                    stream.write_all(&heartbeat).await.map_err(|error| {
                        format!("could not send UDP discovery heartbeat: {error}")
                    })?;
                }
                receive_heartbeat(stream).await
            })
            .await
            .map_err(|_| {
                format!("no MAVLink heartbeat received over {target} within {timeout:?}")
            })??;
            Ok(format!(
                "MAVLink heartbeat received over {target} from sysid={} compid={}",
                result.0, result.1
            ))
        }
        MavlinkTransport::Serial(discovery) => {
            let found = discovery::discover_serial_transport(&discovery)
                .await
                .map_err(|failure| failure.message)?;
            Ok(format!(
                "MAVLink heartbeat received on serial port {} at {} baud",
                found.port, found.baud
            ))
        }
        MavlinkTransport::Tcp(address) => {
            let result = time::timeout(timeout, async {
                let stream = TcpStream::connect(address)
                    .await
                    .map_err(|error| format!("could not connect to {address}: {error}"))?;
                receive_heartbeat(stream).await
            })
            .await
            .map_err(|_| format!("no MAVLink heartbeat received within {timeout:?}"))??;
            Ok(format!(
                "MAVLink heartbeat received over tcp:{address} from sysid={} compid={}",
                result.0, result.1
            ))
        }
        MavlinkTransport::Ble(ble_config) => {
            let connection = ble_config.device_name.clone();
            let ble_config = linkhub::ble::BleConfig::new(connection.clone(), timeout)?;
            let ble = linkhub::ble::connect(&ble_config).await?;
            let result = time::timeout(timeout, receive_heartbeat(ble.stream))
                .await
                .map_err(|_| {
                    format!(
                        "no MAVLink heartbeat received over ble:{connection} within {timeout:?}"
                    )
                })??;
            Ok(format!(
                "MAVLink heartbeat received over ble:{connection} from sysid={} compid={}",
                result.0, result.1
            ))
        }
    }
}

async fn receive_heartbeat<R>(reader: R) -> Result<(u8, u8), String>
where
    R: AsyncRead + Unpin + Send,
{
    let mut frames = FrameReader::new(Box::new(reader));
    loop {
        let message = frames
            .recv()
            .await
            .map_err(|error| format!("MAVLink receive failed: {error}"))?;
        if message.message_id == 0 {
            return Ok((message.system_id, message.component_id));
        }
    }
}

async fn configure_motor(
    name_prefix: Option<String>,
    scan_timeout_ms: u64,
    heartbeat_ms: u64,
    maximum_command_timeout_ms: u64,
) -> Result<Option<MotorHandle>, Box<dyn std::error::Error>> {
    use linkhub::motor::{BleMotorController, MotorConfig};

    let Some(name_prefix) = name_prefix else {
        return Ok(None);
    };
    let controller = BleMotorController::bluetooth(MotorConfig {
        name_prefix,
        scan_timeout: Duration::from_millis(scan_timeout_ms),
        heartbeat_interval: Duration::from_millis(heartbeat_ms),
        maximum_command_timeout_ms,
    })
    .await?;
    controller.connect().await?;
    Ok(Some(std::sync::Arc::new(controller)))
}

/// Build the MAVLink link configuration for `serve`.
///
/// `connection` is `auto` (discover any serial port), `tcp:IP:PORT`,
/// `udp:IP:PORT` (remote peer), `udpin:IP:PORT` (local listener),
/// `ble:DEVICE_NAME_OR_ADDRESS` (BLE GATT), or any other value, which is treated as a
/// serial port name that *restricts* discovery to that one port. `baud`, if
/// set, restricts serial discovery to a single baud rate.
fn link_config(
    connection: &str,
    baud: Option<u32>,
    discovery_bauds: &str,
    probe_timeout: Duration,
) -> Result<MavlinkLinkConfig, String> {
    if let Some(device_name) = connection.strip_prefix("ble:") {
        return linkhub::ble::BleConfig::new(device_name.to_owned(), Duration::from_millis(10_000))
            .map(MavlinkLinkConfig::ble);
    }
    for (prefix, listen) in [("udp:", false), ("udpin:", true)] {
        if let Some(address) = connection.strip_prefix(prefix) {
            return address
                .parse()
                .map(|address| MavlinkLinkConfig::udp(address, listen))
                .map_err(|error| {
                    format!("invalid UDP MAVLink connection {connection:?}: {error}")
                });
        }
    }
    if let Some(address) = connection.strip_prefix("tcp:") {
        return address
            .parse()
            .map(MavlinkLinkConfig::sitl)
            .map_err(|error| format!("invalid TCP MAVLink connection {connection:?}: {error}"));
    }
    if let Ok(address) = connection.parse::<SocketAddr>() {
        return Ok(MavlinkLinkConfig::sitl(address));
    }
    let bauds = match baud {
        Some(baud) => vec![baud],
        None => discovery::parse_bauds(discovery_bauds)?,
    };
    let port_filter = if connection.eq_ignore_ascii_case("auto") {
        None
    } else {
        let port = connection.strip_prefix("serial:").unwrap_or(connection);
        if port.is_empty() {
            return Err("serial port must not be empty".to_owned());
        }
        Some(port.to_owned())
    };
    Ok(MavlinkLinkConfig::serial(DiscoveryConfig {
        port_filter,
        bauds,
        probe_timeout,
    }))
}

fn startup_event(run_id: Uuid) -> DiagnosticEvent {
    DiagnosticEvent {
        schema_version: 1,
        run_id,
        source: "linkhub".to_owned(),
        source_instance: "linkhub-1".to_owned(),
        source_sequence: 1,
        source_wall_time_ns: wall_time_ns(),
        source_monotonic_ns: None,
        sim_time_ns: None,
        sim_time_quality: None,
        level: DiagnosticLevel::Info,
        category: "process".to_owned(),
        event: "process.started".to_owned(),
        message: "LinkHub started".to_owned(),
        correlation_id: None,
        causation_id: None,
        fields: Map::new(),
        related_records: Vec::new(),
    }
}

async fn shutdown_signal() {
    let ctrl_c = async {
        tokio::signal::ctrl_c()
            .await
            .expect("failed to install Ctrl+C handler");
    };
    #[cfg(unix)]
    let terminate = async {
        tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate())
            .expect("failed to install SIGTERM handler")
            .recv()
            .await;
    };
    #[cfg(windows)]
    let terminate = async {
        tokio::signal::windows::ctrl_break()
            .expect("failed to install Ctrl+Break handler")
            .recv()
            .await;
    };
    #[cfg(not(any(unix, windows)))]
    let terminate = std::future::pending::<()>();
    tokio::select! {
        () = ctrl_c => {},
        () = terminate => {},
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use axum::{
        Json, Router,
        routing::{delete, get},
    };
    use std::sync::{
        Arc,
        atomic::{AtomicUsize, Ordering},
    };
    use tokio::io::AsyncWriteExt;

    #[test]
    fn accepts_tcp_auto_and_restricted_serial_connection_syntax() {
        for connection in [
            "udp:127.0.0.1:14550",
            "udpin:0.0.0.0:14550",
            "udp:[::1]:14550",
        ] {
            assert_eq!(
                link_config(connection, None, "115200", Duration::from_secs(3))
                    .unwrap()
                    .transport
                    .to_string(),
                connection
            );
        }
        for connection in ["udp:", "udp:127.0.0.1", "udpin:bad:14550"] {
            assert!(link_config(connection, None, "115200", Duration::from_secs(3)).is_err());
        }
        assert!(matches!(
            link_config("tcp:127.0.0.1:5760", None, "115200", Duration::from_secs(3))
                .expect("TCP connection")
                .transport,
            linkhub::mavlink::MavlinkTransport::Tcp(_)
        ));
        assert!(matches!(
            link_config("auto", None, "115200,57600", Duration::from_secs(3))
                .expect("auto connection")
                .transport,
            linkhub::mavlink::MavlinkTransport::Serial(discovery)
                if discovery.port_filter.is_none() && discovery.bauds == [115_200, 57_600]
        ));
        assert!(matches!(
            link_config("COM4", Some(57_600), "115200", Duration::from_secs(3))
                .expect("restricted serial connection")
                .transport,
            linkhub::mavlink::MavlinkTransport::Serial(discovery)
                if discovery.port_filter.as_deref() == Some("COM4") && discovery.bauds == [57_600]
        ));
        assert!(matches!(
            link_config("ble:DroneBridge", None, "115200", Duration::from_secs(3))
                .expect("BLE connection")
                .transport,
            linkhub::mavlink::MavlinkTransport::Ble(config)
                if config.device_name == "DroneBridge"
        ));
    }

    #[test]
    fn parses_test_connection_for_dronebridge() {
        assert!(matches!(
            Args::try_parse_from([
                "linkhub",
                "test-connection",
                "--connection",
                "ble:DroneBridge"
            ])
            .expect("test-connection arguments")
            .command,
            Command::TestConnection { connection, .. } if connection == "ble:DroneBridge"
        ));
        for command in ["serve", "test-connection"] {
            let args = Args::try_parse_from([
                "linkhub",
                command,
                "--connection",
                "udp:192.168.9.143:14550",
                "--dronebridge-url",
                "http://192.168.9.143",
            ])
            .unwrap();
            let url = match args.command {
                Command::Serve {
                    dronebridge_url, ..
                }
                | Command::TestConnection {
                    dronebridge_url, ..
                } => dronebridge_url,
                _ => panic!("unexpected command"),
            };
            assert_eq!(url.as_deref(), Some("http://192.168.9.143"));
        }
    }

    #[tokio::test]
    async fn exclusive_bridge_cleanup_rejects_non_remote_udp_connections() {
        for connection in ["tcp:127.0.0.1:5760", "udpin:0.0.0.0:14550", "auto"] {
            let config = link_config(connection, None, "115200", Duration::from_secs(3)).unwrap();
            assert!(
                configure_dronebridge(Some("http://127.0.0.1"), &config.transport)
                    .await
                    .is_err()
            );
            assert!(
                configure_dronebridge(None, &config.transport)
                    .await
                    .unwrap()
                    .is_none()
            );
        }
    }

    #[tokio::test]
    async fn connection_check_cleans_up_after_success_and_failure() {
        let clears = Arc::new(AtomicUsize::new(0));
        let delete_state = clears.clone();
        let router = Router::new()
            .route(
                "/api/settings/clients/clear_udp",
                delete(move || {
                    let state = delete_state.clone();
                    async move {
                        state.fetch_add(1, Ordering::SeqCst);
                        axum::http::StatusCode::OK
                    }
                }),
            )
            .route(
                "/api/system/stats",
                get(|| async { Json(json!({"udp_connected": 0, "udp_clients": []})) }),
            );
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let bridge =
            DroneBridge::new(&format!("http://{}", listener.local_addr().unwrap())).unwrap();
        let server = tokio::spawn(async move { axum::serve(listener, router).await.unwrap() });
        for success in [true, false] {
            let socket = tokio::net::UdpSocket::bind("127.0.0.1:0").await.unwrap();
            let config = MavlinkLinkConfig::udp(socket.local_addr().unwrap(), false);
            let peer = tokio::spawn(async move {
                let mut packet = [0; 1024];
                let (length, address) = socket.recv_from(&mut packet).await.unwrap();
                if success {
                    socket.send_to(&packet[..length], address).await.unwrap();
                } else {
                    time::sleep(Duration::from_secs(1)).await;
                }
            });
            let result =
                test_connection_with_cleanup(config, Duration::from_millis(100), Some(&bridge))
                    .await;
            assert_eq!(result.is_ok(), success);
            peer.abort();
        }
        assert_eq!(clears.load(Ordering::SeqCst), 2);
        server.abort();
    }

    #[tokio::test]
    async fn connection_check_waits_for_a_mavlink_heartbeat() {
        let (mut writer, reader) = tokio::io::duplex(64);
        let heartbeat = [
            0xfd, 0x09, 0x00, 0x00, 0x09, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x06, 0x08, 0x00, 0x04, 0x03, 0x51, 0x82,
        ];
        let sender = tokio::spawn(async move {
            writer.write_all(&heartbeat).await.expect("write heartbeat");
        });

        assert_eq!(
            receive_heartbeat(reader).await.expect("heartbeat"),
            (255, 0)
        );
        sender.await.expect("heartbeat sender");
    }

    #[tokio::test]
    async fn udp_connection_check_registers_with_remote_peer() {
        let server = tokio::net::UdpSocket::bind("127.0.0.1:0").await.unwrap();
        let address = server.local_addr().unwrap();
        let peer = tokio::spawn(async move {
            let mut buffer = [0; 1024];
            let (length, client) = server.recv_from(&mut buffer).await.unwrap();
            let heartbeat = mavlink_protocol::codec::decode_raw_bytes(&buffer[..length]).unwrap();
            assert_eq!(heartbeat.message_id, 0);
            server.send_to(&buffer[..length], client).await.unwrap();
        });
        let result = test_connection(
            MavlinkLinkConfig::udp(address, false),
            Duration::from_secs(1),
        )
        .await
        .unwrap();
        assert!(result.contains("sysid=255 compid=0"));
        peer.await.unwrap();
    }
}
