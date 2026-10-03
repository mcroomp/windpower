use std::{net::SocketAddr, path::PathBuf, time::Duration};

use clap::{Parser, Subcommand};
use linkhub::{
    http,
    journal::{JournalConfig, JournalHandle},
    mavlink::{MavlinkLinkConfig, start_link},
    motor::MotorHandle,
    records::{DiagnosticEvent, DiagnosticLevel, wall_time_ns},
};
use serde_json::{Map, json};
use tokio::{fs, net::TcpListener};
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
    Serve {
        #[arg(long, env = "LINKHUB_CONNECTION")]
        connection: String,
        #[arg(long, default_value = "127.0.0.1")]
        listen: String,
        #[arg(long, default_value_t = 8999)]
        port: u16,
        #[arg(long, default_value = "linkhub-data")]
        data_dir: PathBuf,
        #[arg(long)]
        run_id: Option<Uuid>,
        #[arg(long, default_value_t = 50)]
        flush_ms: u64,
        #[arg(long, default_value_t = 4)]
        chunk_mb: usize,
        #[arg(long, default_value_t = 255)]
        source_system: u8,
        #[arg(long, default_value_t = 0)]
        source_component: u8,
        #[arg(long, default_value_t = 115_200)]
        baud: u32,
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
        Command::Serve {
            connection,
            listen,
            port,
            data_dir,
            run_id,
            flush_ms,
            chunk_mb,
            source_system,
            source_component,
            baud,
            motor_name_prefix,
            motor_scan_timeout_ms,
            motor_heartbeat_ms,
            motor_max_command_timeout_ms,
        } => {
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

            let mut link_config = link_config(&connection, baud)?;
            link_config.source_system = source_system;
            link_config.source_component = source_component;
            let (link, link_task) = start_link(link_config, journal.clone());
            let motor = configure_motor(
                motor_name_prefix,
                motor_scan_timeout_ms,
                motor_heartbeat_ms,
                motor_max_command_timeout_ms,
            )
            .await?;

            let bind: SocketAddr = format!("{listen}:{port}").parse()?;
            let listener = TcpListener::bind(bind).await?;
            tracing::info!(%bind, %run_id, "LinkHub listening");
            axum::serve(
                listener,
                http::router_with_motor(journal.clone(), Some(link), motor.clone()),
            )
            .with_graceful_shutdown(shutdown_signal())
            .await?;

            if let Some(motor) = motor {
                motor.close().await?;
            }
            link_task.abort();
            journal.shutdown().await?;
            journal_task.await?;
        }
    }
    Ok(())
}

#[cfg(feature = "bluetooth")]
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

#[cfg(not(feature = "bluetooth"))]
async fn configure_motor(
    name_prefix: Option<String>,
    _scan_timeout_ms: u64,
    _heartbeat_ms: u64,
    _maximum_command_timeout_ms: u64,
) -> Result<Option<MotorHandle>, Box<dyn std::error::Error>> {
    if name_prefix.is_some() {
        return Err("LinkHub was built without the bluetooth feature".into());
    }
    Ok(None)
}

fn link_config(connection: &str, baud: u32) -> Result<MavlinkLinkConfig, String> {
    if let Some(address) = connection.strip_prefix("tcp:") {
        return address
            .parse()
            .map(MavlinkLinkConfig::sitl)
            .map_err(|error| format!("invalid TCP MAVLink connection {connection:?}: {error}"));
    }
    if let Ok(address) = connection.parse::<SocketAddr>() {
        return Ok(MavlinkLinkConfig::sitl(address));
    }
    let port = connection.strip_prefix("serial:").unwrap_or(connection);
    if port.is_empty() {
        return Err("serial port must not be empty".to_owned());
    }
    Ok(MavlinkLinkConfig::serial(port, baud))
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
    #[cfg(not(unix))]
    let terminate = std::future::pending::<()>();
    tokio::select! {
        () = ctrl_c => {},
        () = terminate => {},
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn accepts_tcp_and_serial_connection_syntax() {
        assert!(matches!(
            link_config("tcp:127.0.0.1:5760", 115_200)
                .expect("TCP connection")
                .transport,
            linkhub::mavlink::MavlinkTransport::Tcp(_)
        ));
        assert!(matches!(
            link_config("COM4", 115_200)
                .expect("serial connection")
                .transport,
            linkhub::mavlink::MavlinkTransport::Serial { port, baud }
                if port == "COM4" && baud == 115_200
        ));
    }
}
