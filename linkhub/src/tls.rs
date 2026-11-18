//! Self-signed HTTPS for LAN clients. Phone browsers only expose the camera
//! (`getUserMedia`/`MediaRecorder`) to secure contexts, so the experiment
//! camera page is served over HTTPS with a certificate LinkHub generates and
//! keeps under `<data-dir>/tls/` so phones accept it once per host set.

use std::{io, net::IpAddr, path::Path};

use axum_server::tls_rustls::RustlsConfig;
use tokio::fs;

const CERT_FILE: &str = "cert.pem";
const KEY_FILE: &str = "key.pem";
const HOSTS_FILE: &str = "hosts.txt";

/// The address phones should use to reach a server listening on `listen`.
pub fn lan_host(listen: IpAddr) -> Result<String, String> {
    if listen.is_loopback() {
        return Err(format!(
            "LinkHub listens on {listen} only; start it with --listen 0.0.0.0 for phones"
        ));
    }
    if !listen.is_unspecified() {
        return Ok(listen.to_string());
    }
    let name = hostname::get()
        .map_err(|error| format!("could not read this machine's computer name: {error}"))?
        .into_string()
        .map_err(|_| "this machine's computer name is not valid Unicode".to_owned())?;
    local_hostname(&name)
}

fn local_hostname(name: &str) -> Result<String, String> {
    let label = name.split('.').next().unwrap_or_default();
    if label.is_empty()
        || label.len() > 63
        || !label
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || byte == b'-')
        || label.starts_with('-')
        || label.ends_with('-')
    {
        return Err(format!(
            "computer name {name:?} cannot be used as a .local DNS hostname"
        ));
    }
    Ok(format!("{}.local", label.to_ascii_lowercase()))
}

/// Loads the persisted certificate, regenerating it when the host set changed.
pub async fn load_or_create(directory: &Path, listen: IpAddr) -> io::Result<RustlsConfig> {
    let mut hosts = vec!["localhost".to_owned(), "127.0.0.1".to_owned()];
    if !listen.is_loopback() {
        hosts.push(lan_host(listen).map_err(io::Error::other)?);
    }
    hosts.sort();
    hosts.dedup();
    let wanted_hosts = hosts.join("\n");
    let cert_path = directory.join(CERT_FILE);
    let key_path = directory.join(KEY_FILE);
    let hosts_path = directory.join(HOSTS_FILE);
    let current = fs::read_to_string(&hosts_path).await.ok();
    if current.as_deref() != Some(wanted_hosts.as_str())
        || !fs::try_exists(&cert_path).await?
        || !fs::try_exists(&key_path).await?
    {
        let certified = rcgen::generate_simple_self_signed(hosts).map_err(io::Error::other)?;
        fs::create_dir_all(directory).await?;
        fs::write(&cert_path, certified.cert.pem()).await?;
        fs::write(&key_path, certified.key_pair.serialize_pem()).await?;
        fs::write(&hosts_path, &wanted_hosts).await?;
    }
    RustlsConfig::from_pem_file(cert_path, key_path).await
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn loopback_listen_has_no_lan_host() {
        assert!(lan_host("127.0.0.1".parse().unwrap()).is_err());
        assert_eq!(
            lan_host("192.168.1.20".parse().unwrap()),
            Ok("192.168.1.20".to_owned())
        );
    }

    #[test]
    fn computer_names_use_a_single_local_suffix_and_valid_dns_labels() {
        for name in ["BENCH-PC", "bench-pc.local", "bench-pc.example.com"] {
            assert_eq!(local_hostname(name).unwrap(), "bench-pc.local");
        }
        for name in [
            "",
            ".local",
            "-pc",
            "pc-",
            "bench_pc",
            "pc name",
            &"x".repeat(64),
        ] {
            assert!(local_hostname(name).is_err(), "{name}");
        }
    }

    #[tokio::test]
    async fn wildcard_certificate_authenticates_the_computer_local_hostname() {
        let _ = rustls::crypto::ring::default_provider().install_default();
        let temp = tempfile::tempdir().unwrap();
        let listen = "0.0.0.0".parse().unwrap();
        let host = lan_host(listen).unwrap();
        assert!(host.ends_with(".local"));
        let config = load_or_create(temp.path(), listen).await.unwrap();
        let certificate = fs::read(temp.path().join(CERT_FILE)).await.unwrap();
        let hosts = fs::read_to_string(temp.path().join(HOSTS_FILE))
            .await
            .unwrap();
        assert!(hosts.lines().any(|name| name == host));
        let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        listener.set_nonblocking(true).unwrap();
        let address = listener.local_addr().unwrap();
        let server = axum_server::from_tcp_rustls(listener, config)
            .unwrap()
            .serve(
                axum::Router::new()
                    .route("/", axum::routing::get(|| async { "ok" }))
                    .into_make_service(),
            );
        let task = tokio::spawn(server);
        let result = async {
            reqwest::Client::builder()
                .no_proxy()
                .add_root_certificate(reqwest::Certificate::from_pem(&certificate).unwrap())
                .resolve(&host, address)
                .timeout(std::time::Duration::from_secs(5))
                .build()
                .unwrap()
                .get(format!("https://{host}:{}/", address.port()))
                .send()
                .await?
                .text()
                .await
        }
        .await;
        task.abort();
        assert_eq!(result.unwrap(), "ok");
    }

    #[tokio::test]
    async fn reuses_certificate_until_hosts_change() {
        rustls::crypto::ring::default_provider()
            .install_default()
            .ok();
        let temp = tempfile::tempdir().unwrap();
        load_or_create(temp.path(), "127.0.0.1".parse().unwrap())
            .await
            .unwrap();
        let first = std::fs::read_to_string(temp.path().join(CERT_FILE)).unwrap();
        load_or_create(temp.path(), "127.0.0.1".parse().unwrap())
            .await
            .unwrap();
        assert_eq!(
            std::fs::read_to_string(temp.path().join(CERT_FILE)).unwrap(),
            first
        );
        load_or_create(temp.path(), "10.1.2.3".parse().unwrap())
            .await
            .unwrap();
        assert_ne!(
            std::fs::read_to_string(temp.path().join(CERT_FILE)).unwrap(),
            first
        );
    }
}
