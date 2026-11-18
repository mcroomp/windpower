//! Self-signed HTTPS for LAN clients. Phone browsers only expose the camera
//! (`getUserMedia`/`MediaRecorder`) to secure contexts, so the experiment
//! camera page is served over HTTPS with a certificate LinkHub generates and
//! keeps under `<data-dir>/tls/` so phones accept it once per host set.

use std::{
    io,
    net::{IpAddr, SocketAddr, UdpSocket},
    path::Path,
};

use axum_server::tls_rustls::RustlsConfig;
use tokio::fs;

const CERT_FILE: &str = "cert.pem";
const KEY_FILE: &str = "key.pem";
const HOSTS_FILE: &str = "hosts.txt";

/// The address phones should use to reach a server listening on `listen`.
pub fn lan_host(listen: IpAddr) -> Result<IpAddr, String> {
    if listen.is_loopback() {
        return Err(format!(
            "LinkHub listens on {listen} only; start it with --listen 0.0.0.0 for phones"
        ));
    }
    if !listen.is_unspecified() {
        return Ok(listen);
    }
    default_route_address().ok_or_else(|| "could not determine this machine's LAN address".into())
}

/// Address of the interface holding the default route. Connecting a UDP
/// socket only selects a route; no packet is sent.
fn default_route_address() -> Option<IpAddr> {
    let socket = UdpSocket::bind("0.0.0.0:0").ok()?;
    socket.connect(SocketAddr::from(([192, 0, 2, 1], 9))).ok()?;
    let address = socket.local_addr().ok()?.ip();
    (!address.is_loopback() && !address.is_unspecified()).then_some(address)
}

/// Loads the persisted certificate, regenerating it when the host set changed.
pub async fn load_or_create(directory: &Path, listen: IpAddr) -> io::Result<RustlsConfig> {
    let mut hosts = vec!["localhost".to_owned(), "127.0.0.1".to_owned()];
    if let Ok(host) = lan_host(listen) {
        hosts.push(host.to_string());
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
            Ok("192.168.1.20".parse().unwrap())
        );
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
