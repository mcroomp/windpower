use std::time::Duration;

use reqwest::{Client, Url};
use serde::Deserialize;

const REQUEST_TIMEOUT: Duration = Duration::from_secs(2);

pub struct DroneBridge {
    client: Client,
    origin: Url,
}

#[derive(Deserialize)]
struct ClientList {
    udp_connected: usize,
    udp_clients: Vec<String>,
}

impl DroneBridge {
    pub fn new(origin: &str) -> Result<Self, String> {
        let origin =
            Url::parse(origin).map_err(|error| format!("invalid DroneBridge URL: {error}"))?;
        if origin.scheme() != "http"
            || origin.host_str().is_none()
            || !origin.username().is_empty()
            || origin.password().is_some()
            || origin.path() != "/"
            || origin.query().is_some()
            || origin.fragment().is_some()
        {
            return Err(
                "DroneBridge URL must be an HTTP origin such as http://192.168.9.143".to_owned(),
            );
        }
        let client = Client::builder()
            .timeout(REQUEST_TIMEOUT)
            .redirect(reqwest::redirect::Policy::none())
            .no_proxy()
            .build()
            .map_err(|error| format!("could not create DroneBridge HTTP client: {error}"))?;
        Ok(Self { client, origin })
    }

    pub async fn clients(&self) -> Result<Vec<String>, String> {
        let url = self
            .origin
            .join("api/system/stats")
            .expect("static API path");
        let response = self
            .client
            .get(url)
            .send()
            .await
            .map_err(|error| format!("could not query DroneBridge UDP clients: {error}"))?;
        if !response.status().is_success() {
            return Err(format!(
                "DroneBridge client query rejected: HTTP {}",
                response.status()
            ));
        }
        let clients = response
            .json::<ClientList>()
            .await
            .map_err(|error| format!("invalid DroneBridge UDP client list: {error}"))?;
        if clients.udp_connected != clients.udp_clients.len() {
            return Err(format!(
                "inconsistent DroneBridge UDP client list: count={}, clients={:?}",
                clients.udp_connected, clients.udp_clients,
            ));
        }
        Ok(clients.udp_clients)
    }

    /// Only call after stopping all UDP writers; otherwise they can register again.
    pub async fn clear_clients(&self) -> Result<(), String> {
        let url = self
            .origin
            .join("api/settings/clients/clear_udp")
            .expect("static API path");
        match self.client.delete(url).send().await {
            Ok(response) => {
                let status = response.status();
                if !status.is_success() {
                    return Err(format!("DroneBridge UDP cleanup rejected: HTTP {status}"));
                }
            }
            Err(error) if error.is_timeout() => {
                // Firmware 2.4.1 clears the list without sending an HTTP response.
                tracing::warn!(%error, "DroneBridge cleanup response timed out; verifying UDP client list");
            }
            Err(error) => return Err(format!("DroneBridge UDP cleanup request failed: {error}")),
        }
        let remaining = self.clients().await?;
        if !remaining.is_empty() {
            return Err(format!(
                "DroneBridge UDP cleanup failed; clients remain: {remaining:?}"
            ));
        }
        tracing::info!("DroneBridge UDP cleanup verified: no clients remain");
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use axum::{
        Json, Router,
        http::StatusCode,
        routing::{delete, get},
    };
    use serde_json::json;
    use std::sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    };
    use tokio::{net::TcpListener, task::JoinHandle};

    async fn bridge(router: Router) -> (DroneBridge, JoinHandle<()>) {
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let mut bridge =
            DroneBridge::new(&format!("http://{}", listener.local_addr().unwrap())).unwrap();
        bridge.client = Client::builder()
            .timeout(Duration::from_millis(100))
            .no_proxy()
            .build()
            .unwrap();
        let task = tokio::spawn(async move { axum::serve(listener, router).await.unwrap() });
        (bridge, task)
    }

    #[test]
    fn requires_an_explicit_http_origin() {
        for origin in [
            "https://bridge",
            "http://bridge/path",
            "http://user@bridge",
            "http://bridge/?x=1",
            "http://bridge/#x",
        ] {
            assert!(DroneBridge::new(origin).is_err(), "{origin}");
        }
        assert!(DroneBridge::new("http://127.0.0.1:8080").is_ok());
    }

    #[tokio::test]
    async fn cleanup_verifies_the_list_after_delete() {
        let cleared = Arc::new(AtomicBool::new(false));
        let delete_state = cleared.clone();
        let query_state = cleared.clone();
        let router = Router::new()
            .route(
                "/api/settings/clients/clear_udp",
                delete(move || async move {
                    delete_state.store(true, Ordering::SeqCst);
                    StatusCode::OK
                }),
            )
            .route(
                "/api/system/stats",
                get(move || async move {
                    assert!(query_state.load(Ordering::SeqCst));
                    Json(json!({"udp_connected": 0, "udp_clients": []}))
                }),
            );
        let (bridge, task) = bridge(router).await;
        bridge.clear_clients().await.unwrap();
        assert!(cleared.load(Ordering::SeqCst));
        task.abort();
    }

    #[tokio::test]
    async fn missing_delete_response_succeeds_only_after_verification() {
        let cleared = Arc::new(AtomicBool::new(false));
        let delete_state = cleared.clone();
        let query_state = cleared.clone();
        let router = Router::new()
            .route(
                "/api/settings/clients/clear_udp",
                delete(move || async move {
                    delete_state.store(true, Ordering::SeqCst);
                    std::future::pending::<StatusCode>().await
                }),
            )
            .route(
                "/api/system/stats",
                get(move || async move {
                    assert!(query_state.load(Ordering::SeqCst));
                    Json(json!({"udp_connected": 0, "udp_clients": []}))
                }),
            );
        let (bridge, task) = bridge(router).await;
        bridge.clear_clients().await.unwrap();
        task.abort();
    }

    #[tokio::test]
    async fn timeout_does_not_hide_remaining_clients() {
        let router = Router::new()
            .route(
                "/api/settings/clients/clear_udp",
                delete(|| async { std::future::pending::<StatusCode>().await }),
            )
            .route(
                "/api/system/stats",
                get(|| async {
                    Json(json!({"udp_connected": 1, "udp_clients": ["127.0.0.1:14550"]}))
                }),
            );
        let (bridge, task) = bridge(router).await;
        let error = bridge.clear_clients().await.unwrap_err();
        assert!(error.contains("clients remain"));
        task.abort();
    }

    #[tokio::test]
    async fn cleanup_rejects_http_errors() {
        let router = Router::new().route(
            "/api/settings/clients/clear_udp",
            delete(|| async { StatusCode::FORBIDDEN }),
        );
        let (bridge, task) = bridge(router).await;
        assert!(bridge.clear_clients().await.unwrap_err().contains("403"));
        task.abort();
    }

    #[tokio::test]
    async fn cleanup_rejects_unverifiable_results() {
        for body in [
            json!({}),
            json!({"udp_connected": 0, "udp_clients": ["127.0.0.1:14550"]}),
        ] {
            let router = Router::new()
                .route(
                    "/api/settings/clients/clear_udp",
                    delete(|| async { StatusCode::OK }),
                )
                .route("/api/system/stats", get(move || async move { Json(body) }));
            let (bridge, task) = bridge(router).await;
            assert!(bridge.clear_clients().await.is_err());
            task.abort();
        }
        let router = Router::new()
            .route(
                "/api/settings/clients/clear_udp",
                delete(|| async { StatusCode::OK }),
            )
            .route(
                "/api/system/stats",
                get(|| async { StatusCode::SERVICE_UNAVAILABLE }),
            );
        let (bridge, task) = bridge(router).await;
        assert!(bridge.clear_clients().await.unwrap_err().contains("503"));
        task.abort();
    }
}
