//! Host-owned message transport. The protocol client never opens a connection
//! or assigns observation cursors; the host supplies opaque correlation tokens.

use std::{error::Error, future::Future, pin::Pin, sync::Arc};

use serde_json::{Map, Value};
use tokio::sync::broadcast;

use crate::codec::MavMessage;

pub type BoxFuture<'a, T> = Pin<Box<dyn Future<Output = T> + Send + 'a>>;

#[derive(Clone, Debug)]
pub struct ReceivedMessage {
    pub cursor: String,
    pub system_id: u8,
    pub component_id: u8,
    pub name: String,
    pub fields: Map<String, Value>,
    pub message: MavMessage,
}

/// Preserves the host's concrete error, its display text and source chain.
#[derive(Debug, thiserror::Error)]
#[error(transparent)]
pub struct TransportError(pub Box<dyn Error + Send + Sync>);

pub trait Transport: Send + Sync {
    fn target(&self) -> (u8, u8);
    fn subscribe_messages(&self) -> broadcast::Receiver<Arc<ReceivedMessage>>;
    fn send_message(
        &self,
        message: MavMessage,
        source_system: Option<u8>,
        source_component: Option<u8>,
    ) -> BoxFuture<'_, Result<(), TransportError>>;
}
