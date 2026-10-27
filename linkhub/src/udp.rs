use std::{
    io,
    net::SocketAddr,
    pin::Pin,
    task::{Context, Poll, Waker, ready},
    time::Duration,
};

use tokio::{
    io::{AsyncRead, AsyncWrite, ReadBuf},
    net::UdpSocket,
    time::{self, Sleep},
};

const IDLE_TIMEOUT: Duration = Duration::from_secs(5);

/// Adapts MAVLink UDP packets to the shared frame reader and frame writer.
/// The full datagram is retained even when the frame reader requests fewer bytes.
pub struct UdpStream {
    socket: UdpSocket,
    peer: Option<SocketAddr>,
    packet: Vec<u8>,
    start: usize,
    end: usize,
    writer_waker: Option<Waker>,
    idle: Pin<Box<Sleep>>,
}

impl UdpStream {
    pub async fn open(address: SocketAddr, listen: bool) -> io::Result<Self> {
        let bind = if listen {
            address
        } else if address.is_ipv4() {
            "0.0.0.0:0".parse().expect("IPv4 wildcard")
        } else {
            "[::]:0".parse().expect("IPv6 wildcard")
        };
        Ok(Self {
            socket: UdpSocket::bind(bind).await?,
            peer: (!listen).then_some(address),
            packet: vec![0; 65_536],
            start: 0,
            end: 0,
            writer_waker: None,
            idle: Box::pin(time::sleep(IDLE_TIMEOUT)),
        })
    }
}

impl AsyncRead for UdpStream {
    fn poll_read(
        self: Pin<&mut Self>,
        cx: &mut Context<'_>,
        output: &mut ReadBuf<'_>,
    ) -> Poll<io::Result<()>> {
        let this = self.get_mut();
        if output.remaining() == 0 {
            return Poll::Ready(Ok(()));
        }
        while this.start == this.end {
            if this.peer.is_some() && this.idle.as_mut().poll(cx).is_ready() {
                return Poll::Ready(Err(io::Error::new(
                    io::ErrorKind::TimedOut,
                    "UDP peer silent for 5 seconds",
                )));
            }
            let mut packet = ReadBuf::new(&mut this.packet);
            let sender = ready!(this.socket.poll_recv_from(cx, &mut packet))?;
            if packet.filled().is_empty() || this.peer.is_some_and(|peer| peer != sender) {
                continue;
            }
            this.start = 0;
            this.end = packet.filled().len();
            this.peer = Some(sender);
            this.idle
                .as_mut()
                .reset(time::Instant::now() + IDLE_TIMEOUT);
            if let Some(waker) = this.writer_waker.take() {
                waker.wake();
            }
        }
        let length = output.remaining().min(this.end - this.start);
        output.put_slice(&this.packet[this.start..this.start + length]);
        this.start += length;
        Poll::Ready(Ok(()))
    }
}

impl AsyncWrite for UdpStream {
    fn poll_write(
        self: Pin<&mut Self>,
        cx: &mut Context<'_>,
        bytes: &[u8],
    ) -> Poll<io::Result<usize>> {
        let this = self.get_mut();
        let Some(peer) = this.peer else {
            this.writer_waker = Some(cx.waker().clone());
            return Poll::Pending;
        };
        this.socket.poll_send_to(cx, bytes, peer)
    }

    fn poll_flush(self: Pin<&mut Self>, _cx: &mut Context<'_>) -> Poll<io::Result<()>> {
        Poll::Ready(Ok(()))
    }

    fn poll_shutdown(self: Pin<&mut Self>, _cx: &mut Context<'_>) -> Poll<io::Result<()>> {
        Poll::Ready(Ok(()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tokio::io::{AsyncReadExt, AsyncWriteExt};

    #[tokio::test]
    async fn udp_remote_preserves_datagrams_and_filters_other_senders() {
        let peer = UdpSocket::bind("127.0.0.1:0").await.unwrap();
        let mut stream = UdpStream::open(peer.local_addr().unwrap(), false)
            .await
            .unwrap();
        stream.write_all(b"hello").await.unwrap();
        let mut buffer = [0; 64];
        let (length, address) = peer.recv_from(&mut buffer).await.unwrap();
        assert_eq!(&buffer[..length], b"hello");
        let other = UdpSocket::bind("127.0.0.1:0").await.unwrap();
        other.send_to(b"wrong peer", address).await.unwrap();
        peer.send_to(&[], address).await.unwrap();
        peer.send_to(b"abcdef", address).await.unwrap();
        let mut received = [0; 6];
        stream.read_exact(&mut received[..2]).await.unwrap();
        stream.read_exact(&mut received[2..]).await.unwrap();
        assert_eq!(&received, b"abcdef");
    }

    #[tokio::test]
    async fn udp_listener_learns_peer_and_unblocks_writer() {
        let stream = UdpStream::open("127.0.0.1:0".parse().unwrap(), true)
            .await
            .unwrap();
        let address = stream.socket.local_addr().unwrap();
        let peer = UdpSocket::bind("127.0.0.1:0").await.unwrap();
        let (mut reader, mut writer) = tokio::io::split(stream);
        let send = tokio::spawn(async move { writer.write_all(b"reply").await });
        tokio::task::yield_now().await;
        assert!(!send.is_finished());
        peer.send_to(b"hello", address).await.unwrap();
        let mut received = [0; 5];
        reader.read_exact(&mut received).await.unwrap();
        assert_eq!(&received, b"hello");
        time::timeout(Duration::from_secs(1), send)
            .await
            .unwrap()
            .unwrap()
            .unwrap();
        let mut reply = [0; 64];
        let length = peer.recv(&mut reply).await.unwrap();
        assert_eq!(&reply[..length], b"reply");
        let other = UdpSocket::bind("127.0.0.1:0").await.unwrap();
        other.send_to(b"wrong", address).await.unwrap();
        peer.send_to(b"again", address).await.unwrap();
        reader.read_exact(&mut received).await.unwrap();
        assert_eq!(&received, b"again");
    }

    #[tokio::test(start_paused = true)]
    async fn udp_silence_expires_remote_but_not_unpaired_listener() {
        let peer = UdpSocket::bind("127.0.0.1:0").await.unwrap();
        let mut remote = UdpStream::open(peer.local_addr().unwrap(), false)
            .await
            .unwrap();
        time::advance(IDLE_TIMEOUT).await;
        let error = remote.read_u8().await.unwrap_err();
        assert_eq!(error.kind(), io::ErrorKind::TimedOut);
        let mut listener = UdpStream::open("127.0.0.1:0".parse().unwrap(), true)
            .await
            .unwrap();
        assert!(
            time::timeout(IDLE_TIMEOUT * 2, listener.read_u8())
                .await
                .is_err()
        );
    }
}
