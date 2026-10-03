from __future__ import annotations

import queue
import threading
import time
from types import SimpleNamespace
from pathlib import Path

from groundstation.gcs import RawesGCS, SimClock


def test_send_message_serializes_background_and_foreground_writes():
    state_lock = threading.Lock()
    barrier = threading.Barrier(8)
    active = 0
    max_active = 0

    class Message:
        def send(self, _mav) -> None:
            nonlocal active, max_active
            with state_lock:
                active += 1
                max_active = max(max_active, active)
            time.sleep(0.01)
            with state_lock:
                active -= 1

    gcs = RawesGCS()

    def send() -> None:
        barrier.wait()
        gcs.send_message(Message())

    threads = [
        threading.Thread(target=send)
        for _ in range(8)
    ]

    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(timeout=1.0)

    assert all(not thread.is_alive() for thread in threads)
    assert max_active == 1


class _FakeMavSender:
    def set_send_callback(self, _callback) -> None:
        pass


class _FakeMavConnection:
    def __init__(self) -> None:
        self.mav = _FakeMavSender()
        self.target_system = 1
        self.target_component = 1
        self._messages: queue.Queue[object] = queue.Queue()
        self._closed = threading.Event()
        self.recv_calls: list[bool] = []
        self.logfile_raw = None

    def push(self, msg: object) -> None:
        self._messages.put(msg)

    def recv_match(self, *_, blocking=False, timeout=None, **__):
        self.recv_calls.append(bool(blocking))
        if self._closed.is_set():
            return None
        try:
            if blocking:
                return self._messages.get(timeout=timeout or 0.0)
            return self._messages.get_nowait()
        except queue.Empty:
            return None

    def wait_heartbeat(self, *_, **__):
        return None

    def close(self) -> None:
        self._closed.set()


class _FakeMsg:
    def __init__(self, msg_type: str, *, time_boot_ms: int = 0, base_mode: int = 0):
        self._msg_type = msg_type
        self.time_boot_ms = time_boot_ms
        self.base_mode = base_mode
        self._header = SimpleNamespace(srcSystem=42, srcComponent=7)

    def get_type(self) -> str:
        return self._msg_type


def _start_fake_gcs(receive_mode: str, *, clock=None) -> tuple[RawesGCS, _FakeMavConnection]:
    fake = _FakeMavConnection()
    gcs = RawesGCS(receive_mode=receive_mode, clock=clock)
    gcs._mav = fake
    gcs._start_recv_worker()
    return gcs, fake


def test_background_receive_worker_buffers_hardware_style_messages():
    gcs, fake = _start_fake_gcs("background")
    try:
        msg = _FakeMsg("LOCAL_POSITION_NED", time_boot_ms=100)
        fake.push(msg)

        assert gcs._recv(type="LOCAL_POSITION_NED", blocking=True, timeout=1.0) is msg
        assert True in fake.recv_calls
    finally:
        gcs.close()


def test_recv_filters_nonmatching_messages_in_fifo_order():
    gcs, fake = _start_fake_gcs("background", clock=SimClock())
    try:
        skipped = _FakeMsg("STATUSTEXT", time_boot_ms=10)
        wanted = _FakeMsg("ATTITUDE", time_boot_ms=20)
        fake.push(skipped)
        fake.push(wanted)

        assert gcs._recv(type="ATTITUDE", blocking=True, timeout=1.0) is wanted
        assert gcs.sim_now() == 0.02
        assert gcs._recv(type="STATUSTEXT", blocking=False) is None
    finally:
        gcs.close()


def test_close_unblocks_pending_receive():
    gcs, _fake = _start_fake_gcs("background")
    received = []

    def receive() -> None:
        received.append(gcs._recv(type="ATTITUDE", blocking=True, timeout=30.0))

    thread = threading.Thread(target=receive)
    thread.start()
    time.sleep(0.05)
    gcs.close()
    thread.join(timeout=1.0)

    assert not thread.is_alive()
    assert received == [None]


def test_lockstep_receive_worker_waits_for_explicit_recv_handshake():
    gcs, fake = _start_fake_gcs("lockstep")
    try:
        msg = _FakeMsg("HEARTBEAT", base_mode=128)
        fake.push(msg)
        time.sleep(0.05)

        assert fake.recv_calls == []
        assert gcs._recv(type="HEARTBEAT", blocking=False) is msg
        assert gcs.is_armed
        assert gcs._target_system == 42
        assert gcs._target_component == 7
    finally:
        gcs.close()


def test_start_mavlog_attaches_native_raw_logfile(tmp_path: Path):
    gcs, fake = _start_fake_gcs("background")
    jsonl_path = tmp_path / "traffic.mavlink.jsonl"
    raw_path = tmp_path / "traffic.tlog.raw"
    try:
        gcs.start_mavlog(jsonl_path, raw_path)
        assert fake.logfile_raw is not None
        fake.logfile_raw.write(b"\xFE\x00")
        fake.logfile_raw.flush()
        assert raw_path.read_bytes() == b"\xFE\x00"
    finally:
        gcs.close()

    assert fake.logfile_raw is None


def test_stop_mavlog_rebuilds_canonical_ndjson_from_raw(monkeypatch, tmp_path: Path):
    gcs, _fake = _start_fake_gcs("background")
    calls = []
    jsonl_path = tmp_path / "traffic.mavlink.jsonl"
    raw_path = tmp_path / "traffic.tlog.raw"

    def convert(raw, ndjson):
        calls.append((Path(raw), Path(ndjson)))
        Path(ndjson).write_text('{"_source":"native_raw"}\n', encoding="utf-8")
        return 1

    monkeypatch.setattr("groundstation.gcs.convert_raw_to_ndjson", convert)
    try:
        gcs.start_mavlog(jsonl_path, raw_path)
    finally:
        gcs.close()

    assert calls == [(raw_path, jsonl_path)]
    assert jsonl_path.read_text(encoding="utf-8") == '{"_source":"native_raw"}\n'
