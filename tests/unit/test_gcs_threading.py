from __future__ import annotations

import threading
import time

from groundstation.gcs import RawesGCS


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
