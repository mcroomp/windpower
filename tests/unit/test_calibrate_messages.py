"""calibrate.messages.read_one keeps polling past LinkHub's early empty batches."""
from dataclasses import dataclass

import calibrate.messages as messages_module
from calibrate.messages import read_one


@dataclass
class _Batch:
    messages: tuple
    next_cursor: str


class _Clock:
    """Fake monotonic clock that advances by `step` on every read."""

    def __init__(self, step: float) -> None:
        self.now = 100.0
        self.step = step

    def __call__(self) -> float:
        value = self.now
        self.now += self.step
        return value


class _Session:
    def __init__(self, batches: list[_Batch]) -> None:
        self._batches = iter(batches)
        self.calls: list[dict] = []

    def read_messages(self, after, message_types, **kwargs):
        self.calls.append({"after": after, "types": message_types, **kwargs})
        return next(self._batches)


def test_read_one_continues_from_the_advanced_cursor_until_a_match(monkeypatch) -> None:
    monkeypatch.setattr(messages_module.time, "monotonic", _Clock(step=0.5))
    heartbeat = object()
    session = _Session(
        [
            _Batch((), "v1:11"),  # LinkHub returned early: only non-matching records
            _Batch((), "v1:25"),
            _Batch((heartbeat,), "v1:30"),
        ]
    )

    message, cursor = read_one(session, "v1:10", "HEARTBEAT", wait=5.0)

    assert message is heartbeat
    assert cursor == "v1:30"
    assert [call["after"] for call in session.calls] == ["v1:10", "v1:11", "v1:25"]
    waits = [call["wait"] for call in session.calls]
    assert waits == sorted(waits, reverse=True) and waits[0] <= 5.0
    assert all(call["direction"] == "rx" and call["limit"] == 1 for call in session.calls)


def test_read_one_returns_none_with_the_scanned_cursor_after_the_deadline(
    monkeypatch,
) -> None:
    monkeypatch.setattr(messages_module.time, "monotonic", _Clock(step=1.0))
    session = _Session([_Batch((), f"v1:{n}") for n in range(20, 40)])

    message, cursor = read_one(session, "v1:10", "HEARTBEAT", wait=2.5)

    assert message is None
    assert cursor == f"v1:{20 + len(session.calls) - 1}"  # the last batch's cursor
    assert session.calls[-1]["wait"] == 0.0
    assert 2 <= len(session.calls) <= 5


def test_read_one_with_zero_wait_polls_exactly_once() -> None:
    session = _Session([_Batch((), "v1:11")])

    message, cursor = read_one(session, "v1:10", "ATTITUDE", wait=0.0)

    assert (message, cursor) == (None, "v1:11")
    assert len(session.calls) == 1


def test_read_one_passes_the_generation_guard_on_every_poll(monkeypatch) -> None:
    monkeypatch.setattr(messages_module.time, "monotonic", _Clock(step=0.5))
    session = _Session([_Batch((), "v1:11"), _Batch((object(),), "v1:12")])

    read_one(session, "v1:10", "HEARTBEAT", wait=5.0, expected_generation="v1:run:1")

    assert [call["expected_generation"] for call in session.calls] == ["v1:run:1"] * 2
