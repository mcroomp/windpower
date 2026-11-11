"""Structural contracts for the shared calibration observation engine."""

from typing import Protocol, TypedDict

from linkhub_client.client import MessageBatch
from linkhub_client.messages import MavDataStream, Message, RawMessage


class _ObservationState(TypedDict):
    armed: bool
    pending_text: list[str]


class _ObservationLog(Protocol):
    n_rows: int

    def write_header(self, columns: list[str], /) -> None: ...

    def row(self, values: list[object], /) -> None: ...


class _ObservationSession(Protocol):
    @property
    def _target_system(self) -> int: ...

    @property
    def _target_component(self) -> int: ...

    def send_message(self, message: Message | RawMessage, /) -> object: ...

    def request_data_stream(self, stream: MavDataStream, rate_hz: int, /) -> object: ...

    def current_cursor(self) -> str: ...

    def read_messages(
        self,
        after: str,
        message_types: str | list[str] | tuple[str, ...] | set[str],
        /,
        *,
        direction: str,
        wait: float,
        limit: int,
    ) -> MessageBatch: ...
