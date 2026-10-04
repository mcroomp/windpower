from __future__ import annotations

from linkhub_client import LinkHubClient
from linkhub_client.messages import RawMessage


def read_one(
    session: LinkHubClient,
    after: str,
    message_types: str | list[str] | tuple[str, ...] | set[str],
    *,
    wait: float,
) -> tuple[RawMessage | None, str]:
    batch = session.read_messages(
        after,
        message_types,
        direction="rx",
        wait=wait,
        limit=1,
    )
    message = batch.messages[0] if batch.messages else None
    return message, batch.next_cursor
