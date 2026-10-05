from __future__ import annotations

from linkhub_client import LinkHubClient
from linkhub_client.messages import RawMessage


def read_one(
    session: LinkHubClient,
    after: str,
    message_types: str | list[str] | tuple[str, ...] | set[str],
    *,
    wait: float,
    expected_generation: str | None = None,
) -> tuple[RawMessage | None, str]:
    """Read the next matching message, or `(None, after)` on a plain timeout.

    Pass `expected_generation` (captured once at the start of a multi-step
    sequence, e.g. `session.generation` or a status's `"generation"`) to have
    LinkHub itself raise `LinkHubGenerationChanged` the moment its live
    generation no longer matches -- a service restart, serial reconnect, or
    vehicle reboot mid-sequence -- instead of silently continuing to poll a
    link that is no longer trustworthy. Leave it unset for a one-shot read.
    """
    batch = session.read_messages(
        after,
        message_types,
        direction="rx",
        wait=wait,
        limit=1,
        expected_generation=expected_generation,
    )
    message = batch.messages[0] if batch.messages else None
    return message, batch.next_cursor
