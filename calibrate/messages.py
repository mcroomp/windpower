from __future__ import annotations

import time

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
    """Read the next matching message within `wait` seconds.

    Returns `(message, cursor)`, or `(None, cursor)` once `wait` has elapsed
    without a match; `cursor` is always the scanned-through journal position.

    LinkHub's long-poll returns early, with an advanced cursor and no records,
    whenever new non-matching records arrive while it scans. On a busy link that
    happens constantly, so a single poll can miss a rare message such as a
    1 Hz HEARTBEAT; this keeps polling from the advanced cursor until the
    deadline.

    Pass `expected_generation` (captured once at the start of a multi-step
    sequence, e.g. `session.generation` or a status's `"generation"`) to have
    LinkHub itself raise `LinkHubGenerationChanged` the moment its live
    generation no longer matches -- a service restart, serial reconnect, or
    vehicle reboot mid-sequence -- instead of silently continuing to poll a
    link that is no longer trustworthy. Leave it unset for a one-shot read.
    """
    deadline = time.monotonic() + wait
    cursor = after
    while True:
        remaining = max(0.0, deadline - time.monotonic())
        batch = session.read_messages(
            cursor,
            message_types,
            direction="rx",
            wait=remaining,
            limit=1,
            expected_generation=expected_generation,
        )
        cursor = batch.next_cursor
        if batch.messages:
            return batch.messages[0], cursor
        if remaining <= 0.0:
            return None, cursor
