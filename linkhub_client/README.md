# LinkHub Python Protocol Client

This dependency-free project defines LinkHub's typed HTTP/JSON records and
its standard-library Python client. It contains no MAVLink parser, transport,
or hardware dependency.

LinkHub is the only MAVLink owner. Calibration, ground-station code, simulation
observers, and SITL tests use `LinkHubClient`.

```python
from linkhub_client import LinkHubClient

hub = LinkHubClient("http://127.0.0.1:8999")
hub.connect()
try:
    cursor = hub.current_cursor()
    batch = hub.read_messages(cursor, "ATTITUDE", wait=1.0)
    cursor = batch.next_cursor
    attitudes = batch.messages
finally:
    hub.close()
```

LinkHub owns message retention and vehicle state. Every read is a finite HTTP
request with an explicit input cursor. The response contains matching records
and a scanned-through `next_cursor`, including when no records match. Callers
own their cursors; the client has no global receive cursor, queue, follower, or
receive thread. `sim_now()`, mode confirmation, and arm state use LinkHub's
authoritative MAVLink status rather than consuming telemetry.
