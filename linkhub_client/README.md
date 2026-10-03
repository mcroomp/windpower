# LinkHub Python Protocol Client

This dependency-free project defines LinkHub's typed HTTP/NDJSON records and
its standard-library Python client. It contains no MAVLink parser, transport,
or hardware dependency.

LinkHub is the only MAVLink owner. Calibration, ground-station code, simulation
observers, and SITL tests use `LinkHubClient`.

```python
from linkhub_client import LinkHubClient

hub = LinkHubClient("http://127.0.0.1:8999")
hub.connect()
try:
    attitude = hub.receive("ATTITUDE", timeout=1.0)
finally:
    hub.close()
```

The client maintains one lossless NDJSON subscription cursor independently
from stateful operation cursors. In SITL, `sim_now()` advances only when a
message is delivered to the caller.
