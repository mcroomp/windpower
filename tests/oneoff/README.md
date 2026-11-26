# One-off diagnostics

These scripts are manual investigations, not normal unit tests. Run them with
`uv run python` from the repository root.

## Independent pymavlink FTP probe

[probe_pymavlink_ftp.py](probe_pymavlink_ftp.py) tests native Pixhawk USB using
the upstream `pymavlink.mavutil` and `pymavlink.mavftp.MAVFTP` client, without
LinkHub's transport or FTP implementation. Keep this probe for comparing
client implementations when an FTP operation fails.

### Running

1. Stop applications holding the selected USB port, including any LinkHub
   instance using it. Do not run competing FTP transactions.
2. Identify the live device:

   ```powershell
   uv run python -m serial.tools.list_ports -v
   ```

   The probe requires native Pixhawk USB `VID:PID=3162:0053`, with a device
   serial number and location. COM names can change; never reuse an earlier
   session's port without checking identity.
3. Run with the verified port and baud, for example:

   ```powershell
   $env:MAVLINK20 = '1'
   uv run python tests\oneoff\probe_pymavlink_ftp.py --port $env:PIXHAWK_USB_PORT --baud $env:PIXHAWK_USB_BAUD
   ```

   Set `PIXHAWK_USB_PORT` and `PIXHAWK_USB_BAUD` to the port and baud confirmed
   in the current session. Neither value is a persistent device identity.

### Behavior and results

The probe sends a normal GCS heartbeat, waits for the autopilot heartbeat,
and reads `MAV_SYSID` as a control check (one-second retries within a
five-second deadline). It also requests firmware capabilities and reads
`BRD_OPTIONS` without changing it. It then uses upstream MAVFTP
to reset its FTP sessions and list `/APM/scripts`. It prints the result code,
FTP request/reply counts and addresses, heartbeat count, firmware information,
observed radio-buffer reports, decoder errors, and returned directory entries.
Add `--debug` to print the FTP wire frames and upstream listing debug output.
Status texts received during the test are printed, including controller-side
FTP-disabled or initialization-failure warnings.

A failed heartbeat or parameter control check stops the probe **before FTP**;
that is not an FTP test result. A failed listing exits nonzero. Zero FTP
replies with successful control checks demonstrates missing FTP responses
on that connection, but does not alone identify the controller-side cause.
Upstream's generic `Fail` result can be its initial return value when its idle
check ends a no-response operation; it does not prove a remote FTP NACK.

### Raw-port comparison

To bypass both `mavutil.mavlink_connection` and the `MAVFTP` client while keeping
the standard pymavlink decoder and checksum implementation:

```powershell
$env:MAVLINK20 = '1'
uv run python tests\oneoff\probe_pymavlink_ftp.py --port $env:PIXHAWK_USB_PORT --baud $env:PIXHAWK_USB_BAUD --raw
```

This opens pyserial directly, verifies heartbeat and `MAV_SYSID`, and sends
hand-encoded ResetSessions/ListDirectory requests in trimmed MAVLink 2,
full-length MAVLink 2, and MAVLink 1. Each operation retries every 0.5 seconds
within a five-second deadline. Wire frames and actual replies are printed.
The raw listing reads only its first directory page: this is a transport
diagnostic, not a complete FTP client. Use `--source-component N` to compare
client identities (default 190, source system 255) in either mode.

To check whether USB replies appear on the bridge connection instead:

```powershell
uv run python tests\oneoff\probe_pymavlink_ftp.py --port $env:PIXHAWK_USB_PORT --baud $env:PIXHAWK_USB_BAUD --raw --source-component 191 --observe-tcp "$($env:BRIDGE_HOST):$($env:BRIDGE_PORT)"
```

Set `BRIDGE_HOST` and `BRIDGE_PORT` to the bridge endpoint verified in the
current session. This sends only on USB and listens on both connections. The TCP observer never
sends a heartbeat or request, so it does not advertise the USB client's identity
on the other channel. This paired test uses standard trimmed MAVLink 2 only;
the summary labels each observed stream and the channel of any matching reply.

The probe closes USB on success or failure. It does not arm, change parameters
or modes, reboot, upload scripts, or write remote files. The upstream FTP reset
can close existing sessions belonging to this client's source identity, so
use the port exclusively. It does not create another MAVLink transport log;
output is printed to the console. For production/run diagnosis, query
LinkHub's canonical journal first, as described in
[the LinkHub owner](../../design/linkhub.md).
