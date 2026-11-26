"""Read-only native-USB FTP comparison using upstream pymavlink.

Run with MAVLINK20=1 and an explicitly identified native-USB port/baud.
See tests/oneoff/README.md for usage, ownership, and result interpretation.
"""

import argparse
from collections import Counter
import json
import logging
import select
import socket
import struct
import time

from pymavlink import mavftp, mavutil
from serial import Serial
from serial.tools import list_ports


def encode_raw_ftp_request(
    framing: str, mav_sequence: int, ftp_sequence: int, source_component: int,
    system: int, component: int, opcode: int,
) -> bytes:
    data = b"/APM/scripts" if opcode == mavftp.OP_ListDirectory else b""
    ftp_payload = struct.pack("<HBBBBBBI", ftp_sequence, 0, opcode, len(data), 0, 0, 0, 0)
    payload = bytes((0, system, component)) + (ftp_payload + data).ljust(251, b"\0")
    message_id = mavutil.mavlink.MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL
    if framing == "mavlink1":
        header = bytes((0xFE, len(payload), mav_sequence, 255, source_component, message_id))
    elif framing in ("mavlink2-trimmed", "mavlink2-full"):
        if framing == "mavlink2-trimmed":
            payload = payload.rstrip(b"\0")
        header = bytes((0xFD, len(payload), 0, 0, mav_sequence, 255, source_component))
        header += message_id.to_bytes(3, "little")
    else:
        raise ValueError(f"Unknown MAVLink framing: {framing}")
    checksum = mavutil.x25crc(header[1:] + payload)
    checksum.accumulate(bytes((mavutil.mavlink.MAVLink_file_transfer_protocol_message.crc_extra,)))
    return header + payload + struct.pack("<H", checksum.crc)


def probe_raw_port(
    port_name: str, baud: int, source_component: int, observe_tcp: str | None = None,
) -> None:
    port = Serial(port_name, baudrate=baud, timeout=0.05, write_timeout=2)
    parser = mavutil.mavlink.MAVLink(port, srcSystem=255, srcComponent=source_component)
    parser.robust_parsing = True
    received = Counter()
    next_heartbeat = 0.0
    observer = None
    tcp_parser = mavutil.mavlink.MAVLink(None)
    tcp_parser.robust_parsing = True

    def exchange(send, predicate, timeout_s: float = 5):
        nonlocal next_heartbeat
        deadline = time.monotonic() + timeout_s
        next_send = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_heartbeat:
                parser.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, mavutil.mavlink.MAV_STATE_ACTIVE,
                )
                next_heartbeat = now + 1
            if send is not None and now >= next_send:
                send()
                next_send = now + 0.5
            chunk = port.read(max(1, min(port.in_waiting, 4096)))
            chunks = [("usb", parser, chunk)]
            if observer is not None and select.select([observer], [], [], 0)[0]:
                tcp_chunk = observer.recv(4096)
                if not tcp_chunk:
                    raise ConnectionError("The receive-only TCP observer connection closed")
                chunks.append(("tcp", tcp_parser, tcp_chunk))
            for channel, decoder, data in chunks:
                for message in decoder.parse_buffer(data) or ():
                    received[f"{channel}:{message.get_type()}"] += 1
                    if message.get_type() == "STATUSTEXT":
                        print(f"{channel} STATUSTEXT severity={message.severity}: {message.text}", flush=True)
                    elif message.get_type() == "FILE_TRANSFER_PROTOCOL":
                        print(f"Raw FTP RX on {channel}: {message.to_dict()}", flush=True)
                    elif message.get_type() == "BAD_DATA" and received[f"{channel}:BAD_DATA"] <= 5:
                        logging.warning("%s decoder rejected data: %s", channel, message.reason)
                    if predicate(message, channel):
                        return message, channel
        return None

    try:
        if observe_tcp is not None:
            host, port_text = observe_tcp.rsplit(":", 1)
            observer = socket.create_connection((host, int(port_text)), timeout=5)
            observer.setblocking(False)
            print(f"Observing TCP {observe_tcp}; no packets will be sent on TCP", flush=True)
        heartbeat_reply = exchange(
            None,
            lambda m, channel: channel == "usb" and m.get_type() == "HEARTBEAT" and m.get_srcComponent() == 1
            and m.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            timeout_s=10,
        )
        if heartbeat_reply is None:
            raise RuntimeError("No autopilot heartbeat received by raw-port decoder")
        heartbeat, _ = heartbeat_reply
        system, component = heartbeat.get_srcSystem(), heartbeat.get_srcComponent()
        parameter_reply = exchange(
            lambda: parser.param_request_read_send(system, component, b"MAV_SYSID", -1),
            lambda m, channel: channel == "usb" and m.get_type() == "PARAM_VALUE" and m.get_srcSystem() == system
            and m.get_srcComponent() == component and m.param_id == "MAV_SYSID",
        )
        if parameter_reply is None:
            raise RuntimeError("Raw-port parameter-read control check failed")
        parameter, _ = parameter_reply
        print(f"Raw-port control: {system}/{component}, MAV_SYSID={parameter.param_value}", flush=True)
        results = []
        ftp_sequence = 0
        framings = (
            ("mavlink2-trimmed",) if observe_tcp is not None
            else ("mavlink2-trimmed", "mavlink2-full", "mavlink1")
        )
        for framing in framings:
            for opcode in (mavftp.OP_ResetSessions, mavftp.OP_ListDirectory):
                attempts = 0

                def send_request() -> None:
                    nonlocal attempts
                    frame = encode_raw_ftp_request(
                        framing, parser.seq, ftp_sequence, source_component, system, component, opcode,
                    )
                    if port.write(frame) != len(frame):
                        raise OSError("Incomplete raw FTP serial write")
                    parser.seq = (parser.seq + 1) % 256
                    attempts += 1
                    if attempts == 1:
                        print(f"Raw FTP TX {framing}, opcode={opcode}, bytes={len(frame)}: {frame.hex()}", flush=True)

                reply_result = exchange(
                    send_request,
                    lambda m, _channel: m.get_type() == "FILE_TRANSFER_PROTOCOL"
                    and m.get_srcSystem() == system and m.get_srcComponent() == component
                    and m.target_system == 255 and m.target_component == source_component
                    and m.payload[5] == opcode,
                )
                item = {"framing": framing, "opcode": opcode, "attempts": attempts, "reply": None}
                if reply_result is not None:
                    reply, reply_channel = reply_result
                    payload = reply.payload
                    item["reply"] = {
                        "channel": reply_channel,
                        "opcode": payload[3],
                        "sequence": int.from_bytes(payload[:2], "little"),
                        "expected_sequence": (ftp_sequence + 1) % 65536,
                        "data": repr(bytes(payload[12:12 + payload[4]])),
                    }
                else:
                    logging.error("No raw FTP reply: %s opcode=%s after five seconds", framing, opcode)
                results.append(item)
                ftp_sequence += 1
        print(json.dumps({"raw_port": True, "source_component": source_component,
                          "receive_only_tcp": observe_tcp,
                          "received_messages": dict(received), "results": results}, indent=2), flush=True)
        if not any(
            item["opcode"] == mavftp.OP_ListDirectory
            and item["reply"] is not None and item["reply"]["opcode"] == mavftp.OP_Ack
            for item in results
        ):
            raise RuntimeError("No raw-port directory-list request succeeded")
    finally:
        try:
            if observer is not None:
                observer.close()
        finally:
            port.close()
            print("USB connection closed", flush=True)


def read_parameter(master, system: int, component: int, name: str) -> float | None:
    deadline = time.monotonic() + 5
    next_request = 0.0
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_request:
            master.mav.param_request_read_send(system, component, name.encode("ascii"), -1)
            next_request = now + 1
        candidate = master.recv_match(
            type="PARAM_VALUE", blocking=True, timeout=min(1, deadline - time.monotonic()),
        )
        if (
            candidate is not None
            and candidate.get_srcSystem() == system
            and candidate.get_srcComponent() == component
            and candidate.param_id == name
        ):
            return float(candidate.param_value)
    logging.warning("No matching parameter reply for %s within five seconds", name)
    return None


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", required=True, type=int)
    parser.add_argument("--debug", action="store_true", help="Print FTP wire frames and upstream debug output")
    parser.add_argument("--raw", action="store_true", help="Use pyserial directly and test three FTP wire framings")
    parser.add_argument("--source-component", type=int, default=190)
    parser.add_argument("--observe-tcp", help="Receive-only HOST:PORT observer for the raw routing test")
    args = parser.parse_args()
    if not 0 <= args.source_component <= 255:
        parser.error("--source-component must be in [0, 255]")
    if args.observe_tcp is not None and not args.raw:
        parser.error("--observe-tcp requires --raw")
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    identity = next((port for port in list_ports.comports() if port.device == args.port), None)
    if (
        identity is None
        or (identity.vid, identity.pid) != (0x3162, 0x0053)
        or not identity.serial_number
        or not identity.location
    ):
        raise RuntimeError("The selected port is not positively identified native Pixhawk USB")
    if mavutil.mavlink.WIRE_PROTOCOL_VERSION != "2.0":
        raise RuntimeError("Set MAVLINK20=1 before running this diagnostic")
    print(f"Native USB: {args.port}, VID:PID=3162:0053, location={identity.location}", flush=True)
    if args.raw:
        probe_raw_port(args.port, args.baud, args.source_component, args.observe_tcp)
        return
    received = Counter()
    sent = Counter()
    sent_opcodes = Counter()
    version = {}
    radio_status = []

    def describe_ftp(direction: str, message) -> None:
        payload = message.payload
        print(
            f"FTP {direction}: {message.get_srcSystem()}/{message.get_srcComponent()} -> "
            f"{message.target_system}/{message.target_component}; "
            f"seq={int.from_bytes(payload[:2], 'little')}, session={payload[2]}, "
            f"opcode={payload[3]}, size={payload[4]}, requested opcode={payload[5]}",
            flush=True,
        )
        if args.debug:
            print(f"FTP {direction} wire: {message.get_msgbuf().hex()}", flush=True)

    def observe_send(message) -> None:
        sent[message.get_type()] += 1
        if message.get_type() == "FILE_TRANSFER_PROTOCOL":
            sent_opcodes[message.payload[3]] += 1
            describe_ftp("TX", message)

    def observe(_connection, message) -> None:
        received[message.get_type()] += 1
        if message.get_type() == "FILE_TRANSFER_PROTOCOL":
            describe_ftp("RX", message)
        elif message.get_type() == "STATUSTEXT":
            print(f"STATUSTEXT severity={message.severity}: {message.text}", flush=True)
        elif message.get_type() == "AUTOPILOT_VERSION":
            value = message.flight_sw_version
            version.update({
                "version": f"{value >> 24}.{(value >> 16) & 255}.{(value >> 8) & 255}",
                "release_type": value & 255,
                "capabilities": int(message.capabilities),
                "ftp_advertised": bool(message.capabilities & mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_FTP),
            })
        elif message.get_type() == "RADIO_STATUS":
            radio_status.append({
                "system": message.get_srcSystem(),
                "component": message.get_srcComponent(),
                "txbuf": message.txbuf,
            })
        elif message.get_type() == "BAD_DATA" and received["BAD_DATA"] <= 5:
            logging.warning("MAVLink decoder rejected data: %s", message.reason)

    master = mavutil.mavlink_connection(
        args.port, baud=args.baud, source_system=255, source_component=args.source_component,
        dialect="ardupilotmega",
    )
    master.message_hooks.append(observe)
    master.mav.set_send_callback(observe_send)
    last_heartbeat = 0.0

    def send_heartbeat(_connection, _message=None) -> None:
        nonlocal last_heartbeat
        now = time.monotonic()
        if now - last_heartbeat >= 1:
            master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, mavutil.mavlink.MAV_STATE_ACTIVE,
            )
            last_heartbeat = now

    master.message_hooks.append(send_heartbeat)
    master.idle_hooks.append(send_heartbeat)
    try:
        send_heartbeat(master)
        deadline = time.monotonic() + 10
        heartbeat = None
        while time.monotonic() < deadline:
            candidate = master.recv_match(
                type="HEARTBEAT", blocking=True, timeout=deadline - time.monotonic(),
            )
            if (
                candidate is not None
                and candidate.get_srcComponent() == 1
                and candidate.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID
            ):
                heartbeat = candidate
                break
        if heartbeat is None:
            raise RuntimeError("No autopilot heartbeat received over native USB")
        system = heartbeat.get_srcSystem()
        component = heartbeat.get_srcComponent()
        print(f"Autopilot heartbeat: {system}/{component}; MAVLink 2", flush=True)
        parameter = read_parameter(master, system, component, "MAV_SYSID")
        if parameter is None:
            raise RuntimeError("The ordinary parameter-read control check did not succeed")
        print(f"Ordinary parameter read: MAV_SYSID={parameter}", flush=True)
        master.mav.command_long_send(
            system, component, mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
            mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION, 0, 0, 0, 0, 0, 0,
        )
        board_options = read_parameter(master, system, component, "BRD_OPTIONS")
        print(f"BRD_OPTIONS={board_options}", flush=True)

        # The upstream client performs ResetSessions during construction.
        started = time.monotonic()
        client = mavftp.MAVFTP(master, system, component)
        client.ftp_settings.debug = 2 if args.debug else 0
        result = client.cmd_list(["/APM/scripts"])
        result.display_message()
        print(json.dumps({
            "library": "pymavlink.mavftp.MAVFTP",
            "directory": "/APM/scripts",
            "ftp_return_code": int(result.return_code),
            "ftp_error": mavftp.FtpError(result.return_code).name,
            "ftp_requests_sent": sent["FILE_TRANSFER_PROTOCOL"],
            "ftp_requests_by_opcode": dict(sent_opcodes),
            "ftp_replies_received": received["FILE_TRANSFER_PROTOCOL"],
            "heartbeats_received": received["HEARTBEAT"],
            "elapsed_ftp_s": round(time.monotonic() - started, 3),
            "firmware": version,
            "board_options": board_options,
            "radio_status": radio_status,
            "decoder_errors": received["BAD_DATA"],
            "directory_entries": [
                {"name": entry.name, "size": entry.size_b, "is_dir": entry.is_dir}
                for entry in client.list_result
            ],
        }, indent=2), flush=True)
        if result.return_code != mavftp.FtpError.Success:
            raise RuntimeError("Upstream pymavlink FTP directory listing failed")
    finally:
        master.close()
        print("USB connection closed", flush=True)


if __name__ == "__main__":
    main()
