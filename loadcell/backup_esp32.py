# /// script
# requires-python = ">=3.14"
# dependencies = ["esptool==5.1.0"]
#
# [[tool.uv.index]]
# name = "microsoft-python"
# url = "https://packagefeedproxy.microsoft.io/pypi/simple/"
# default = true
# ///
"""Read and verify a complete ESP32 flash backup without writing device flash."""

import argparse
from datetime import datetime, UTC
import hashlib
from importlib.metadata import version
import json
from pathlib import Path
import re
import subprocess
import sys
from typing import TextIO

from serial.tools import list_ports
from serial.tools.list_ports_common import ListPortInfo


DEFAULT_BACKUP_ROOT = Path(__file__).resolve().parent / "backups"
READ_ONLY_COMMANDS = frozenset({"flash-id", "read-flash", "verify-flash"})


def flash_size_bytes(output: str) -> int:
    match = re.search(r"Detected flash size:\s*(\d+)\s*(KB|MB)\b", output)
    if match is None:
        raise ValueError("esptool did not report a recognized flash size; refusing to guess.")
    size = int(match[1]) * {"KB": 1024, "MB": 1024 * 1024}[match[2]]
    if size <= 0:
        raise ValueError("esptool reported an invalid flash size.")
    return size


def run_esptool(
    port: str, baud: int, command: str, arguments: list[str], log: TextIO,
) -> str:
    if command not in READ_ONLY_COMMANDS:
        raise ValueError(f"Device command is not read-only: {command}")
    invocation = [
        sys.executable, "-u", "-m", "esptool", "--chip", "esp32",
        "--port", port, "--baud", str(baud),
        "--before", "default-reset", "--after", "no-reset",
        command, *arguments,
    ]
    log.write(f"\nCOMMAND: {subprocess.list2cmdline(invocation)}\n")
    log.flush()
    output: list[str] = []
    next_percent = 5
    with subprocess.Popen(
        invocation, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, encoding="utf-8", errors="replace",
    ) as process:
        if process.stdout is None:
            raise RuntimeError("Cannot read esptool output.")
        for line in process.stdout:
            progress = re.search(r"(\d+)/([1-9]\d*) bytes", line) if command == "read-flash" else None
            if progress is not None:
                completed, total = int(progress[1]), int(progress[2])
                while next_percent <= 100 and completed * 100 >= total * next_percent:
                    message = f"Flash read: {next_percent}% ({completed:,}/{total:,} bytes)\n"
                    print(message, end="", flush=True)
                    log.write(message)
                    log.flush()
                    output.append(message)
                    next_percent += 5
                continue
            if not line.strip():
                continue
            print(line, end="", flush=True)
            log.write(line)
            log.flush()
            output.append(line)
        log.flush()
        returncode = process.wait()
    if returncode != 0:
        raise RuntimeError(
            f"esptool {command} failed with exit code {returncode}; see esptool.log."
        )
    return "".join(output)


def port_identity(port: ListPortInfo) -> dict[str, str | int | None]:
    return {
        "device": port.device,
        "description": port.description,
        "hardware_id": port.hwid,
        "usb_vid": port.vid,
        "usb_pid": port.pid,
        "usb_serial_number": port.serial_number,
        "usb_location": port.location,
    }


def backup(port: ListPortInfo, baud: int, root: Path) -> Path:
    started = datetime.now(UTC)
    directory = root / started.strftime("esp32-%Y%m%dT%H%M%S%fZ")
    directory.mkdir(parents=True, exist_ok=False)
    partial = directory / "full_flash.partial.bin"
    image = directory / "full_flash.bin"
    print(f"Backup directory: {directory}")
    print("Device will reset into its serial bootloader. Flash is not written or erased.")
    print("Close serial monitors first. Keep the device powered and USB connected.")
    with (directory / "esptool.log").open("x", encoding="utf-8") as log:
        identity = run_esptool(port.device, baud, "flash-id", [], log)
        expected_size = flash_size_bytes(identity)
        print(f"Reading all {expected_size:,} flash bytes; this may take several minutes.", flush=True)
        run_esptool(
            port.device, baud, "read-flash", ["0", "ALL", str(partial)], log,
        )
        actual_size = partial.stat().st_size
        if actual_size != expected_size:
            raise ValueError(
                f"Incomplete flash read: expected {expected_size} bytes, got {actual_size}."
            )
        print("Read complete; verifying the entire image against the device.", flush=True)
        run_esptool(
            port.device, baud, "verify-flash", ["0", str(partial)], log,
        )
    with partial.open("rb") as stream:
        digest = hashlib.file_digest(stream, "sha256").hexdigest()
    manifest = {
        "status": "verified_against_device",
        "started_utc": started.isoformat(),
        "completed_utc": datetime.now(UTC).isoformat(),
        "chip": "esp32",
        "port": port_identity(port),
        "baud": baud,
        "esptool_version": version("esptool"),
        "flash_size_bytes": expected_size,
        "image": image.name,
        "sha256": digest,
        "verification": "esptool verify-flash against the complete device flash",
        "includes_efuses": False,
    }
    partial.rename(image)
    (directory / "manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8",
    )
    print(f"\nVERIFIED: {image}\nSHA-256: {digest}")
    print("Backup may contain credentials. Keep this directory private and out of Git.")
    print("Press the board's EN/RESET button or reconnect USB to resume the old firmware.")
    return directory


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--list", action="store_true", help="List serial ports; do not open them.")
    parser.add_argument("--port", help="Explicit USB serial port, e.g. COM12; never guessed.")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud (default: 115200).")
    parser.add_argument(
        "--output-dir", type=Path, default=DEFAULT_BACKUP_ROOT,
        help="Backup parent directory (default: loadcell/backups, ignored by Git).",
    )
    args = parser.parse_args(argv)
    if args.list and args.port:
        parser.error("--list and --port cannot be combined.")
    if args.baud <= 0:
        parser.error("--baud must be positive.")
    if not args.list and not args.port:
        parser.error("Use --list to identify the board, then --port COM_NUMBER to back it up.")
    ports = list(list_ports.comports())
    if args.list:
        print(json.dumps([port_identity(port) for port in ports], indent=2))
        return 0
    matches = [port for port in ports if port.device.casefold() == args.port.casefold()]
    if len(matches) != 1:
        parser.error("Selected port is not uniquely present. Run --list and identify the board.")
    port = matches[0]
    if port.vid is None or port.pid is None:
        parser.error("Selected port has no USB VID/PID; connect and identify the USB adapter.")
    try:
        backup(port, args.baud, args.output_dir)
    except (OSError, ValueError, RuntimeError) as error:
        print(f"BACKUP FAILED: {error}", file=sys.stderr)
        print(
            "Do not flash based on this attempt. Partial files are not verified backups. "
            "The board may remain in its bootloader; press EN/RESET to resume.",
            file=sys.stderr,
        )
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
