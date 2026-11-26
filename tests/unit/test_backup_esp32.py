import hashlib
import io
import json
from pathlib import Path
import re
from typing import TextIO
from unittest.mock import MagicMock

import pytest
from serial.tools.list_ports_common import ListPortInfo

from loadcell import backup_esp32


@pytest.fixture
def usb_port() -> ListPortInfo:
    port = ListPortInfo("COM42")
    port.vid = 0x10C4
    port.pid = 0xEA60
    port.serial_number = "test-board"
    return port


@pytest.mark.parametrize("size,expected", [("4MB", 4194304), ("16 MB", 16777216), ("512KB", 524288)])
def test_detected_flash_size(size: str, expected: int) -> None:
    assert backup_esp32.flash_size_bytes(f"Device: 4016\nDetected flash size: {size}\n") == expected


@pytest.mark.parametrize("output", ["Detected flash size: Unknown", "", "Detected flash size: 0MB"])
def test_unknown_flash_size_is_not_guessed(output: str) -> None:
    with pytest.raises(ValueError):
        backup_esp32.flash_size_bytes(output)


@pytest.mark.parametrize("failure", [None, "short-read", "verify-failed"])
def test_backup_requires_complete_verified_image(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, usb_port: ListPortInfo, failure: str | None,
) -> None:
    image = bytes(range(256)) * 4
    commands: list[str] = []

    def fake_esptool(
        port: str, baud: int, command: str, arguments: list[str], log: TextIO,
    ) -> str:
        commands.append(command)
        assert port == "COM42"
        assert baud == 115200
        if command == "flash-id":
            return "Detected flash size: 1KB\n"
        if command == "read-flash":
            assert arguments[:2] == ["0", "ALL"]
            assert len(arguments) == 3
            Path(arguments[2]).write_bytes(image[:-1] if failure == "short-read" else image)
        if command == "verify-flash":
            assert Path(arguments[1]).read_bytes() == image
            if failure == "verify-failed":
                raise RuntimeError("Verification mismatch")
        return ""

    monkeypatch.setattr(backup_esp32, "run_esptool", fake_esptool)
    monkeypatch.setattr(backup_esp32, "version", lambda _: "test-version")
    if failure:
        with pytest.raises((ValueError, RuntimeError)):
            backup_esp32.backup(usb_port, 115200, tmp_path)
        directory, = tmp_path.iterdir()
        assert (directory / "full_flash.partial.bin").exists()
        assert not (directory / "full_flash.bin").exists()
        assert not (directory / "manifest.json").exists()
        if failure == "short-read":
            assert commands == ["flash-id", "read-flash"]
    else:
        directory = backup_esp32.backup(usb_port, 115200, tmp_path)
        assert (directory / "full_flash.bin").read_bytes() == image
        manifest = json.loads((directory / "manifest.json").read_text())
        assert manifest["status"] == "verified_against_device"
        assert manifest["sha256"] == hashlib.sha256(image).hexdigest()
        assert manifest["flash_size_bytes"] == len(image)
        assert commands == ["flash-id", "read-flash", "verify-flash"]


def test_unknown_capacity_never_reads_flash(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, usb_port: ListPortInfo,
) -> None:
    commands: list[str] = []

    def fake_esptool(
        port: str, baud: int, command: str, arguments: list[str], log: TextIO,
    ) -> str:
        commands.append(command)
        return "Detected flash size: Unknown"

    monkeypatch.setattr(backup_esp32, "run_esptool", fake_esptool)
    with pytest.raises(ValueError, match="refusing to guess"):
        backup_esp32.backup(usb_port, 115200, tmp_path)
    assert commands == ["flash-id"]


def test_write_commands_are_rejected(tmp_path: Path) -> None:
    with (tmp_path / "log").open("w") as log, pytest.raises(ValueError, match="not read-only"):
        backup_esp32.run_esptool("COM42", 115200, "write-flash", [], log)


def test_read_progress_reports_each_five_percent_once(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str],
) -> None:
    lines = [
        "Reading from 0x00000031 [] 5.0% 49/1000 bytes...\n",
        "Reading from 0x00000032 [] 5.0% 50/1000 bytes...\n",
        "\n",
        "Reading from 0x00000033 [] 5.1% 51/1000 bytes...\n",
        "Reading from 0x0000015d [] 34.9% 349/1000 bytes...\n",
        "Reading from 0x0000015e [] 35.0% 350/1000 bytes...\n",
        "Reading from 0x000003e8 [] 100.0% 1000/1000 bytes...\n",
        "Read complete.\n",
    ]
    process = MagicMock()
    process.__enter__.return_value = process
    process.stdout = iter(lines)
    process.wait.return_value = 0
    popen = MagicMock(return_value=process)
    monkeypatch.setattr(backup_esp32.subprocess, "Popen", popen)
    log = io.StringIO()

    backup_esp32.run_esptool("COM42", 115200, "read-flash", ["0", "ALL", "image.bin"], log)

    output = capsys.readouterr().out
    assert [int(value) for value in re.findall(r"Flash read: (\d+)%", output)] == list(range(5, 101, 5))
    assert "Flash read: 5% (50/1,000 bytes)" in output
    assert "Flash read: 35% (350/1,000 bytes)" in output
    assert "Reading from" not in output
    assert "Read complete." in output
    assert log.getvalue().endswith(output)
    assert popen.call_args.args[0][:4] == [backup_esp32.sys.executable, "-u", "-m", "esptool"]


def test_failed_read_keeps_progress_and_errors_without_claiming_completion(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str],
) -> None:
    process = MagicMock()
    process.__enter__.return_value = process
    process.stdout = iter([
        "Reading from 0x00000032 [] 5.0% 50/1000 bytes...\n",
        "A fatal error occurred: Serial data stream stopped.\n",
    ])
    process.wait.return_value = 2
    monkeypatch.setattr(backup_esp32.subprocess, "Popen", MagicMock(return_value=process))
    log = io.StringIO()

    with pytest.raises(RuntimeError, match="read-flash failed"):
        backup_esp32.run_esptool("COM42", 115200, "read-flash", ["0", "ALL", "image.bin"], log)

    output = capsys.readouterr().out
    assert "Flash read: 5%" in output
    assert "100%" not in output
    assert "Serial data stream stopped" in output
    assert "Serial data stream stopped" in log.getvalue()


def test_missing_port_does_not_discover_or_connect(monkeypatch: pytest.MonkeyPatch) -> None:
    def forbidden() -> None:
        pytest.fail("Port discovery should not run without an explicit action")

    monkeypatch.setattr(backup_esp32.list_ports, "comports", forbidden)
    with pytest.raises(SystemExit) as error:
        backup_esp32.main([])
    assert error.value.code == 2


def test_listing_does_not_open_ports(
    monkeypatch: pytest.MonkeyPatch, usb_port: ListPortInfo, capsys: pytest.CaptureFixture[str],
) -> None:
    monkeypatch.setattr(backup_esp32.list_ports, "comports", lambda: [usb_port])

    def forbidden(*args: object) -> None:
        pytest.fail("Listing must not start a backup")

    monkeypatch.setattr(backup_esp32, "backup", forbidden)
    assert backup_esp32.main(["--list"]) == 0
    assert json.loads(capsys.readouterr().out)[0]["device"] == "COM42"


@pytest.mark.parametrize("connected", [False, True])
def test_absent_or_non_usb_port_is_rejected(
    monkeypatch: pytest.MonkeyPatch, usb_port: ListPortInfo, connected: bool,
) -> None:
    usb_port.vid = None
    usb_port.pid = None
    monkeypatch.setattr(backup_esp32.list_ports, "comports", lambda: [usb_port] if connected else [])
    with pytest.raises(SystemExit) as error:
        backup_esp32.main(["--port", "COM42"])
    assert error.value.code == 2
