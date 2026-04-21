#!/usr/bin/env python3
"""
am32config -- Read and write AM32 ESC settings via direct serial connection.

Reverse-engineered from am32-firmware/am32-configurator (src/communication/direct.ts,
src/mcu.ts, src/eeprom.ts).  Source: https://github.com/am32-firmware/am32-configurator

Protocol: direct serial (NOT 4WIF/BLHeli FC passthrough).
The ESC must be powered with no valid PWM signal so it waits in config mode.
Baud: 19200 8N1.

Commands
---------
  scan              Scan COM ports for an AM32 ESC and identify the MCU
  probe <port>      Connect and show MCU variant / signature / firmware version
  read  <port>      Display all settings
  write <port> k=v  Write field=raw_integer pairs
  dump  <port> f    Save JSON snapshot
  load  <port> f    Restore JSON snapshot

Examples
---------
  python am32config.py scan
  python am32config.py probe COM4
  python am32config.py read  COM4
  python am32config.py write COM4 MOTOR_DIRECTION=1 TIMING_ADVANCE=26
  python am32config.py dump  COM4 backup.json
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass, field as dc_field

try:
    import serial
except ImportError:
    print("ERROR: pyserial not installed -- run: pip install pyserial")
    sys.exit(1)


# ---------------------------------------------------------------------------
# Direct protocol constants  (from direct.ts)
# ---------------------------------------------------------------------------

# Magic 21-byte init preamble sent to the ESC on direct serial connect.
# After 12 null bytes: \r B L H e l i \xF4 }
INIT_PREAMBLE = bytes([
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x0D, 0x42, 0x4C, 0x48, 0x65, 0x6C, 0x69, 0xF4, 0x7D,
])

# Command bytes (DIRECT_COMMANDS enum in direct.ts)
CMD_SET_ADDRESS    = 0xFF
CMD_SET_BUF_SIZE   = 0xFE
CMD_WRITE_FLASH    = 0x01
CMD_READ_FLASH     = 0x03
CMD_SEND_BUFFER    = 0x04
CMD_RESET          = 0x05

# Ack values (DIRECT_RESPONSES enum in direct.ts)
GOOD_ACK = 0x30
BAD_ACK  = 0xC1
BAD_CRC  = 0xC2

# Settings layout size in bytes  (Mcu.LAYOUT_SIZE = 0xB8 in mcu.ts)
LAYOUT_SIZE = 0xB8   # 184

# Chunk size used by readChunked (matches their default chunkSize=64)
CHUNK_SIZE = 64

# ---------------------------------------------------------------------------
# MCU variants  (from src/mcu.ts -- Mcu.variants)
# Keyed by the upper-cased hex signature string, e.g. "1F06"
# ---------------------------------------------------------------------------

@dataclass
class McuVariant:
    name:          str
    signature_hex: str
    page_size:     int
    flash_size:    int
    flash_offset:  int
    firmware_start: int
    eeprom_offset: int


MCU_VARIANTS: dict[str, McuVariant] = {
    "1F06": McuVariant(
        name="STM32F051",  signature_hex="0x1F06",
        page_size=1024, flash_size=65536,
        flash_offset=0x08000000, firmware_start=0x1000, eeprom_offset=0x7C00,
    ),
    "3506": McuVariant(
        name="ARM64K",     signature_hex="0x3506",
        page_size=1024, flash_size=65536,
        flash_offset=0x08000000, firmware_start=0x1000, eeprom_offset=0xF800,
    ),
    # G071 not yet in the official table -- add when signature is confirmed
}


def lookup_mcu(signature: int) -> McuVariant | None:
    """Look up an MCU variant by 16-bit signature integer."""
    key = format(signature, "X").upper()
    # Try exact match first, then zero-padded 4-char
    return MCU_VARIANTS.get(key) or MCU_VARIANTS.get(f"{key:0>4}")


# ---------------------------------------------------------------------------
# CRC-16/ARC  (makeCRC in direct.ts)
# poly=0x8005 reflected=0xA001, init=0, LSB-first
# Used for all command packets.
# ---------------------------------------------------------------------------

def make_crc(data: bytes | list[int]) -> int:
    crc = 0
    for byte in data:
        xb = byte & 0xFF
        for _ in range(8):
            if (xb & 0x01) ^ (crc & 0x0001):
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
            xb >>= 1
    return crc & 0xFFFF


# ---------------------------------------------------------------------------
# EEPROM layout  (from src/eeprom.ts -- EepromLayout)
# Offsets are relative to the start of the 184-byte settings buffer.
# ---------------------------------------------------------------------------

@dataclass
class Field:
    name:             str
    offset:           int
    size:             int   = 1
    min_eeprom_ver:   int | None = None
    max_eeprom_ver:   int | None = None
    choices:          dict  = dc_field(default_factory=dict)
    unit:             str   = ""
    scale:            float = 1.0


FIELDS: list[Field] = [
    Field("BOOT_BYTE",              0x00, 1),
    Field("LAYOUT_REVISION",        0x01, 1),
    Field("BOOT_LOADER_REVISION",   0x02, 1),
    Field("MAIN_REVISION",          0x03, 1),
    Field("SUB_REVISION",           0x04, 1),
    Field("MAX_RAMP",               0x05, 1, min_eeprom_ver=3),
    Field("MINIMUM_DUTY_CYCLE",     0x06, 1, min_eeprom_ver=3),
    Field("DISABLE_STICK_CALIBRATION", 0x07, 1, min_eeprom_ver=3,
          choices={0: "no", 1: "yes"}),
    Field("ABSOLUTE_VOLTAGE_CUTOFF",0x08, 1, min_eeprom_ver=3),
    Field("CURRENT_P",              0x09, 1, min_eeprom_ver=3),
    Field("CURRENT_I",              0x0A, 1, min_eeprom_ver=3),
    Field("CURRENT_D",              0x0B, 1, min_eeprom_ver=3),
    Field("ACTIVE_BRAKE_POWER",     0x0C, 1, min_eeprom_ver=3),
    Field("MOTOR_DIRECTION",        0x11, 1, choices={0: "forward", 1: "reverse"}),
    Field("BIDIRECTIONAL_MODE",     0x12, 1, choices={0: "off", 1: "on"}),
    Field("SINUSOIDAL_STARTUP",     0x13, 1, choices={0: "off", 1: "on"}),
    Field("COMPLEMENTARY_PWM",      0x14, 1, choices={0: "off", 1: "on"}),
    Field("VARIABLE_PWM_FREQUENCY", 0x15, 1,
          choices={0: "fixed", 1: "variable", 2: "auto"}),
    Field("STUCK_ROTOR_PROTECTION", 0x16, 1, choices={0: "off", 1: "on"}),
    Field("TIMING_ADVANCE",         0x17, 1, scale=0.9375, unit="deg"),
    Field("PWM_FREQUENCY",          0x18, 1, unit="kHz"),
    Field("STARTUP_POWER",          0x19, 1, unit="%"),
    Field("MOTOR_KV",               0x1A, 1, scale=40.0, unit="KV"),
    Field("MOTOR_POLES",            0x1B, 1),
    Field("BRAKE_ON_STOP",          0x1C, 1,
          choices={0: "off", 1: "drag", 2: "active"}),
    Field("STALL_PROTECTION",       0x1D, 1, choices={0: "off", 1: "on"}),
    Field("BEEP_VOLUME",            0x1E, 1, unit="0-11"),
    Field("INTERVAL_TELEMETRY",     0x1F, 1, choices={0: "off", 1: "on"}),
    Field("SERVO_LOW_THRESHOLD",    0x20, 1, unit="raw*2+750 us"),
    Field("SERVO_HIGH_THRESHOLD",   0x21, 1, unit="raw*2+1750 us"),
    Field("SERVO_NEUTRAL",          0x22, 1, unit="1374+raw us"),
    Field("SERVO_DEAD_BAND",        0x23, 1, unit="us per side"),
    Field("LOW_VOLTAGE_CUTOFF",     0x24, 1, choices={0: "off", 1: "on"}),
    Field("LOW_VOLTAGE_THRESHOLD",  0x25, 1, scale=0.1, unit="V (+2.5V)"),
    Field("RC_CAR_REVERSING",       0x26, 1, choices={0: "off", 1: "on"}),
    Field("USE_HALL_SENSORS",       0x27, 1, choices={0: "off", 1: "on"}),
    Field("SINE_MODE_RANGE",        0x28, 1, unit="% (5-25)"),
    Field("BRAKE_STRENGTH",         0x29, 1, unit="1-10"),
    Field("RUNNING_BRAKE_LEVEL",    0x2A, 1, unit="%"),
    Field("TEMPERATURE_LIMIT",      0x2B, 1, unit="degC"),
    Field("CURRENT_LIMIT",          0x2C, 1, scale=2.0, unit="A"),
    Field("SINE_MODE_POWER",        0x2D, 1, unit="1-10"),
    Field("ESC_PROTOCOL",           0x2E, 1,
          choices={0: "auto", 1: "dshot", 2: "servo", 3: "pwm",
                   4: "serial", 5: "bf_serial"}),
    Field("AUTO_ADVANCE",           0x2F, 1, choices={0: "off", 1: "on"}),
    Field("STARTUP_MELODY",         0x30, 128),   # bytes; not decoded as scalar
    Field("CAN_SETTINGS",           0xB0, 16),    # bytes; not decoded as scalar
]

_BY_NAME: dict[str, Field] = {f.name: f for f in FIELDS}


def _decode_field(f: Field, buf: bytes) -> str:
    if f.offset + f.size > len(buf):
        return "<out of range>"
    if f.size == 1:
        raw = buf[f.offset]
        if f.choices:
            return f.choices.get(raw, f"unknown({raw})")
        if f.scale != 1.0:
            return f"{round(raw * f.scale, 4)} {f.unit}".strip()
        return f"{raw} {f.unit}".strip()
    else:
        chunk = buf[f.offset: f.offset + f.size]
        return chunk.hex()


# ---------------------------------------------------------------------------
# Error type
# ---------------------------------------------------------------------------

class Am32Error(Exception):
    pass


# ---------------------------------------------------------------------------
# Low-level serial helpers
# ---------------------------------------------------------------------------

def _exchange(ser: serial.Serial, out: bytes, response_bytes: int,
              timeout: float = 2.0) -> bytes:
    """
    Send `out`, read back len(out) echo bytes + response_bytes data bytes.
    Returns only the response_bytes (echo is stripped).

    Uses a fixed-count blocking read (ser.read(total)) with `timeout` seconds.
    This matches the browser's approach and avoids quiet-timeout races where
    the ack byte arrives after a short silence gap.
    """
    ser.write(out)
    ser.flush()
    total = len(out) + response_bytes
    old_timeout = ser.timeout
    ser.timeout = timeout
    buf = bytearray(ser.read(total))
    ser.timeout = old_timeout

    if len(buf) < len(out):
        raise Am32Error(
            f"No echo: got {len(buf)} bytes, expected at least {len(out)} "
            f"(sent {len(out)}, want {response_bytes} back)"
        )
    if len(buf) < total:
        raise Am32Error(
            f"Short response: got {len(buf) - len(out)} response bytes, "
            f"expected {response_bytes} (total {len(buf)}/{total})"
        )
    return bytes(buf[len(out):])


# ---------------------------------------------------------------------------
# Direct protocol command builders
# ---------------------------------------------------------------------------

def _build_cmd(cmd: int, extra: list[int] | None = None) -> bytes:
    """Build a command buffer: [cmd, ...extra, crc_lo, crc_hi]."""
    buf = [cmd] + (extra or [])
    crc = make_crc(buf)
    buf.append(crc & 0xFF)
    buf.append((crc >> 8) & 0xFF)
    return bytes(buf)


def cmd_set_address(addr: int) -> bytes:
    return _build_cmd(CMD_SET_ADDRESS, [0x00, (addr >> 8) & 0xFF, addr & 0xFF])


def cmd_read_flash(n: int) -> bytes:
    return _build_cmd(CMD_READ_FLASH, [n & 0xFF])


def cmd_set_buf_size(n: int) -> bytes:
    n_enc = 0 if n == 256 else n
    return _build_cmd(CMD_SET_BUF_SIZE, [0x00, 0x00, n_enc])


def cmd_send_buffer(data: bytes) -> bytes:
    buf = list(data)
    crc = make_crc(buf)
    buf.append(crc & 0xFF)
    buf.append((crc >> 8) & 0xFF)
    return bytes(buf)


def cmd_write_flash() -> bytes:
    return _build_cmd(CMD_WRITE_FLASH, [0x01])


# ---------------------------------------------------------------------------
# ESC session
# ---------------------------------------------------------------------------

@dataclass
class EscInfo:
    """Parsed result of the direct-connect init handshake."""
    signature:     int
    mcu:           McuVariant | None
    input_byte:    int
    fw_major:      int | None = None
    fw_minor:      int | None = None
    file_name:     str | None = None
    mcu_type:      str | None = None
    eeprom_offset: int | None = None

    def mcu_name(self) -> str:
        if self.mcu:
            return self.mcu.name
        return f"UNKNOWN(0x{self.signature:04X})"

    def display(self) -> None:
        print(f"  Signature:     0x{self.signature:04X}")
        print(f"  MCU:           {self.mcu_name()}")
        if self.mcu:
            print(f"  EEPROM offset: 0x{self.mcu.eeprom_offset:04X}")
        if self.file_name:
            print(f"  Firmware file: {self.file_name}")
        if self.mcu_type:
            print(f"  MCU type:      {self.mcu_type}")
        if self.fw_major is not None:
            print(f"  FW version:    {self.fw_major}.{self.fw_minor}")
        print(f"  Input byte:    0x{self.input_byte:02X}")


class Am32Session:
    """
    Direct serial session to an AM32 ESC.

        with Am32Session("COM4") as s:
            info = s.handshake()
            info.display()
            buf = s.read_settings(info)
    """

    def __init__(self, port: str, baud: int = 19200, timeout: float = 2.0):
        self._port    = port
        self._baud    = baud
        self._timeout = timeout
        self._ser: serial.Serial | None = None

    def __enter__(self) -> "Am32Session":
        self._ser = serial.Serial(
            self._port, baudrate=self._baud,
            bytesize=8, parity=serial.PARITY_NONE, stopbits=1,
            timeout=self._timeout,
        )
        self._ser.reset_input_buffer()
        return self

    def __exit__(self, *_):
        if self._ser and self._ser.is_open:
            self._ser.close()

    # -- init ------------------------------------------------------------------

    def handshake(self, verbose: bool = False) -> EscInfo:
        """
        Send the 21-byte BLHeli preamble and parse the ESC's response.

        The ESC echoes the preamble then appends at least 9 response bytes:
          [0] [1] [2]        misc
          [3]                input byte (bootloader pin encoding)
          [4] [5]            signature  (MCU identifier)
          [6]                misc
          [7] [8]            checksum (CRC of response)

        Returns an EscInfo regardless of whether the MCU variant is known.
        Raises Am32Error if nothing comes back.
        """
        preamble = INIT_PREAMBLE

        self._ser.write(preamble)
        self._ser.flush()

        # Browser reads exactly len(preamble)+9 = 30 bytes: echo + 9 info bytes.
        # Use a fixed-count read with a generous timeout.
        TOTAL = len(preamble) + 9
        old_timeout = self._ser.timeout
        self._ser.timeout = self._timeout

        buf = bytearray(self._ser.read(TOTAL))

        self._ser.timeout = old_timeout

        if verbose:
            print(f"  Sent {len(preamble)} bytes, received {len(buf)} bytes total")
            print(f"  Raw hex: {buf.hex()}")
            echo    = buf[:len(preamble)]
            rest    = buf[len(preamble):]
            print(f"  Echo ({len(echo)}): {echo.hex()}")
            print(f"  Response ({len(rest)}): {rest.hex()}")
            if rest:
                print(f"  Response bytes: {[f'0x{b:02X}' for b in rest]}")

        if len(buf) < len(preamble) + 2:
            raise Am32Error(
                f"No response to init preamble ({len(buf)} bytes received). "
                "Is the ESC powered and in config mode (no PWM input)?"
            )

        info_buf = buf[len(preamble):]
        signature  = (info_buf[4] << 8) | info_buf[5] if len(info_buf) >= 6 else 0
        input_byte = info_buf[3] if len(info_buf) >= 4 else 0

        mcu = lookup_mcu(signature)

        info = EscInfo(
            signature=signature,
            mcu=mcu,
            input_byte=input_byte,
            eeprom_offset=mcu.eeprom_offset if mcu else None,
        )
        return info

    # -- address + read -------------------------------------------------------

    def set_address(self, addr: int) -> bool:
        """Set the ESC's read/write pointer.  Returns True on GOOD_ACK."""
        resp = _exchange(self._ser, cmd_set_address(addr), 1, self._timeout)
        return len(resp) > 0 and resp[0] == GOOD_ACK

    def read_flash(self, n: int) -> bytes:
        """Read n bytes from the current address pointer."""
        # Response = echo(4) + data(n) + crc(2) + ack(1); strip trailing 3 bytes.
        resp = _exchange(self._ser, cmd_read_flash(n), n + 3, self._timeout)
        return resp[:n]

    def read_at(self, addr: int, n: int) -> bytes:
        """Set address then read n bytes.  Raises Am32Error on bad ack."""
        if not self.set_address(addr):
            raise Am32Error(f"set_address(0x{addr:04X}) failed -- no GOOD_ACK")
        return self.read_flash(n)

    # -- file name preamble ---------------------------------------------------

    def read_file_name(self, eeprom_offset: int) -> str | None:
        """
        Read the 32-byte filename preamble at (eeprom_offset - 32).
        Returns the null-terminated ASCII string, or None if unreadable.
        """
        try:
            time.sleep(0.2)
            raw = self.read_at(eeprom_offset - 32, 32)
            null = raw.find(0x00)
            name = raw[:null].decode("ascii", errors="replace").strip()
            import re
            return name if re.match(r'^[A-Z0-9_]+$', name) else None
        except Am32Error:
            return None

    # -- settings read --------------------------------------------------------

    def read_settings(self, info: EscInfo) -> bytes:
        """
        Read LAYOUT_SIZE (184) bytes from the ESC EEPROM.

        Reads in chunks of CHUNK_SIZE (64) bytes, matching the configurator.
        eeprom_offset comes from info.mcu.eeprom_offset; if the MCU is unknown
        the user must supply an override via the --eeprom-offset flag.
        """
        if info.eeprom_offset is None:
            raise Am32Error(
                f"MCU signature 0x{info.signature:04X} is not in the known variants "
                "table.  Use --eeprom-offset 0x7E00 (G071) or 0x7C00 (F051) to override."
            )

        addr = info.eeprom_offset
        result = bytearray()
        remaining = LAYOUT_SIZE

        while remaining > 0:
            n = min(CHUNK_SIZE, remaining)
            if not self.set_address(addr + len(result)):
                raise Am32Error(
                    f"set_address(0x{addr + len(result):04X}) failed during read"
                )
            chunk = self.read_flash(n)
            result.extend(chunk)
            remaining -= n
            if remaining > 0:
                time.sleep(0.2)

        return bytes(result)

    # -- settings write -------------------------------------------------------

    def write_settings(self, info: EscInfo, buf: bytes) -> None:
        """Write buf to eeprom_offset in chunks."""
        if info.eeprom_offset is None:
            raise Am32Error("Unknown MCU -- cannot write without eeprom_offset")
        if len(buf) != LAYOUT_SIZE:
            raise Am32Error(f"Buffer must be {LAYOUT_SIZE} bytes, got {len(buf)}")

        addr = info.eeprom_offset
        offset = 0
        while offset < LAYOUT_SIZE:
            n = min(CHUNK_SIZE, LAYOUT_SIZE - offset)
            chunk = buf[offset: offset + n]

            if not self.set_address(addr + offset):
                raise Am32Error(f"set_address(0x{addr + offset:04X}) failed")
            time.sleep(0.2)

            # set buffer size — ESC echoes the command only, no ack byte
            _exchange(self._ser, cmd_set_buf_size(n), 0, self._timeout)

            # send buffer
            resp = _exchange(self._ser, cmd_send_buffer(chunk), 1, self._timeout)
            if not resp or resp[0] != GOOD_ACK:
                raise Am32Error("send_buffer failed")

            # write flash
            resp = _exchange(self._ser, cmd_write_flash(), 1, self._timeout)
            if not resp or resp[0] != GOOD_ACK:
                raise Am32Error("write_flash failed")

            offset += n
            if offset < LAYOUT_SIZE:
                time.sleep(0.2)


# ---------------------------------------------------------------------------
# Settings decode/encode
# ---------------------------------------------------------------------------

def decode_settings(buf: bytes, eeprom_ver: int | None = None) -> dict[str, str]:
    """Return {field_name: human_readable_value} for all scalar fields."""
    out = {}
    for f in FIELDS:
        if f.min_eeprom_ver and eeprom_ver and eeprom_ver < f.min_eeprom_ver:
            continue
        if f.max_eeprom_ver and eeprom_ver and eeprom_ver > f.max_eeprom_ver:
            continue
        out[f.name] = _decode_field(f, buf)
    return out


def apply_updates(buf: bytes, updates: dict[str, int]) -> bytes:
    out = bytearray(buf)
    for name, val in updates.items():
        f = _BY_NAME.get(name)
        if f is None:
            raise Am32Error(f"Unknown field {name!r}  (valid: {sorted(_BY_NAME)})")
        if f.size != 1:
            raise Am32Error(f"Field {name!r} is {f.size} bytes; only 1-byte fields can be set")
        out[f.offset] = int(val) & 0xFF
    return bytes(out)


# ---------------------------------------------------------------------------
# COM port scanner
# ---------------------------------------------------------------------------

def scan_ports(baud: int = 19200, timeout: float = 1.5) -> list[dict]:
    try:
        import serial.tools.list_ports as lp
        ports = sorted(lp.comports(), key=lambda p: p.device)
    except ImportError:
        print("  ERROR: pyserial not installed")
        return []

    if not ports:
        print("  No COM ports found.")
        return []

    print(f"  Scanning {len(ports)} port(s) ...")
    print()
    results = []
    for info in ports:
        port = info.device
        desc = (info.description or "").strip()
        print(f"  {port:<12} {desc:<40} ", end="", flush=True)
        entry = {"port": port, "description": desc, "ok": False,
                 "signature": None, "mcu": None, "detail": ""}
        try:
            with Am32Session(port, baud=baud, timeout=timeout) as s:
                esc = s.handshake(verbose=False)
            entry.update({
                "ok": True,
                "signature": f"0x{esc.signature:04X}",
                "mcu": esc.mcu_name(),
                "detail": esc.mcu_name(),
            })
            print(f"[OK]  {esc.mcu_name()}  sig=0x{esc.signature:04X}")
        except Am32Error as e:
            entry["detail"] = str(e)
            print(f"[--]  {str(e)[:55]}")
        except Exception as e:
            entry["detail"] = str(e)
            print(f"[ERR] {str(e)[:55]}")
        results.append(entry)

    print()
    ok = [r for r in results if r["ok"]]
    if ok:
        print(f"  Found {len(ok)} ESC(s):")
        for r in ok:
            print(f"    {r['port']}  {r['mcu']}  sig={r['signature']}")
        print()
        print(f"  To read settings:  python am32config.py read {ok[0]['port']}")
    else:
        print("  No AM32 ESC found.  Check port, power, and that no PWM signal is active.")
    return results


# ---------------------------------------------------------------------------
# CLI command handlers
# ---------------------------------------------------------------------------

def _get_session_and_info(args) -> tuple["Am32Session.__class__", EscInfo]:
    """Open session, handshake, optionally read filename.  Returns (session, info)."""
    s = Am32Session(args.port, baud=args.baud, timeout=args.timeout)
    s.__enter__()
    info = s.handshake()

    # Override eeprom_offset if user specified one
    if hasattr(args, "eeprom_offset") and args.eeprom_offset is not None:
        info.eeprom_offset = args.eeprom_offset

    # Try to read the filename preamble if we have an eeprom_offset
    if info.eeprom_offset is not None:
        name = s.read_file_name(info.eeprom_offset)
        if name:
            info.file_name = name
            # mcuType is the part after the last _
            info.mcu_type = name.split("_")[-1] if "_" in name else name

    return s, info


def cmd_probe(args) -> None:
    with Am32Session(args.port, baud=args.baud, timeout=args.timeout) as s:
        print(f"\n  Connecting to {args.port} at {args.baud} baud ...")
        info = s.handshake(verbose=True)

        if hasattr(args, "eeprom_offset") and args.eeprom_offset is not None:
            info.eeprom_offset = args.eeprom_offset

        if info.eeprom_offset is not None:
            name = s.read_file_name(info.eeprom_offset)
            if name:
                info.file_name = name
                info.mcu_type = name.split("_")[-1] if "_" in name else name

        print()
        info.display()

        if info.mcu is None:
            print()
            print("  NOTE: Signature not in known MCU table.")
            print("  If this is a G071-based ESC (e.g. REVVitRC), use:")
            print("  --eeprom-offset 0x7E00")
            print()
            print("  To add this MCU to the table, report the signature at:")
            print("  https://github.com/am32-firmware/am32-configurator/issues")


def cmd_scan(args) -> None:
    scan_ports(baud=args.baud, timeout=args.timeout)


def cmd_read(args) -> None:
    s, info = _get_session_and_info(args)
    try:
        print(f"\n  {args.port}  baud={args.baud}")
        info.display()

        buf = s.read_settings(info)
        eeprom_ver = buf[_BY_NAME["LAYOUT_REVISION"].offset] if len(buf) > 0x01 else None
        decoded = decode_settings(buf, eeprom_ver)
    finally:
        s.__exit__(None, None, None)

    if args.raw:
        print(f"\n  Raw {LAYOUT_SIZE} bytes from 0x{info.eeprom_offset:04X}:\n")
        for i in range(0, len(buf), 16):
            chunk   = buf[i:i+16]
            hex_str = " ".join(f"{b:02X}" for b in chunk)
            asc_str = "".join(chr(b) if 0x20 <= b < 0x7F else "." for b in chunk)
            print(f"  +{i:04X}  {hex_str:<47}  {asc_str}")
        return

    print(f"\n  {'Field':<28} {'Offset':>6}  {'Raw':>4}  Value")
    print(f"  {'-'*28}  {'-'*6}  {'-'*4}  {'-'*40}")
    for f in FIELDS:
        if f.name not in decoded:
            continue
        if f.size > 1:
            val = decoded[f.name]
            if len(val) > 40:
                val = val[:38] + ".."
            print(f"  {f.name:<28}  [0x{f.offset:02X}]  {'-':>4}   {val}")
        else:
            raw = buf[f.offset] if f.offset < len(buf) else -1
            print(f"  {f.name:<28}  [0x{f.offset:02X}]  {raw:4d}   {decoded[f.name]}")


def cmd_write(args) -> None:
    updates: dict[str, int] = {}
    for tok in args.assignments:
        if "=" not in tok:
            raise SystemExit(f"Error: expected FIELD=value, got {tok!r}")
        name, val = tok.split("=", 1)
        try:
            updates[name] = int(val, 0)
        except ValueError:
            raise SystemExit(f"Error: value must be an integer, got {val!r}")

    s, info = _get_session_and_info(args)
    try:
        buf = s.read_settings(info)
        new_buf = apply_updates(buf, updates)
        print(f"  Writing {len(updates)} field(s):")
        for name, val in updates.items():
            f = _BY_NAME[name]
            old = buf[f.offset]
            print(f"    {name}: {old} -> {val}")
        s.write_settings(info, new_buf)
        print("  [OK] Settings written.")
    finally:
        s.__exit__(None, None, None)


def cmd_dump(args) -> None:
    s, info = _get_session_and_info(args)
    try:
        buf = s.read_settings(info)
    finally:
        s.__exit__(None, None, None)
    eeprom_ver = buf[_BY_NAME["LAYOUT_REVISION"].offset] if len(buf) > 0x01 else None
    decoded = decode_settings(buf, eeprom_ver)
    out = {
        "port":           args.port,
        "signature":      f"0x{info.signature:04X}",
        "mcu":            info.mcu_name(),
        "eeprom_offset":  f"0x{info.eeprom_offset:04X}" if info.eeprom_offset else None,
        "file_name":      info.file_name,
        "mcu_type":       info.mcu_type,
        "raw_hex":        buf.hex(),
        "settings":       decoded,
    }
    with open(args.file, "w") as fh:
        json.dump(out, fh, indent=2)
    print(f"  [OK] {LAYOUT_SIZE} bytes / {len(decoded)} fields -> {args.file}")


def cmd_load(args) -> None:
    with open(args.file) as fh:
        snap = json.load(fh)

    s, info = _get_session_and_info(args)
    try:
        # Start from current ESC state, apply stored raw values
        live_buf = s.read_settings(info)
        raw_hex  = snap.get("raw_hex", "")

        if raw_hex and len(raw_hex) == LAYOUT_SIZE * 2:
            new_buf = bytes.fromhex(raw_hex)
        else:
            # Apply each scalar field from the snapshot
            updates = {}
            for name, val in snap.get("settings", {}).items():
                f = _BY_NAME.get(name)
                if f and f.size == 1:
                    try:
                        updates[name] = int(val) if isinstance(val, (int, float)) else int(val, 0)
                    except (ValueError, TypeError):
                        pass
            new_buf = apply_updates(live_buf, updates)

        print(f"  Writing snapshot from {args.file} ...")
        s.write_settings(info, new_buf)
        print("  [OK] Done.")
    finally:
        s.__exit__(None, None, None)


# ---------------------------------------------------------------------------
# Argument parser
# ---------------------------------------------------------------------------

def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="am32config",
        description="AM32 ESC configuration via direct serial (USB)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument("--baud",          type=int,   default=19200)
    p.add_argument("--timeout",       type=float, default=2.0,
                   help="Serial read timeout in seconds (default 2.0)")
    p.add_argument("--eeprom-offset", type=lambda x: int(x, 0), default=None,
                   help="Override EEPROM base address, e.g. 0x7E00 for G071")
    sub = p.add_subparsers(dest="cmd", required=True)

    # scan
    sc = sub.add_parser("scan", help="Scan all COM ports for an AM32 ESC")
    sc.set_defaults(func=cmd_scan)

    # probe
    pr = sub.add_parser("probe", help="Connect and show MCU variant / signature")
    pr.add_argument("port")
    pr.set_defaults(func=cmd_probe)

    # read
    r = sub.add_parser("read", help="Read and display all settings")
    r.add_argument("port")
    r.add_argument("--raw", action="store_true", help="Print raw hex instead of table")
    r.set_defaults(func=cmd_read)

    # write
    w = sub.add_parser("write", help="Write field=value pairs")
    w.add_argument("port")
    w.add_argument("assignments", nargs="+", metavar="FIELD=value")
    w.set_defaults(func=cmd_write)

    # dump
    d = sub.add_parser("dump", help="Save settings snapshot to JSON")
    d.add_argument("port")
    d.add_argument("file", help="Output .json path")
    d.set_defaults(func=cmd_dump)

    # load
    lo = sub.add_parser("load", help="Restore settings from JSON snapshot")
    lo.add_argument("port")
    lo.add_argument("file", help="Input .json produced by dump")
    lo.set_defaults(func=cmd_load)

    return p


def main() -> None:
    args = _build_parser().parse_args()
    try:
        args.func(args)
    except Am32Error as e:
        print(f"\n[FAIL] {e}")
        sys.exit(1)
    except FileNotFoundError as e:
        print(f"\n[FAIL] {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print()


if __name__ == "__main__":
    main()
