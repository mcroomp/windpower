"""Stream decoded records from LinkHub's canonical journal."""

from __future__ import annotations

import json
import os
import shutil
import subprocess
from collections.abc import Iterator, Sequence
from pathlib import Path


def _linkhub_binary() -> Path:
    configured = os.environ.get("LINKHUB_BIN")
    if configured:
        binary = Path(configured)
    elif discovered := shutil.which("linkhub"):
        binary = Path(discovered)
    else:
        name = "linkhub.exe" if os.name == "nt" else "linkhub"
        binary = Path(__file__).resolve().parents[1] / "linkhub" / "target" / "release" / name
    if not binary.is_file():
        raise FileNotFoundError(
            f"LinkHub binary not found at {binary}; build LinkHub first"
        )
    return binary


def cursor_sequence(cursor: str) -> int:
    prefix, separator, value = cursor.partition(":")
    if prefix != "v1" or not separator:
        raise ValueError(f"Unsupported LinkHub cursor: {cursor!r}")
    return int(value)


def iter_messages(
    journal: str | Path,
    *,
    after: int = 0,
    through: int | None = None,
    message_types: Sequence[str] = (),
    direction: str | None = None,
) -> Iterator[dict]:
    command = [
        str(_linkhub_binary()),
        "query",
        str(journal),
        "--after",
        str(after),
    ]
    if through is not None:
        command.extend(("--through", str(through)))
    command.extend(("show", "--json"))
    for message_type in message_types:
        command.extend(("--type", message_type))
    if direction is not None:
        command.extend(("--dir", direction))

    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        encoding="utf-8",
    )
    assert process.stdout is not None
    try:
        for line in process.stdout:
            if line.strip():
                yield json.loads(line)
    finally:
        process.stdout.close()
    stderr = process.stderr.read() if process.stderr is not None else ""
    return_code = process.wait()
    if return_code:
        raise RuntimeError(
            f"LinkHub journal query failed with exit code {return_code}: {stderr.strip()}"
        )
