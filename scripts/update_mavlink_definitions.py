"""Vendor ArduPilot's MAVLink definitions into linkhub/dialect/definitions.

LinkHub generates its MAVLink dialect from the XML that the simulated ArduPilot
release builds from: ArduPilot pins ``ArduPilot/mavlink`` as its
``modules/mavlink`` submodule, so the commit that ``ARDUPILOT_TAG`` (in
simulation/Dockerfile) points at fixes the definitions. This resolves that
commit through the GitHub API, downloads ``ardupilotmega.xml`` and everything it
includes at exactly that commit, and records the tag, the commit and a SHA-256
per file in linkhub/dialect/ARDUPILOT_VERSION.

    uv run python scripts/update_mavlink_definitions.py           # tag from the Dockerfile
    uv run python scripts/update_mavlink_definitions.py --tag Copter-4.8.0
    uv run python scripts/update_mavlink_definitions.py --check   # verify against GitHub

Set GITHUB_TOKEN to avoid the unauthenticated API rate limit.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import sys
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DIALECT = ROOT / "linkhub" / "dialect"
DEFINITIONS = DIALECT / "definitions"
VERSION_FILE = DIALECT / "ARDUPILOT_VERSION"
DOCKERFILE = ROOT / "simulation" / "Dockerfile"
ROOT_DEFINITION = "ardupilotmega.xml"

API = "https://api.github.com/repos/ArduPilot/ardupilot/contents/modules/mavlink"
RAW = "https://raw.githubusercontent.com/ArduPilot/mavlink/{commit}/message_definitions/v1.0/{name}"


def dockerfile_tag() -> str:
    match = re.search(r"^ARG ARDUPILOT_TAG=(\S+)", DOCKERFILE.read_text(), re.MULTILINE)
    if not match:
        sys.exit(f"ARDUPILOT_TAG not found in {DOCKERFILE}")
    return match.group(1)


def fetch(url: str) -> bytes:
    request = urllib.request.Request(url, headers={"User-Agent": "rawes-update-mavlink"})
    token = os.environ.get("GITHUB_TOKEN")
    if token and url.startswith("https://api.github.com"):
        request.add_header("Authorization", f"Bearer {token}")
    with urllib.request.urlopen(request, timeout=60) as response:
        return response.read()


def mavlink_commit(tag: str) -> str:
    """The ArduPilot/mavlink commit that ArduPilot `tag` pins as modules/mavlink."""
    entry = json.loads(fetch(f"{API}?ref={tag}"))
    if entry.get("type") != "submodule":
        sys.exit(f"modules/mavlink at {tag} is not a submodule: {entry.get('type')}")
    return entry["sha"]


def download(commit: str) -> dict[str, bytes]:
    """ardupilotmega.xml plus every file it (transitively) includes."""
    files: dict[str, bytes] = {}
    pending = [ROOT_DEFINITION]
    while pending:
        name = pending.pop()
        if name in files:
            continue
        files[name] = fetch(RAW.format(commit=commit, name=name))
        for include in re.findall(rb"<include>([^<]+)</include>", files[name]):
            pending.append(include.decode().strip())
    return files


def parse_version() -> dict[str, str]:
    values: dict[str, str] = {}
    for line in VERSION_FILE.read_text().splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            values[key.strip()] = value.strip()
    return values


def write(tag: str, commit: str, files: dict[str, bytes]) -> None:
    DEFINITIONS.mkdir(parents=True, exist_ok=True)
    for stale in DEFINITIONS.glob("*.xml"):
        if stale.name not in files:
            stale.unlink()
    for name, content in files.items():
        (DEFINITIONS / name).write_bytes(content)
    lines = [f"tag={tag}", f"mavlink_commit={commit}"]
    lines += [
        f"sha256.{name}={hashlib.sha256(content).hexdigest()}"
        for name, content in sorted(files.items())
    ]
    VERSION_FILE.write_text("\n".join(lines) + "\n")
    print(f"vendored {len(files)} files from ArduPilot/mavlink {commit[:12]} ({tag})")


def check() -> int:
    recorded = parse_version()
    commit = recorded["mavlink_commit"]
    problems = []
    if recorded["tag"] != dockerfile_tag():
        problems.append(f"ARDUPILOT_VERSION tag {recorded['tag']} != Dockerfile {dockerfile_tag()}")
    if mavlink_commit(recorded["tag"]) != commit:
        problems.append(f"ArduPilot {recorded['tag']} pins a different mavlink commit than {commit}")
    for name, content in download(commit).items():
        local = DEFINITIONS / name
        if not local.is_file() or local.read_bytes() != content:
            problems.append(f"{name} differs from ArduPilot/mavlink {commit[:12]}")
    for problem in problems:
        print("MISMATCH:", problem)
    print("definitions match ArduPilot" if not problems else "definitions are out of sync")
    return 1 if problems else 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--tag", help="ArduPilot tag (default: ARDUPILOT_TAG in simulation/Dockerfile)")
    parser.add_argument("--check", action="store_true", help="verify the vendored files against GitHub")
    args = parser.parse_args()
    if args.check:
        return check()
    tag = args.tag or dockerfile_tag()
    commit = mavlink_commit(tag)
    write(tag, commit, download(commit))
    return 0


if __name__ == "__main__":
    sys.exit(main())
