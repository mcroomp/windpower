#!/usr/bin/env python3
"""
validate.py -- MBDyn syntax validator for RAWES simulation files

Runs MBDyn inside the rawes-sim Docker container and checks for parse errors.
MBDyn parses the entire .mbd file before touching any sockets, so:
  - syntax errors  → MBDyn exits immediately with an error message  → FAIL
  - syntax OK      → MBDyn hangs waiting for socket connections     → PASS

Usage:
  python simulation/validate.py
  python simulation/validate.py --file simulation/mbdyn/rotor.mbd
  python simulation/validate.py --image my-custom-image
"""

import argparse
import os
import re
import shutil
import subprocess
import sys
import threading

SIM_DIR    = os.path.dirname(os.path.abspath(__file__))
MBD_FILE   = os.path.join(SIM_DIR, "mbdyn", "rotor.mbd")
IMAGE      = "rawes-sim"
# How long to wait before declaring syntax OK (MBDyn is hanging on sockets = good)
TIMEOUT    = 8.0

# MBDyn output lines that indicate a parse/runtime error
ERROR_PATTERNS = [
    re.compile(r"An error occurred", re.IGNORECASE),
    re.compile(r"aborting\.\.\.", re.IGNORECASE),
    re.compile(r"error return from", re.IGNORECASE),
    re.compile(r"unable to find", re.IGNORECASE),
    re.compile(r"\(unable to find", re.IGNORECASE),
    re.compile(r"expected at line", re.IGNORECASE),
    re.compile(r"unknown (keyword|var|element|joint|force)", re.IGNORECASE),
    re.compile(r"invalid (syntax|token|value)", re.IGNORECASE),
    re.compile(r"parse error", re.IGNORECASE),
]

# Lines to skip (MBDyn noise that isn't an error)
SKIP_PATTERNS = [
    re.compile(r"unknown var type <MBDYN_PREFIX>"),  # harmless install artifact
]


def is_error_line(line):
    if any(p.search(line) for p in SKIP_PATTERNS):
        return False
    return any(p.search(line) for p in ERROR_PATTERNS)


def run_validation(mbd_file, image):
    # Path inside container (simulation dir is mounted at /rawes/simulation)
    rel = os.path.relpath(mbd_file, SIM_DIR)
    container_path = "/rawes/simulation/" + rel.replace("\\", "/")

    cmd = [
        "docker", "run", "--rm",
        "-v", f"{SIM_DIR}:/rawes/simulation",
        image,
        "mbdyn", "-f", container_path, "-s",  # -s = silent (suppress banner)
    ]

    print(f"[INFO]  Validating: {mbd_file}")
    print(f"[INFO]  Container:  {image}")
    print(f"[CMD]   {' '.join(cmd)}\n")

    errors   = []
    output   = []
    timed_out = [False]
    proc     = None

    try:
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            encoding="utf-8",
            errors="replace",
            bufsize=1,
        )
    except FileNotFoundError:
        print("[ERROR] docker not found on PATH.", file=sys.stderr)
        sys.exit(1)

    def _kill_after_timeout():
        import time
        time.sleep(TIMEOUT)
        if proc.poll() is None:
            timed_out[0] = True
            proc.kill()

    timer = threading.Thread(target=_kill_after_timeout, daemon=True)
    timer.start()

    for line in proc.stdout:
        line = line.rstrip("\n")
        output.append(line)
        if is_error_line(line):
            errors.append(line)
            # print errors in real-time so you can see them as they appear
            print(f"  {line}")

    proc.wait()

    return errors, output, timed_out[0]


def main():
    parser = argparse.ArgumentParser(description="Validate MBDyn .mbd syntax via Docker.")
    parser.add_argument("--file",  default=MBD_FILE,  help="Path to .mbd file to validate")
    parser.add_argument("--image", default=IMAGE,     help="Docker image name")
    args = parser.parse_args()

    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")

    if not shutil.which("docker"):
        print("[ERROR] docker not found on PATH.", file=sys.stderr)
        sys.exit(1)

    if not os.path.isfile(args.file):
        print(f"[ERROR] File not found: {args.file}", file=sys.stderr)
        sys.exit(1)

    errors, output, timed_out = run_validation(args.file, args.image)

    print()
    if timed_out and not errors:
        print("=" * 60)
        print("  PASS  — MBDyn parsed the file successfully.")
        print(f"          (No errors in {TIMEOUT:.0f}s; MBDyn waiting for sockets is expected)")
        print("=" * 60)
        sys.exit(0)
    elif errors:
        print("=" * 60)
        print(f"  FAIL  — {len(errors)} error(s) found:")
        for e in errors:
            print(f"    {e}")
        print()
        # Print surrounding context from full output
        for i, line in enumerate(output):
            if is_error_line(line):
                start = max(0, i - 3)
                end   = min(len(output), i + 4)
                print("  Context:")
                for j in range(start, end):
                    marker = ">>>" if j == i else "   "
                    print(f"  {marker} {output[j]}")
                print()
        print("=" * 60)
        sys.exit(1)
    else:
        # MBDyn exited cleanly with no errors and no timeout — unusual
        print("=" * 60)
        print("  PASS  — MBDyn exited cleanly with no errors.")
        print("=" * 60)
        sys.exit(0)


if __name__ == "__main__":
    main()
