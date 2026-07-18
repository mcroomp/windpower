#!/usr/bin/env python3
"""
build.py -- RAWES simulation Docker build + run script

Usage:
  python simulation/build.py                  # build base image
  python simulation/build.py --ardupilot      # build with ArduPilot SITL (~30 min)
  python simulation/build.py --run            # build then start container shell
  python simulation/build.py --no-build --run # start container shell without rebuilding
"""

import argparse
import os
import shutil
import subprocess
import sys
import threading
import time

# Force UTF-8 output so Docker's Unicode progress chars don't crash on Windows
sys.stdout.reconfigure(encoding='utf-8', errors='replace')
sys.stderr.reconfigure(encoding='utf-8', errors='replace')

IMAGE = "rawes-sim"
SIM_DIR = os.path.dirname(os.path.abspath(__file__))

# Warn if no output for this many seconds
STUCK_WARN_SECS = 60
# Print a heartbeat every this many seconds even when output is flowing
HEARTBEAT_SECS = 30


def stream_with_monitor(cmd):
    """Run cmd, stream output, and warn if it goes silent (likely stuck)."""
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        encoding='utf-8',
        errors='replace',
        bufsize=1,
    )

    last_output_time = [time.time()]
    last_heartbeat_time = [time.time()]
    done = [False]

    def monitor():
        while not done[0]:
            time.sleep(5)
            silent_for = time.time() - last_output_time[0]
            since_heartbeat = time.time() - last_heartbeat_time[0]
            if silent_for >= STUCK_WARN_SECS:
                mins = int(silent_for // 60)
                secs = int(silent_for % 60)
                print(
                    f"\n[MONITOR] No output for {mins}m{secs:02d}s -- build may be stuck!",
                    flush=True,
                )
                last_heartbeat_time[0] = time.time()
            elif since_heartbeat >= HEARTBEAT_SECS:
                elapsed = int(time.time() - start_time)
                print(
                    f"[MONITOR] Still running... {elapsed//60}m{elapsed%60:02d}s elapsed",
                    flush=True,
                )
                last_heartbeat_time[0] = time.time()

    start_time = time.time()
    t = threading.Thread(target=monitor, daemon=True)
    t.start()

    for line in proc.stdout:
        print(line, end="", flush=True)
        last_output_time[0] = time.time()
        last_heartbeat_time[0] = time.time()

    done[0] = True
    proc.wait()
    return proc.returncode


def main():
    parser = argparse.ArgumentParser(description="Build/run the RAWES simulation Docker image.")
    parser.add_argument("--ardupilot", action="store_true", help="Include ArduPilot SITL (~30 min build)")
    parser.add_argument("--run", action="store_true", help="Start a container shell after building")
    parser.add_argument("--no-build", action="store_true", help="Skip build, only run")
    args = parser.parse_args()

    if not shutil.which("docker"):
        print("[ERROR] docker not found on PATH.", file=sys.stderr)
        sys.exit(1)

    if not args.no_build:
        build_cmd = ["docker", "build", SIM_DIR, "-t", IMAGE, "--progress=plain"]
        if args.ardupilot:
            build_cmd += ["--build-arg", "INSTALL_ARDUPILOT=true"]
            print("[WARN]  ArduPilot SITL build enabled -- expect ~30 minutes.", flush=True)
        print(f"[INFO]  Building image '{IMAGE}' from {SIM_DIR} ...", flush=True)
        # Remove existing image so BuildKit doesn't fail with "already exists"
        subprocess.run(["docker", "rmi", "-f", IMAGE],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print(f"[CMD]   {' '.join(build_cmd)}\n", flush=True)

        rc = stream_with_monitor(build_cmd)
        if rc != 0:
            print(f"\n[ERROR] Docker build failed (exit {rc}).", file=sys.stderr)
            sys.exit(rc)
        print(f"\n[INFO]  Build complete: {IMAGE}", flush=True)

    if args.run:
        run_cmd = [
            "docker", "run", "--rm", "-it",
            "-v", f"{SIM_DIR}:/rawes/simulation",
            "-p", "5760:5760",
            "-p", "5761:5761",
            "-p", "5762:5762",
            IMAGE,
        ]
        print(f"\n[INFO]  Starting container ...", flush=True)
        print(f"[CMD]   {' '.join(run_cmd)}\n", flush=True)
        subprocess.run(run_cmd)
    elif not args.no_build:
        print(f"\n[INFO]  To start a shell in the container:")
        print(f"        docker run --rm -it -p 5760:5760 -p 5761:5761 -p 5762:5762 -v \"{SIM_DIR}:/rawes/simulation\" {IMAGE}")
        print(f"\n[INFO]  Or use this script:")
        print(f"        python simulation/build.py --no-build --run\n")


if __name__ == "__main__":
    main()
