"""Linux thread scheduling trace for SITL diagnostics."""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import threading
import time
from pathlib import Path

# gdb Python run inside the traced process: report which thread owns the futex
# the traced thread is blocked on. In a futex syscall %rdi holds the futex
# address; for a glibc pthread_mutex_t, __owner (TID) follows __lock/__count.
_OWNER_PROBE = """
import gdb
tid = {tid}
names = {{t.ptid[1]: t.name for t in gdb.selected_inferior().threads()}}
for thread in gdb.selected_inferior().threads():
    if thread.ptid[1] == tid:
        thread.switch()
        futex = int(gdb.parse_and_eval("$rdi"))
        owner = int(gdb.parse_and_eval("*(int*)%d" % (futex + 8)))
        print("STALL_FUTEX 0x%x OWNER_TID %d OWNER_NAME %s" % (
            futex, owner, names.get(owner, "?")))
"""


class LinuxThreadTrace:
    def __init__(
        self,
        path: Path,
        *,
        process_name: str,
        thread_name: str,
        interval_s: float = 0.02,
        stall_snapshot_s: float | None = None,
        max_stall_snapshots: int = 5,
    ) -> None:
        self._path = path
        self._process_name = process_name
        self._thread_name = thread_name
        self._interval_s = interval_s
        self._stall_snapshot_s = stall_snapshot_s
        self._max_stall_snapshots = max_stall_snapshots
        self._snapshot_path = path.with_name(path.stem + "-stalls.txt")
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._stream = None
        self.pid: int | None = None
        self.tid: int | None = None
        self.samples = 0
        self.stall_snapshots = 0

    def start(self, timeout_s: float = 10.0) -> None:
        if os.name != "posix" or not Path("/proc").is_dir():
            raise RuntimeError("Linux thread tracing requires procfs")
        if self._thread is not None:
            raise RuntimeError("Linux thread trace is already running")
        self.pid, self.tid = self._find_thread(timeout_s)
        self._path.parent.mkdir(parents=True, exist_ok=True)
        self._stream = self._path.open("w", encoding="utf-8", buffering=1)
        self._write({
            "event": "start",
            "wall_time_ns": time.time_ns(),
            "monotonic_ns": time.monotonic_ns(),
            "pid": self.pid,
            "tid": self.tid,
            "process_name": self._process_name,
            "thread_name": self._thread_name,
            "interval_s": self._interval_s,
        })
        self._thread = threading.Thread(
            target=self._run,
            name="sitl-thread-trace",
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        if self._thread is None:
            return
        self._stop.set()
        self._thread.join(timeout=2.0)
        if self._thread.is_alive():
            raise RuntimeError("Linux thread trace did not stop")
        self._write({
            "event": "stop",
            "wall_time_ns": time.time_ns(),
            "monotonic_ns": time.monotonic_ns(),
            "samples": self.samples,
        })
        assert self._stream is not None
        self._stream.close()
        self._stream = None
        self._thread = None

    def _find_thread(self, timeout_s: float) -> tuple[int, int]:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            matches: list[tuple[int, int]] = []
            for process_path in Path("/proc").iterdir():
                if not process_path.name.isdigit():
                    continue
                try:
                    if (
                        process_path.joinpath("comm").read_text().strip()
                        != self._process_name
                    ):
                        continue
                    for task_path in process_path.joinpath("task").iterdir():
                        if (
                            task_path.joinpath("comm").read_text().strip()
                            == self._thread_name
                        ):
                            matches.append((
                                int(process_path.name),
                                int(task_path.name),
                            ))
                except (FileNotFoundError, PermissionError, ProcessLookupError):
                    continue
            if len(matches) == 1:
                return matches[0]
            if len(matches) > 1:
                raise RuntimeError(
                    f"Multiple {self._process_name}/{self._thread_name} "
                    f"threads found: {matches}"
                )
            time.sleep(0.05)
        raise RuntimeError(
            f"Timed out finding {self._process_name}/{self._thread_name}"
        )

    def _run(self) -> None:
        assert self.pid is not None
        assert self.tid is not None
        task_path = Path(f"/proc/{self.pid}/task/{self.tid}")
        previous_sched: tuple[int, int, int] | None = None
        futex_since: float | None = None
        snapshot_taken = False
        while not self._stop.is_set():
            sample_started = time.monotonic()
            try:
                sched = tuple(
                    int(value)
                    for value in task_path.joinpath("schedstat")
                    .read_text()
                    .split()
                )
                status = self._read_status(task_path / "status")
                stat = (task_path / "stat").read_text()
                stat_fields = stat[stat.rfind(")") + 2:].split()
                sample = {
                    "event": "sample",
                    "wall_time_ns": time.time_ns(),
                    "monotonic_ns": time.monotonic_ns(),
                    "cpu_runtime_ns": sched[0],
                    "runqueue_wait_ns": sched[1],
                    "timeslices": sched[2],
                    "state": stat_fields[0],
                    "processor": int(stat_fields[36]),
                    "wchan": (task_path / "wchan").read_text().strip(),
                    "voluntary_context_switches": int(
                        status["voluntary_ctxt_switches"]
                    ),
                    "involuntary_context_switches": int(
                        status["nonvoluntary_ctxt_switches"]
                    ),
                }
                if previous_sched is not None:
                    sample.update({
                        "cpu_delta_ns": sched[0] - previous_sched[0],
                        "runqueue_delta_ns": sched[1] - previous_sched[1],
                        "timeslices_delta": sched[2] - previous_sched[2],
                    })
                previous_sched = sched
                self._write(sample)
                self.samples += 1
                if "futex" in sample["wchan"]:
                    if futex_since is None:
                        futex_since = sample_started
                        snapshot_taken = False
                    if (
                        self._stall_snapshot_s is not None
                        and not snapshot_taken
                        and self.stall_snapshots < self._max_stall_snapshots
                        and sample_started - futex_since >= self._stall_snapshot_s
                    ):
                        snapshot_taken = True
                        self._snapshot_stall(sample_started - futex_since)
                else:
                    futex_since = None
            except (FileNotFoundError, ProcessLookupError) as error:
                self._write({
                    "event": "thread-exited",
                    "wall_time_ns": time.time_ns(),
                    "monotonic_ns": time.monotonic_ns(),
                    "error": str(error),
                })
                return
            remaining = self._interval_s - (time.monotonic() - sample_started)
            self._stop.wait(max(0.0, remaining))

    def _snapshot_stall(self, blocked_s: float) -> None:
        """Attach gdb once and record every thread's stack plus the futex owner."""
        self.stall_snapshots += 1
        gdb = shutil.which("gdb")
        if gdb is None:
            raise RuntimeError("stall snapshots require gdb in the SITL image")
        started_ns = time.monotonic_ns()
        probe_path = self._snapshot_path.with_suffix(".gdb.py")
        probe_path.write_text(_OWNER_PROBE.format(tid=self.tid), encoding="utf-8")
        result = subprocess.run(
            [
                gdb, "-p", str(self.pid), "-batch", "-nx",
                "-ex", "set pagination off",
                "-x", str(probe_path),
                "-ex", "info threads",
                "-ex", "thread apply all bt 30",
            ],
            capture_output=True,
            text=True,
            timeout=60,
            check=False,
        )
        probe_path.unlink()
        owner = next(
            (line for line in result.stdout.splitlines()
             if line.startswith("STALL_FUTEX")),
            None,
        )
        self._write({
            "event": "stall-snapshot",
            "wall_time_ns": time.time_ns(),
            "monotonic_ns": started_ns,
            "blocked_s": round(blocked_s, 3),
            "gdb_returncode": result.returncode,
            "owner": owner,
            "snapshot_file": self._snapshot_path.name,
        })
        with self._snapshot_path.open("a", encoding="utf-8") as stream:
            stream.write(
                f"===== stall snapshot {self.stall_snapshots}: traced thread "
                f"blocked {blocked_s:.2f} s, monotonic_ns={started_ns} =====\n"
            )
            stream.write(result.stdout)
            if result.stderr:
                stream.write("----- gdb stderr -----\n" + result.stderr)
            stream.write("\n")

    @staticmethod
    def _read_status(path: Path) -> dict[str, str]:
        values = {}
        for line in path.read_text().splitlines():
            key, separator, value = line.partition(":")
            if separator:
                values[key] = value.strip()
        return values

    def _write(self, record: dict) -> None:
        assert self._stream is not None
        self._stream.write(json.dumps(record, separators=(",", ":")) + "\n")
