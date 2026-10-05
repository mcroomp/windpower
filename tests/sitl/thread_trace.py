"""Linux thread scheduling trace for SITL diagnostics."""

from __future__ import annotations

import json
import os
import threading
import time
from pathlib import Path


class LinuxThreadTrace:
    def __init__(
        self,
        path: Path,
        *,
        process_name: str,
        thread_name: str,
        interval_s: float = 0.02,
    ) -> None:
        self._path = path
        self._process_name = process_name
        self._thread_name = thread_name
        self._interval_s = interval_s
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._stream = None
        self.pid: int | None = None
        self.tid: int | None = None
        self.samples = 0

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
