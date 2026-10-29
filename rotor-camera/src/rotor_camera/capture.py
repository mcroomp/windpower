"""Bounded latest-frame handoff: the GUI never queues camera frames."""

from dataclasses import dataclass
import math
from pathlib import Path
import sys
from threading import Lock
import time

import cv2
from PySide6.QtCore import QThread, Signal

from rotor_camera.tracking import Frame, MarkerTracker, Measurement, Settings


@dataclass(frozen=True)
class Packet:
    frame: Frame
    measurement: Measurement
    processing_ms: float


class CaptureWorker(QThread):
    error = Signal(str)
    notice = Signal(str)

    def __init__(self, source: int | Path) -> None:
        super().__init__()
        self.source = source
        self._lock = Lock()
        self._settings = Settings()
        self._packet: Packet | None = None

    def configure(self, settings: Settings) -> None:
        with self._lock:
            self._settings = settings

    def take_latest(self) -> Packet | None:
        with self._lock:
            packet, self._packet = self._packet, None
        return packet

    def run(self) -> None:
        capture = cv2.VideoCapture()
        recorded = isinstance(self.source, Path)
        backend = cv2.CAP_DSHOW if sys.platform == "win32" and not recorded else cv2.CAP_ANY
        try:
            if not capture.open(str(self.source) if recorded else self.source, backend):
                backend_name = "DirectShow" if backend == cv2.CAP_DSHOW else "automatic"
                self.error.emit(f"Cannot open video source: {self.source} (backend: {backend_name})")
                return
            fps = capture.get(cv2.CAP_PROP_FPS)
            if recorded and (not math.isfinite(fps) or fps <= 0):
                self.error.emit("Video has no valid frame rate; reliable playback timing is unavailable.")
                return
            tracker = MarkerTracker()
            start = time.monotonic()
            index = 0
            while not self.isInterruptionRequested():
                if recorded:
                    deadline = start + index / fps
                    while time.monotonic() < deadline and not self.isInterruptionRequested():
                        self.msleep(2)
                    if self.isInterruptionRequested():
                        break
                ok, frame = capture.read()
                received = time.monotonic()
                if not ok:
                    if recorded:
                        self.notice.emit("Playback ended (EOF or decoder stopped).")
                    else:
                        self.error.emit("Camera stopped delivering frames.")
                    break
                timestamp = index / fps if recorded else received - start
                with self._lock:
                    settings = self._settings
                measurement = tracker.analyze(frame, timestamp, settings)
                packet = Packet(frame, measurement, (time.monotonic() - received) * 1000)
                with self._lock:
                    self._packet = packet
                index += 1
        except (cv2.error, ValueError, OSError) as exc:
            self.error.emit(f"Capture/analysis failed: {exc}")
        finally:
            capture.release()
