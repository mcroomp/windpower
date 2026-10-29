"""Qt desktop application for live and recorded rotor marker analysis."""

from collections import deque
from dataclasses import replace
from pathlib import Path
import sys

import cv2
import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import QPointF, Qt, QTimer, Signal
from PySide6.QtGui import QCloseEvent, QImage, QMouseEvent, QPixmap
from PySide6.QtWidgets import (
    QApplication,
    QFileDialog,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from rotor_camera.capture import CaptureWorker
from rotor_camera.tracking import Frame, Settings, marker_hue


class VideoView(QLabel):
    selected = Signal(float, float, bool)

    def __init__(self) -> None:
        super().__init__("Open a camera or video to begin.")
        self.setMinimumSize(480, 300)
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._image_size = (0, 0)
        self._display_size = (0, 0)

    def show_frame(self, frame: Frame) -> None:
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        height, width = rgb.shape[:2]
        image = QImage(rgb.data, width, height, rgb.strides[0], QImage.Format.Format_RGB888).copy()
        pixmap = QPixmap.fromImage(image).scaled(
            self.size(), Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation
        )
        self._image_size = (width, height)
        self._display_size = (pixmap.width(), pixmap.height())
        self.setPixmap(pixmap)

    def image_position(self, point: QPointF) -> tuple[float, float] | None:
        width, height = self._display_size
        if width == 0 or height == 0:
            return None
        x = point.x() - (self.width() - width) / 2
        y = point.y() - (self.height() - height) / 2
        if 0 <= x < width and 0 <= y < height:
            return x * self._image_size[0] / width, y * self._image_size[1] / height
        return None

    def mousePressEvent(self, event: QMouseEvent) -> None:
        position = self.image_position(event.position())
        if position is not None and event.button() in (
            Qt.MouseButton.LeftButton, Qt.MouseButton.RightButton
        ):
            self.selected.emit(*position, event.button() == Qt.MouseButton.RightButton)
        super().mousePressEvent(event)


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Rotor Camera")
        self.resize(1200, 800)
        self.worker: CaptureWorker | None = None
        self.settings = Settings()
        self.frame: Frame | None = None
        self._terminal_status: str | None = None
        self._history: deque[tuple[float, float]] = deque(maxlen=600)
        self._path: deque[tuple[float, float]] = deque(maxlen=600)

        self.video = VideoView()
        self.video.selected.connect(self.select_point)
        self.camera_index = QSpinBox()
        self.camera_index.setRange(0, 20)
        self.camera_index.setPrefix("Camera ")
        self.camera_button = QPushButton("Open camera")
        self.file_button = QPushButton("Open video")
        self.stop_button = QPushButton("Stop")
        self.stop_button.setEnabled(False)
        reset = QPushButton("Reset selection")
        self.camera_button.clicked.connect(lambda: self.start_source(self.camera_index.value()))
        self.file_button.clicked.connect(self.open_video)
        self.stop_button.clicked.connect(self.stop)
        reset.clicked.connect(self.reset_selection)

        controls = QHBoxLayout()
        for widget in (self.camera_index, self.camera_button, self.file_button, self.stop_button, reset):
            controls.addWidget(widget)
        self.readout = QLabel("Azimuth: --    RPM: --")
        self.status = QLabel("Open a source. Left-click rotor centre; right-click the unique colour marker.")
        self.status.setWordWrap(True)
        instructions = QLabel(
            "Left-click: centre | Right-click: saturated marker | "
            "Signed RPM: positive counterclockwise in the image. "
            "Sampling aliasing cannot be ruled out from video alone."
        )
        instructions.setWordWrap(True)
        self.rpm_plot = pg.PlotWidget(title="Signed image-plane rotor speed")
        self.rpm_plot.setLabel("left", "RPM")
        self.rpm_plot.setLabel("bottom", "Source time", units="s")
        self.rpm_curve = self.rpm_plot.plot(pen="y", connect="finite")
        self.path_plot = pg.PlotWidget(title="Marker path relative to selected centre")
        self.path_plot.setLabel("bottom", "Image x", units="px")
        self.path_plot.setLabel("left", "Image y (up)", units="px")
        self.path_plot.setAspectLocked(True)
        self.path_curve = self.path_plot.plot(pen="c", connect="finite")
        plots = QHBoxLayout()
        plots.addWidget(self.rpm_plot)
        plots.addWidget(self.path_plot)
        layout = QVBoxLayout()
        layout.addLayout(controls)
        layout.addWidget(instructions)
        layout.addWidget(self.video, stretch=3)
        layout.addWidget(self.readout)
        layout.addWidget(self.status)
        layout.addLayout(plots, stretch=2)
        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(33)

    def open_video(self) -> None:
        filename, _ = QFileDialog.getOpenFileName(
            self, "Open rotor recording", "", "Video (*.mp4 *.avi *.mov *.mkv);;All files (*)"
        )
        if filename:
            self.start_source(Path(filename))

    def start_source(self, source: int | Path) -> None:
        if self.worker is not None:
            return
        self.reset_selection()
        self.frame = None
        self.video.clear()
        self._terminal_status = None
        self.status.setText(f"Opening {source}...")
        self.worker = CaptureWorker(source)
        self.worker.error.connect(self.source_error)
        self.worker.notice.connect(self.source_notice)
        self.worker.finished.connect(self.source_finished)
        self.camera_button.setEnabled(False)
        self.file_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.worker.start()

    def source_error(self, message: str) -> None:
        self.source_notice(message)
        QMessageBox.warning(self, "Video source error", message)

    def source_notice(self, message: str) -> None:
        self._terminal_status = message
        self.status.setText(message)

    def source_finished(self) -> None:
        self.refresh()
        if self.worker is not None:
            self.worker.deleteLater()
        self.worker = None
        self.camera_button.setEnabled(True)
        self.file_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.status.setText(self._terminal_status or "Stopped.")

    def stop(self) -> None:
        if self.worker is not None:
            self.worker.requestInterruption()
            self.status.setText("Stopping capture...")

    def reset_selection(self) -> None:
        self.settings = Settings()
        self._clear_plots()
        self.readout.setText("Azimuth: --    RPM: --")
        if self.worker is not None:
            self.worker.configure(self.settings)

    def _clear_plots(self) -> None:
        self._history.clear()
        self._path.clear()
        self.rpm_curve.setData([], [])
        self.path_curve.setData([], [])

    def select_point(self, x: float, y: float, marker: bool) -> None:
        if self.frame is None:
            return
        if marker:
            try:
                hue = marker_hue(self.frame, int(x), int(y))
            except ValueError as exc:
                self.status.setText(str(exc))
                return
            self.settings = replace(self.settings, hue=hue)
        else:
            self.settings = replace(self.settings, center=(x, y))
        self._clear_plots()
        if self.worker is not None:
            self.worker.configure(self.settings)

    def refresh(self) -> None:
        if self.worker is None:
            return
        packet = self.worker.take_latest()
        if packet is None:
            return
        self.frame = packet.frame
        display = self.frame.copy()
        measurement = packet.measurement
        if self.settings.center is not None:
            center = tuple(round(v) for v in self.settings.center)
            cv2.drawMarker(display, center, (255, 255, 255), cv2.MARKER_CROSS, 20, 2)
            if measurement.tip is not None:
                cv2.line(display, center, tuple(round(v) for v in measurement.tip), (0, 255, 255), 2)
        if measurement.tip is not None:
            cv2.circle(display, tuple(round(v) for v in measurement.tip), 8, (0, 255, 0), 2)
        self.video.show_frame(display)
        azimuth = "--" if measurement.azimuth_deg is None else f"{measurement.azimuth_deg:.1f} deg"
        rpm = "--" if measurement.rpm is None else f"{measurement.rpm:.1f}"
        self.readout.setText(f"Azimuth: {azimuth}    RPM: {rpm}    Analysis: {packet.processing_ms:.1f} ms")
        if self._terminal_status is None:
            self.status.setText(measurement.status)
        self._history.append((measurement.timestamp, measurement.rpm if measurement.rpm is not None else np.nan))
        if measurement.tip is not None and self.settings.center is not None:
            self._path.append((
                measurement.tip[0] - self.settings.center[0],
                self.settings.center[1] - measurement.tip[1],
            ))
        else:
            self._path.append((np.nan, np.nan))
        times, speeds = np.array(self._history).T
        xs, ys = np.array(self._path).T
        self.rpm_curve.setData(times, speeds)
        self.path_curve.setData(xs, ys)

    def closeEvent(self, event: QCloseEvent) -> None:
        if self.worker is not None:
            self.worker.requestInterruption()
            if not self.worker.wait(3000):
                self.status.setText("Camera read is still blocked; close again after capture stops.")
                event.ignore()
                return
        event.accept()


def main() -> int:
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    return app.exec()
