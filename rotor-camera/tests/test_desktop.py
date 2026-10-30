import os
from pathlib import Path
import time
from types import SimpleNamespace

os.environ["QT_QPA_PLATFORM"] = "offscreen"

import cv2
import numpy as np
import pytest
from PySide6.QtCore import QPointF
from PySide6.QtWidgets import QApplication

from rotor_camera.app import MainWindow, VideoView
from rotor_camera import capture as capture_module
from rotor_camera.capture import CaptureWorker
from rotor_camera.tracking import Settings


@pytest.fixture(scope="module")
def app():
    instance = QApplication.instance() or QApplication([])
    yield instance


@pytest.fixture
def video(tmp_path):
    path = tmp_path / "rotor.avi"
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"MJPG"), 30, (200, 200))
    assert writer.isOpened(), "MJPG encoder must be available for the integration test"
    try:
        for i in range(12):
            frame = np.zeros((200, 200, 3), dtype=np.uint8)
            angle = i * 0.1
            point = (round(100 + 70 * np.cos(angle)), round(100 - 70 * np.sin(angle)))
            cv2.circle(frame, point, 6, (0, 0, 255), -1)
            writer.write(frame)
    finally:
        writer.release()
    return path


def test_recorded_capture_timestamps_and_rpm(app, video):
    worker = CaptureWorker(video)
    worker.configure(Settings(center=(100, 100), hue=0))
    errors = []
    notices = []
    worker.error.connect(errors.append)
    worker.notice.connect(notices.append)
    worker.start()
    try:
        assert worker.wait(5000)
        app.processEvents()
        packet = worker.take_latest()
        assert not errors
        assert notices and "Playback ended" in notices[0]
        assert packet is not None
        assert packet.measurement.timestamp == pytest.approx(11 / 30)
        assert packet.measurement.rpm == pytest.approx(0.1 * 30 * 60 / (2 * np.pi), abs=1)
        assert worker.take_latest() is None
    finally:
        worker.requestInterruption()
        worker.wait(5000)


def test_invalid_source_reports_error(app, tmp_path):
    worker = CaptureWorker(tmp_path / "missing.avi")
    errors = []
    worker.error.connect(errors.append)
    worker.start()
    try:
        assert worker.wait(5000)
        app.processEvents()
        assert errors and "Cannot open" in errors[0]
    finally:
        worker.requestInterruption()
        worker.wait(5000)


@pytest.mark.parametrize("platform", ["win32", "linux"])
@pytest.mark.parametrize("recorded", [False, True])
def test_capture_backend_selection(app, monkeypatch, tmp_path, platform, recorded):
    class FakeCapture:
        def __init__(self):
            self.open_args = None
            self.released = False

        def open(self, source, backend):
            self.open_args = (source, backend)
            return True

        def get(self, prop):
            return 30.0

        def read(self):
            return False, None

        def release(self):
            self.released = True

    capture = FakeCapture()
    monkeypatch.setattr(capture_module, "sys", SimpleNamespace(platform=platform))
    monkeypatch.setattr(capture_module.cv2, "VideoCapture", lambda: capture)
    source = tmp_path / "input.avi" if recorded else 0
    worker = CaptureWorker(source)
    errors = []
    notices = []
    worker.error.connect(errors.append)
    worker.notice.connect(notices.append)
    worker.run()
    expected_backend = cv2.CAP_DSHOW if platform == "win32" and not recorded else cv2.CAP_ANY
    assert capture.open_args == (str(source) if recorded else source, expected_backend)
    assert capture.released
    if recorded:
        assert not errors
        assert notices and "Playback ended" in notices[0]
    else:
        assert errors == ["Camera stopped delivering frames."]
        assert not notices


def test_video_coordinate_mapping(app):
    view = VideoView()
    view.resize(600, 300)
    view.show_frame(np.zeros((200, 200, 3), dtype=np.uint8))
    assert view.image_position(QPointF(300, 150)) == pytest.approx((100, 100))
    assert view.image_position(QPointF(0, 0)) is None
    view.close()


def test_desktop_video_lifecycle(app, video):
    window = MainWindow()
    window.show()
    window.start_source(Path(video))
    deadline = time.monotonic() + 5
    try:
        while window.worker is not None and time.monotonic() < deadline:
            app.processEvents()
            if window.frame is not None and window.settings.hue is None:
                window.select_point(100, 100, False)
                hsv = cv2.cvtColor(window.frame, cv2.COLOR_BGR2HSV)
                ys, xs = np.where((hsv[:, :, 1] > 200) & (hsv[:, :, 2] > 200))
                assert len(xs) > 0
                window.select_point(float(xs[len(xs) // 2]), float(ys[len(ys) // 2]), True)
            time.sleep(0.01)
        assert window.worker is None
        assert window.frame is not None
        assert window.settings.hue is not None
        assert "Playback ended" in window.status.text()
        assert window.camera_button.isEnabled()
        assert not window.stop_button.isEnabled()
        assert window.rpm_curve.getData()[0] is not None
    finally:
        window.close()
        app.processEvents()
