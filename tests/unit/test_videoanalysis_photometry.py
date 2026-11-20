import pytest

cv2 = pytest.importorskip("cv2", reason="Requires the videoanalysis extra")
import numpy as np  # noqa: E402 - skip before importing optional dependencies

from videoanalysis.__main__ import main  # noqa: E402 - skip before importing optional dependencies
from videoanalysis.photometry import brightness_diagnostics, normalize_illumination  # noqa: E402 - skip before importing optional dependencies


def test_normalization_is_spatial_exposure_invariant_and_does_not_modify_input():
    image = np.random.default_rng(42).integers(50, 180, (100, 160), dtype=np.uint8)
    cv2.rectangle(image, (50, 25), (100, 75), 20, -1)
    original = image.copy()
    exposed = (image.astype(float) * 0.8 + 20).astype(np.uint8)
    normalized = normalize_illumination(image)
    corrected = normalize_illumination(exposed)
    assert normalized.dtype == np.uint8 and normalized.shape == image.shape
    assert np.mean(np.abs(normalized.astype(float) - corrected)) < 1.5
    assert np.array_equal(image, original)
    normalize_illumination(exposed)
    assert np.array_equal(normalized, normalize_illumination(image))


@pytest.mark.parametrize("image", [
    np.zeros((10, 10, 3), dtype=np.uint8),
    np.zeros((10, 10), dtype=np.float32),
    np.zeros((0, 10), dtype=np.uint8),
])
def test_normalization_rejects_invalid_images(image: np.ndarray):
    with pytest.raises(ValueError, match="nonempty 8-bit grayscale"):
        normalize_illumination(image)


def test_brightness_diagnostics_measures_the_observed_frequency_and_reduction():
    times = np.arange(600) / 30
    wave = np.sin(2 * np.pi * 12.5 * times)
    metrics = brightness_diagnostics((140 + 20 * wave).tolist(), (128 + 2 * wave).tolist(), times.tolist())
    assert metrics["status"] == "ok"
    assert metrics["observed_peak_hz"] == pytest.approx(12.5, abs=0.05)
    assert metrics["tracking_brightness_amplitude_reduction_percent"] == pytest.approx(90, abs=0.1)


def test_brightness_diagnostics_explicitly_marks_unmeasurable_cases():
    assert brightness_diagnostics([1], [1], [0])["status"] == "too_short"
    assert brightness_diagnostics([1] * 30, [1] * 30, (np.arange(30) / 30).tolist())["status"] == "flat_reference"
    times = (np.arange(30) / 30).tolist()
    times[10] += 0.01
    assert brightness_diagnostics(list(range(30)), list(range(30)), times)["status"] == "irregular_sampling"
    assert brightness_diagnostics(list(range(30)), list(range(30)), list(range(30)))["status"] == "insufficient_sample_rate"


def test_cli_wires_flicker_compensation(
    monkeypatch: pytest.MonkeyPatch,
):
    calls = []

    def analyze(*args: object, **kwargs: object):
        calls.append((args, kwargs))
        return {"frames_processed": 1, "tracking_status_counts": {}}

    monkeypatch.setattr("videoanalysis.__main__.analyze", analyze)
    monkeypatch.setattr("sys.argv", [
        "videoanalysis", "example.mp4", "--compensate-flicker",
    ])
    assert main() == 0
    assert calls[0][0][-1] is True
