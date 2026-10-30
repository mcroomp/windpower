# Rotor Camera

Independent, local-only Python desktop application for tracking one uniquely
coloured rotor marker. Managed with uv; it has its own environment and lockfile,
does not import RAWES packages, and is not a member of the root project.
Requires Python 3.12+.

## Run

From this directory:

```powershell
uv sync --dev
uv run rotor-camera
```

Or from the repository root:

```powershell
uv sync --project .\rotor-camera --dev
uv run --project .\rotor-camera rotor-camera
```

Run `uv sync` explicitly after dependency changes: this workstation disables
automatic uv synchronization with `UV_NO_SYNC=1`.
The package index matches the parent repository's Microsoft Python feed because
direct package downloads fail TLS negotiation on this workstation.

## Measure

1. Use **Open camera** with the correct device index, or **Open video**.
   Windows cameras use OpenCV's DirectShow backend explicitly: automatic backend
   selection stalled when opening the Logitech c922 on this workstation.
   Startup can take several seconds. Other platforms and video files keep
   automatic backend selection.
2. Left-click the centre of rotation.
3. Right-click a bright, saturated, uniquely coloured blade/tip marker.
   The initial detector selects that hue within +/-12 OpenCV hue units, with
   saturation/value at least 60/255 and contour area at least 12 square pixels.
   Use a contrasting background without other large objects of that colour.
4. Inspect the tracked-marker overlay, signed RPM, azimuth and pixel trajectory.
   Use **Reset selection** to start over. Reopen a recording to replay it.

Capture and analysis run in a worker thread. A single latest-frame slot prevents
GUI frame backlog. Plots keep the latest 600 displayed samples; they are not an
archival measurement log. There is no recording/export or hardware-control path.

Azimuth is zero to image-right and increases counterclockwise (image-up is
positive). RPM is signed in that same image-plane convention, estimated by a
linear fit to unwrapped marker phase over approximately 0.3 seconds. Tracking
loss, ambiguous similarly sized colour blobs, configuration changes, time gaps
over 0.5 seconds, and phase steps of 120 degrees or more reset the estimator.
Three samples are required before RPM is displayed. The detector chooses the
largest qualifying blob; it cannot establish physical blade identity.

These are **image-plane measurements**, not calibrated blade flapping,
feathering, physical displacement, or 3D rotor attitude. Oblique viewing can
distort phase and apparent instantaneous speed. Camera axes are not RAWES
body/NED axes; do not feed these values directly into flight control.

## Camera and timing limitations

- Prefer a global shutter, short exposure and a fixed camera with the complete
  rotor visible. Place a marker only using appropriate rotor hardware procedures.
- A unique marker avoids four-blade permutation ambiguity, but **does not remove
  temporal aliasing**. More than two samples per revolution is a theoretical
  minimum for phase unwrapping; use substantially more for useful measurements.
  Phase checks cannot detect every aliased speed or reversed apparent direction.
  Validate RPM against an independent tachometer.
- Live timing uses host monotonic timestamps after frame reads, not sensor
  exposure timestamps. Camera/driver buffering and dropped frames affect RPM.
- Recordings use frame index divided by reported FPS and play at that rate.
  Variable-frame-rate files, bad FPS metadata, and missing frames are not
  correctly timestamped by this initial implementation. Use constant-rate input.
  A decoder stopping cannot always be distinguished from normal end-of-file.
- **Analysis ms** measures tracking work, not capture-to-display latency.
- Stop requests are cooperative. Some camera drivers block in reads; closing
  waits up to three seconds, then keeps the window open with an explicit warning
  instead of destroying a running capture thread.

## Tests

From this directory:

```powershell
uv run python -m pytest
```

Tests cover synthetic positive/negative RPM, hue wrap, loss/ambiguity,
discontinuities, real generated AVI playback, timestamps, error reporting,
letterboxed click mapping and the offscreen desktop lifecycle. Camera hardware
and real-rotor measurement accuracy require separate validation.
