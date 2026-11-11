# Rotor-camera video analysis

Local Python/OpenCV tool for tracking the four physical corners of blade tape
relative to a rigid hub reference. Measures the projected quadrilateral, not a
fixed rectangle. Works offline; no cloud service or hardware connection.

## Setup and run

From the repository root (PowerShell):

```powershell
uv sync --dev --extra simulation --extra videoanalysis
uv run python -m videoanalysis videoanalysis\footage\your-video.mp4 --max-frames 300
```

The simulation extra preserves the scientific stack already used in this repo.
For a lightweight video-only environment, use `uv sync --dev --extra videoanalysis`.
Source footage is not included. Put a local video in `videoanalysis\footage\` or
pass its path directly; generated output and local footage are ignored by Git.

1. Drag a rectangle around a distinctive, rigid **hub** patch; Enter confirms.
   Do not select a moving linkage, flexible part, or the background.
2. Box the **blade tape** with some margin. This opens a magnified view.
   Click its four physical corners **clockwise around the boundary**, starting
   near the top left. Enter confirms; R resets; C/Esc cancels. Do not select
   four corners of an enclosing rectangle: click the tape boundary itself.
3. In the next window, press A to add another blade patch, or any other key to
   start processing. Multiple patches help compare movement along the blade.

Corner numbering (0-3) stays attached to the selected physical corners; it is
never re-sorted as the shape changes. Zoomed selection preserves subpixel
coordinates in original decoded pixels. OpenCV applies MP4 rotation metadata.
The source video is never modified. The exported video has no audio.

The checked-in `example-regions.json` and `example-regions-20s.json` are
clip-specific sample selections for 1080x1920 footage. The source recording is
not included; use interactive selection or edit the examples for your video:

```powershell
uv run python -m videoanalysis videoanalysis\footage\your-video.mp4 --regions videoanalysis\example-regions.json --max-frames 300 --output videoanalysis\results\my-run
```

Omit `--max-frames` to process the whole video. Use `--start-frame N` to select
patches on a later frame. Saved configurations must match the decoded frame size
and starting frame, and should only be reused for the same recording.
The hub entry uses `{"name": "hub", "rect": [x, y, width, height]}`; blade entries
require `{"name": "tape", "corners": [[x0,y0], [x1,y1], [x2,y2], [x3,y3]]}`.
Blade rectangles are rejected rather than interpreted as physical tape corners.

### Compensating flicker in the 20-40 second segment

For the sample recording, frames 600-1198 cover about 20 seconds when recorded
at 29.97 fps. A separate selection is provided for that starting frame:

```powershell
uv run python -m videoanalysis videoanalysis\footage\your-video.mp4 --regions videoanalysis\example-regions-20s.json --start-frame 600 --max-frames 599 --compensate-flicker --output videoanalysis\results\my-20-40s
```

`--compensate-flicker` is optional and changes the exported video to a
**contrast-normalized grayscale analysis view**, not a natural-color restoration.
Without the flag, the original-color annotated output is unchanged.

Compensation removes a spatially Gaussian-weighted local mean and scales local
contrast (sigma 5 pixels, noise floor 5 intensity units). The same normalized
image drives recovery templates and corner flow; an additional CLAHE step is
disabled in this mode. Recovery proposals use one-pixel Gaussian smoothing to
avoid matching only high-frequency edge noise. The measured corners still come
from the unsmoothed normalized image and must pass all existing confidence gates.

There is **no temporal frame blending, coordinate smoothing, or motion-frequency
notch filter**. This reduces brightness/color-flicker sensitivity without deciding
which movement frequencies are real. Clipped glare, motion blur, and invisible
corners cannot be recovered; those measurements remain missing.

## Results

Default output: `videoanalysis\results\<video-stem>\` (ignored by Git).
Existing result files are never overwritten; choose a fresh `--output` directory.

| File | Contents |
|------|----------|
| `annotated.mp4` | Numbered, changing tape quadrilaterals; hub box; status/method and displacement |
| `displacement.csv` | One row per region per frame, including corner coordinates and shape measurements |
| `regions.json` | Saved selection; reuse with `--regions` to skip windows |
| `summary.json` | Source metadata, settings, frame counts, and valid/lost tracking totals |

`raw_patch_median` and `tracking_patch_median` record grayscale median brightness
within each fixed reference region's bounding box, before and after compensation.
The summary reports the strongest observed brightness peak above 1 Hz and the
tracking-view amplitude reduction within +/-0.5 Hz of that peak. These are
brightness diagnostics, **not rotor frequency or radiometric calibration**;
movement through the fixed box can also affect them. Short, constant, irregularly
sampled, or undersampled series are explicitly identified instead of assigned
frequency estimates.

CSV `frame` is the zero-based source frame index. `time_s` is the decoder's
source timestamp; `elapsed_s` starts at the selected reference frame. Missing or
nonmonotonic timestamps and premature decode failures stop processing explicitly.
The MP4 export uses the reported nominal frame rate; variable-frame-rate source
timing remains in the CSV, not necessarily in exported playback.

Coordinates use image **x right, y down**. For the tape, `dx_px`/`dy_px` measure
the movement of the quadrilateral's diagonal intersection from the reference
frame. This is the projected center of a planar square, not its bounding-box
center. For the hub, they remain patch translations. `relative_dx_px`/
`relative_dy_px` subtract hub translation; the hub's relative displacement is zero.
If either patch is lost, relative fields are **empty**, not zero or carried forward;
`relative_status` identifies the missing measurement.

### Distortion measurements

For each blade marker, the CSV exports:

- `corner_i_x_px` / `corner_i_y_px`: four independently measured corner positions.
- `relative_corner_i_dx_px` / `relative_corner_i_dy_px`: each corner's movement
  from its own reference position, minus hub translation.
- `edge_i_px`: length from corner i to (i+1) modulo 4.
- `edge_i_ratio`: each edge's length divided by its reference-frame length.
- `angle_i_deg` / `angle_i_delta_deg`: projected interior angle and its change.
- `area_px2` / `area_ratio`: projected area and ratio to the reference frame.

`shape_status=ok` means all four corners were measured. Shape fields are blank on
corner loss, not copied from an old shape. Shape measurements remain available
when the hub is lost; only hub-relative fields are blank. Hub shape fields are
blank with `shape_status=not_applicable`. The summary counts shape-valid frames.

A physical square need not look square in the image. No right angles, parallel
opposite edges, or equal edge lengths are imposed on the tracked quadrilateral.
These measurements describe **image-plane distortion**, including perspective
and foreshortening, not necessarily material strain or physical blade pitch.

### Approximate square-normal roll and pitch

Post-process an existing tracking CSV without decoding the video again:

```powershell
uv run python -m videoanalysis.orientation videoanalysis\results\20-40s-compensated\displacement.csv
```

For the whole recording, omit the tracking frame limit and then plot its CSV:

```powershell
uv run python -m videoanalysis videoanalysis\footage\your-video.mp4 --regions videoanalysis\example-regions.json --compensate-flicker --output videoanalysis\results\full-video-compensated
uv run python -m videoanalysis.orientation videoanalysis\results\full-video-compensated\displacement.csv --smooth-seconds 0.5
```

By default this creates an `orientation` subdirectory next to the input CSV.
Use `--output <new-directory>` to choose another destination. Outputs are
`orientation.csv`, `orientation_summary.json`, and `normal_orientation.png`
(one numbered graph per additional marker). Existing outputs are not overwritten.

This explicitly assumes a **flat physical square, zero lens distortion, and
local orthographic projection** (parallel viewing rays across the marker).
Ignoring lens distortion alone does not establish the latter assumption.
No focal length or field of view is inferred. Small size in the image does not
guarantee negligible lens distortion, especially away from the image center.
These are conditional camera-relative estimates, **not calibrated blade/body
pitch, roll, or bending angles**.

An affine least-squares fit maps a unit square to the four measured corners.
The ratio of its minor/major singular values gives `cos(tilt)`; the minor image
axis gives the projected square normal. The normal points into the image:
camera axes are x right, y down, z forward. Angles obey
`normal = Ry(pitch) Rx(roll) [0,0,1]`, so
`roll = atan2(-ny, hypot(nx,nz))` and `pitch = atan2(nx,nz)`.
In-plane square rotation is not interpreted as roll.

Two mirror tilts, `(nx,ny,nz)` and `(-nx,-ny,nz)`, produce the same projection.
Only one tilt branch is displayed. Its sign is chosen for visual continuity,
**not physical disambiguation**. Source timestamps are preserved; lost corners
and model-rejected shapes leave blank CSV angles and gaps in the graph.

The graph uses a **centered 0.5-second moving mean** by default, with raw angles
shown faintly for comparison. `--smooth-seconds N` changes the time window;
`--smooth-seconds 0` disables smoothing. Each sample averages available angles
within half the window on either side, using source timestamps. Smoothing runs
separately within each contiguous valid segment and marker; it never crosses a
missing measurement or fills a gap. Windows are truncated at segment endpoints,
so very short segments receive less smoothing. It is an offline, centered filter,
not a causal real-time estimate.

CSV `roll_deg` / `pitch_deg` and `normal_x` / `normal_y` / `normal_z` remain raw
estimates. Separate `smoothed_roll_deg` / `smoothed_pitch_deg` columns hold the
displayed angles (equal to raw when smoothing is disabled). The summary records
the method, window, and gap handling. Smoothing changes only the presentation
and derived angle columns, not corner tracking, confidence gates, or source CSV.
It attenuates rapid real movement as well as jitter; use raw values for motion
frequency/amplitude analysis.

`affine_residual_px` is RMS corner-fit error;
`affine_residual_fraction` divides it by the shortest measured edge. Fractions
above 0.1 produce `perspective_mismatch` and blank angles, rather than a pose from
a strongly non-parallelogram shape. This heuristic catches gross model departure;
a low residual is **not proof** of correct perspective, corner placement, or
angle accuracy. The roughly 20-pixel short edge in this recording makes estimates
sensitive to corner errors. `orientation_summary.json` records assumptions and
per-marker status counts. No graph is produced if there are no acceptable fits.

## Tracking and limits

The tape's four corners are followed independently by pyramidal Lucas-Kanade
optical flow, using local CLAHE contrast normalization for changing lighting.
Corner windows and pyramid depth are limited by the shortest edge so nearby
corners do not share an oversized tracking neighborhood. Four ordered corners
can represent a projective deformation; the tape is never constrained to an
affine parallelogram or replaced with a translated first-frame rectangle.

Every corner must pass forward/backward error of at most one pixel and LK
intensity error of at most 30. The shape must remain convex, clockwise, inside
the image, have edges at least four pixels long, and area at least 32 pixels
squared. Edge length changes outside 0.7-1.4 per step and per-corner movements
beyond `--search-radius` are rejected. If even one corner fails, the entire
quadrilateral is missing.

An unchanged first-frame template supplies a **location proposal** for recovery.
`--min-score` (default 0.75), peak-ambiguity, and search-boundary gates apply to
that proposal. Recovery additionally measures and verifies all four corners
against the reference image; template correlation alone never supplies shape.
There is no whole-image search or template replacement.

The hub continues to use template translation anchors and contrast-normalized
sparse flow with affine fitting between anchors. Hub flow requires six features,
60% surviving features, 70% affine inliers (1.5-pixel RANSAC threshold), the same
one-pixel/30-intensity flow gates, and per-frame affine scales within 0.7-1.4.

CSV `method` distinguishes tape `reference`, `corner_flow`, `corner_reacquire`
from hub `template`/`flow`. `score` and `peak_gap` describe the template proposal,
not the reliability of the measured quadrilateral. `tracked_points=4` describes
accepted corner flow; `fb_error_px` is its worst corner error. Hub flow reports
its inlier count, ratio, and median forward/backward error. The initial reference
uses the manually selected corners, not a new flow measurement.

The default per-axis `--search-radius` is 60 original pixels. After loss, the same
local area is searched for recovery. Flat patches fail initial validation. Corner
flow can drift and manual corner placement affects the baseline, so review the
annotated video. A red polygon shows the reference shape on loss, not an estimated
current shape. Console and summary report losses even when export completes.

Hub subtraction removes **2D translation only**, not camera rotation, perspective
changes, scale changes, or mount flex. Tracking itself is pixel-motion analysis,
not a physical pitch, flap, bending, or RPM estimator. The separate square-normal
postprocessor provides only the conditional approximation described above.
Background inside a patch can corrupt tracking.
Strong appearance changes, out-of-plane motion, blur, occlusion, lens distortion,
and rolling shutter can bias measurements or cause gaps. No physical angles,
distances, or vibration bandwidth are inferred without appropriate calibration
and sampling evidence.

On cancellation or failure, partial files may remain and no successful summary
is produced. Rerun into a new output directory. Successful completion verifies
the exported frame count and decodes the first and last exported frames.

## Tests

```powershell
uv run python -m pytest tests\unit\test_videoanalysis.py tests\unit\test_videoanalysis_quad.py tests\unit\test_videoanalysis_photometry.py tests\unit\test_videoanalysis_orientation.py -q
```

Synthetic tests verify actual non-affine projective corner coordinates, unequal
opposite edge lengths, angle/area changes, changing illumination, corner loss
and verified recovery, blank missing measurements, zoomed selection, overlays,
timestamped CSVs, seeking, and decoded MP4 frame counts.
Flicker tests also verify 12.5 Hz exposure compensation while preserving the
amplitude of a known 3 Hz physical marker movement.
Orientation tests verify known orthographically projected 3D normals, roll/pitch
signs, both possible mirror poses, scale/translation/in-plane-spin invariance,
model departure rejection, missing-data gaps, and actual PNG/CSV export.
Smoothing tests cover centered time windows, irregular timestamps, endpoint
handling, jitter reduction, raw-data preservation, and never filtering across gaps.
