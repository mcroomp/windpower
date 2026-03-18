"""
Digitize curves from original thesis figure PNGs.

Extracts colored pixel traces from each subplot, maps pixel coordinates
to data coordinates, and saves CSV files for comparison.
"""

import numpy as np
from PIL import Image
import os
import csv


# ---------------------------------------------------------------------------
# Color detection
# ---------------------------------------------------------------------------

COLOR_THRESHOLDS = {
    # name: (R_range, G_range, B_range)
    "blue":  ((0, 120), (0, 120), (150, 255)),
    "green": ((0, 120), (100, 220), (0, 120)),
    "red":   ((150, 255), (0, 120), (0, 120)),
    "black": ((0, 60), (0, 60), (0, 60)),
}


def color_mask(img_rgb, color_name):
    """Return boolean mask of pixels matching the named color."""
    rr, gr, br = COLOR_THRESHOLDS[color_name]
    return (
        (img_rgb[:, :, 0] >= rr[0]) & (img_rgb[:, :, 0] <= rr[1]) &
        (img_rgb[:, :, 1] >= gr[0]) & (img_rgb[:, :, 1] <= gr[1]) &
        (img_rgb[:, :, 2] >= br[0]) & (img_rgb[:, :, 2] <= br[1])
    )


def extract_curve(img_rgb, color_name, x_range, y_range):
    """
    Extract a curve from a subplot image by scanning for colored pixels.

    img_rgb:  numpy array (H, W, 3) -- cropped subplot region
    color_name: one of COLOR_THRESHOLDS keys
    x_range:  (x_min, x_max) data coordinates for the x-axis
    y_range:  (y_min, y_max) data coordinates for the y-axis

    Returns: (x_data, y_data) arrays
    """
    mask = color_mask(img_rgb, color_name)
    h, w = mask.shape

    x_data = []
    y_data = []

    for col in range(w):
        rows = np.where(mask[:, col])[0]
        if len(rows) == 0:
            continue
        # Take median y-pixel for this column (handles thick/anti-aliased lines)
        y_px = np.median(rows)
        # Map pixel coords to data coords
        # x: col 0 -> x_min, col w-1 -> x_max
        x_val = x_range[0] + (x_range[1] - x_range[0]) * col / (w - 1)
        # y: row 0 -> y_max (top), row h-1 -> y_min (bottom) -- image y is inverted
        y_val = y_range[1] - (y_range[1] - y_range[0]) * y_px / (h - 1)
        x_data.append(x_val)
        y_data.append(y_val)

    return np.array(x_data), np.array(y_data)


def extract_multi_curve(img_rgb, color_name, x_range, y_range, n_curves=1):
    """
    Extract multiple curves of the same color from a subplot
    (e.g. dashed lines that appear as separate segments).

    For single curves, delegates to extract_curve.
    For n_curves > 1, clusters y-pixels per column into separate traces.
    """
    if n_curves == 1:
        return [extract_curve(img_rgb, color_name, x_range, y_range)]

    mask = color_mask(img_rgb, color_name)
    h, w = mask.shape

    # For multiple curves, collect all points then cluster by y
    all_cols = []
    all_rows = []
    for col in range(w):
        rows = np.where(mask[:, col])[0]
        for r in rows:
            all_cols.append(col)
            all_rows.append(r)

    if len(all_cols) == 0:
        return [([],[])] * n_curves

    all_cols = np.array(all_cols)
    all_rows = np.array(all_rows)

    # Simple clustering: sort by row, split into n_curves groups at gaps
    from scipy.ndimage import label
    # Create a 2D boolean image and label connected components
    labeled, n_found = label(mask)

    curves = []
    for lbl in range(1, n_found + 1):
        pts = np.where(labeled == lbl)
        if len(pts[0]) < 10:
            continue
        # For this component, extract median y per column
        rows_c = pts[0]
        cols_c = pts[1]
        x_arr = []
        y_arr = []
        for col in sorted(set(cols_c)):
            rs = rows_c[cols_c == col]
            y_px = np.median(rs)
            x_val = x_range[0] + (x_range[1] - x_range[0]) * col / (w - 1)
            y_val = y_range[1] - (y_range[1] - y_range[0]) * y_px / (h - 1)
            x_arr.append(x_val)
            y_arr.append(y_val)
        curves.append((np.array(x_arr), np.array(y_arr)))

    return curves


def save_csv(filename, headers, *columns):
    """Save columns to CSV."""
    with open(filename, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(headers)
        for row in zip(*columns):
            w.writerow(["%.6f" % v for v in row])
    print("  saved %s (%d rows)" % (filename, len(columns[0])))


# ---------------------------------------------------------------------------
# Figure 10 digitizer
# ---------------------------------------------------------------------------

def digitize_fig10(img_path, out_dir):
    """
    Fig 10: 3x2 grid of subplots.
    Left:  Apparent wind v_ap, AoA alpha, Blade pitch beta
    Right: Force e'_x, Force e'_y, Force e'_z
    Each subplot has 3 curves: blue (blade 1), green (blade 2), red (blade 3)
    """
    print("Digitizing Fig 10...")
    img = np.array(Image.open(img_path).convert("RGB"))
    h, w, _ = img.shape

    # Subplot regions (fractional coordinates: top, bottom, left, right)
    # Determined from gridline pixel analysis of the actual image
    rows_frac = [
        (0.087, 0.274),   # row 0 (v_ap)
        (0.370, 0.529),   # row 1 (alpha)
        (0.635, 0.846),   # row 2 (beta)
    ]
    cols_frac = [
        (0.126, 0.438),   # col 0 (left)
        (0.561, 0.873),   # col 1 (right)
    ]

    # Data ranges for each subplot [row][col] = (x_range, y_range)
    # Time axis matches visible labels: 0.00 to 0.14
    t_range = (0.0, 0.14)
    data_ranges = {
        (0, 0): (t_range, (40.0, 55.0)),    # v_ap [m/s]
        (1, 0): (t_range, (-10.0, 20.0)),   # alpha [deg]
        (2, 0): (t_range, (-20.0, 20.0)),   # beta [deg]
        (0, 1): (t_range, (-100.0, 0.0)),   # F_ex [N]
        (1, 1): (t_range, (-60.0, 40.0)),   # F_ey [N]
        (2, 1): (t_range, (75.0, 225.0)),   # F_ez [N]
    }

    names = {
        (0, 0): "v_ap",
        (1, 0): "alpha",
        (2, 0): "beta",
        (0, 1): "F_ex",
        (1, 1): "F_ey",
        (2, 1): "F_ez",
    }

    for ri, (rt, rb) in enumerate(rows_frac):
        for ci, (cl, cr) in enumerate(cols_frac):
            # Crop subplot
            y0, y1 = int(rt * h), int(rb * h)
            x0, x1 = int(cl * w), int(cr * w)
            sub = img[y0:y1, x0:x1, :]

            x_range, y_range = data_ranges[(ri, ci)]
            name = names[(ri, ci)]

            # Extract each blade's curve
            all_t = {}
            all_v = {}
            for color, blade_id in [("blue", 1), ("green", 2), ("red", 3)]:
                t_arr, v_arr = extract_curve(sub, color, x_range, y_range)
                if len(t_arr) > 0:
                    all_t[blade_id] = t_arr
                    all_v[blade_id] = v_arr

            # Save -- interpolate all blades to common time grid
            if all_t:
                t_common = np.linspace(x_range[0], x_range[1], 200)
                headers = ["time"]
                cols = [t_common]
                for bid in [1, 2, 3]:
                    if bid in all_t and len(all_t[bid]) > 3:
                        v_interp = np.interp(t_common, all_t[bid], all_v[bid])
                        cols.append(v_interp)
                        headers.append("blade_%d" % bid)
                    else:
                        cols.append(np.full_like(t_common, np.nan))
                        headers.append("blade_%d" % bid)

                save_csv(os.path.join(out_dir, "orig_fig10_%s.csv" % name),
                         headers, *cols)

def extract_curve_with_margin(img_rgb, color_name, x_range, y_range, margin=5):
    """
    Extract curve with a pixel margin excluded from edges.
    Removes axis spine/tick contamination for black dashed lines.
    """
    h, w = img_rgb.shape[:2]
    # Zero out border pixels
    cropped = img_rgb[margin:h-margin, margin:w-margin, :]
    t_arr, y_arr = extract_curve(cropped, color_name, x_range, y_range)
    return t_arr, y_arr


# ---------------------------------------------------------------------------
# Figure 14 digitizer
# ---------------------------------------------------------------------------

def digitize_fig14(img_path, out_dir):
    """
    Fig 14: 4 panels (a,b,c,d) in 2x2, each with tracking + error subplots.
    Tracking: blue = beta, black dashed = beta_ref
    Error: green
    """
    print("Digitizing Fig 14...")
    img = np.array(Image.open(img_path).convert("RGB"))
    h, w, _ = img.shape

    # 4 panels determined from axis spine pixel analysis
    # Corrected from gridline positions in the actual image
    panels = {
        "a": {"track": (0.247, 0.352, 0.218, 0.483),
               "error": (0.392, 0.477, 0.218, 0.483)},
        "b": {"track": (0.247, 0.352, 0.568, 0.833),
               "error": (0.392, 0.477, 0.568, 0.833)},
        "c": {"track": (0.593, 0.696, 0.218, 0.483),
               "error": (0.734, 0.838, 0.218, 0.483)},
        "d": {"track": (0.593, 0.696, 0.568, 0.833),
               "error": (0.734, 0.838, 0.568, 0.833)},
    }

    t_range = (0.0, 6.0)
    beta_range = (-30.0, 30.0)
    error_range = (-20.0, 20.0)

    for panel_name, regions in panels.items():
        # Tracking subplot
        rt, rb, cl, cr = regions["track"]
        sub = img[int(rt*h):int(rb*h), int(cl*w):int(cr*w), :]
        # Extract blue (beta)
        t_beta, beta = extract_curve(sub, "blue", t_range, beta_range)
        # Extract black (beta_ref) with margin to exclude axis spines
        t_ref, bref = extract_curve_with_margin(sub, "black", t_range,
                                                 beta_range, margin=5)

        # Error subplot
        rt, rb, cl, cr = regions["error"]
        sub_e = img[int(rt*h):int(rb*h), int(cl*w):int(cr*w), :]
        t_err, err = extract_curve(sub_e, "green", t_range, error_range)

        # Save
        t_common = np.linspace(0, 6, 500)
        headers = ["time", "beta", "beta_ref", "error"]
        cols_out = [t_common]

        for t_arr, v_arr, label in [
            (t_beta, beta, "beta"),
            (t_ref, bref, "beta_ref"),
            (t_err, err, "error"),
        ]:
            if len(t_arr) > 3:
                cols_out.append(np.interp(t_common, t_arr, v_arr))
            else:
                cols_out.append(np.full_like(t_common, np.nan))

        save_csv(os.path.join(out_dir, "orig_fig14_%s.csv" % panel_name),
                 headers, *cols_out)


# ---------------------------------------------------------------------------
# Figure 15 digitizer
# ---------------------------------------------------------------------------

def digitize_fig15(img_path, out_dir):
    """
    Fig 15: 2 columns (a=without CL, b=with CL), 2 rows (tracking, error).
    """
    print("Digitizing Fig 15...")
    img = np.array(Image.open(img_path).convert("RGB"))
    h, w, _ = img.shape

    # Corrected panel regions from pixel analysis
    # Fig15 is cropped from page_43 at ~49-73% of page height
    panels = {
        "open_loop":   {"track": (0.05, 0.42, 0.218, 0.483),
                         "error": (0.50, 0.88, 0.218, 0.483)},
        "closed_loop": {"track": (0.05, 0.42, 0.568, 0.833),
                         "error": (0.50, 0.88, 0.568, 0.833)},
    }

    t_range = (0.0, 6.0)
    beta_range = (-30.0, 30.0)
    error_range = (-15.0, 15.0)

    for panel_name, regions in panels.items():
        rt, rb, cl, cr = regions["track"]
        sub = img[int(rt*h):int(rb*h), int(cl*w):int(cr*w), :]
        t_beta, beta = extract_curve(sub, "blue", t_range, beta_range)
        t_ref, bref = extract_curve_with_margin(sub, "black", t_range,
                                                 beta_range, margin=5)

        rt, rb, cl, cr = regions["error"]
        sub_e = img[int(rt*h):int(rb*h), int(cl*w):int(cr*w), :]
        t_err, err = extract_curve(sub_e, "green", t_range, error_range)

        t_common = np.linspace(0, 6, 500)
        headers = ["time", "beta", "beta_ref", "error"]
        cols_out = [t_common]

        for t_arr, v_arr, label in [
            (t_beta, beta, "beta"),
            (t_ref, bref, "beta_ref"),
            (t_err, err, "error"),
        ]:
            if len(t_arr) > 3:
                cols_out.append(np.interp(t_common, t_arr, v_arr))
            else:
                cols_out.append(np.full_like(t_common, np.nan))

        save_csv(os.path.join(out_dir, "orig_fig15_%s.csv" % panel_name),
                 headers, *cols_out)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    orig_dir = "fig_original"
    csv_dir  = "csv_original"
    os.makedirs(csv_dir, exist_ok=True)

    digitize_fig10(os.path.join(orig_dir, "fig10_original.png"), csv_dir)
    digitize_fig14(os.path.join(orig_dir, "fig14_original.png"), csv_dir)
    digitize_fig15(os.path.join(orig_dir, "fig15_original.png"), csv_dir)

    print("\nDone. CSVs in %s/" % csv_dir)
