"""
flight_report.py — Standalone flight report plotting for RAWES simulation.

No test dependencies — safe to import from scripts, notebooks, or tests.
"""
import logging
import math


def plot_flight_report(
    pos_history:      list,   # [(t, N, E, D), ...]
    attitude_history: list,   # [(t, roll_deg, pitch_deg, yaw_deg), ...]
    servo_history:    list,   # [(t, s1, s2, s3, s4_pwm), ...]
    events:           dict,   # {label: t_relative_to_obs_start}
    target:           tuple,  # (N, E, D) target position in LOCAL_NED from home
    out_path,                 # Path or str
    telemetry_path=None,      # Path or None — mediator per-step CSV
    tether_length_m: float = 200.0,
    anchor_ned: tuple = None, # (N, E, D) anchor position in LOCAL_NED from home.
                              # When provided, space-time panel shows horizontal
                              # distance from anchor vs altitude above anchor.
) -> None:
    """
    Save a flight report PNG with up to 6 panels.

    Panels (top to bottom):
      1. Space-time diagram — East vs Altitude, time as colour
      2. 3-D displacement from home vs time  — with pass threshold
      3. Attitude  — roll, pitch, yaw in degrees
      4. Swashplate commands  — collective, tilt X (lon), tilt Y (lat) in degrees; S4 ESC µs
      5. Aero thrust & hub altitude (from mediator telemetry, if available)
      6. Rotor spin & torque balance (from mediator telemetry, if available)
    Event markers are drawn as vertical lines across all panels.
    """
    from pathlib import Path
    out_path = Path(out_path)

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import matplotlib.gridspec as gridspec
    except ImportError:
        logging.getLogger("flight_report").warning(
            "matplotlib not available — skipping flight report plot"
        )
        return

    log = logging.getLogger("flight_report")

    if not pos_history:
        log.warning("No position data — skipping flight report plot")
        return

    ts_pos  = [r[0] for r in pos_history]
    ns      = [r[1] for r in pos_history]
    es      = [r[2] for r in pos_history]
    ds      = [r[3] for r in pos_history]
    disps   = [math.sqrt(n**2 + e**2 + d**2) for n, e, d in zip(ns, es, ds)]

    target_n, target_e, target_d = target

    # Load mediator telemetry CSV if available
    telem: dict = {}
    if telemetry_path is not None and Path(telemetry_path).exists():
        try:
            from telemetry_csv import read_csv as _read_csv
            rows = _read_csv(telemetry_path)
            if rows:
                t_end_telem      = rows[-1].t_sim
                t_end_obs        = ts_pos[-1] if ts_pos else 0.0
                offset           = t_end_obs - t_end_telem
                telem["t"]           = [r.t_sim + offset      for r in rows]
                telem["T"]           = [r.aero_T              for r in rows]
                telem["v_axial"]     = [r.aero_v_axial        for r in rows]
                telem["v_inplane"]   = [r.aero_v_inplane      for r in rows]
                telem["omega_rotor"] = [r.omega_rotor         for r in rows]
                telem["pos_z"]       = [r.pos_z               for r in rows]
                telem["F_z"]         = [r.F_z                 for r in rows]
                telem["collective"]  = [math.degrees(r.collective_rad) for r in rows]
                telem["tilt_lon"]    = [r.tilt_lon             for r in rows]
                telem["tilt_lat"]    = [r.tilt_lat             for r in rows]
                telem["rpy_roll"]    = [math.degrees(r.rpy_roll)  for r in rows]
                telem["rpy_pitch"]   = [math.degrees(r.rpy_pitch) for r in rows]
                telem["rpy_yaw"]     = [math.degrees(r.rpy_yaw)   for r in rows]
                log.info("Telemetry loaded: %d rows from %s", len(rows), telemetry_path)
        except Exception as exc:
            log.warning("Could not load telemetry CSV: %s", exc)

    n_panels = 4 + (2 if telem else 0)
    fig = plt.figure(figsize=(14, 4 * n_panels + 2))
    fig.suptitle("RAWES Guided Flight Test — Flight Report", fontsize=14, fontweight="bold")
    gs = gridspec.GridSpec(n_panels, 1, hspace=0.45, top=0.97 - 0.02 * n_panels,
                           bottom=0.05, left=0.08, right=0.97)

    event_colors = {
        "EKF lock":        "#2ca02c",
        "Armed":           "#d62728",
        "GUIDED":          "#9467bd",
        "Position target": "#8c564b",
    }

    t_max = max(ts_pos) if ts_pos else 60.0
    x_min = min(min(events.values(), default=0.0) - 2.0, -2.0)

    def _add_events(ax):
        ax.set_xlim(x_min, t_max + 1.0)
        ylo, yhi = ax.get_ylim()
        for label, t in sorted(events.items(), key=lambda x: x[1]):
            c = event_colors.get(label, "#7f7f7f")
            ax.axvline(t, color=c, linewidth=1.2, linestyle="--", alpha=0.8)
            ax.text(t + 0.4, yhi - (yhi - ylo) * 0.02,
                    label, color=c, fontsize=7.5, va="top",
                    rotation=90, alpha=0.9, fontweight="bold")

    # ── Panel 1: Space-time diagram ───────────────────────────────────────────
    import numpy as _np
    t_arr = _np.array(ts_pos)
    ax1 = fig.add_subplot(gs[0])

    if anchor_ned is not None:
        # Show position relative to anchor: horizontal distance vs altitude.
        # Convert LOCAL_NED (from home) to position relative to anchor.
        #   hub_from_anchor_ned = hub_ned - anchor_ned
        #   ENU: E=hub_fa_ned[1], N=hub_fa_ned[0], Z=-hub_fa_ned[2]
        an, ae, ad = anchor_ned
        horiz = _np.array([math.sqrt((n - an)**2 + (e - ae)**2) for n, e in zip(ns, es)])
        alt_from_anc = _np.array([ad - d for d in ds])  # D=down, so alt = anchor_d - pos_d

        # Target in anchor-relative coords
        tn, te, td = target
        tgt_horiz = math.sqrt((tn - an)**2 + (te - ae)**2)
        tgt_alt   = ad - td

        sc = ax1.scatter(horiz, alt_from_anc, c=t_arr, cmap="viridis",
                         s=4, linewidths=0, zorder=3)
        cb = fig.colorbar(sc, ax=ax1, pad=0.01)
        cb.set_label("Time (s)", fontsize=8)
        cb.ax.tick_params(labelsize=7)

        theta_arc = _np.linspace(0, _np.pi / 2, 200)
        ax1.plot(tether_length_m * _np.cos(theta_arc),
                 tether_length_m * _np.sin(theta_arc),
                 color="grey", linewidth=1.0, linestyle=":", alpha=0.6,
                 label=f"Tether arc (r={tether_length_m:.0f} m)")
        ax1.scatter([tgt_horiz], [tgt_alt], marker="*", s=180,
                    color="#d62728", zorder=5,
                    label=f"Target ({tgt_horiz:.0f} m horiz, {tgt_alt:.0f} m alt)")
        ax1.scatter([horiz[0]], [alt_from_anc[0]], marker="o", s=60,
                    color="#2ca02c", zorder=5, label="Start")
        ax1.set_xlabel("Horizontal distance from anchor (m)")
        ax1.set_ylabel("Altitude above anchor (m)")
        ax1.set_title("Space–time diagram  (tether plane: horiz distance vs altitude, colour = time)")
    else:
        # Fallback: East vs Altitude from home
        alts  = [-d for d in ds]
        e_arr = _np.array(es)
        a_arr = _np.array(alts)
        sc = ax1.scatter(e_arr, a_arr, c=t_arr, cmap="viridis",
                         s=4, linewidths=0, zorder=3)
        cb = fig.colorbar(sc, ax=ax1, pad=0.01)
        cb.set_label("Time (s)", fontsize=8)
        cb.ax.tick_params(labelsize=7)
        theta_arc = _np.linspace(0, _np.pi / 2, 200)
        ax1.plot(tether_length_m * _np.cos(theta_arc),
                 tether_length_m * _np.sin(theta_arc),
                 color="grey", linewidth=1.0, linestyle=":", alpha=0.6,
                 label=f"Tether boundary (r={tether_length_m:.0f} m)")
        ax1.scatter([target_e], [-target_d], marker="*", s=180,
                    color="#d62728", zorder=5,
                    label=f"Target ({target_e:.0f} m E, {-target_d:.0f} m alt)")
        ax1.scatter([e_arr[0]], [a_arr[0]], marker="o", s=60,
                    color="#2ca02c", zorder=5, label="Start")
        ax1.set_xlabel("East relative to home (m)")
        ax1.set_ylabel("Altitude relative to home (m)")
        ax1.set_title("Space–time diagram  (East vs Altitude, colour = time)")

    ax1.legend(loc="lower right", fontsize=8)
    ax1.set_aspect("equal", adjustable="datalim")
    ax1.grid(True, alpha=0.3)
    ax1.axhline(0, color="black", linewidth=0.5, alpha=0.3)
    ax1.axvline(0, color="black", linewidth=0.5, alpha=0.3)

    # ── Panel 2: 3-D displacement ─────────────────────────────────────────────
    ax2 = fig.add_subplot(gs[1])
    ax2.plot(ts_pos, disps, color="#2ca02c", linewidth=1.8, label="3-D displacement (m)")
    ax2.axhline(5.0, color="#e377c2", linestyle="--", linewidth=1.2,
                label="Pass threshold (5 m)")
    ax2.set_ylabel("Displacement (m)")
    ax2.set_title("3-D displacement from home")
    ax2.legend(loc="upper right", fontsize=8)
    ax2.grid(True, alpha=0.3)
    _add_events(ax2)

    # ── Panel 3: Attitude (physical orbital frame) ───────────────────────────
    ax3 = fig.add_subplot(gs[2])
    if telem and "rpy_roll" in telem:
        # Prefer mediator ground-truth physical attitude from telemetry
        ax3.plot(telem["t"], telem["rpy_roll"],  color="#1f77b4", linewidth=1.2, label="Roll  (°)  ← tilt_lat")
        ax3.plot(telem["t"], telem["rpy_pitch"], color="#ff7f0e", linewidth=1.2, label="Pitch (°)  ← tilt_lon")
        ax3.plot(telem["t"], telem["rpy_yaw"],   color="#2ca02c", linewidth=1.2, label="Yaw   (°)  (velocity heading)")
        ax3.axhline(0, color="black", linewidth=0.5, alpha=0.4)
    elif attitude_history:
        # Fallback: EKF attitude from ArduPilot MAVLink
        ts_att = [r[0] for r in attitude_history]
        rolls  = [r[1] for r in attitude_history]
        pitchs = [r[2] for r in attitude_history]
        yaws   = [r[3] for r in attitude_history]
        ax3.plot(ts_att, rolls,  color="#1f77b4", linewidth=1.2, label="Roll  (°) EKF  ← tilt_lat")
        ax3.plot(ts_att, pitchs, color="#ff7f0e", linewidth=1.2, label="Pitch (°) EKF  ← tilt_lon")
        ax3.plot(ts_att, yaws,   color="#2ca02c", linewidth=1.2, label="Yaw   (°) EKF  (velocity heading)")
        ax3.axhline(0, color="black", linewidth=0.5, alpha=0.4)
    else:
        ax3.text(0.5, 0.5, "No attitude data", transform=ax3.transAxes,
                 ha="center", va="center", color="grey")
    ax3.set_ylabel("Angle (°)")
    ax3.set_title("Attitude relative to tether direction  (0° = axle aligned with tether)")
    ax3.legend(loc="upper right", fontsize=8)
    ax3.grid(True, alpha=0.3)
    _add_events(ax3)

    # ── Panel 4: Swashplate mixed outputs (collective, tilt X/Y) + S4 ESC ──────
    _COLL_MAX_DEG   = math.degrees(0.35)   # 20.05°
    _PITCH_GAIN_DEG = math.degrees(0.3)    # 17.19°

    ax4  = fig.add_subplot(gs[3])
    ax4b = ax4.twinx()
    if servo_history:
        ts_srv = [r[0] for r in servo_history]
        coll_deg, tilt_x_deg, tilt_y_deg, s4_pwm = [], [], [], []
        for _, p1, p2, p3, p4 in servo_history:
            n1 = (p1 - 1500) / 500
            n2 = (p2 - 1500) / 500
            n3 = (p3 - 1500) / 500
            coll_deg.append(((n1 + n2 + n3) / 3) * _COLL_MAX_DEG)
            tilt_x_deg.append(((2/3) * (n2 * 0.866 - n3 * 0.866)) * _PITCH_GAIN_DEG)
            tilt_y_deg.append(((2/3) * (n1 - 0.5*n2 - 0.5*n3)) * _PITCH_GAIN_DEG)
            s4_pwm.append(p4)
        ax4.plot(ts_srv, coll_deg,   linewidth=1.4, label="Collective (°)")
        ax4.plot(ts_srv, tilt_x_deg, linewidth=1.2, label="tilt_lon / longitudinal (°)  → pitch")
        ax4.plot(ts_srv, tilt_y_deg, linewidth=1.2, label="tilt_lat / lateral (°)  → roll")
        ax4.axhline(0, color="black", linewidth=0.7, linestyle=":", alpha=0.5)
        ax4b.plot(ts_srv, s4_pwm, linewidth=1.0, linestyle="--", color="grey",
                  label="S4 ESC / anti-rotation motor (µs)")
    else:
        ax4.text(0.5, 0.5, "No servo data", transform=ax4.transAxes,
                 ha="center", va="center", color="grey")
    ax4.set_xlabel("Time since position target (s)")
    ax4.set_ylabel("Angle (°)")
    ax4b.set_ylabel("S4 ESC (µs)", color="grey")
    ax4b.tick_params(axis="y", labelcolor="grey")
    ax4.set_title("ArduPilot swashplate commands  (H3-120 inverse mix)")
    lines4a, labels4a = ax4.get_legend_handles_labels()
    lines4b, labels4b = ax4b.get_legend_handles_labels()
    ax4.legend(lines4a + lines4b, labels4a + labels4b, loc="upper right", fontsize=7, ncol=2)
    ax4.grid(True, alpha=0.3)
    _add_events(ax4)

    # ── Panel 5: Aero thrust / hub altitude / collective (from telemetry) ────
    if telem:
        ax5 = fig.add_subplot(gs[4])
        ax5b = ax5.twinx()
        ax5.plot(telem["t"], telem["T"],         color="#2ca02c", linewidth=1.0, label="Thrust T (N)")
        ax5.plot(telem["t"], telem["F_z"],        color="#1f77b4", linewidth=0.8,
                 linestyle="--", label="Fz world (N)")
        ax5b.plot(telem["t"], [-z for z in telem["pos_z"]], color="#d62728", linewidth=1.0,
                  linestyle="-.", label="Hub altitude (m)")
        ax5.set_ylabel("Force (N)", color="#2ca02c")
        ax5b.set_ylabel("Altitude (m)", color="#d62728")
        ax5.set_title("Aero thrust & hub altitude (mediator telemetry)")
        lines5a, labels5a = ax5.get_legend_handles_labels()
        lines5b, labels5b = ax5b.get_legend_handles_labels()
        ax5.legend(lines5a + lines5b, labels5a + labels5b, loc="upper right", fontsize=7)
        ax5.grid(True, alpha=0.3)
        _add_events(ax5)

        # ── Panel 6: Rotor spin & torque / wind components ────────────────────
        ax6 = fig.add_subplot(gs[5])
        ax6b = ax6.twinx()
        ax6.plot(telem["t"], telem["omega_rotor"],  color="#9467bd", linewidth=1.0, label="ω rotor (rad/s)")
        ax6b.plot(telem["t"], telem["v_axial"],  color="#17becf", linewidth=0.8,
                  linestyle=":",  label="v_axial (m/s)")
        ax6b.plot(telem["t"], telem["v_inplane"],color="#8c564b", linewidth=0.8,
                  linestyle=":",  label="v_inplane (m/s)")
        ax6.set_ylabel("ω rotor (rad/s)", color="#9467bd")
        ax6b.set_ylabel("Torque (N·m) / Wind (m/s)", color="#e377c2")
        ax6.set_xlabel("Time since position target (s)")
        ax6.set_title("Rotor spin & torque balance / relative wind (mediator telemetry)")
        lines6a, labels6a = ax6.get_legend_handles_labels()
        lines6b, labels6b = ax6b.get_legend_handles_labels()
        ax6.legend(lines6a + lines6b, labels6a + labels6b, loc="upper right", fontsize=7)
        ax6.grid(True, alpha=0.3)
        _add_events(ax6)

    fig.savefig(str(out_path), dpi=120)
    plt.close(fig)
    log.info("Flight report saved → %s", out_path)
