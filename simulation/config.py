"""
config.py — Shared mediator configuration schema.

Used by mediator.py (to read config at startup) and by test fixtures
(to write per-test JSON config files before launching the mediator).

Schema
------
All keys and their defaults are defined in DEFAULTS below.
Pass a path to ``load(path)`` to get a config dict with defaults filled in.
Use ``save(cfg, path)`` to write a JSON file for the mediator to read.
Use ``defaults()`` to get a fresh copy of the defaults dict.

Example (test fixture):
    import config as mcfg
    cfg = mcfg.defaults()
    cfg["internal_controller"] = True
    cfg["base_k_ang"] = 50.0
    cfg_path = tmp_path / "mediator_config.json"
    mcfg.save(cfg, cfg_path)
    # pass --config str(cfg_path) to mediator subprocess

Example (mediator):
    import config as mcfg
    cfg = mcfg.load(args.config)   # args.config may be None → pure defaults
"""

import json
import copy

# ---------------------------------------------------------------------------
# Schema and defaults
# ---------------------------------------------------------------------------
DEFAULTS: dict = {
    # ── Rotor definition ─────────────────────────────────────────────────────
    "rotor_definition": "beaupoil_2026",  # built-in name or path to .yaml file

    # ── Environment ──────────────────────────────────────────────────────────
    "wind": [0.0, 10.0, 0.0],          # ambient wind NED [m/s]  (East = NED Y axis)

    # ── Initial hub state ─────────────────────────────────────────────────────
    # Warmup-settled equilibrium from test_steady_flight.py.
    # 50 m tether at ~30° elevation; body_z aligned with tether direction.
    "pos0":       [14.241, 46.258, -12.530],       # NED [m]  (T @ ENU)
    "vel0":       [ 0.916, -0.257,   0.093],       # NED [m/s]
    "body_z":     [0.305391, 0.851018, -0.427206], # unit vector (axle direction, NED)
    "omega_spin": 20.148,                           # rotor spin [rad/s]

    # ── Tether & Winch ────────────────────────────────────────────────────────
    "tether_rest_length": 99.949,   # unstretched tether length [m]
    "anchor_ned": [0.0, 0.0, 0.0],  # tether anchor position NED [m]
                                     # mirrors RAWES_ANCHOR_LAT/LON/ALT converted to local NED
    "tension_safety_n":  496.0,     # WinchController: stop paying out above this tension [N]
                                     # ≈ 80% of Dyneema SK75 1.9 mm break load (620 N)

    # ── Startup kinematic ramp ────────────────────────────────────────────────
    # Hub moves at vel0 from launch_pos for startup_damp_seconds, then physics
    # takes over.  Gives EKF time to lock on GPS before free flight.
    #
    # kinematic_traj_type selects the trajectory shape:
    #   "linear"   — constant vel0 from launch_pos, optional linear ramp to zero.
    #                Hub starts at pos0 - vel0*duration; vel unchanged unless ramp_s>0.
    #   "min_jerk" — 5th-order polynomial from [0,0,0] at rest to pos0 at vel0.
    #                Zero accel at both ends; accel_body computed analytically.
    #                Kinematic ends immediately on arrival (startup_damp_seconds
    #                must be long enough for GPS to fuse before arrival — 65 s
    #                gives ~11 s after fusion at ~54 s).
    #                kinematic_min_jerk_margin_s: optional extra hold at
    #                (pos0, vel0) before free flight (0 = hand over on arrival).
    #
    # kinematic_vel_ramp_s: vel taper window at end of kinematic (0 = no taper).
    # NOTE: ramp > 0 reduces GPS horizontal position change → slower EKF fusion.
    # Default 0.0 keeps vel0 constant so GPS fuses quickly (within kinematic window).
    "startup_damp_seconds":         30.0,   # kinematic duration [s]
    "kinematic_vel_ramp_s":         15.0,   # linear: vel ramp-to-zero window [s]
    "kinematic_traj_type":          "min_jerk",  # "linear" or "min_jerk"
    "kinematic_min_jerk_margin_s":   0.0,   # min_jerk: extra hold after trajectory [s]; 0 = hand over immediately on arrival
    "startup_damp_k_ang":          500.0,   # peak angular drag [N·m·s/rad] during kinematic phase

    # ── Attitude damping ──────────────────────────────────────────────────────
    "base_k_ang": 50.0,             # permanent angular drag [N·m·s/rad]
                                    # τ = I/k = 5/50 = 0.1 s settling time
                                    # Increase to 300 for 10 Hz MAVLink controller

    # ── Internal controller ───────────────────────────────────────────────────
    "internal_controller":      False,  # truth-state swashplate at 400 Hz
    "internal_controller_ramp":   3.0,  # ramp-in time after kinematic end [s]
    "lock_orientation":         False,  # debug: lock R to initial value every step

    # ── Swashplate phase compensation ─────────────────────────────────────────
    # Rotates the (tilt_lon, tilt_lat) cyclic command before it reaches the aero model,
    # compensating for gyroscopic precession when I_spin_kgm2 > 0.
    # 0.0 = disabled (use when I_spin = 0).  Theoretical: 90° for CCW rotor.
    # Determined by step-response test on hardware (corresponds to H_PHANG in ArduPilot).
    "swashplate_phase_deg":   0.0,
    "cyclic_kp":              0.30,  # truth-state cyclic proportional gain [rad/s / rad]
    "cyclic_kd":              0.12,  # truth-state cyclic derivative gain [rad/s / (rad/s)]

    # ── Mode_RAWES attitude parameters ───────────────────────────────────────
    # body_z_slew_rate_rad_s: max angular slew rate for body_z transitions [rad/s].
    #   Derived from RotorDefinition.body_z_slew_rate_rad_s = 2% of gyroscopic limit.
    #   beaupoil_2026: max_body_z_rate=20.1 rad/s → 0.02×20.1 = 0.40 rad/s.
    #   At 0.40 rad/s a 50° reel-in transition (xi=80°) completes in ~2.2 s.
    #   Update when rotor changes: rd.default().body_z_slew_rate_rad_s.
    "body_z_slew_rate_rad_s":    0.40,

    # ── Sensor model ──────────────────────────────────────────────────────────
    # Rotor spin speed is an internal simulation state (omega_spin ODE).
    # In hardware, rotor spin speed measurement is handled separately from
    # anti-rotation motor control — the gear coupling keeps the hub stationary
    # without requiring spin feedback; spin measurement is TBD for hardware.
    # Simulation noise: set 0 to use true omega_spin (ideal, recommended for unit tests).
    "spin_sensor_sigma": 0.0,           # Gaussian noise sigma [rad/s]; 0 = ideal

    # ── Trajectory controller ─────────────────────────────────────────────────
    # "type" selects the active controller.  Each type has its own sub-dict so
    # all parameters are self-contained and callers only touch one section.
    #
    # Supported types:
    #   "hold"        — orbit-track tether direction, zero collective, no winch.
    #                   No parameters required.
    #   "deschutter"  — De Schutter (2018) reel-out/reel-in pumping cycle.
    #                   Requires internal_controller=True.
    "trajectory": {
        "type": "hold",
        "hold": {},
        "landing": {
            # Vertical descent from high-tilt reel-in position directly above anchor.
            # Hub holds body_z fixed at the orientation captured at planner start
            # (initial_body_z=None → captured from first step's state_pkt["body_z"]).
            "v_land":           0.5,    # target descent rate [m/s]
            "col_cruise":       0.079,  # altitude-neutral collective at xi=80 deg [rad]
            "min_tether_m":     2.0,    # tether length at which final_drop begins [m]
            "tension_target_n": 80.0,   # winch PI tension setpoint [N]
            "k_winch":          0.005,  # winch PI gain [m/s per N]
            "v_pay_out":        0.5,    # max pay-out speed [m/s]
            "kp_vz":            0.05,   # descent-rate P gain [rad/(m/s)]
            "col_min_rad":     -0.28,   # collective floor [rad]
            "col_max_rad":      0.10,   # collective ceiling [rad]
        },
        "deschutter": {
            "t_hold_s":         0.0,   # hold phase before first reel-out [s]; use ~20s in stack test to settle orbit
            "t_reel_out":      30.0,   # reel-out phase duration [s]
            "t_reel_in":       30.0,   # reel-in phase duration [s]
            # t_transition derived: radians(xi_reel_in - 30) / body_z_slew_rate + 1.5 s margin.
            # At slew=0.40 rad/s and xi=80°: (50°×π/180)/0.40 + 1.5 ≈ 3.7 s.
            "t_transition":     3.7,   # body_z slew window [s]
            "v_reel_out":       0.4,   # winch pay-out speed [m/s]
            "v_reel_in":        0.4,   # winch reel-in speed [m/s]
            "tension_out":    200.0,   # reel-out tension setpoint [N]
            "tension_in":      55.0,   # reel-in tension setpoint [N] — above min at xi=80°
            # xi=80°: aerodynamic equilibrium settles ~80°; SkewedWakeBEM valid to ~85°.
            "xi_reel_in_deg":  80.0,   # De Schutter reel-in tilt [deg] from wind
            # Tension PI (trajectory planner — ground station, raws_mode.md §3.2)
            "tension_kp":      5e-4,   # proportional gain [rad/N]
            "tension_ki":      1e-4,   # integral gain [rad/(N·s)]
            # col_min_reel_in derived: binary search Fz=weight at xi=80° → 0.069+0.01=0.079 rad.
            "col_min_rad":         -0.28,  # reel-out floor [rad]
            "col_min_reel_in_rad":  0.079, # reel-in floor [rad]: altitude floor at xi=80°
            "col_max_rad":          0.10,  # extended for high-tilt altitude support [rad]
            # TensionPI warm-start: collective output at zero error on first step.
            # Overridden by initial_state["coll_eq_rad"] when available so PI
            # starts at the exact equilibrium collective after kinematic exit.
            "warm_coll_rad":       -0.20,  # [rad] default from TensionPI.WARM_COLL_RAD
        },
    },
}


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def defaults() -> dict:
    """Return a deep copy of DEFAULTS (safe to mutate)."""
    return copy.deepcopy(DEFAULTS)


def load(path: "str | None") -> dict:
    """
    Load a mediator config from a JSON file, filling missing keys from DEFAULTS.

    If path is None, returns a copy of DEFAULTS unchanged — useful for unit
    tests that bypass the mediator subprocess entirely.

    The ``"trajectory"`` key is deep-merged so callers can override only
    ``type`` or individual deschutter params without clobbering the rest.
    """
    cfg = defaults()
    if path is not None:
        with open(path, encoding="utf-8") as fh:
            overrides = json.load(fh)
        # Validate top-level keys — warn on unknown keys to catch typos
        unknown = set(overrides) - set(DEFAULTS)
        if unknown:
            import warnings
            warnings.warn(
                f"config.py: unknown keys in {path}: {sorted(unknown)}\n"
                "These will be ignored.  Check for typos.",
                stacklevel=2,
            )
        for k, v in overrides.items():
            if k not in DEFAULTS:
                continue
            if k == "trajectory" and isinstance(v, dict):
                # Deep merge: keep default sub-dicts, override only what's given
                cfg["trajectory"] = copy.deepcopy(DEFAULTS["trajectory"])
                cfg["trajectory"].update(v)
                for sub_key in ("hold", "deschutter", "landing"):
                    if sub_key in v and isinstance(v[sub_key], dict):
                        merged = copy.deepcopy(DEFAULTS["trajectory"].get(sub_key, {}))
                        merged.update(v[sub_key])
                        cfg["trajectory"][sub_key] = merged
            else:
                cfg[k] = v
    return cfg


def save(cfg: dict, path: str) -> None:
    """Write cfg as a pretty-printed JSON file."""
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(cfg, fh, indent=2)
        fh.write("\n")


def make_trajectory(cfg: dict, wind_ned):
    """
    Build a TrajectoryController from the ``cfg["trajectory"]`` section.

    Parameters
    ----------
    cfg      : full mediator config dict (as returned by load/defaults)
    wind_ned : array-like [3] — ambient wind NED [m/s]

    Returns
    -------
    TrajectoryController instance (HoldTrajectory or DeschutterTrajectory).

    Note: body_z_eq0 is no longer a parameter — orbit tracking runs inside
    Mode_RAWES (mediator), not in the trajectory planner.

    Example
    -------
    >>> cfg = defaults()
    >>> cfg["trajectory"]["type"] = "deschutter"
    >>> traj = make_trajectory(cfg, [0,10,0])  # East wind in NED
    """
    import sys, os
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from planner import HoldPlanner, DeschutterPlanner, WindEstimator

    import numpy as _np
    traj_cfg  = cfg.get("trajectory", {})
    traj_type = traj_cfg.get("type", "hold")

    if traj_type == "deschutter":
        # config.load() always starts from DEFAULTS so every key is guaranteed
        # present.  Use direct key access — no fallback values here so that
        # a misconfigured (incomplete) dict produces a clear KeyError rather
        # than silently using a wrong default.
        p = traj_cfg["deschutter"]
        _xi = p["xi_reel_in_deg"]
        # WindEstimator seeded from the ground station anemometer reading.
        # Provides wind direction immediately (seed), then converges toward
        # the orbital GPS estimate over the first few pumping cycles.
        estimator = WindEstimator(seed_wind_ned=_np.asarray(wind_ned, dtype=float))
        return DeschutterPlanner(
            t_reel_out          = float(p["t_reel_out"]),
            t_reel_in           = float(p["t_reel_in"]),
            t_transition        = float(p["t_transition"]),
            v_reel_out          = float(p["v_reel_out"]),
            v_reel_in           = float(p["v_reel_in"]),
            tension_out         = float(p["tension_out"]),
            tension_in          = float(p["tension_in"]),
            wind_estimator      = estimator,
            xi_reel_in_deg      = float(_xi) if _xi is not None else None,
            tension_kp          = float(p["tension_kp"]),
            tension_ki          = float(p["tension_ki"]),
            col_min_rad         = float(p["col_min_rad"]),
            col_min_reel_in_rad = float(p["col_min_reel_in_rad"]),
            col_max_rad         = float(p["col_max_rad"]),
            t_hold_s            = float(p.get("t_hold_s", 0.0)),
            warm_coll_rad       = float(p.get("warm_coll_rad", -0.20)),
        )

    if traj_type == "landing":
        from landing_planner import LandingPlanner
        p = traj_cfg["landing"]
        anchor = cfg.get("anchor_ned", [0.0, 0.0, 0.0])
        return LandingPlanner(
            initial_body_z   = None,   # captured from first step's state_pkt["body_z"]
            v_land           = float(p["v_land"]),
            col_cruise       = float(p["col_cruise"]),
            min_tether_m     = float(p["min_tether_m"]),
            anchor_ned       = _np.asarray(anchor, dtype=float),
            tension_target_n = float(p["tension_target_n"]),
            k_winch          = float(p["k_winch"]),
            v_pay_out        = float(p["v_pay_out"]),
            kp_vz            = float(p["kp_vz"]),
            col_min_rad      = float(p["col_min_rad"]),
            col_max_rad      = float(p["col_max_rad"]),
        )

    return HoldPlanner()
