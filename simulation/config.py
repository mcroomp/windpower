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
    "wind": [10.0, 0.0, 0.0],          # ambient wind ENU [m/s]

    # ── Initial hub state ─────────────────────────────────────────────────────
    # Warmup-settled equilibrium from test_steady_flight.py.
    # 50 m tether at ~30° elevation; body_z aligned with tether direction.
    "pos0":       [46.258, 14.241, 12.530],      # ENU [m]
    "vel0":       [-0.257,  0.916, -0.093],      # ENU [m/s]
    "body_z":     [0.851018, 0.305391, 0.427206],  # unit vector (axle direction)
    "omega_spin": 20.148,                          # rotor spin [rad/s]

    # ── Tether ────────────────────────────────────────────────────────────────
    "tether_rest_length": 49.949,   # unstretched tether length [m]
    "anchor_enu": [0.0, 0.0, 0.0],  # tether anchor position ENU [m]
                                     # mirrors RAWES_ANCHOR_LAT/LON/ALT converted to local ENU

    # ── Startup kinematic ramp ────────────────────────────────────────────────
    # Hub moves at constant vel0 from launch_pos for startup_damp_seconds,
    # then physics takes over.  Gives EKF time to lock on GPS before free flight.
    "startup_damp_seconds": 30.0,   # kinematic ramp duration [s]
    "startup_damp_k_vel":  200.0,   # peak translational drag [N·s/m]
    "startup_damp_k_ang":  500.0,   # peak angular drag [N·m·s/rad]
    "startup_damp_k_pos": 2000.0,   # peak position spring [N/m]

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

    # ── Mode_RAWES attitude and tension parameters ────────────────────────────
    # tension_max_n: Mode_RAWES maps thrust [0..1] to setpoint via
    #   tension_setpoint_n = thrust × tension_max_n.
    #   The planner normalises against the same value.
    "tension_max_n":           200.0,  # max operating tension [N]
    # body_z_slew_rate_rad_s: max angular slew rate for body_z transitions [rad/s].
    #   At 0.12 rad/s a 0.57 rad (33°) reel-in transition takes ~5 s.
    #   Replaces the blend_alpha ramp that formerly lived in the trajectory planner.
    "body_z_slew_rate_rad_s":    0.12,

    # ── Sensor model ──────────────────────────────────────────────────────────
    "sensor_mode": "tether_relative",   # "tether_relative" | "physical"
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
        "deschutter": {
            "t_reel_out":      30.0,   # reel-out phase duration [s]
            "t_reel_in":       30.0,   # reel-in phase duration [s]
            "t_transition":     5.0,   # body_z blend window at phase boundary [s]
            "v_reel_out":       0.4,   # winch pay-out speed [m/s]
            "v_reel_in":        0.4,   # winch reel-in speed [m/s]
            "tension_out":    200.0,   # reel-out tension setpoint [N]
            "tension_in":      20.0,   # reel-in tension setpoint [N]
            "xi_reel_in_deg":  55.0,   # disk tilt from wind during reel-in [deg]
                                       # None → no tilt change (simple tension cycle)
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
                for sub_key in ("hold", "deschutter"):
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


def make_trajectory(cfg: dict, wind_enu, rest_length_min: float):
    """
    Build a TrajectoryController from the ``cfg["trajectory"]`` section.

    Parameters
    ----------
    cfg            : full mediator config dict (as returned by load/defaults)
    wind_enu       : array-like [3] — ambient wind ENU [m/s]
    rest_length_min: float — tether rest_length floor for reel-in [m]
                     Enforced by Mode_RAWES; passed here for informational use.

    Returns
    -------
    TrajectoryController instance (HoldTrajectory or DeschutterTrajectory).

    Note: body_z_eq0 is no longer a parameter — orbit tracking runs inside
    Mode_RAWES (mediator), not in the trajectory planner.

    Example
    -------
    >>> cfg = defaults()
    >>> cfg["trajectory"]["type"] = "deschutter"
    >>> traj = make_trajectory(cfg, [10,0,0], 49.9)
    """
    import sys, os
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from trajectory import HoldTrajectory, DeschutterTrajectory

    import numpy as _np
    traj_cfg  = cfg.get("trajectory", {})
    traj_type = traj_cfg.get("type", "hold")

    if traj_type == "deschutter":
        p = traj_cfg.get("deschutter", {})
        return DeschutterTrajectory(
            t_reel_out      = float(p.get("t_reel_out",      30.0)),
            t_reel_in       = float(p.get("t_reel_in",       30.0)),
            t_transition    = float(p.get("t_transition",     5.0)),
            v_reel_out      = float(p.get("v_reel_out",       0.4)),
            v_reel_in       = float(p.get("v_reel_in",        0.4)),
            tension_out     = float(p.get("tension_out",    200.0)),
            tension_in      = float(p.get("tension_in",      20.0)),
            wind_enu        = _np.asarray(wind_enu, dtype=float),
            rest_length_min = float(rest_length_min),
            xi_reel_in_deg  = p.get("xi_reel_in_deg", 55.0),
        )

    return HoldTrajectory()
