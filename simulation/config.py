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
    cfg["base_k_ang"] = 0.0
    cfg_path = tmp_path / "mediator_config.json"
    mcfg.save(cfg, cfg_path)
    # pass --config str(cfg_path) to mediator subprocess

Example (mediator):
    import config as mcfg
    cfg = mcfg.load(args.config)   # args.config may be None → pure defaults
"""

import json
import copy
from pathlib import Path

# ---------------------------------------------------------------------------
# Load warmup-settled initial conditions via the shared IC loader (ic.py) so
# the mediator config uses the exact same IC as simtests and the SITL stack.
# The JSON is written only by test_generate_ic.py::test_create_ic.
# ---------------------------------------------------------------------------
import ic as _ic

_ss: dict = _ic.load_ic_dict()

# ---------------------------------------------------------------------------
# Schema and defaults
# ---------------------------------------------------------------------------
DEFAULTS: dict = {
    # ── Rotor definition ─────────────────────────────────────────────────────
    "rotor_definition": "beaupoil_2026",  # built-in name or path to .yaml file

    # ── Environment ──────────────────────────────────────────────────────────
    "wind": [0.0, 10.0, 0.0],          # ambient wind NED [m/s]  (East = NED Y axis)

    # ── Initial hub state ─────────────────────────────────────────────────────
    # Warmup-settled equilibrium from steady_state_starting.json (auto-loaded above).
    # Regenerate by running test_steady_state_hub_does_not_drift.
    "pos0":       _ss["pos"],    # NED [m]
    "vel0":       _ss["vel"],    # NED [m/s]
    "R0":         _ss["R0"],     # full body-to-NED rotation matrix (3x3 list); GPS-friendly yaw; body_z = R0[:, 2]
    "omega_spin": _ss["omega_spin"],  # rotor spin [rad/s]

    # ── Tether & Winch ────────────────────────────────────────────────────────
    "rest_length": _ss["rest_length"],  # unstretched tether length [m]
    "anchor_ned": [0.0, 0.0, 0.0],  # tether anchor position NED [m]
                                     # mirrors RAWES_ANCHOR_LAT/LON/ALT converted to local NED
    "tension_safety_n":  496.0,     # WinchController T_max_n: reel-out threshold [N]
                                     # ≈ 80% of Dyneema SK75 1.9 mm break load (620 N)

    # Optional: replace the elastic tether with a constant-tension model.
    # Set "tether_model": "constant_tension" and supply the desired load.
    # Direction follows hub→anchor geometry; no spring, no damping, no slack.
    # Useful for steady-flight sensitivity studies where tether elasticity is
    # a disturbance rather than a feature under test.
    # "tether_model": "constant_tension",
    # "tether_constant_tension_n": 300.0,

    # ── Startup kinematic ramp ────────────────────────────────────────────────
    # Hub moves at vel0 from launch_pos for startup_damp_seconds, then physics
    # takes over.  Gives EKF time to lock on GPS before free flight.
    #
    # With dual GPS (EK3_SRC1_YAW=2), yaw is known from the first GPS fix and
    # delAngBiasLearned converges with constant-zero gyro (~21 s after arm).
    # A stationary hold (vel0=[0,0,0], ramp_s=0) is the standard pattern.
    #
    # kinematic_vel_ramp_s: vel taper window at end of kinematic (0 = no taper).
    "startup_damp_seconds":         30.0,   # kinematic duration [s]
    "kinematic_vel_ramp_s":         15.0,   # vel ramp-to-zero window at end [s]; 0 = constant vel throughout
    "startup_damp_k_ang":          500.0,   # peak angular drag [N·m·s/rad] during kinematic phase
    # Smooth trapezoidal kinematic motion (overrides the linear vel0 path when
    # kinematic_cruise_speed > 0).  The hub accelerates from rest to
    # kinematic_cruise_speed along the IC yaw heading, cruises, then decelerates
    # back to rest, arriving exactly at pos0 at kinematic exit.  Raised-cosine
    # ramps give continuous (smooth) acceleration with no jerk steps.  This gives
    # the EKF velocity observability during the hold while leaving zero residual
    # position/velocity error at release.
    "kinematic_cruise_speed":        0.0,   # cruise speed [m/s]; 0 = use linear vel0 path
    "kinematic_accel_s":             5.0,   # accel ramp window at start [s]
    "kinematic_decel_s":             5.0,   # decel ramp window at end [s]
    # Kinematic aero behavior:
    #   "locked" (default): legacy full kinematic lock (no physics response)
    #   "nul"            : simplified debug response where acceleration is
    #                        proportional to cyclic body-rate command while
    #                        translation remains kinematically locked.
    "kinematic_aero_mode": "locked",
    "kinematic_nul_rate_gain_rads_per_rad": 0.2,

    # ── Attitude damping ──────────────────────────────────────────────────────
    "base_k_ang": 0.0,              # optional diagnostic angular drag [N·m·s/rad]
                                    # default is no artificial free-flight damping

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

    # ── Post-kinematic cyclic handoff (mediator-side) ───────────────────────
    # Optional blend window after kinematic release to avoid an abrupt first
    # cyclic command from AP. During the window, mediator applies:
    #   tilt = (1-alpha)*IC_tilt + alpha*AP_tilt, alpha=t/blend_s
    # 0.0 disables blending.
    "post_release_cyclic_blend_s": 0.0,
    # IC cyclic trim [rad] used as blend anchor.
    "ic_tilt_lon_rad": float(_ss["trim_tilt_lon"]),
    "ic_tilt_lat_rad": float(_ss["trim_tilt_lat"]),

    # ── Sensor model ──────────────────────────────────────────────────────────
    # Rotor spin speed is an internal simulation state (omega_spin ODE).
    # In hardware, rotor spin speed measurement is handled separately from
    # anti-rotation motor control — the gear coupling keeps the hub stationary
    # without requiring spin feedback; spin measurement is TBD for hardware.
    # Simulation noise: set 0 to use true omega_spin (ideal, recommended for unit tests).
    "spin_sensor_sigma": 0.0,           # Gaussian noise sigma [rad/s]; 0 = ideal

    # ── Winch command socket (pumping stack tests) ────────────────────────────
    # When > 0, mediator opens a UDP server on this port so the test process can
    # send winch set_target commands and receive physics state at ~10 Hz.
    # 0 = disabled (default: trajectory planner owns the winch).
    "winch_cmd_port": 0,

    # ── Dedicated MAVLink log link (mediator-side, optional) ─────────────────
    # Optional second MAVLink client endpoint used by mediator to sample
    # ATTITUDE/ATTITUDE_TARGET/SERVO_OUTPUT_RAW for telemetry CSV logging.
    # Empty string disables this listener.
    "mavlink_log_connection": "",
    # Requested ATTITUDE_TARGET message rate [Hz] over the dedicated link.
    "mavlink_att_target_hz": 100.0,
    # Requested ATTITUDE message rate [Hz] over the dedicated link.
    "mavlink_attitude_hz": 100.0,
    # Requested SERVO_OUTPUT_RAW message rate [Hz] over the dedicated link.
    "mavlink_servo_output_raw_hz": 100.0,
    # Periodic mediator-side diagnostics log interval for dedicated MAVLink rx.
    "mavlink_log_diag_interval_s": 5.0,

    # ── Winch hardware parameters ─────────────────────────────────────────────
    # GovernedWinchController gains (canonical winch — same control law as the
    # pumping simtest test_pump_cycle_lua.py).  cruise velocity + proportional
    # tension governor, jerk-limited (S-curve) speed.
    "winch_kp_tension":      4.0e-4,  # (m/s)/N  governor gain (~0.5 s settling)
    "winch_v_max_out":       1.5,     # m/s  max pay-out speed (governor headroom)
    "winch_v_max_in":        1.5,     # m/s  max reel-in speed (governor headroom)
    "winch_accel_limit_ms2": 2.0,     # m/s^2  acceleration limit
    "winch_jerk_limit_ms3":  10.0,    # m/s^3  jerk limit (S-curve smoothing)
    "winch_tension_tau_s":   0.08,    # s      load-cell low-pass time constant
    "winch_min_length":      2.0,     # m      hard floor (drum stop)
    "winch_hold_tension_n":  300.0,   # N      idle tension target (no socket cmd)

    # ── Trajectory controller ─────────────────────────────────────────────────
    # "type" selects the active controller.  Each type has its own sub-dict so
    # all parameters are self-contained and callers only touch one section.
    #
    # Supported types:
    #   "hold"        — hold tether direction, zero collective, no winch.
    #                   No parameters required.
    #   "deschutter"  — De Schutter (2018) reel-out/reel-in pumping cycle.
    "trajectory": {
        "type": "hold",
        "hold": {},
        "deschutter": {
            "t_hold_s":         0.0,   # hold phase before first reel-out [s]; use ~20s in stack test to settle guidance
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
            "col_min_reel_in_rad":  0.079, # reel-in floor [rad]: altitude floor at xi=80°
            # TensionPI warm-start: thrust output at zero error on first step.
            # Overridden by initial_state["eq_thrust"] when available so PI
            # starts at the exact equilibrium thrust after kinematic exit.
            "warm_coll_rad":       -0.20,  # [rad] integrator seed near reel-out equilibrium
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

    Note: body_z_eq0 is no longer a parameter — guidance runs inside
    Mode_RAWES (mediator), not in the trajectory planner.
    """
    import sys, os
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from planner import HoldPlanner

    return HoldPlanner()
