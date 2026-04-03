"""
trajectory.py — Pluggable trajectory controllers for the RAWES pumping cycle.

Models the offboard MAVLink trajectory planner that runs on a ground station
or companion computer and sends commands to the custom ArduPilot Mode_RAWES.

Protocol
--------
Two packet types cross the MAVLink boundary:

    STATE packet  (ground station assembles each cycle — MAVLink streams + local sensors):
        "pos_ned"        np.ndarray [3]  — hub position NED [m]        (LOCAL_POSITION_NED)
        "vel_ned"        np.ndarray [3]  — hub velocity NED [m/s]      (LOCAL_POSITION_NED)
        "omega_spin"     float           — rotor spin rate [rad/s]      (ESC_STATUS rpm × gear ratio)
        "body_z"         np.ndarray [3]  — rotor axis NED unit vector   (ATTITUDE_QUATERNION col 2)
        "tension_n"      float           — tether tension [N]           (winch load cell — local)
        "tether_length_m" float          — tether rest length [m]       (winch encoder — local)

    Planner-internal quantities (never on wire):
        collective_rad  float  — previous step collective [rad]  — passed as kwarg to WindEstimator
        phase           str    — current phase name              — passed as kwarg to WindEstimator
        t_free          float  — elapsed free-flight [s]         — planner internal clock

    COMMAND packet  (planner → Pixhawk, ~10 Hz):
        "attitude_q"      np.ndarray [4]  (w,x,y,z)                (SET_ATTITUDE_TARGET quaternion)
            Desired disk orientation in NED.
            [1,0,0,0] (identity) = tether-aligned natural orbit — Mode_RAWES tracks
            the tether direction at 400 Hz without any planner correction.
            Non-identity = desired body_z is quat_apply(attitude_q, [0,0,-1]) in NED.
            Mode_RAWES rate-limits the slew internally (body_z_slew_rate_rad_s).
        "thrust"          float [0..1]                              (SET_ATTITUDE_TARGET thrust)
            Normalised collective, direct output of the tension PI:
              collective_rad = kP * err + kI * integral
              thrust = clamp((collective_rad - col_min) / (col_max - col_min), 0, 1)
            Mode_RAWES calls set_throttle_out(thrust) — direct passthrough, no conversion.
            Simulation mediator denormalises: collective_rad = col_min + thrust*(col_max-col_min).
        "phase"           str — telemetry label ("hold"|"reel-out"|"reel-in"). Not on wire.

    Winch command  (planner → WinchController, ground-local link):
        "winch_speed_ms"  float [m/s]                               (MAV_CMD_DO_WINCH RATE_CONTROL)
            +ve = pay out, −ve = reel in, 0 = hold.
            Passed to WinchController.step() — the Pixhawk is not involved.

Mode_RAWES responsibilities (NOT in this file):
    - Orbit tracking: compute tether-aligned body_z_eq at 400 Hz from current pos
    - Slew body_z_eq toward attitude_q target at body_z_slew_rate_rad_s
    - Attitude error → cyclic tilt  (compute_swashplate_from_state)
    - Collective: set_throttle_out(thrust) — direct passthrough, no PI on Pixhawk
    - Counter-torque motor (inner loop, not commanded by planner)

WinchController responsibilities (NOT in this file — see winch.py):
    - Execute winch_speed_ms commands
    - Enforce tension safety limit (stop paying out if tension too high)
    - Write tension_n and tether_length_m into the STATE packet each step

Available controllers
---------------------
    HoldPlanner()
        Returns natural equilibrium command every step.
        attitude_q = identity → Mode_RAWES stays tether-aligned through orbit tracking.
        Use for: test_closed_loop_60s.py

    DeschutterPlanner(t_reel_out, t_reel_in, t_transition,
                         v_reel_out, v_reel_in, tension_out, tension_in,
                         wind_estimator, xi_reel_in_deg=55.0,
                         tension_kp=5e-4, tension_ki=1e-4,
                         col_min_rad=-0.28, col_max_rad=0.0,
                         tension_max_n=None)
        De Schutter (2018) pumping cycle.
        Owns the tension PI internally; outputs normalised thrust [0..1].
        Reel-out: identity attitude_q (natural tether-aligned orbit).
        Reel-in:  reel-in attitude_q at xi_reel_in_deg from wind_estimator.wind_dir_ned.
        The estimator starts from its seed (ground anemometer) and converges toward
        the orbital GPS estimate over the first few cycles.
        xi_reel_in_deg=80  — validated optimal (default); +24% net energy vs 55°.
        xi_reel_in_deg=None — no tilt change; identity attitude_q all cycle.
        Use for: test_deschutter_cycle.py (xi=80), test_pumping_cycle.py (xi=None)

Wind estimation
---------------
    WindEstimator(seed_wind_ned, window_s=60.0, K_drive=1.4, K_drag=0.01786)
        Estimates wind direction and in-plane speed from STATE packets.

        seed_wind_ned — initial wind vector from the ground station anemometer.
            Used immediately as the wind_dir_ned estimate before the orbital
            GPS average has converged.  This is what the planner uses at first.

        wind_dir_ned  — unit vector in wind direction (NED, horizontal).
            Before ready: returns normalised seed_wind_ned direction.
            After ready: derived from the rolling mean horizontal hub position.
            Over one orbit the hub's mean horizontal position points downwind
            from the anchor.  Requires no extra sensors beyond pos_ned.

        v_inplane_ms  — in-plane wind speed component [m/s].
            Derived from rotor spin rate via the autorotation torque balance:
            v_inplane = omega_spin² × K_drag / K_drive.
            This is the wind component perpendicular to the rotor axle.
            Requires omega_spin in the STATE packet.
"""

import math
import numpy as np
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from controller import TensionPI


# ---------------------------------------------------------------------------
# Quaternion utilities (module-level, importable)
# ---------------------------------------------------------------------------

#: Quaternion identity [w, x, y, z] — represents no rotation.
Q_IDENTITY: np.ndarray = np.array([1.0, 0.0, 0.0, 0.0])


def quat_from_vectors(v1: np.ndarray, v2: np.ndarray) -> np.ndarray:
    """
    Quaternion [w, x, y, z] that rotates unit vector *v1* to unit vector *v2*.

    The quaternion satisfies:  quat_apply(q, v1) ≈ v2.
    Both vectors are normalised internally.
    """
    v1 = np.asarray(v1, dtype=float)
    v2 = np.asarray(v2, dtype=float)
    v1 = v1 / np.linalg.norm(v1)
    v2 = v2 / np.linalg.norm(v2)
    c = float(np.dot(v1, v2))
    if c >= 1.0 - 1e-9:
        return Q_IDENTITY.copy()
    if c <= -1.0 + 1e-9:
        # Anti-parallel: 180° rotation around any perpendicular axis
        perp = np.array([1.0, 0.0, 0.0]) if abs(v1[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        axis = np.cross(v1, perp)
        axis = axis / np.linalg.norm(axis)
        return np.array([0.0, axis[0], axis[1], axis[2]])
    axis = np.cross(v1, v2)
    axis = axis / np.linalg.norm(axis)
    angle = math.acos(c)
    s = math.sin(angle / 2.0)
    return np.array([math.cos(angle / 2.0), s * axis[0], s * axis[1], s * axis[2]])


def quat_apply(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """
    Rotate vector *v* by quaternion *q* = [w, x, y, z].

    Equivalent to:  q ⊗ [0, v] ⊗ q*
    """
    w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    v = np.asarray(v, dtype=float)
    xyz = np.array([x, y, z])
    t = 2.0 * np.cross(xyz, v)
    return v + w * t + np.cross(xyz, t)


def quat_is_identity(q: np.ndarray, atol: float = 1e-6) -> bool:
    """Return True if *q* is the identity quaternion within *atol*."""
    return abs(float(q[0]) - 1.0) < atol and float(np.linalg.norm(q[1:])) < atol


# ---------------------------------------------------------------------------
# WindEstimator
# ---------------------------------------------------------------------------

class WindEstimator:
    """
    Estimates wind direction and in-plane speed from STATE packets.

    Uses two methods that require no hardware beyond what is already present:

    **Direction — orbital mean position (Method 2):**
    Over one full orbit the hub's mean horizontal position points downwind from
    the anchor.  This is because the rotor lives on the downwind side of the
    anchor at all times; the orbit is symmetric about the wind direction, so
    the mean horizontal position is the mean tether azimuth, which equals the
    wind direction.  Only pos_enu is needed.

    **In-plane speed — rotor spin rate (Method 1):**
    The rotor is in autorotation equilibrium when aerodynamic drive torque equals
    drag torque:
        Q_drive(v_inplane) = Q_drag(omega_spin)
        → v_inplane = omega_spin² × K_drag / K_drive
    v_inplane is the wind component perpendicular to the rotor axle.  To convert
    to full wind speed: v_wind ≈ v_inplane / sin(xi), where xi is the disk tilt
    from wind (available from body_z and wind_dir_enu).
    Requires omega_spin in the STATE packet.

    Parameters
    ----------
    seed_wind_ned : array [3] — ground station anemometer reading NED [m/s].
                  Used as wind_dir_ned before the orbital GPS estimate converges.
                  Only direction is used; magnitude does not matter.
    window_s    : float — rolling average window [s].  Should be ≥ one orbit
                  period (~60 s at default equilibrium).  Default: 60.0.
    min_samples : int   — minimum samples required before returning estimates.
    max_samples : int   — hard cap on buffer length.  Oldest entries evicted when
                  full.  Prevents O(n²) behaviour when update() is called at the
                  physics rate (400 Hz) rather than the planner rate (~10 Hz).
                  Default: 600 (60 s at 10 Hz).
    K_drive     : float — autorotation drive constant [N·m·s/m].
                  Default matches beaupoil_2026 rotor (K_DRIVE_SPIN in mediator).
    K_drag      : float — autorotation drag constant [N·m·s²/rad²].
                  Default matches beaupoil_2026 rotor (K_DRAG_SPIN in mediator).
    """

    def __init__(
        self,
        seed_wind_ned: np.ndarray,
        window_s:    float = 60.0,
        min_samples: int   = 20,
        max_samples: int   = 600,
        K_drive:     float = 1.4,
        K_drag:      float = 0.01786,
    ):
        seed = np.asarray(seed_wind_ned, dtype=float)
        seed_horiz = np.array([seed[0], seed[1], 0.0])
        seed_norm = float(np.linalg.norm(seed_horiz))
        if seed_norm < 1e-6:
            raise ValueError(
                "WindEstimator: seed_wind_ned has no horizontal component — "
                "cannot determine initial wind direction"
            )
        self._seed_dir    = seed_horiz / seed_norm   # unit vector, NED horizontal
        self._window_s    = float(window_s)
        self._min_samples = int(min_samples)
        self._max_samples = int(max_samples)
        self._K_drive     = float(K_drive)
        self._K_drag      = float(K_drag)
        self._t           = 0.0   # internal monotonic clock [s]
        # Buffer entries: (t, pos_ned [3], omega_spin, body_z [3]|None, collective_rad|None, phase|None)
        self._buf: "list[tuple]" = []
        # Per-step property cache — invalidated on every update() call.
        # DeschutterPlanner calls several expensive properties (wind_dir_ned,
        # wind_speed_ms, _binned_data) per step; this avoids recomputing them.
        # Each cache entry is a (gen, value) pair; stale when gen != _cache_gen.
        self._cache_gen:  int  = 0
        self._cache_dir:  "tuple[int, np.ndarray] | None"  = None
        self._cache_spd:  "tuple[int, float | None] | None" = None
        self._cache_bins: "tuple[int, list | None] | None"  = None

    def update(self, state: dict, dt: float = 1.0,
               collective_rad: "float | None" = None,
               phase: "str | None" = None) -> None:
        """
        Ingest one STATE packet.  Call every planner step.

        dt advances the internal clock used for window eviction.

        STATE keys used:
            "pos_ned"    np.ndarray [3] — hub position NED (required)
            "omega_spin" float          — rotor spin rate [rad/s]
            "body_z"     np.ndarray [3] — rotor axis NED unit vector (enables wind_speed_ms)

        Planner-internal kwargs (not from sensors — passed by DeschutterPlanner.step()):
            collective_rad : float | None — previous step collective [rad]
            phase          : str | None   — "reel-out" | "reel-in" | "hold"
        """
        self._t    += float(dt)
        pos         = np.asarray(state["pos_ned"], dtype=float).copy()
        omega_spin  = float(state.get("omega_spin", 0.0))
        body_z_raw  = state.get("body_z")
        body_z      = np.asarray(body_z_raw, dtype=float).copy() if body_z_raw is not None else None
        coll        = float(collective_rad) if collective_rad is not None else None
        self._buf.append((self._t, pos, omega_spin, body_z, coll, phase))
        # Evict entries older than window_s
        cutoff = self._t - self._window_s
        self._buf = [e for e in self._buf if e[0] >= cutoff]
        # Enforce hard cap — drop oldest entries if called faster than intended rate
        if len(self._buf) > self._max_samples:
            self._buf = self._buf[-self._max_samples:]
        # Invalidate per-step cache by advancing the generation counter.
        self._cache_gen += 1

    @property
    def ready(self) -> bool:
        """True once enough samples have accumulated to return estimates."""
        return len(self._buf) >= self._min_samples

    @property
    def wind_dir_ned(self) -> np.ndarray:
        """
        Estimated wind direction as a horizontal NED unit vector.

        Before ready: returns the seed direction (ground anemometer).
        After ready: returns the normalised mean horizontal hub position,
        which converges toward the true wind direction over one orbit period.
        Falls back to seed if the orbital mean is degenerate (hub near anchor).
        """
        if self._cache_dir is not None and self._cache_dir[0] == self._cache_gen:
            return self._cache_dir[1]
        result: np.ndarray
        if self.ready:
            positions  = np.array([e[1] for e in self._buf])
            mean_pos   = np.mean(positions, axis=0)
            mean_horiz = np.array([mean_pos[0], mean_pos[1], 0.0])
            norm       = float(np.linalg.norm(mean_horiz))
            result = mean_horiz / norm if norm >= 1.0 else self._seed_dir
        else:
            result = self._seed_dir
        self._cache_dir = (self._cache_gen, result)
        return result

    @property
    def v_inplane_ms(self) -> "float | None":
        """
        Estimated in-plane wind speed [m/s] from rotor spin rate.

        v_inplane is the wind component perpendicular to the rotor axle:
            v_inplane = omega_spin^2 * K_drag / K_drive

        To recover full wind speed use wind_speed_ms (requires body_z in STATE).
        Returns None if not ready or no valid omega_spin samples.
        """
        if not self.ready:
            return None
        spins = [e[2] for e in self._buf if e[2] > 1.0]
        if not spins:
            return None
        omega_mean = float(np.mean(spins))
        return omega_mean ** 2 * self._K_drag / self._K_drive

    @property
    def wind_speed_ms(self) -> "float | None":
        """
        Estimated full wind speed [m/s] at rotor altitude.

        Recovers full wind speed from the in-plane component:
            v_wind = v_inplane / sin(xi)
        where xi is the angle between the rotor axis (body_z) and wind direction.

        Requires "body_z" to be present in STATE packets.  Returns None if
        body_z data is unavailable, xi is near 0 or 180 degrees (degenerate),
        or the estimator is not ready.
        """
        if self._cache_spd is not None and self._cache_spd[0] == self._cache_gen:
            return self._cache_spd[1]
        spd: "float | None" = None
        v_inplane = self.v_inplane_ms
        if v_inplane is not None:
            body_zs = [e[3] for e in self._buf if e[3] is not None]
            if body_zs:
                mean_bz = np.mean(np.array(body_zs), axis=0)
                norm    = float(np.linalg.norm(mean_bz))
                if norm >= 1e-6:
                    mean_bz  = mean_bz / norm
                    wind_dir = self.wind_dir_ned
                    cos_xi   = float(np.clip(np.dot(mean_bz, wind_dir), -1.0, 1.0))
                    sin_xi   = math.sqrt(max(0.0, 1.0 - cos_xi ** 2))
                    if sin_xi >= 0.1:
                        spd = v_inplane / sin_xi
        self._cache_spd = (self._cache_gen, spd)
        return spd

    # ------------------------------------------------------------------
    # Altitude-stratified shear and veer models
    # ------------------------------------------------------------------

    _ALT_BIN_M   = 5.0   # bin width [m]
    _MIN_FIT_BINS = 3     # bins needed before fitting
    _MIN_BIN_SAMPLES = 3  # samples needed per bin to include it

    def _binned_data(self) -> "list[tuple[float, float, float]] | None":
        """
        Return altitude-binned (mean_z, mean_v_wind, mean_azimuth) tuples.

        Requires body_z in the buffer.  Returns None if fewer than
        _MIN_FIT_BINS valid bins are available.

        mean_v_wind  [m/s]  — v_inplane / sin(xi) per sample
        mean_azimuth [deg]  — tether horizontal azimuth (proxy for wind direction)

        Result is cached for the lifetime of one update() call.
        """
        if self._cache_bins is not None and self._cache_bins[0] == self._cache_gen:
            return self._cache_bins[1]

        # Vectorized extraction — avoids per-element Python loops.
        valid_entries = [(e[1], e[2], e[3]) for e in self._buf if e[3] is not None and e[2] > 1.0]
        if not valid_entries:
            self._cache_bins = (self._cache_gen, None)
            return None
        pos_arr   = np.array([p for p, _, _ in valid_entries])   # (n, 3)
        omega_arr = np.array([o for _, o, _ in valid_entries])   # (n,)
        bz_arr    = np.array([b for _, _, b in valid_entries])   # (n, 3)

        z_arr = -pos_arr[:, 2]                                   # altitude [m]
        ok    = z_arr >= 0.5
        if not np.any(ok):
            self._cache_bins = (self._cache_gen, None)
            return None
        pos_arr, omega_arr, bz_arr, z_arr = pos_arr[ok], omega_arr[ok], bz_arr[ok], z_arr[ok]

        bz_norms = np.linalg.norm(bz_arr, axis=1)               # (n,)
        ok2      = bz_norms >= 1e-6
        bz_arr   = bz_arr[ok2] / bz_norms[ok2, np.newaxis]
        omega_arr, z_arr, pos_arr = omega_arr[ok2], z_arr[ok2], pos_arr[ok2]

        wind_dir = self.wind_dir_ned                             # (3,)
        cos_xi   = np.clip(bz_arr @ wind_dir, -1.0, 1.0)        # (n,)
        sin_xi   = np.sqrt(np.maximum(0.0, 1.0 - cos_xi ** 2))  # (n,)
        ok3      = sin_xi >= 0.1
        omega_arr, z_arr, pos_arr, sin_xi = omega_arr[ok3], z_arr[ok3], pos_arr[ok3], sin_xi[ok3]

        v_wind   = (omega_arr ** 2 * self._K_drag / self._K_drive) / sin_xi  # (n,)
        az       = np.degrees(np.arctan2(pos_arr[:, 1], pos_arr[:, 0]))      # (n,)
        bin_idx  = (z_arr / self._ALT_BIN_M).astype(int)                     # (n,)

        result_list = []
        for b in np.unique(bin_idx):
            mask = bin_idx == b
            if np.sum(mask) < self._MIN_BIN_SAMPLES:
                continue
            result_list.append((float(np.mean(z_arr[mask])),
                                 float(np.mean(v_wind[mask])),
                                 float(np.mean(az[mask]))))

        result = result_list if len(result_list) >= self._MIN_FIT_BINS else None
        self._cache_bins = (self._cache_gen, result)
        return result

    @property
    def shear_alpha(self) -> "float | None":
        """
        Power-law shear exponent fitted to buffered (altitude, v_wind) data.

        Fits: log(v) = alpha * log(z) + const  (log-linear regression).

        Returns None if insufficient altitude coverage (< _MIN_FIT_BINS bins).
        Typically available after 3-5 pumping cycles.
        """
        data = self._binned_data()
        if data is None:
            return None
        zs = np.array([d[0] for d in data])
        vs = np.array([d[1] for d in data])
        if np.any(zs <= 0) or np.any(vs <= 0):
            return None
        alpha, _ = np.polyfit(np.log(zs), np.log(vs), 1)
        return float(alpha)

    @property
    def veer_rate_deg_per_m(self) -> "float | None":
        """
        Wind veer rate [degrees per metre altitude] fitted to buffered data.

        Fits: azimuth = veer_rate * z + const  (linear regression).

        Returns None if insufficient altitude coverage.
        A non-zero value means wind direction rotates with altitude.
        """
        data = self._binned_data()
        if data is None:
            return None
        zs  = np.array([d[0] for d in data])
        azs = np.array([d[2] for d in data])
        veer_rate, _ = np.polyfit(zs, azs, 1)
        return float(veer_rate)

    def wind_speed_at(self, altitude_m: float) -> "float | None":
        """
        Estimated wind speed [m/s] at the given altitude.

        Uses the fitted shear model if available:
            v(z) = v_ref * (z / z_ref) ** alpha
        where z_ref and v_ref are the mean altitude and speed from the buffer.

        Falls back to wind_speed_ms (bulk mean) if the shear model is not
        yet fitted.  Returns None if no speed estimate is available.
        """
        v_ref = self.wind_speed_ms
        if v_ref is None:
            return None
        data  = self._binned_data()
        if data is None:
            return v_ref
        z_ref = float(np.mean([d[0] for d in data]))
        if z_ref < 0.5:
            return v_ref
        alpha = self.shear_alpha
        if alpha is None:
            return v_ref
        z = max(float(altitude_m), 0.5)
        return v_ref * (z / z_ref) ** alpha

    def wind_dir_at(self, altitude_m: float) -> np.ndarray:
        """
        Estimated wind direction at the given altitude as a horizontal NED unit vector.

        Applies the fitted veer model on top of wind_dir_ned:
            azimuth(z) = azimuth_ref + veer_rate * (z - z_ref)
        where z_ref is the mean altitude of the buffer.

        Falls back to wind_dir_ned if the veer model is not yet fitted.
        """
        base_dir  = self.wind_dir_ned
        veer_rate = self.veer_rate_deg_per_m
        if veer_rate is None:
            return base_dir
        data  = self._binned_data()
        if data is None:
            return base_dir
        z_ref     = float(np.mean([d[0] for d in data]))
        delta_az  = veer_rate * (float(altitude_m) - z_ref)
        cos_d     = math.cos(math.radians(delta_az))
        sin_d     = math.sin(math.radians(delta_az))
        # Rotate NED horizontal vector around vertical (NED Z-down) axis
        n, e = float(base_dir[0]), float(base_dir[1])
        return np.array([n * cos_d - e * sin_d,
                         n * sin_d + e * cos_d,
                         0.0])


# ---------------------------------------------------------------------------
# Abstract base
# ---------------------------------------------------------------------------

class TrajectoryPlanner:
    """Abstract base class — subclass and implement step()."""

    def step(self, state: dict, dt: float) -> dict:
        """
        Advance one planner step.

        Parameters
        ----------
        state : dict — STATE packet assembled by the ground station each cycle:
            pos_ned          np.ndarray [3] — hub position NED [m]        (LOCAL_POSITION_NED)
            vel_ned          np.ndarray [3] — hub velocity NED [m/s]      (LOCAL_POSITION_NED)
            omega_spin       float          — rotor spin rate [rad/s]      (ESC_STATUS)
            body_z           np.ndarray [3] — rotor axis NED unit vector   (ATTITUDE_QUATERNION)
            tension_n        float          — tether tension [N]           (winch load cell)
            tether_length_m  float          — tether rest length [m]       (winch encoder)
        dt    : float — timestep [s]

        Returns
        -------
        COMMAND packet dict with keys:
            attitude_q, thrust, winch_speed_ms, phase
        """
        raise NotImplementedError(
            f"{type(self).__name__} must implement step()")


# ---------------------------------------------------------------------------
# HoldPlanner
# ---------------------------------------------------------------------------

class HoldPlanner(TrajectoryPlanner):
    """
    Natural equilibrium — no correction, no winch.

    Sends identity attitude_q and zero thrust every step.  Mode_RAWES stays
    tether-aligned through its own orbit-tracking (no planner involvement).

    Use for: closed-loop stability tests where the pumping cycle is inactive.
    """

    def step(self, state: dict, dt: float) -> dict:
        return {
            "attitude_q":     Q_IDENTITY.copy(),
            "thrust":         0.0,
            "winch_speed_ms": 0.0,
            "phase":          "hold",
        }


# ---------------------------------------------------------------------------
# DeschutterPlanner
# ---------------------------------------------------------------------------

class DeschutterPlanner(TrajectoryPlanner):
    """
    De Schutter (2018) reel-out / reel-in pumping cycle.

    The planner manages only mission-level decisions: phase timing, tension
    setpoints, winch speed, and the reel-in tilt target.  Orbit tracking and
    body_z slewing run entirely inside Mode_RAWES — the planner only sends the
    target attitude quaternion and thrust setpoint.

    The reel-in quaternion is recomputed each step from wind_estimator.wind_dir_ned.
    Before the orbital GPS estimate converges, the estimator returns its seed
    direction (ground anemometer reading).  The planner never has direct access
    to the simulated wind vector.

    Reel-out phase:
        attitude_q = identity  →  Mode_RAWES stays tether-aligned naturally.
        thrust = tension_out / tension_max_n.
        winch_speed_ms = +v_reel_out.

    Reel-in phase:
        attitude_q = reel-in quaternion (body_z at xi_reel_in_deg from wind).
        thrust = tension_in / tension_max_n.
        winch_speed_ms = −v_reel_in.

    Parameters
    ----------
    t_reel_out      : float — reel-out phase duration [s]
    t_reel_in       : float — reel-in phase duration [s]
    t_transition    : float — stored for reference; slewing handled by Mode_RAWES
    v_reel_out      : float — winch pay-out speed [m/s]
    v_reel_in       : float — winch reel-in speed [m/s]
    tension_out     : float — reel-out tension setpoint [N]
    tension_in      : float — reel-in tension setpoint [N]
    wind_estimator  : WindEstimator — provides wind direction for reel-in quaternion.
        Seed (ground anemometer) is used until orbital GPS estimate converges.
    xi_reel_in_deg  : float or None
        Angle between body_z and wind during reel-in [degrees].
        80.0 (default) — validated optimal; +24% net energy vs 55°. BEM valid to ~85°.
        None — no tilt change; identity attitude_q all cycle.
    tension_max_n   : float or None
        Max tension for thrust normalisation.  None → use tension_out.
    """

    def __init__(
        self,
        t_reel_out:           float,
        t_reel_in:            float,
        t_transition:         float,
        v_reel_out:           float,
        v_reel_in:            float,
        tension_out:          float,
        tension_in:           float,
        wind_estimator:       WindEstimator,
        xi_reel_in_deg:       "float | None" = 80.0,
        tension_kp:           float = 5e-4,
        tension_ki:           float = 1e-4,
        col_min_rad:          float = TensionPI.COLL_MIN_RAD,
        col_min_reel_in_rad:  "float | None" = None,
        col_max_rad:          float = TensionPI.COLL_MAX_RAD,
    ):
        self._t_reel_out    = float(t_reel_out)
        self._t_reel_in     = float(t_reel_in)
        self._t_transition  = float(max(t_transition, 1e-6))
        self._v_reel_out    = float(v_reel_out)
        self._v_reel_in     = float(v_reel_in)
        self._tension_out   = float(tension_out)
        self._tension_in    = float(tension_in)
        self._t_cycle       = self._t_reel_out + self._t_reel_in
        self._col_min_rad   = float(col_min_rad)
        # Reel-in may need a less-negative minimum to stay above zero-thrust
        # when body_z is tilted toward vertical (xi_reel_in_deg).
        self._col_min_reel_in_rad = float(col_min_reel_in_rad) if col_min_reel_in_rad is not None else float(col_min_rad)
        self._col_max_rad   = float(col_max_rad)
        self._xi_reel_in_deg: "float | None" = float(xi_reel_in_deg) if xi_reel_in_deg is not None else None
        self._wind_estimator = wind_estimator
        self._t_free         = 0.0   # internal elapsed free-flight time [s]

        # Tension PI — owned by the planner (raws_mode.md §3.2)
        # Single controller whose setpoint changes at the phase boundary so the
        # integral state carries across smoothly (no warm-start discontinuity).
        self._tension_ctrl = TensionPI(
            setpoint_n=float(tension_out),
            kp=float(tension_kp),
            ki=float(tension_ki),
            coll_min=float(col_min_rad),
            coll_max=float(col_max_rad),
        )

        # Snapshots of PI integral at each phase boundary, used to ramp
        # collective smoothly across transitions without step jumps.
        self._reel_out_integral: float = self._tension_ctrl._integral
        self._reel_in_integral:  float = self._tension_ctrl._integral

        # Previous step collective [rad] — passed to WindEstimator.update() each step
        # so it can use collective as a proxy for operating point (future: K_drive correction).
        self._prev_collective: float = 0.0

        # Wind estimate refresh rate — how many planner steps between recomputing
        # wind_dir_at / wind_speed_at.  Wind changes on a ~60 s timescale so 10 Hz
        # (every 40 steps at 400 Hz) is more than adequate.  The estimator still
        # ingests every STATE packet; only the expensive property lookups are throttled.
        _WIND_UPDATE_EVERY   = 10
        self._wind_update_every: int  = _WIND_UPDATE_EVERY
        # Initialise to N-1 so the first update fires on step 1, populating
        # _wind_vec_cached immediately rather than waiting N steps.
        self._wind_step_count:  int   = _WIND_UPDATE_EVERY - 1
        self._wind_vec_cached: "np.ndarray | None" = None   # last computed wind vector

        # Initial reel-in attitude from estimator seed (ground anemometer direction).
        # Updated at wind-estimate rate as the orbital GPS estimate converges.
        self._attitude_q_reel_in: "np.ndarray | None" = self._q_from_wind(
            wind_estimator.wind_dir_ned)

    # ------------------------------------------------------------------
    def _q_from_wind(self, wind_ned: np.ndarray) -> "np.ndarray | None":
        """Compute reel-in quaternion from a wind direction vector (NED)."""
        if self._xi_reel_in_deg is None:
            return None
        xi_rad     = math.radians(self._xi_reel_in_deg)
        wind_horiz = np.array([float(wind_ned[0]), float(wind_ned[1]), 0.0])
        wh_norm    = np.linalg.norm(wind_horiz)
        if wh_norm < 1e-6:
            return None
        wind_horiz = wind_horiz / wh_norm
        # In NED: up = [0,0,-1].  body_z_target tilts xi from horizontal toward up.
        body_z_target = (
            math.cos(xi_rad) * wind_horiz
            + math.sin(xi_rad) * np.array([0.0, 0.0, -1.0])
        )
        # quat_from_vectors: rotation from NED up [0,0,-1] to body_z_target
        return quat_from_vectors(np.array([0.0, 0.0, -1.0]), body_z_target)

    # ------------------------------------------------------------------
    def step(self, state: dict, dt: float) -> dict:
        self._t_free += dt

        # Determine phase before calling WindEstimator so we can pass it as context.
        t_cyc     = self._t_free % self._t_cycle
        phase_out = t_cyc < self._t_reel_out

        # Read sensor values from STATE — tension and tether_length are local ground
        # station measurements (load cell + encoder), not MAVLink streams.
        tension_n       = float(state.get("tension_n", 0.0))
        tether_length_m = state.get("tether_length_m")  # noqa: F841 — reserved for future use

        # Update wind estimator with full STATE plus planner-internal context.
        # collective_rad from the PREVIOUS step is the best available operating-point
        # proxy (current step's collective hasn't been computed yet).
        self._wind_estimator.update(state, dt,
                                    collective_rad=self._prev_collective,
                                    phase="reel-out" if phase_out else "reel-in")

        # Refresh reel-in quaternion at wind-estimate rate (10 Hz by default).
        # Wind changes on a ~60 s timescale — no benefit to recomputing at 400 Hz.
        # The estimator still ingests every STATE packet; only the expensive
        # property lookups (wind_dir_at, wind_speed_at) are throttled.
        self._wind_step_count += 1
        if self._wind_step_count >= self._wind_update_every:
            self._wind_step_count = 0
            hub_alt  = -float(np.asarray(state["pos_ned"])[2])
            wind_dir = self._wind_estimator.wind_dir_at(hub_alt)
            v_wind   = self._wind_estimator.wind_speed_at(hub_alt) or self._wind_estimator.v_inplane_ms
            new_wind_vec = wind_dir * v_wind if (v_wind is not None and v_wind > 0.5) else wind_dir
            self._wind_vec_cached = new_wind_vec
            q_new = self._q_from_wind(new_wind_vec)
            if q_new is not None:
                self._attitude_q_reel_in = q_new

        # PI floor: keep col_min_rad throughout both phases.
        # col_min_reel_in_rad is the minimum collective for altitude maintenance at
        # xi_reel_in.  It is NOT used as a hard PI floor because that would force
        # a sudden output jump from the reel-out equilibrium collective to the
        # reel-in minimum the instant the phase changes.  Instead the PI drives
        # collective up naturally as tension drops during the body_z tilt — the
        # winch speed ramp (via alpha) ensures the tether doesn't shorten faster
        # than the disk tilts, so tension drops predictably.
        _col_min_now = self._col_min_rad
        self._tension_ctrl.coll_min = _col_min_now

        # Snapshot the PI integral at each phase boundary so each transition
        # can ramp collective smoothly from the previous equilibrium.
        _just_entered_reel_in  = (not phase_out) and (t_cyc - self._t_reel_out < dt * 1.5)
        _just_entered_reel_out = phase_out and (t_cyc < dt * 1.5)
        if _just_entered_reel_in:
            self._reel_out_integral = self._tension_ctrl._integral
        if _just_entered_reel_out:
            self._reel_in_integral = self._tension_ctrl._integral

        # Transition progress scalars: 0 at phase start → 1 after t_transition.
        # Both are always defined so winch and collective logic can share them.
        if phase_out:
            # alpha_out: reel-in → reel-out transition (body_z slewing back to tether)
            alpha_out = min(1.0, t_cyc / self._t_transition)
            alpha     = 1.0   # unused sentinel
            tension_setpoint = self._tension_in + alpha_out * (self._tension_out - self._tension_in)
        else:
            # alpha: reel-out → reel-in transition (body_z slewing to xi)
            t_into_reel_in = t_cyc - self._t_reel_out
            alpha     = min(1.0, t_into_reel_in / self._t_transition)
            alpha_out = 1.0   # unused sentinel
            tension_setpoint = self._tension_out + alpha * (self._tension_in - self._tension_out)
        self._tension_ctrl.setpoint = tension_setpoint

        if not phase_out and alpha < 1.0:
            # ── Reel-in transition: ramp collective from reel-out equilibrium
            # to col_min_reel_in over t_transition, then hand off to the PI.
            #
            # Without this ramp the PI carries the large reel-out integral into
            # reel-in, causing a sudden collective change when either the integral
            # is reset (step down → tether snap) or left unchanged (step up due to
            # new coll_min floor).  The ramp spreads the mandatory increase from the
            # reel-out equilibrium (~-0.15 rad) to the reel-in minimum (0.079 rad)
            # over t_transition seconds, synchronised with the body_z tilt.
            coll_reel_out = self._reel_out_integral * self._tension_ctrl.ki
            coll_transition = coll_reel_out + alpha * (self._col_min_reel_in_rad - coll_reel_out)
            collective_rad = float(np.clip(coll_transition, _col_min_now, self._col_max_rad))
            # Keep PI integral in sync so it continues smoothly when alpha reaches 1
            self._tension_ctrl._integral = collective_rad / max(self._tension_ctrl.ki, 1e-12)
        elif phase_out and alpha_out < 1.0:
            # ── Reel-out transition: ramp collective from reel-in equilibrium
            # back to the natural reel-out collective over t_transition.
            #
            # At the reel-in → reel-out boundary body_z is still tilted at xi_reel_in
            # (mostly vertical).  The winch has reversed but the disk hasn't re-tilted
            # yet.  Without this ramp:
            #   - tension setpoint jumps to tension_out (200 N)
            #   - PI sees a large error and drives collective hard
            #   - tether is already extending at v_reel_out
            #   - collective overshoot → altitude spike or descent before disk re-tilts
            # The ramp holds collective at approximately the reel-in equilibrium and
            # lets the PI take over smoothly once the disk is back in reel-out geometry.
            coll_reel_in    = self._reel_in_integral * self._tension_ctrl.ki
            coll_reel_out_target = float(np.clip(
                self._tension_ctrl.kp * (self._tension_out - float(tension_n))
                + coll_reel_in,
                _col_min_now, self._col_max_rad,
            ))
            coll_transition = coll_reel_in + alpha_out * (coll_reel_out_target - coll_reel_in)
            collective_rad  = float(np.clip(coll_transition, _col_min_now, self._col_max_rad))
            # Keep PI integral in sync so it continues smoothly when alpha_out reaches 1
            self._tension_ctrl._integral = collective_rad / max(self._tension_ctrl.ki, 1e-12)
        else:
            collective_rad = self._tension_ctrl.update(float(tension_n), dt)

        # Normalise collective → thrust [0..1] for SET_ATTITUDE_TARGET.
        # Always use the full hardware range (col_min_rad..col_max_rad) so that
        # Mode_RAWES can denormalise with fixed parameters regardless of the PI's
        # internal coll_min limit (col_min_reel_in_rad during reel-in).
        col_range = self._col_max_rad - self._col_min_rad
        thrust = float(max(0.0, min(1.0,
            (collective_rad - self._col_min_rad) / col_range if col_range > 1e-9 else 0.5
        )))

        # Winch speed.
        # Both transitions are ramped over t_transition, synchronised with the
        # body_z slerp:
        #   Reel-out → reel-in: ramp 0 → -v_reel_in  (prevents shortening before disk tilts)
        #   Reel-in → reel-out: ramp 0 → +v_reel_out  (prevents extending before disk re-tilts;
        #                                               gives altitude time to stabilise)
        if phase_out:
            winch_speed = self._v_reel_out * alpha_out
        else:
            winch_speed = -(self._v_reel_in * alpha)

        if phase_out and self._attitude_q_reel_in is None:
            attitude_q = Q_IDENTITY.copy()
        elif phase_out:
            attitude_q = Q_IDENTITY.copy()
        else:
            attitude_q = self._attitude_q_reel_in.copy() if self._attitude_q_reel_in is not None else Q_IDENTITY.copy()

        self._prev_collective = collective_rad

        return {
            "attitude_q":     attitude_q,
            "thrust":         thrust,
            "collective_rad": collective_rad,  # raw collective [rad] for direct use by internal controller
            "winch_speed_ms": winch_speed,
            "phase":          "reel-out" if phase_out else "reel-in",
        }
