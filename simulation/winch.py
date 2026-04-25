"""
winch.py — Ground-station Winch Controller.

Two-layer design (program mode — requires set_phase):

  Layer 1 — Nominal program (efficiency-optimised):
      reel-out : reel at v_reel_out until delta_l metres have been paid out, then hold
      reel-in  : reel at v_reel_in  back to the length recorded at reel-out start
      transition / hold : stop

      The cycle is guaranteed to close — reel-in target is exactly the length
      recorded when reel-out began, regardless of safety-layer speed reductions.

  Layer 2 — Safety speed multiplier (tension-reactive, layered on top):
      reel-out : full speed below T_soft_max, linear ramp to zero at T_hard_max
      reel-in  : full speed above T_soft_min, linear ramp to zero at T_hard_min

  Call set_phase(phase) at 10 Hz from the ground controller.
  Call step(T_actual, dt) at 400 Hz.

Legacy 3-zone mode (no set_phase needed):
  When set_phase() is never called, falls back to the original proportional
  tension regulator for callers that don't have phase knowledge (landing, etc.).
"""


def _ramp_down(T: float, T_soft: float, T_hard: float) -> float:
    """Safety factor: 1.0 below T_soft, linear to 0.0 at T_hard (T_hard > T_soft)."""
    if T <= T_soft:
        return 1.0
    if T >= T_hard:
        return 0.0
    return (T_hard - T) / (T_hard - T_soft)


def _ramp_up(T: float, T_soft: float, T_hard: float) -> float:
    """Safety factor: 1.0 above T_soft, linear to 0.0 at T_hard (T_hard < T_soft)."""
    if T >= T_soft:
        return 1.0
    if T <= T_hard:
        return 0.0
    return (T - T_hard) / (T_soft - T_hard)


class WinchController:
    """
    Two-layer winch controller.

    Program-mode parameters
    -----------------------
    rest_length   : float  initial unstretched tether length [m]
    delta_l       : float  symmetric reel-out / reel-in distance per cycle [m]
    v_reel_out    : float  nominal reel-out speed [m/s]  (generator optimal RPM point)
    v_reel_in     : float  nominal reel-in speed [m/s]   (fast; maximise cycle rate)
    T_soft_max    : float  start slowing during reel-out [N]
    T_hard_max    : float  stop during reel-out — tether safety ceiling [N]
    T_soft_min    : float  start slowing during reel-in [N]  (slack warning)
    T_hard_min    : float  stop during reel-in — slack floor [N]
    min_length    : float  hard floor on rest_length [m]
    max_length    : float  hard ceiling on rest_length [m]

    Legacy 3-zone parameters (used when set_phase is never called)
    --------------------------------------------------------------
    T_min_n       : float  reel-in threshold [N]
    T_max_n       : float  reel-out threshold [N]
    v_reel_in_ms  : float  max reel-in speed [m/s]
    v_reel_out_ms : float  max reel-out speed [m/s]
    kp            : float  proportional gain [(m/s)/N]
    """

    def __init__(
        self,
        rest_length:    float,
        # ── program-mode ──────────────────────────────────────────────
        delta_l:        float = 12.0,
        v_reel_out:     float = 0.40,
        v_reel_in:      float = 0.80,
        T_soft_max:     float = 470.0,
        T_hard_max:     float = 496.0,
        T_soft_min:     float =  30.0,
        T_hard_min:     float =  10.0,
        T_reel_in_start:float = 200.0,
        # ── shared ────────────────────────────────────────────────────
        min_length:     float =   2.0,
        max_length:     float = 300.0,
        # ── legacy 3-zone ─────────────────────────────────────────────
        T_min_n:        float = 150.0,
        T_max_n:        float = 496.0,
        v_reel_in_ms:   float =   0.4,
        v_reel_out_ms:  float =   0.4,
        kp:             float =  0.01,
    ):
        self.rest_length    = float(rest_length)

        self._delta_l       = float(delta_l)
        self._v_out         = float(v_reel_out)
        self._v_in          = float(v_reel_in)
        self._T_soft_max    = float(T_soft_max)
        self._T_hard_max    = float(T_hard_max)
        self._T_soft_min    = float(T_soft_min)
        self._T_hard_min    = float(T_hard_min)
        self._T_reel_in_start = float(T_reel_in_start)
        self._min_length    = float(min_length)
        self._max_length    = float(max_length)

        # legacy
        self._T_min         = float(T_min_n)
        self._T_max         = float(T_max_n)
        self._v_in_leg      = float(v_reel_in_ms)
        self._v_out_leg     = float(v_reel_out_ms)
        self._kp            = float(kp)

        # program state
        self._phase         : str | None = None
        self._start_length  : float = float(rest_length)

    # ── phase control (10 Hz from ground controller) ───────────────────────

    def set_phase(self, phase: str) -> None:
        """
        Notify the winch of the current pumping phase.  Call at 10 Hz.

        Captures rest_length at the start of reel-out so reel-in can return
        to exactly the same length, guaranteeing a closed cycle.
        """
        if phase == self._phase:
            return
        if phase == "reel-out":
            self._start_length = self.rest_length
        self._phase = phase

    # ── 400 Hz step ────────────────────────────────────────────────────────

    def step(self, T_actual_n: float, dt: float) -> None:
        """
        Advance winch by one 400 Hz step.

        Parameters
        ----------
        T_actual_n : float  tether tension from load cell [N]
        dt         : float  timestep [s]
        """
        T = float(T_actual_n)
        dt = float(dt)

        if self._phase is None:
            speed = self._legacy_speed(T)
        elif self._phase == "reel-out":
            speed = self._reel_out_speed(T, dt)
        elif self._phase == "reel-in":
            speed = self._reel_in_speed(T, dt)
        else:
            speed = 0.0   # "transition" or "hold"

        new_length = self.rest_length + speed * dt
        self.rest_length = max(self._min_length, min(self._max_length, new_length))

    # ── private helpers ────────────────────────────────────────────────────

    def _reel_out_speed(self, T: float, dt: float) -> float:
        target    = self._start_length + self._delta_l
        remaining = target - self.rest_length
        if remaining <= 0.0:
            return 0.0
        speed = min(self._v_out, remaining / dt)
        return speed * _ramp_down(T, self._T_soft_max, self._T_hard_max)

    def _reel_in_speed(self, T: float, dt: float) -> float:
        if T > self._T_reel_in_start:
            return 0.0   # wait for AP to reach low-tension mode
        remaining = self.rest_length - self._start_length
        if remaining <= 0.0:
            return 0.0
        speed = min(self._v_in, remaining / dt)
        return -speed * _ramp_up(T, self._T_soft_min, self._T_hard_min)

    def _legacy_speed(self, T: float) -> float:
        error_high = T - self._T_max
        error_low  = self._T_min - T
        if error_high > 0.0:
            return min(self._kp * error_high, self._v_out_leg)
        if error_low > 0.0:
            return -min(self._kp * error_low, self._v_in_leg)
        return 0.0

    # ── properties ─────────────────────────────────────────────────────────

    @property
    def tether_length_m(self) -> float:
        """Current tether rest length [m] — encoder reading."""
        return self.rest_length

    @property
    def phase(self) -> str | None:
        """Current phase label, or None in legacy mode."""
        return self._phase
