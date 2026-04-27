"""
winch.py — Ground-station Winch Controller.

Tension-controlled motion-profile design:

  The caller sets a target length and a target tension at ~10 Hz.
  The winch runs a trapezoidal motion profile toward the target length.
  Cruise speed is proportional to tension error:

    Reeling out (paying out, target_length > rest_length):
        v_cruise = kp * max(0, tension_measured - target_tension)
        Generator loads up as tension rises above target; holds when below.

    Reeling in (motoring, target_length < rest_length):
        v_cruise = kp * max(0, target_tension - tension_measured)
        Motor slows when kite resists (high tension); speeds up when slack.

  The motion profile decelerates in time to arrive at zero speed exactly at
  target_length, so there is no overshoot and no abrupt stop.

  Hardware limits:
    v_max_out        : generator max speed (motor rating)
    v_max_in         : motor max reel-in speed
    accel_limit_ms2  : acceleration/deceleration ramp [m/s²]
    min_length       : hard floor — drum never reels past this
    max_length       : hard ceiling

Interface:
    set_target(length_m, tension_n)   10 Hz — from ground planner
    step(tension_measured, dt)        400 Hz
"""


class WinchController:
    """
    Tension-controlled motion-profile winch.

    Parameters
    ----------
    rest_length      : float  initial tether rest length [m]
    kp_tension       : float  speed-per-tension-error gain [(m/s)/N]
    v_max_out        : float  max reel-out (pay-out) speed [m/s]
    v_max_in         : float  max reel-in speed [m/s]
    accel_limit_ms2  : float  acceleration / deceleration limit [m/s²]
    min_length       : float  hard floor on rest_length [m]
    max_length       : float  hard ceiling on rest_length [m]
    """

    def __init__(
        self,
        rest_length:     float,
        kp_tension:      float,
        v_max_out:       float,
        v_max_in:        float,
        accel_limit_ms2: float,
        min_length:      float,
        max_length:      float = 300.0,
    ):
        self.rest_length       = float(rest_length)
        self._kp               = float(kp_tension)
        self._v_max_out        = float(v_max_out)
        self._v_max_in         = float(v_max_in)
        self._accel            = float(accel_limit_ms2)
        self._min_length       = float(min_length)
        self._max_length       = float(max_length)

        self._target_length    = float(rest_length)   # hold by default
        self._target_tension   = 0.0                  # set by first set_target() call
        self._speed            = 0.0                  # current ramped speed [m/s]

        self._energy_out_j     = 0.0   # cumulative generator harvest [J]
        self._energy_in_j      = 0.0   # cumulative motor consumption [J]

    # ── command interface (10 Hz from ground planner) ──────────────────────

    def set_target(self, length_m: float, tension_n: float) -> None:
        """
        Set the winch target length and tension setpoint.  Call at ~10 Hz.

        length_m   : desired tether rest length [m]
        tension_n  : desired tether tension [N] — controls cruise speed
        """
        self._target_length  = float(length_m)
        self._target_tension = float(tension_n)

    # ── 400 Hz step ────────────────────────────────────────────────────────

    def step(self, tension_measured: float, dt: float) -> None:
        """
        Advance winch by one 400 Hz step.

        tension_measured : current load-cell reading [N]
        dt               : timestep [s]
        """
        T  = float(tension_measured)
        dt = float(dt)

        remaining = self._target_length - self.rest_length

        if remaining > 1e-6:
            # Reel out: generator loads up as tension rises above target
            v_cruise = min(self._v_max_out,
                           max(0.0, self._kp * (T - self._target_tension)))
        elif remaining < -1e-6:
            # Reel in: motor backs off as tension rises; boosts when slack
            v_cruise = min(self._v_max_in,
                           max(0.0, self._kp * (self._target_tension - T)))
        else:
            v_cruise = 0.0   # at target — decelerate to stop

        self._speed = self._profile_step(
            self.rest_length, self._target_length,
            self._speed, v_cruise, self._accel, dt,
        )

        new_length = self.rest_length + self._speed * dt
        self.rest_length = max(self._min_length, min(self._max_length, new_length))

        power = T * self._speed
        if power > 0.0:
            self._energy_out_j += power * dt
        elif power < 0.0:
            self._energy_in_j  += -power * dt

    # ── motion profile ─────────────────────────────────────────────────────

    @staticmethod
    def _profile_step(
        x: float, x_target: float,
        v: float, v_cruise: float,
        accel: float, dt: float,
    ) -> float:
        """
        One step of a trapezoidal motion profile.

        x          : current position [m]
        x_target   : target position [m]
        v          : current speed [m/s], signed
        v_cruise   : tension-derived cruise speed magnitude [m/s] >= 0
        accel      : acceleration/deceleration limit [m/s²]
        dt         : timestep [s]

        Returns the new signed speed.  Positive = paying out, negative = reeling in.
        Decelerates early enough to arrive at zero speed exactly at x_target.
        If v_cruise == 0, decelerates smoothly to a stop regardless of position.
        """
        # Tension limit or at-target: decelerate to stop wherever we are.
        if v_cruise == 0.0:
            sign = 1.0 if v > 0.0 else (-1.0 if v < 0.0 else 0.0)
            return sign * max(0.0, abs(v) - accel * dt)

        remaining = x_target - x
        if abs(remaining) < 1e-9:
            return 0.0

        direction = 1.0 if remaining > 0.0 else -1.0

        # Stopping distance from current speed
        d_stop = (v * direction) ** 2 / (2.0 * accel) if accel > 0.0 else 0.0

        if abs(remaining) <= d_stop:
            # In deceleration zone: slow down toward stop
            new_speed = direction * max(0.0, abs(v) - accel * dt)
        elif abs(v) > v_cruise:
            # Over cruise speed (e.g. tension dropped mid-cruise): decelerate toward v_cruise
            new_speed = direction * max(v_cruise, abs(v) - accel * dt)
        else:
            # Accelerate toward cruise speed
            new_speed = direction * min(v_cruise, abs(v) + accel * dt)

        return new_speed

    # ── properties ─────────────────────────────────────────────────────────

    @property
    def speed_ms(self) -> float:
        """Current motor speed [m/s], signed (+ve = paying out)."""
        return self._speed

    @property
    def target_length(self) -> float:
        """Current target tether length [m]."""
        return self._target_length

    @property
    def target_tension(self) -> float:
        """Current target tension [N]."""
        return self._target_tension

    @property
    def energy_out_j(self) -> float:
        """Cumulative generator harvest since construction [J]."""
        return self._energy_out_j

    @property
    def energy_in_j(self) -> float:
        """Cumulative motor consumption since construction [J]."""
        return self._energy_in_j

    @property
    def net_energy_j(self) -> float:
        """Cumulative net energy (harvest minus consumption) [J]."""
        return self._energy_out_j - self._energy_in_j

    def log_fields(self) -> dict:
        """Standard winch state fields for telemetry kwargs."""
        return dict(winch_speed_ms=self._speed)
