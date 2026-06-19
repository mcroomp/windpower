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
            # Reel in: reel in faster when tension is low (slack) to maintain
            # minimum tension. Stops when T >= target.
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


# ---------------------------------------------------------------------------
# Velocity-commanded winch with soft tension limits
# ---------------------------------------------------------------------------

class VelocityWinchController:
    """
    Velocity-commanded winch with predictive tension limits and acceleration ramp.

    The caller sets a target velocity at ~10 Hz.  The winch tracks that
    velocity but predicts whether the current tension trend will hit a hard
    limit before the controller can stop. If so, the target speed is reduced
    toward zero early, symmetrically for both tension bounds. An acceleration
    ramp prevents velocity discontinuities.

    Predictive braking:
        Estimate dT/dt from successive tension measurements.
        Predict tension after the current stopping horizon abs(speed)/accel.
        If that prediction would cross T_min or T_max, command zero speed.

    Interface:
        set_velocity(v_ms)           ~10 Hz from planner (+ve=out, -ve=in, 0=hold)
        step(tension_measured, dt)    400 Hz

    Parameters
    ----------
    rest_length      : float  initial tether rest length [m]
    v_max_out        : float  max reel-out (pay-out) speed [m/s]
    v_max_in         : float  max reel-in speed [m/s]
    accel_limit_ms2  : float  acceleration / deceleration limit [m/s^2]
    T_min            : float  lower tension hard limit [N]
    T_max            : float  upper tension hard limit [N]
    soft_zone_n      : float  predictive guard band before the hard limit [N]
    min_length       : float  hard floor on rest_length [m]
    max_length       : float  hard ceiling on rest_length [m]
    """

    def __init__(
        self,
        rest_length:     float,
        v_max_out:       float,
        v_max_in:        float,
        accel_limit_ms2: float,
        T_min:           float,
        T_max:           float,
        soft_zone_n:     float = 25.0,
        min_length:      float = 2.0,
        max_length:      float = 300.0,
    ):
        self.rest_length   = float(rest_length)
        self._v_max_out    = float(v_max_out)
        self._v_max_in     = float(v_max_in)
        self._accel        = float(accel_limit_ms2)
        self._T_min        = float(T_min)
        self._T_max        = float(T_max)
        self._predict_band  = max(1.0, float(soft_zone_n))
        self._min_length   = float(min_length)
        self._max_length   = float(max_length)

        self._v_target     = 0.0
        self._speed        = 0.0
        self._prev_tension = None
        self._tension_rate = 0.0
        self._energy_out_j = 0.0
        self._energy_in_j  = 0.0

    def set_velocity(self, v_ms: float) -> None:
        """Set target velocity [m/s].  +ve = pay out, -ve = reel in, 0 = hold."""
        self._v_target = float(v_ms)

    def step(self, tension_measured: float, dt: float) -> None:
        """400 Hz step.  Updates rest_length and energy accumulators."""
        T  = float(tension_measured)
        dt = float(dt)

        if self._prev_tension is not None and dt > 0.0:
            raw_rate = (T - self._prev_tension) / dt
            # Lightweight smoothing keeps the predictor from overreacting to noise.
            self._tension_rate = 0.8 * self._tension_rate + 0.2 * raw_rate
        self._prev_tension = T

        v = self._v_target
        if v > 0.0:
            v_setpoint = min(v, self._v_max_out)
        elif v < 0.0:
            v_setpoint = max(v, -self._v_max_in)
        else:
            v_setpoint = 0.0

        # Predictive braking: if the current tension trend would cross a hard
        # limit before we can stop, taper the commanded speed smoothly now.
        if self._accel > 0.0 and self._speed != 0.0:
            t_stop = abs(self._speed) / self._accel
            if self._tension_rate > 1e-9:
                predicted_t = T + self._tension_rate * t_stop
                margin = self._T_max - predicted_t
            elif self._tension_rate < -1e-9:
                predicted_t = T + self._tension_rate * t_stop
                margin = predicted_t - self._T_min
            else:
                margin = float("inf")

            if margin != float("inf"):
                brake = max(0.0, min(1.0, margin / self._predict_band))
                # Smoothstep gives a continuous first derivative at the band edges.
                brake = brake * brake * (3.0 - 2.0 * brake)
                v_setpoint *= brake

        # Ramp speed toward setpoint at accel_limit
        dv = self._accel * dt
        if self._speed < v_setpoint:
            self._speed = min(self._speed + dv, v_setpoint)
        elif self._speed > v_setpoint:
            self._speed = max(self._speed - dv, v_setpoint)

        # Integrate position
        new_length       = self.rest_length + self._speed * dt
        self.rest_length = max(self._min_length, min(self._max_length, new_length))

        # Energy accounting
        power = T * self._speed
        if power > 0.0:
            self._energy_out_j += power * dt
        elif power < 0.0:
            self._energy_in_j  += -power * dt

    # ── properties ─────────────────────────────────────────────────────────

    @property
    def speed_ms(self) -> float:
        """Current motor speed [m/s], signed (+ve = paying out)."""
        return self._speed

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
