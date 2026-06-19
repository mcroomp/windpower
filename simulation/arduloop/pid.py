"""
AC_PID port. Signal flow matches `AC_PID::update_all` exactly:

    target   ──► [target notch] ──► [FLTT lowpass] ──► _target
                                                         │
    measurement ─────────────────────────────────────────┤
                                          error = _target - measurement
                                                         │
                                       ┌── [error notch] ┴── [FLTE lowpass] ──► _error
                                       │
                                       ├── P     = _kp * _error
                                       ├── D     = _kd * d/dt(_error)  through [FLTD lowpass]
                                       ├── I     = ∫ _ki * _error dt    clamped to ±IMAX
                                       ├── FF    = _kff * _target
                                       └── D_FF  = _kdff * d/dt(_target)

Returns P + D + FF + DFF + I.

Notes
-----
* Notches are applied **before** the smoothing low-pass — this matches the C++
  ordering "apply notch filters before FLTD/FLTE to avoid shot noise".
* I-term update is gated by `limit` (set true when the output is saturated)
  to implement integrator clamping/anti-windup, identical to AP semantics.
* Slew rate limiter (SMAX): reduces P and D gains when the P+D sum changes
  faster than `SMAX` output units/s.  Matches Filter/SlewLimiter.cpp.
* Leaky integrator (ILMI): decays the integrator toward ±ILMI each tick.
  Matches AC_HELI_PID::update_leaky_i().
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from .filters import LowPassFilter1p, NotchFilter
from .params import RateAxisParams


# ---------------------------------------------------------------------------
# SlewLimiter  (Filter/SlewLimiter.cpp)
# ---------------------------------------------------------------------------

_SLEW_WINDOW_S       = 0.300           # WINDOW_MS / 1000
_SLEW_MODIFIER_GAIN  = 1.5             # MODIFIER_GAIN
_SLEW_CUTOFF_FREQ    = 25.0            # DERIVATIVE_CUTOFF_FREQ (Hz)
_SLEW_N_EVENTS       = 2               # SLEWLIMITER_N_EVENTS


class SlewLimiter:
    """Port of Filter/SlewLimiter — reduces P+D gain when output slew rate exceeds SMAX.

    Uses sim_time_s (float seconds) instead of AP_HAL::millis().
    """

    def __init__(self, smax: float, tau: float = 1.0):
        """
        Parameters
        ----------
        smax : float  Max slew rate (output units/s).  0 = disabled.
        tau  : float  Decay time constant (s) for peak tracking.  AP default 1.0.
        """
        self._smax = smax
        self._tau  = max(tau, 1e-6)
        self._flt  = LowPassFilter1p(_SLEW_CUTOFF_FREQ)
        self._flt.reset(0.0)
        self._last_sample: float = 0.0
        self._max_pos: float = 0.0
        self._max_neg: float = 0.0
        self._max_pos_t: float = 0.0   # sim-time (s) of last positive peak
        self._max_neg_t: float = 0.0
        # Exceedance event ring buffers (sim-time, s)
        self._pos_event_t: list[float] = [0.0] * _SLEW_N_EVENTS
        self._neg_event_t: list[float] = [0.0] * _SLEW_N_EVENTS
        self._pos_event_idx: int = 0
        self._neg_event_idx: int = 0
        self._pos_event_stored: bool = False
        self._neg_event_stored: bool = False
        self._output_slew_rate: float = 0.0
        self._modifier_slew_rate: float = 0.0

    def update_smax(self, smax: float) -> None:
        self._smax = smax

    def modifier(self, sample: float, dt: float, sim_time_s: float = 0.0) -> float:
        """Return a scale factor in [0,1] to apply to P+D.  Updates internal state."""
        if dt <= 0.0:
            return 1.0

        # LP-filtered derivative of sample.
        slew_rate = self._flt.apply((sample - self._last_sample) / dt, dt)
        self._last_sample = sample

        now = sim_time_s

        # Decay/attack alphas.
        decay_alpha  = min(dt, self._tau) / self._tau
        attack_alpha = min(2.0 * decay_alpha, 1.0)

        # Track positive peak with timestamp-based decay.
        if slew_rate > self._max_pos:
            self._max_pos = slew_rate
            self._max_pos_t = now
        elif now - self._max_pos_t > _SLEW_WINDOW_S:
            self._max_pos *= (1.0 - decay_alpha)

        # Track negative peak (stored as positive magnitude).
        if -slew_rate > self._max_neg:
            self._max_neg = -slew_rate
            self._max_neg_t = now
        elif now - self._max_neg_t > _SLEW_WINDOW_S:
            self._max_neg *= (1.0 - decay_alpha)

        raw_slew = 0.5 * (self._max_pos + self._max_neg)
        self._output_slew_rate = (1.0 - attack_alpha) * self._output_slew_rate + attack_alpha * raw_slew
        self._output_slew_rate = min(self._output_slew_rate, raw_slew)

        if self._smax <= 0.0:
            return 1.0

        smax = self._smax
        # Store exceedance events.
        if not self._pos_event_stored and slew_rate > smax:
            self._pos_event_t[self._pos_event_idx % _SLEW_N_EVENTS] = now
            self._pos_event_idx = (self._pos_event_idx + 1) % _SLEW_N_EVENTS
            self._pos_event_stored = True
            self._neg_event_stored = False
        if not self._neg_event_stored and -slew_rate > smax:
            self._neg_event_t[self._neg_event_idx % _SLEW_N_EVENTS] = now
            self._neg_event_idx = (self._neg_event_idx + 1) % _SLEW_N_EVENTS
            self._neg_event_stored = True
            self._pos_event_stored = False

        # Find oldest event time.
        oldest = now
        for t in self._pos_event_t:
            oldest = min(oldest, t)
        for t in self._neg_event_t:
            oldest = min(oldest, t)

        # Clamp raw slew to 10x smax before computing modifier_input.
        lim_pos = min(self._max_pos, 10.0 * smax)
        lim_neg = min(self._max_neg, 10.0 * smax)
        modifier_input = 0.5 * (lim_pos + lim_neg)

        # Reduce modifier_input if oldest event is too far in the past.
        age_threshold = (_SLEW_N_EVENTS + 1) * _SLEW_WINDOW_S
        if now - oldest > age_threshold:
            oldest_age = now - oldest - age_threshold
            modifier_input *= math.exp(-oldest_age / self._tau)

        self._modifier_slew_rate = (
            (1.0 - attack_alpha) * self._modifier_slew_rate + attack_alpha * modifier_input
        )
        self._modifier_slew_rate = min(self._modifier_slew_rate, modifier_input)

        if self._modifier_slew_rate <= smax:
            return 1.0
        return smax / (smax + _SLEW_MODIFIER_GAIN * (self._modifier_slew_rate - smax))

    def get_slew_rate(self) -> float:
        return self._output_slew_rate


@dataclass
class PIDDebug:
    target:    float = 0.0
    actual:    float = 0.0
    error:     float = 0.0
    P:         float = 0.0
    I:         float = 0.0
    D:         float = 0.0
    FF:        float = 0.0
    DFF:       float = 0.0
    out:       float = 0.0


class AC_PID:
    """Single-axis PID with target / error notches and three low-passes.

    Parameters
    ----------
    p : RateAxisParams
        Gain / filter configuration. Stored by reference so live tuning works.
    sample_hz : float
        Loop rate used to discretise the notch biquads. Must match the rate
        at which `update_all` is called (use `set_sample_rate` to update).
    """

    def __init__(self, p: RateAxisParams, sample_hz: float):
        self.p = p
        self.sample_hz = float(sample_hz)

        # Three low-passes
        self._flt_T = LowPassFilter1p(p.FLTT)
        self._flt_E = LowPassFilter1p(p.FLTE)
        self._flt_D = LowPassFilter1p(p.FLTD)

        # Two notches (target & error)
        self._notch_T = NotchFilter()
        self._notch_E = NotchFilter()
        self._configure_notches()

        # Slew rate limiter (SMAX).  _slew_limit_scale=1 matches AP default.
        self._slew_limiter = SlewLimiter(p.SMAX, p.SRTAU)
        self._slew_limit_scale: float = 1.0

        # State
        self._target = 0.0
        self._error = 0.0
        self._derivative = 0.0
        self._target_derivative = 0.0
        self._integrator = 0.0
        self._reset_filter = True

        self.debug = PIDDebug()

    # ------------------------------------------------------------------
    # Configuration
    # ------------------------------------------------------------------

    def set_sample_rate(self, sample_hz: float) -> None:
        self.sample_hz = float(sample_hz)
        self._configure_notches()

    def reload_params(self) -> None:
        """Re-read cutoff/notch from `self.p`. Call after live edits."""
        self._flt_T.set_cutoff(self.p.FLTT)
        self._flt_E.set_cutoff(self.p.FLTE)
        self._flt_D.set_cutoff(self.p.FLTD)
        self._configure_notches()

    def _configure_notches(self) -> None:
        p = self.p
        if p.NTF_center_hz > 0.0 and p.NTF_bandwidth_hz > 0.0:
            self._notch_T.set(p.NTF_center_hz, p.NTF_bandwidth_hz,
                              self.sample_hz, p.NTF_attn_db)
        else:
            self._notch_T.disable()
        if p.NEF_center_hz > 0.0 and p.NEF_bandwidth_hz > 0.0:
            self._notch_E.set(p.NEF_center_hz, p.NEF_bandwidth_hz,
                              self.sample_hz, p.NEF_attn_db)
        else:
            self._notch_E.disable()

    def reset(self, target: float = 0.0, measurement: float = 0.0) -> None:
        self._reset_filter = True
        self._integrator = 0.0
        self._target = target
        self._error = target - measurement
        self._derivative = 0.0
        self._target_derivative = 0.0
        self._flt_T.reset(target)
        self._flt_E.reset(self._error)
        self._flt_D.reset(0.0)
        self._notch_T.reset(target)
        self._notch_E.reset(self._error)

    def reset_I(self) -> None:
        self._integrator = 0.0

    # ------------------------------------------------------------------
    # Main update
    # ------------------------------------------------------------------

    def update_all(self, target: float, measurement: float, dt: float,
                   limit: bool = False, boost: float = 1.0,
                   sim_time_s: float = 0.0) -> float:
        p = self.p

        if self._reset_filter:
            self._reset_filter = False
            # Reset target path
            self._target = self._notch_T.apply(target) if self._notch_T.enabled else target
            self._notch_T.reset(self._target)
            self._flt_T.reset(self._target)
            # Reset error path
            err = self._target - measurement
            self._error = self._notch_E.apply(err) if self._notch_E.enabled else err
            self._notch_E.reset(self._error)
            self._flt_E.reset(self._error)
            self._derivative = 0.0
            self._target_derivative = 0.0
            self._flt_D.reset(0.0)
        else:
            target_last = self._target
            error_last = self._error

            # Notch before LPF
            t = self._notch_T.apply(target)
            self._target = self._flt_T.apply(t, dt)

            err = self._target - measurement
            err = self._notch_E.apply(err)
            self._error = self._flt_E.apply(err, dt)

            if dt > 0.0:
                raw_deriv = (self._error - error_last) / dt
                self._derivative = self._flt_D.apply(raw_deriv, dt)
                self._target_derivative = (self._target - target_last) / dt

        # I-term update (anti-windup via `limit`)
        if p.I != 0.0 and dt > 0.0:
            if not limit or (self._error * self._integrator) < 0.0:
                self._integrator += p.I * self._error * dt
                imax = abs(p.IMAX)
                if imax > 0.0:
                    self._integrator = max(-imax, min(imax, self._integrator))

        P_out  = p.P  * self._error
        D_out  = p.D  * self._derivative
        FF_out = p.FF * self._target
        DFF_out = p.D_FF * self._target_derivative

        # Slew rate limiter: reduce P+D when output changes faster than SMAX.
        # Matches AC_PID::update_all line: Dmod = _slew_limiter.modifier(P+D, dt)
        if p.SMAX > 0.0:
            Dmod = self._slew_limiter.modifier(
                (P_out + D_out) * self._slew_limit_scale, dt, sim_time_s)
            P_out *= Dmod
            D_out *= Dmod

        # Optional PD sum limit (PDMX)
        if p.PDMX > 0.0:
            pd_abs = abs(P_out + D_out)
            if pd_abs > p.PDMX:
                scale = p.PDMX / pd_abs
                P_out *= scale
                D_out *= scale

        P_out *= boost
        D_out *= boost

        out = P_out + D_out + FF_out + DFF_out + self._integrator

        d = self.debug
        d.target = self._target
        d.actual = measurement
        d.error = self._error
        d.P = P_out
        d.I = self._integrator
        d.D = D_out
        d.FF = FF_out
        d.DFF = DFF_out
        d.out = out
        return out

    # ------------------------------------------------------------------
    # Introspection helpers
    # ------------------------------------------------------------------

    def get_i(self) -> float:
        return self._integrator

    def set_i(self, value: float) -> None:
        self._integrator = value

    def update_leaky_i(self, leak_rate: float) -> None:
        """Port of AC_HELI_PID::update_leaky_i().

        Decays the integrator toward ±ILMI (the leak minimum) at `leak_rate`
        per tick.  With ILMI=0 (default) this decays toward zero.

        Call before ``update_all`` to match AP call order.
        ``leak_rate`` = AC_ATTITUDE_HELI_RATE_INTEGRATOR_LEAK_RATE = 0.02.
        """
        p = self.p
        if p.I == 0.0:
            return
        leak_min = p.ILMI
        if self._integrator > leak_min:
            self._integrator -= (self._integrator - leak_min) * leak_rate
        elif self._integrator < -leak_min:
            self._integrator -= (self._integrator + leak_min) * leak_rate
