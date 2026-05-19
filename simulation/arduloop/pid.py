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
"""

from __future__ import annotations

from dataclasses import dataclass, field

from .filters import LowPassFilter1p, NotchFilter
from .params import RateAxisParams


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
                   limit: bool = False, boost: float = 1.0) -> float:
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
