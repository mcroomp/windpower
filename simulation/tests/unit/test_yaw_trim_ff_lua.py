"""Unit tests for the yaw-rate PID (H_YAW_TRIM feedforward) in rawes.lua.

``yaw_trim_ff_step(dt, psi_dot, kp, ki, kd)`` regulates the measured body yaw
rate ``psi_dot = gyro:z()`` to zero and writes the result to H_YAW_TRIM,
downstream of the ArduPilot attitude clamp (so it holds even when the native
rate PID is frozen).  The counter-torque motor is one-directional, so the
output and the integrator are clamped to ``[0, YFF_MAX]``:

    err  = -psi_dot                      (setpoint 0)
    I   += ki*err*dt                     (clamped [0, YFF_MAX] -- the DC hold)
    D    = kd * lowpass(d(err)/dt)
    out  = clamp(I + kp*err + D, 0, YFF_MAX)

Gains are the live params KP=SCR_USER1, KI=SCR_USER2, KD=SCR_USER3.  The step
function is pure (no sensor/param access) so it is unit-testable in isolation.
"""
from __future__ import annotations

from rawes_lua_harness import RawesLua


_DT = 0.01  # 100 Hz nominal observer tick


def _sim() -> RawesLua:
    sim = RawesLua()
    sim.fns.yaw_ff_reset()
    return sim


# ---------------------------------------------------------------------------
# Closed-loop equilibrium: the integrator holds the zero-rate throttle
# ---------------------------------------------------------------------------

def _run_closed_loop(sim: RawesLua, a_true: float, omega: float,
                     ki: float = 1.0, n: int = 4000) -> tuple[float, float]:
    """Drive the PID as the sole controller of an affine static-gain plant:

        psi_dot = a_true * out - omega      (out = motor throttle in [0, YFF_MAX])

    At equilibrium the PID output settles to u_eq = omega/a_true (rotor reaction
    torque exactly cancelled).  Returns (final_out, final_psi_dot).

    ki is chosen so ki*dt*a_true < 2 (discrete-integrator stability on the
    static-gain plant).  The fixed point u_eq is independent of ki.
    """
    psi_dot = -omega           # t0: motor off, body drifts at -omega
    out = 0.0
    for _ in range(n):
        out = float(sim.fns.yaw_trim_ff_step(_DT, psi_dot, 0.0, ki, 0.0))
        psi_dot = a_true * out - omega
    return out, psi_dot


class TestClosedLoopEquilibrium:
    def test_trim_converges_to_equilibrium_throttle(self):
        sim = _sim()
        a_true, omega = 57.75, 20.94
        u_eq = omega / a_true            # ~0.3626 (< YFF_MAX, so the clamp is idle)
        out, psi_dot = _run_closed_loop(sim, a_true, omega)
        assert abs(out - u_eq) < 0.02, f"out={out:.4f} vs u_eq={u_eq:.4f}"
        assert abs(psi_dot) < 0.5, f"|psi_dot|={abs(psi_dot):.3f} not converged"

    def test_equilibrium_tracks_plant_slope(self):
        """The integrator settles to u_eq = omega/a_true for a different plant
        slope; the fixed point depends on the plant, not on the loop gain ki."""
        sim = _sim()
        a_true, omega = 45.0, 20.94
        u_eq = omega / a_true            # ~0.4653
        out, psi_dot = _run_closed_loop(sim, a_true, omega, ki=1.0, n=6000)
        assert abs(out - u_eq) < 0.03, f"out={out:.4f} vs u_eq={u_eq:.4f}"
        assert abs(psi_dot) < 0.5, f"|psi_dot|={abs(psi_dot):.3f} not converged"

    def test_trim_clamped_non_negative(self):
        """Over-driven spin (psi_dot > 0 with the motor already off) must not
        push the one-directional output/integrator negative."""
        sim = _sim()
        for _ in range(50):
            out = float(sim.fns.yaw_trim_ff_step(_DT, 5.0, 0.0, 5.0, 0.0))
            assert out >= 0.0
        assert float(sim.fns.yaw_ff_trim()) >= 0.0

    def test_output_clamped_at_max(self):
        """A large sustained drift winds the integrator up to YFF_MAX, no more."""
        sim = _sim()
        ymax = float(sim.fns.YFF_MAX)
        for _ in range(2000):
            out = float(sim.fns.yaw_trim_ff_step(_DT, -50.0, 0.0, 1.0, 0.0))
            assert out <= ymax + 1e-9
        assert abs(float(sim.fns.yaw_ff_trim()) - ymax) < 1e-6


# ---------------------------------------------------------------------------
# Calibrated model slope (used by the Smith predictor, not adapted online)
# ---------------------------------------------------------------------------

class TestCalibratedSlope:
    def test_slope_is_fixed_and_not_adapted(self):
        """yaw_a_set is a no-op; yaw_a_hat returns the bench-calibrated constant
        (~63 rad/s per u) regardless of excitation."""
        sim = _sim()
        a0 = float(sim.fns.yaw_a_hat())
        assert 55.0 < a0 < 72.0, f"calibrated slope out of range: {a0:.2f}"
        sim.fns.yaw_a_set(120.0)         # no-op now
        for i in range(200):
            psi_dot = (0.2 if (i % 2 == 0) else 0.6) * a0 - 10.0
            sim.fns.yaw_trim_ff_step(_DT, psi_dot, 0.0, 0.02, 0.0)
        assert abs(float(sim.fns.yaw_a_hat()) - a0) < 1e-9


# ---------------------------------------------------------------------------
# Yaw-D (damping) term (gain kd = SCR_USER3)
# ---------------------------------------------------------------------------

class TestYawDTerm:
    def test_kd_zero_small_response_to_rate_spike(self):
        """With kd=0 a single rate spike moves the output only by the integral
        step (~ki*err*dt), i.e. there is no large derivative kick."""
        sim = _sim()
        kp, ki = 0.0, 0.5
        for _ in range(50):
            sim.fns.yaw_trim_ff_step(_DT, -0.4, kp, ki, 0.0)   # build a positive I
        prev = float(sim.fns.yaw_ff_trim())
        out = float(sim.fns.yaw_trim_ff_step(_DT, 0.5, kp, ki, 0.0))  # rate spike
        assert abs(out - prev) < 0.01

    def test_kd_opposes_rising_rate(self):
        """A positive kd subtracts when the yaw rate is rising (d(psi_dot)/dt>0),
        so the kd>0 output is below the kd=0 output on the same rising sample."""
        kp, ki, kd = 0.0, 0.5, 0.02
        sim0 = _sim()
        simd = _sim()
        # Prime both identically at a steady rate (the D term settles to 0).
        for _ in range(50):
            sim0.fns.yaw_trim_ff_step(_DT, -0.4, kp, ki, 0.0)
            simd.fns.yaw_trim_ff_step(_DT, -0.4, kp, ki, kd)
        # Apply the same rising-rate sample; only the kd term should differ.
        out0 = float(sim0.fns.yaw_trim_ff_step(_DT, 0.5, kp, ki, 0.0))
        outd = float(simd.fns.yaw_trim_ff_step(_DT, 0.5, kp, ki, kd))
        assert outd < out0

    def test_output_clamped_non_negative_with_kd(self):
        sim = _sim()
        kd = 0.05
        for _ in range(30):
            out = float(sim.fns.yaw_trim_ff_step(_DT, 5.0, 0.0, 0.0, kd))
            assert out >= 0.0
