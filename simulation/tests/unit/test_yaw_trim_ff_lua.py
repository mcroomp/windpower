"""Unit tests for the adaptive yaw-trim throttle observer in rawes.lua.

The counter-torque plant is affine in the applied motor throttle u:

    psi_dot = a * u + b        a = slope (constant, gear ratio unknown)
                               b = -omega_rotor (slowly varying)

The observer (``yaw_trim_ff_step``) learns the equilibrium throttle u_eq = -b/a
online from the *actual applied* throttle (read back from SERVO4) and the
measured spin gyro:z().  Two properties are verified:

1.  Slope identification — the learned slope a_hat converges to the true slope
    from a wrong prior when the throttle is excited (normalised LMS on
    increments, which cancel the slow offset b).

2.  Equilibrium invariance — in closed loop with a one-directional P actuator,
    the trim converges to u_eq = omega/a and the spin is driven to ~0, and this
    holds even when the learned slope is inaccurate (the slope only sets the
    correction rate, not the fixed point).
"""
from __future__ import annotations

from rawes_lua_harness import RawesLua


_DT = 0.01  # 100 Hz observer tick


def _sim() -> RawesLua:
    sim = RawesLua()
    sim.fns.yaw_ff_reset()
    return sim


# ---------------------------------------------------------------------------
# 1. Slope identification
# ---------------------------------------------------------------------------

class TestSlopeIdentification:
    def test_learns_true_slope_from_wrong_prior(self):
        sim = _sim()
        a_true = 57.75      # RPM_SCALE / GEAR (unknown to the observer)
        b_true = -20.94     # -omega_rotor
        u_eq = -b_true / a_true
        sim.fns.yaw_a_set(120.0)   # deliberately wrong prior

        # Dither the applied throttle around the operating point so psi_dot stays
        # in the quiescent band (|psi_dot| < YFF_A_QUIET) where the slope updates.
        for i in range(600):
            u = u_eq + (0.02 if (i % 2 == 0) else -0.02)
            psi_dot = a_true * u + b_true        # = a_true*(u-u_eq) ~ +-1.15 rad/s
            sim.fns.yaw_trim_ff_step(_DT, u, psi_dot)

        a_hat = float(sim.fns.yaw_a_hat())
        assert abs(a_hat - a_true) < 0.1 * a_true, (
            f"slope not learned: a_hat={a_hat:.2f} vs a_true={a_true:.2f}"
        )

    def test_no_slope_update_when_not_quiescent(self):
        """Large |psi_dot| (fast oscillation) must not corrupt the slope."""
        sim = _sim()
        sim.fns.yaw_a_set(90.0)
        for i in range(200):
            u = 0.2 if (i % 2 == 0) else 0.6     # big du, but psi_dot far from 0
            psi_dot = 8.0 if (i % 2 == 0) else -8.0
            sim.fns.yaw_trim_ff_step(_DT, u, psi_dot)
        assert abs(float(sim.fns.yaw_a_hat()) - 90.0) < 1e-6


# ---------------------------------------------------------------------------
# 2. Closed-loop equilibrium (invariant to learned slope)
# ---------------------------------------------------------------------------

def _run_closed_loop(sim: RawesLua, a_true: float, omega: float,
                     kp: float = 0.008, n: int = 6000) -> tuple[float, float]:
    """Simulate the affine plant + one-directional P actuator driven by the
    observer trim.  Returns (final_trim, final_psi_dot).

    The test P gain is kept below 1/a_true so the no-lag algebraic inner loop is
    stable (the real system is stabilised by the motor lag at 400 Hz); this test
    isolates the observer's steady-state behaviour, not the fast loop dynamics.
    """
    trim = 0.0
    psi_dot = -omega           # t0: motor off, body drifts at -omega
    for _ in range(n):
        # One-directional P actuator on top of the learned trim.
        u = trim + kp * (-psi_dot)
        u = max(0.0, min(1.0, u))
        # Affine steady-state plant.
        psi_dot = a_true * u - omega
        trim = float(sim.fns.yaw_trim_ff_step(_DT, u, psi_dot))
    return trim, psi_dot


class TestClosedLoopEquilibrium:
    def test_trim_converges_to_equilibrium_throttle(self):
        sim = _sim()
        a_true, omega = 57.75, 20.94
        u_eq = omega / a_true            # ~0.3626
        trim, psi_dot = _run_closed_loop(sim, a_true, omega)
        assert abs(trim - u_eq) < 0.02, f"trim={trim:.4f} vs u_eq={u_eq:.4f}"
        assert abs(psi_dot) < 0.5, f"|psi_dot|={abs(psi_dot):.3f} not converged"

    def test_equilibrium_invariant_to_wrong_slope(self):
        """Even with a badly wrong (frozen-ish) slope prior, the fixed point is
        still u_eq = omega/a_true."""
        sim = _sim()
        a_true, omega = 57.75, 20.94
        u_eq = omega / a_true
        sim.fns.yaw_a_set(200.0)         # wrong prior; slow correction rate
        trim, psi_dot = _run_closed_loop(sim, a_true, omega, n=12000)
        assert abs(trim - u_eq) < 0.03, f"trim={trim:.4f} vs u_eq={u_eq:.4f}"
        assert abs(psi_dot) < 0.5, f"|psi_dot|={abs(psi_dot):.3f} not converged"

    def test_trim_clamped_non_negative(self):
        """Over-driven spin (psi_dot > 0 at u=0) must not push trim negative."""
        sim = _sim()
        for _ in range(50):
            sim.fns.yaw_trim_ff_step(_DT, 0.0, 5.0)
        assert float(sim.fns.yaw_ff_trim()) >= 0.0
