"""Unit tests for the yaw trim observer (servo-readback) in rawes.lua.

``yaw_trim_step(dt, u, psi_dot)`` computes the equilibrium H_YAW_TRIM using the
affine plant model  psi_dot = YFF_A * (u - u_eq)  and low-passes toward it:

    trim_target = clamp(u - psi_dot / YFF_A,  0,  YFF_MAX)
    trim       += alpha * (trim_target - trim)     alpha = dt / (YFF_TRIM_TAU + dt)

The fixed point is  trim = u_eq = omega_rotor / YFF_A  (independent of dt or TAU).
"""
from __future__ import annotations

import math

from rawes_lua_harness import RawesLua

_DT = 0.01   # 100 Hz nominal tick


def _sim() -> RawesLua:
    sim = RawesLua()
    sim.fns.yaw_trim_reset()
    return sim


# ---------------------------------------------------------------------------
# Trim-target formula: trim_target = u - psi_dot / YFF_A
# ---------------------------------------------------------------------------

class TestTrimTarget:
    def test_zero_rate_trim_target_equals_u(self):
        """With psi_dot=0 the deadbeat target is just u."""
        sim = _sim()
        u   = 0.4
        # dt=1e4 => alpha = 1e4/(TAU+1e4) ≈ 1; trim snaps to target.
        out = float(sim.fns.yaw_trim_step(1e4, u, 0.0))
        assert abs(out - u) < 1e-3, f"trim={out:.5f} should equal u={u}"

    def test_nonzero_rate_trim_target(self):
        """trim_target = u - psi_dot/YFF_A (verified via big-dt snap)."""
        sim = _sim()
        a   = float(sim.fns.YFF_A)
        u, psi_dot = 0.5, -1.0          # body yawing CCW under rotor torque
        expected = u - psi_dot / a      # should be > u (motor needs more throttle)
        out = float(sim.fns.yaw_trim_step(1e4, u, psi_dot))
        assert abs(out - expected) < 1e-3, f"out={out:.5f} expected={expected:.5f}"

    def test_low_pass_slows_convergence(self):
        """With small dt the trim moves only a fraction (alpha) per step."""
        sim = _sim()
        a   = float(sim.fns.YFF_A)
        tau = float(sim.fns.YFF_TAU)
        u, psi_dot = 0.4, 0.0
        alpha = _DT / (tau + _DT)
        out = float(sim.fns.yaw_trim_step(_DT, u, psi_dot))
        # After one step from 0: trim ≈ alpha * u
        assert abs(out - alpha * u) < 1e-6


# ---------------------------------------------------------------------------
# Clamp behaviour
# ---------------------------------------------------------------------------

class TestClamp:
    def test_trim_never_negative(self):
        """Over-spin (psi_dot > 0, motor already off) must not push trim < 0."""
        sim = _sim()
        for _ in range(50):
            out = float(sim.fns.yaw_trim_step(_DT, 0.0, 5.0))
            assert out >= 0.0

    def test_trim_never_exceeds_max(self):
        """Sustained large drift must not push trim above YFF_MAX."""
        sim = _sim()
        ymax = float(sim.fns.YFF_MAX)
        for _ in range(2000):
            out = float(sim.fns.yaw_trim_step(_DT, 0.0, -50.0))
            assert out <= ymax + 1e-9


# ---------------------------------------------------------------------------
# Closed-loop equilibrium: observer converges to u_eq = omega_rotor / YFF_A
# ---------------------------------------------------------------------------

def _run_plant_loop(sim: RawesLua, a: float, omega: float,
                    n: int = 3000) -> tuple[float, float]:
    """Simulate the plant+observer closed loop without an AP P-term.

    Plant: psi_dot = a * (u - u_eq)   with u_eq = omega / a
    Starting from motor off, the observer adapts H_YAW_TRIM toward u_eq.
    Returns (final_trim, final_psi_dot).
    """
    u_eq  = omega / a
    trim  = 0.0                 # initial H_YAW_TRIM
    psi_dot = -omega            # t=0: motor off, body drifts at -omega rad/s
    for _ in range(n):
        # The ONLY motor command is H_YAW_TRIM (no AP P-term).
        u = trim
        out = float(sim.fns.yaw_trim_step(_DT, u, psi_dot))
        trim = out
        psi_dot = a * (trim - u_eq)
    return trim, psi_dot


class TestEquilibrium:
    def test_converges_to_u_eq(self):
        """Trim converges to omega/YFF_A when driving a static-gain plant."""
        sim = _sim()
        a     = float(sim.fns.YFF_A)
        omega = 28.0             # ~nominal rotor rad/s
        u_eq  = omega / a        # ≈ 0.53 for a≈52.8
        trim, psi_dot = _run_plant_loop(sim, a, omega)
        assert abs(trim - u_eq) < 0.01,  f"trim={trim:.4f} vs u_eq={u_eq:.4f}"
        assert abs(psi_dot)     < 0.05,  f"|psi_dot|={abs(psi_dot):.4f} not converged"

    def test_equilibrium_independent_of_omega(self):
        """Fixed point u_eq = omega/a for a different rotor speed."""
        sim = _sim()
        a     = float(sim.fns.YFF_A)
        omega = 20.0
        u_eq  = omega / a
        trim, psi_dot = _run_plant_loop(sim, a, omega, n=3000)
        assert abs(trim - u_eq) < 0.02, f"trim={trim:.4f} vs u_eq={u_eq:.4f}"

    def test_state_reset_zeroes_trim(self):
        """yaw_trim_reset() brings trim back to 0 regardless of prior state."""
        sim = _sim()
        float(sim.fns.yaw_trim_step(10.0, 0.5, 0.0))   # push trim up
        sim.fns.yaw_trim_reset()
        assert float(sim.fns.yaw_ff_trim()) == 0.0
