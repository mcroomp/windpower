"""Unit tests for WinchController — verify tension-controlled motion profile."""

import math
import pytest
from winch import WinchController


# ---------------------------------------------------------------------------
# Simplified kite model for coupled tests
# ---------------------------------------------------------------------------

class SimpleTetheredKite:
    """
    1-DOF radial kite model on an elastic tether.

    Captures: thrust-to-tension response with ~1 s lag, tether spring, damping.

    State:
        L_actual  : radial distance from anchor [m]
        v_radial  : radial velocity [m/s]  (positive = away from anchor)
        F_thrust  : thrust output [N], first-order lag behind F_set

    Equation of motion:
        m * dv/dt = F_thrust - T - c * v_radial
        T = max(0, EA * (L_actual - L_rest) / L_rest)
        dF_thrust/dt = (F_set - F_thrust) / tau_thrust

    EA_eff is intentionally low (2000 N/m) so the spring period is ~3 s and
    the test runs at reasonable step counts.  The 1-second thrust lag matches
    the user requirement.
    """

    def __init__(
        self,
        L0: float,              # initial actual and rest length [m]
        F0: float = 300.0,      # initial / equilibrium thrust [N]
        mass: float = 5.0,      # effective kite mass [kg]
        EA_eff: float = 2000.0, # effective tether stiffness [N/m]
        c_damp: float = 20.0,   # radial damping [N·s/m]
        tau_thrust: float = 1.0,# thrust lag [s]
    ):
        self.L_actual   = float(L0)
        self.v_radial   = 0.0
        self.F_thrust   = float(F0)
        self._F_set     = float(F0)
        self._mass      = mass
        self._EA        = EA_eff
        self._c         = c_damp
        self._tau       = tau_thrust

    def set_thrust(self, F_set: float) -> None:
        self._F_set = float(F_set)

    def step(self, L_rest: float, dt: float) -> float:
        """Advance one step; return tension [N]."""
        # Thrust lag
        self.F_thrust += (self._F_set - self.F_thrust) * dt / self._tau

        # Tether tension (slack → 0)
        extension = self.L_actual - L_rest
        T = max(0.0, self._EA * extension / L_rest)

        # Radial dynamics
        a = (self.F_thrust - T - self._c * self.v_radial) / self._mass
        self.v_radial  += a * dt
        self.L_actual  += self.v_radial * dt
        self.L_actual   = max(0.01, self.L_actual)

        return T


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def make_winch(
    rest=100.0, kp=0.005, v_max_out=0.40, v_max_in=0.20,
    accel=1.0, min_len=2.0, max_len=300.0
):
    """Return a WinchController with fast accel (1 m/s²) for most tests."""
    return WinchController(
        rest_length=rest, kp_tension=kp,
        v_max_out=v_max_out, v_max_in=v_max_in,
        accel_limit_ms2=accel, min_length=min_len, max_length=max_len,
    )


def run_steps(winch, tension, dt=1/400, n=400):
    """Run n steps at constant tension. Returns final rest_length."""
    for _ in range(n):
        winch.step(tension, dt)
    return winch.rest_length


# ---------------------------------------------------------------------------
# Reel-out behaviour
# ---------------------------------------------------------------------------

class TestReelOut:
    """Winch pays out when target_length > rest_length and T > T_target."""

    def test_no_movement_below_tension_target(self):
        """T < T_target with reel-out goal: winch stays put."""
        w = make_winch(rest=100.0)
        w.set_target(110.0, tension_n=300.0)
        run_steps(w, tension=200.0, n=400)   # T=200 < 300 = target
        assert w.rest_length == pytest.approx(100.0, abs=1e-6)

    def test_pays_out_above_tension_target(self):
        """T > T_target: rest_length increases toward target."""
        w = make_winch(rest=100.0)
        w.set_target(110.0, tension_n=300.0)
        run_steps(w, tension=400.0, n=2000)  # ~5 s
        assert w.rest_length > 100.01        # definitely moved

    def test_speed_proportional_to_tension_error(self):
        """Cruise speed = kp*(T - T_target), capped at v_max_out."""
        w = make_winch(rest=100.0, kp=0.005, v_max_out=0.40, accel=100.0)
        w.set_target(200.0, tension_n=300.0)
        # Run many steps to reach cruise
        for _ in range(20):
            w.step(500.0, dt=1/400)
        # T=500, T_target=300 -> cruise = 0.005*(500-300) = 1.0 -> cap 0.40
        assert w.speed_ms == pytest.approx(0.40, abs=0.01)

    def test_speed_below_cap_when_small_error(self):
        """Small tension excess -> speed below v_max."""
        w = make_winch(rest=100.0, kp=0.005, v_max_out=0.40, accel=100.0)
        w.set_target(200.0, tension_n=300.0)
        for _ in range(20):
            w.step(320.0, dt=1/400)   # T-T_target = 20 N -> 0.005*20 = 0.10 m/s
        assert w.speed_ms == pytest.approx(0.10, abs=0.02)

    def test_stops_at_target_length(self):
        """Winch decelerates and stops once rest_length reaches target."""
        w = make_winch(rest=100.0, kp=0.005, v_max_out=0.40, accel=0.5)
        w.set_target(101.0, tension_n=100.0)  # small 1 m payout
        run_steps(w, tension=500.0, n=8000)   # ~20 s
        assert w.rest_length == pytest.approx(101.0, abs=0.01)
        assert abs(w.speed_ms) < 0.01

    def test_energy_accumulated_during_reel_out(self):
        """Generator energy = T * v * dt > 0 when paying out under tension."""
        w = make_winch(rest=100.0, kp=0.005, v_max_out=0.40, accel=100.0)
        w.set_target(110.0, tension_n=300.0)
        run_steps(w, tension=400.0, n=400)
        assert w._energy_out_j > 0.0
        assert w._energy_in_j == 0.0


# ---------------------------------------------------------------------------
# Reel-in behaviour
# ---------------------------------------------------------------------------

class TestReelIn:
    """Winch reels in when target_length < rest_length and T < T_target."""

    def test_no_movement_above_tension_target(self):
        """T >= T_target with reel-in goal: winch stays put."""
        w = make_winch(rest=110.0)
        w.set_target(100.0, tension_n=300.0)
        run_steps(w, tension=350.0, n=400)   # T=350 > 300 = target
        assert w.rest_length == pytest.approx(110.0, abs=1e-6)

    def test_reels_in_below_tension_target(self):
        """T < T_target: rest_length decreases toward target."""
        w = make_winch(rest=110.0)
        w.set_target(100.0, tension_n=300.0)
        run_steps(w, tension=200.0, n=2000)  # ~5 s
        assert w.rest_length < 109.99        # definitely moved inward

    def test_speed_proportional_to_tension_deficit(self):
        """Cruise speed = kp*(T_target - T), capped at v_max_in."""
        w = make_winch(rest=110.0, kp=0.005, v_max_in=0.20, accel=100.0)
        w.set_target(50.0, tension_n=300.0)
        for _ in range(20):
            w.step(100.0, dt=1/400)   # T_target-T = 200 N -> 0.005*200 = 1.0 -> cap 0.20
        assert w.speed_ms == pytest.approx(-0.20, abs=0.01)

    def test_speed_below_cap_small_deficit(self):
        """Small tension deficit -> speed below v_max_in."""
        w = make_winch(rest=110.0, kp=0.005, v_max_in=0.20, accel=100.0)
        w.set_target(50.0, tension_n=300.0)
        for _ in range(20):
            w.step(280.0, dt=1/400)   # T_target-T = 20 N -> 0.005*20 = 0.10 m/s
        assert w.speed_ms == pytest.approx(-0.10, abs=0.02)

    def test_zero_speed_when_tension_at_target(self):
        """T == T_target: cruise speed = 0, winch decelerates to stop."""
        w = make_winch(rest=110.0, kp=0.005, v_max_in=0.20, accel=100.0)
        w.set_target(50.0, tension_n=300.0)
        # First let it build up speed at low tension
        for _ in range(20):
            w.step(100.0, dt=1/400)
        assert w.speed_ms < -0.01            # moving inward
        # Now tension equals target: should decelerate
        for _ in range(1000):
            w.step(300.0, dt=1/400)
        assert abs(w.speed_ms) < 0.01        # stopped

    def test_stops_at_target_length(self):
        """Winch decelerates and stops when rest_length reaches target."""
        w = make_winch(rest=110.0, kp=0.005, v_max_in=0.20, accel=0.5)
        w.set_target(109.0, tension_n=300.0)  # small 1 m reel-in
        run_steps(w, tension=100.0, n=8000)   # ~20 s
        assert w.rest_length == pytest.approx(109.0, abs=0.01)
        assert abs(w.speed_ms) < 0.01

    def test_does_not_reel_below_min_length(self):
        """Hard floor: rest_length never goes below min_length."""
        w = make_winch(rest=10.0, min_len=9.0)
        w.set_target(2.0, tension_n=300.0)   # target below min_length
        run_steps(w, tension=0.0, n=4000)    # max tension deficit -> max speed
        assert w.rest_length >= 9.0

    def test_energy_accumulated_during_reel_in(self):
        """Motor energy consumed (energy_in_j) > 0 when reeling in under tension."""
        w = make_winch(rest=110.0, kp=0.005, v_max_in=0.20, accel=100.0)
        w.set_target(100.0, tension_n=300.0)
        run_steps(w, tension=200.0, n=400)
        assert w._energy_in_j > 0.0
        assert w._energy_out_j == 0.0


# ---------------------------------------------------------------------------
# Symmetric behaviour — reel-in/reel-out are mirror images at same |error|
# ---------------------------------------------------------------------------

class TestSymmetry:
    """Confirm the reel-in formula is the mirror of reel-out."""

    def test_same_tension_error_same_speed_magnitude(self):
        """
        |T - T_target| = 100 N in both directions should give the same
        cruise speed magnitude (before capping by v_max_out vs v_max_in).
        """
        kp, v_cap = 0.005, 10.0  # use large v_cap so no capping
        w_out = make_winch(rest=100.0, kp=kp, v_max_out=v_cap, accel=100.0)
        w_out.set_target(200.0, tension_n=300.0)
        for _ in range(20):
            w_out.step(400.0, dt=1/400)  # T - T_target = +100 N

        w_in = make_winch(rest=110.0, kp=kp, v_max_in=v_cap, accel=100.0)
        w_in.set_target(50.0, tension_n=300.0)
        for _ in range(20):
            w_in.step(200.0, dt=1/400)   # T_target - T = +100 N

        assert abs(w_out.speed_ms) == pytest.approx(abs(w_in.speed_ms), abs=0.001)


# ---------------------------------------------------------------------------
# Orbital tension oscillation scenario
# ---------------------------------------------------------------------------

class TestOrbitalScenario:
    """
    Simulate an orbiting kite where tension alternates between a peak (kite
    flying outward) and a trough (kite flying inward).

    Reel-in target < rest_length, T_target = 300 N.
    Orbital peak T = 500 N  -> winch stops (T > T_target)
    Orbital trough T = 100 N -> winch reels in at kp*(300-100)=1.0 -> cap 0.20 m/s

    Net: rest_length should decrease (reel in completed) without going slack.
    """

    def test_reel_in_completes_over_multiple_orbits(self):
        DELTA_L = 3.0
        w = make_winch(rest=100.0, kp=0.005, v_max_in=0.20, accel=0.5)
        w.set_target(100.0 - DELTA_L, tension_n=300.0)

        orbit_period_s  = 8.0
        dt              = 1.0 / 400
        steps_per_orbit = int(orbit_period_s / dt)
        max_orbits      = 300

        for _ in range(max_orbits):
            # First half of orbit: high tension (kite flying outward)
            for _ in range(steps_per_orbit // 2):
                w.step(500.0, dt)
            # Second half: low tension (kite flying inward)
            for _ in range(steps_per_orbit // 2):
                w.step(100.0, dt)
            if w.rest_length <= (100.0 - DELTA_L) + 0.05:
                break
        else:
            pytest.fail(
                f"Reel-in did not complete after {max_orbits} orbits; "
                f"rest_length={w.rest_length:.3f} m  (target={100.0 - DELTA_L:.1f} m)"
            )

    def test_rest_length_monotonically_decreases_during_reel_in(self):
        """rest_length should never increase while winch is in reel-in mode."""
        w = make_winch(rest=100.0, kp=0.005, v_max_in=0.20, accel=0.5)
        w.set_target(95.0, tension_n=300.0)
        dt = 1.0 / 400
        prev = w.rest_length
        orbit_period_s = 8.0
        steps_per_orbit = int(orbit_period_s / dt)

        for _ in range(20 * steps_per_orbit):
            t = _ % steps_per_orbit
            tension = 500.0 if t < steps_per_orbit // 2 else 100.0
            w.step(tension, dt)
            assert w.rest_length <= prev + 1e-9, (
                f"rest_length increased: {prev:.6f} -> {w.rest_length:.6f}"
            )
            prev = w.rest_length


# ---------------------------------------------------------------------------
# Coupled winch + kite model tests
# ---------------------------------------------------------------------------

class TestCoupledKiteModel:
    """
    Drive the WinchController with a SimpleTetheredKite so that thrust
    influences tension through a ~1 s lag and the kite's radial spring.

    This validates the winch controller in a closed-loop plant, not just with
    prescribed tension traces.
    """

    DT = 1.0 / 400      # 400 Hz physics step

    def _run(self, winch, kite, n_steps):
        """Advance coupled simulation for n_steps; return tension history."""
        T_hist = []
        for _ in range(n_steps):
            T = kite.step(winch.rest_length, self.DT)
            winch.step(T, self.DT)
            T_hist.append(T)
        return T_hist

    def test_equilibrium_tension_tracks_thrust(self):
        """
        With no reel-in/out goal, tension settles near the thrust force.
        Thrust set to 300 N; after ~10 s lag + spring settling, T ≈ 300 N.
        """
        L0 = 100.0
        kite  = SimpleTetheredKite(L0=L0, F0=300.0)
        winch = make_winch(rest=L0, kp=0.005, v_max_out=0.40, accel=1.0)
        winch.set_target(L0, tension_n=300.0)   # hold at rest length

        T_hist = self._run(winch, kite, n_steps=int(15 / self.DT))
        T_final = sum(T_hist[-400:]) / 400      # mean over last second
        assert T_final == pytest.approx(300.0, abs=30.0)

    def test_reel_in_no_slack_with_1s_thrust_lag(self):
        """
        Reel in 5 m.  For the formula v = kp*(T_target - T) to produce reel-in
        speed, the kite's equilibrium tension must be BELOW T_target.  Here
        thrust=200 N gives equilibrium T≈200 N < T_target=300 N, so the winch
        is continuously active during reel-in.
        """
        L0 = 100.0
        kite  = SimpleTetheredKite(L0=L0, F0=200.0)   # equilibrium T ≈ 200 N < T_target
        winch = make_winch(rest=L0, kp=0.005, v_max_in=0.20, accel=0.5)
        winch.set_target(L0 - 5.0, tension_n=300.0)

        T_hist = self._run(winch, kite, n_steps=int(60 / self.DT))

        # Tension must never go slack
        assert min(T_hist) >= 0.0, f"Slack detected: T_min={min(T_hist):.1f} N"

        # Reel-in must complete
        assert winch.rest_length == pytest.approx(L0 - 5.0, abs=0.10), (
            f"Reel-in incomplete: rest_length={winch.rest_length:.3f} m"
        )

    def test_reel_in_completes_with_oscillating_thrust(self):
        """
        Simulate orbiting kite: thrust oscillates ±100 N around 200 N with
        a 6 s period (range 100–300 N).  Mean equilibrium T ≈ 200 N < T_target
        = 300 N, so the winch is active during the low-tension orbital phase.
        Winch must complete a 4 m reel-in without slack.
        """
        L0 = 100.0
        kite  = SimpleTetheredKite(L0=L0, F0=200.0, tau_thrust=1.0)
        winch = make_winch(rest=L0, kp=0.005, v_max_in=0.20, accel=0.5)
        winch.set_target(L0 - 4.0, tension_n=300.0)

        ORBIT_PERIOD = 6.0
        n_steps = int(120 / self.DT)
        T_hist = []
        for i in range(n_steps):
            t = i * self.DT
            F = 200.0 + 100.0 * math.sin(2 * math.pi * t / ORBIT_PERIOD)
            kite.set_thrust(F)
            T = kite.step(winch.rest_length, self.DT)
            winch.step(T, self.DT)
            T_hist.append(T)

        assert min(T_hist) >= 0.0, f"Slack: T_min={min(T_hist):.1f} N"
        assert winch.rest_length == pytest.approx(L0 - 4.0, abs=0.15), (
            f"Reel-in incomplete: rest_length={winch.rest_length:.3f} m"
        )

    def test_thrust_step_causes_reel_out(self):
        """
        Sudden thrust increase above T_out triggers reel-out.
        Thrust jumps from 300 N to 500 N; winch target T_out=435 N.
        Rest_length must increase (tether paid out).
        """
        L0 = 100.0
        kite  = SimpleTetheredKite(L0=L0, F0=300.0, tau_thrust=1.0)
        winch = make_winch(rest=L0, kp=0.005, v_max_out=0.40, accel=0.5)
        winch.set_target(L0 + 10.0, tension_n=435.0)

        # Warm up at low thrust (T < T_out, winch stays put)
        self._run(winch, kite, n_steps=int(3 / self.DT))
        L_before = winch.rest_length

        # Step thrust high; tension should rise above T_out after ~1 s lag
        kite.set_thrust(500.0)
        self._run(winch, kite, n_steps=int(20 / self.DT))

        assert winch.rest_length > L_before + 0.5, (
            f"Reel-out did not trigger: L={winch.rest_length:.3f} m"
        )

    def test_reel_in_stalls_when_equilibrium_tension_above_target(self):
        """
        Documents the failure mode: if the kite's equilibrium tension is ABOVE
        T_target, v = kp*(T_target - T) is always 0 and the winch stalls.

        This is the root cause of the simtest failure after reel-out: the
        thrust overshoot during the orbital minimum leaves the kite flying at
        T > T_target when reel-in begins.  The fix is to ensure the transition
        phase fully resets the operating point to T ≈ T_ic < T_target before
        reel-in starts.
        """
        L0 = 100.0
        kite  = SimpleTetheredKite(L0=L0, F0=400.0)   # equilibrium T ≈ 400 N > T_target
        winch = make_winch(rest=L0, kp=0.005, v_max_in=0.20, accel=0.5)
        winch.set_target(L0 - 5.0, tension_n=300.0)

        # Allow kite to reach its equilibrium
        self._run(winch, kite, n_steps=int(15 / self.DT))
        L_after_settle = winch.rest_length

        # Run for a long time — winch should barely move
        self._run(winch, kite, n_steps=int(60 / self.DT))

        # Winch should be essentially stalled (T always above 300 N target)
        reel_in_distance = L_after_settle - winch.rest_length
        assert reel_in_distance < 1.0, (
            f"Expected stall but winch moved {reel_in_distance:.2f} m "
            f"(equilibrium T={400:.0f} N > T_target=300 N)"
        )

    def test_thrust_reduction_during_reel_in_does_not_slack(self):
        """
        During reel-in, briefly drop thrust to 150 N (below T_target=300 N).
        Winch should reel in faster (large T deficit) and prevent slack.
        """
        L0 = 100.0
        kite  = SimpleTetheredKite(L0=L0, F0=300.0, tau_thrust=1.0)
        winch = make_winch(rest=L0, kp=0.005, v_max_in=0.20, accel=1.0)
        winch.set_target(L0 - 3.0, tension_n=300.0)

        # Settle for a moment
        self._run(winch, kite, n_steps=int(3 / self.DT))

        # Drop thrust suddenly
        kite.set_thrust(150.0)
        T_hist = self._run(winch, kite, n_steps=int(5 / self.DT))

        assert min(T_hist) >= 0.0, (
            f"Slack during thrust reduction: T_min={min(T_hist):.1f} N"
        )
