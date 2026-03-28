"""
test_setup.py — ACRO stack setup test.

Verifies that the full initialization sequence completes successfully:
    mediator + SITL launch → GCS connect → params → EKF tilt alignment →
    arm → motor interlock → ACRO mode

This test intentionally covers ONLY the setup phase.  It is the first test to
run when debugging a broken stack — if it passes, higher-level tests (hold,
pumping cycle, etc.) can rely on the shared ``acro_armed`` fixture.

Asserts:
  - EKF produced at least one finite ATTITUDE during setup
  - Vehicle is confirmed armed (arm() did not raise)
  - Vehicle is confirmed in ACRO mode (set_mode() did not raise)
  - No CRITICAL errors in mediator log
  - LOCAL_POSITION_NED received if GPS absolute position locked (diagnostic only)
"""
import logging

from conftest import (
    StackContext,
    analyze_startup_logs,
    dump_startup_diagnostics,
    wait_for_acro_stability,
)

log = logging.getLogger("test_setup")


def test_acro_armed(acro_armed: StackContext):
    """
    Setup-only test: verify the stack reaches armed ACRO mode.

    Uses the ``acro_armed`` fixture from conftest.py which handles
    the full initialization sequence.  This test only adds assertions.
    """
    ctx = acro_armed

    try:
        # ── 1. At least one clean ATTITUDE was received during setup ──────────
        assert ctx.setup_samples, (
            "No MAVLink samples received during setup.\n"
            "This means either:\n"
            "  • mediator is not sending UDP packets to SITL\n"
            "  • SITL is not producing MAVLink output\n"
            "  • GCS connection to port 5760 failed silently\n"
            f"STATUSTEXT seen: {ctx.all_statustext}"
        )

        attitude_samples = [s for s in ctx.setup_samples if s["type"] == "ATTITUDE"]
        assert attitude_samples, (
            "No ATTITUDE messages received during setup — EKF never aligned.\n"
            "Likely causes:\n"
            "  • mediator accel_body not producing gravity (check sensor packet)\n"
            "  • SITL JSON backend not parsing our IMU fields correctly\n"
            "  • INITIAL_MODE=1 preventing EKF from initialising attitude\n"
            f"STATUSTEXT seen: {ctx.all_statustext}\n"
            f"Setup samples: {[s['type'] for s in ctx.setup_samples]}"
        )

        # ── 2. Attitude is finite (not NaN/Inf from a broken IMU feed) ────────
        import math
        for s in attitude_samples[-5:]:     # check the most recent samples
            assert all(math.isfinite(v) for v in (s["roll"], s["pitch"], s["yaw"])), (
                f"Non-finite ATTITUDE: rpy=({s['roll']}, {s['pitch']}, {s['yaw']})\n"
                "mediator may be sending NaN in accel_body or gyro_body."
            )

        # ── 3. LOCAL_POSITION_NED (diagnostic only — not required for ACRO) ────
        # ACRO uses raw gyro and does not need absolute GPS position.  The EKF
        # exits the setup window as soon as it has velocity (flags 0x0006), which
        # happens within ~10 s — before GPS absolute position (0x0008) locks.
        # A missing LOCAL_POSITION_NED here is expected and not a failure.
        pos_samples = [s for s in ctx.setup_samples if s["type"] == "LOCAL_POSITION_NED"]
        if pos_samples:
            first_pos = pos_samples[0]
            log.info(
                "GPS position locked during setup: NED=(%.2f, %.2f, %.2f) m",
                first_pos["N"], first_pos["E"], first_pos["D"],
            )
        else:
            log.info(
                "No LOCAL_POSITION_NED during setup (EKF has velocity but not "
                "absolute position — normal for fast setup with COMPASS_USE=0)"
            )

        # ── 4. No CRITICAL errors in mediator ─────────────────────────────────
        if ctx.mediator_log.exists():
            med_text = ctx.mediator_log.read_text(encoding="utf-8", errors="replace")
            critical = [l for l in med_text.splitlines() if "CRITICAL" in l]
            assert not critical, (
                f"{len(critical)} CRITICAL error(s) in mediator log:\n"
                + "\n".join(critical[:10])
            )

        # ── 5. Setup complete event recorded (arm + ACRO both succeeded) ──────
        assert "Setup complete" in ctx.flight_events, (
            "Setup did not complete — arm or mode-set raised an exception.\n"
            f"flight_events: {ctx.flight_events}\n"
            f"STATUSTEXT: {ctx.all_statustext}"
        )

        log.info(
            "test_acro_armed PASSED  "
            "ATTITUDE_samples=%d  pos_samples=%d  statustext=%d  "
            "ekf_aligned=%s",
            len(attitude_samples), len(pos_samples), len(ctx.all_statustext),
            any("tilt alignment" in t.lower() for t in ctx.all_statustext),
        )

    except Exception:
        dump_startup_diagnostics(ctx)
        raise
