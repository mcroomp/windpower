"""
test_nvf_receive.py -- Validate NAMED_VALUE_FLOAT receive in rawes.lua (SITL).

Sends a NAMED_VALUE_FLOAT("RAWES_SUB", 99) from the Python GCS after the Lua
script is active and asserts rawes.lua echoes it back as a STATUSTEXT within
a short window.

This is the gate test for the named-float substate migration:
  1. GCS sends  NAMED_VALUE_FLOAT  to ArduPilot over the normal telemetry link.
  2. ArduPilot routes it to rawes.lua via mavlink.receive_chan() (registered at
     script load with mavlink.register_rx_msgid(251)).
  3. rawes.lua stores the value and emits:
       RAWES: rcvd RAWES_SUB=99
  4. This test asserts that STATUSTEXT appears within _ACK_TIMEOUT_S.

Uses the acro_armed_lua_full fixture (SCR_USER6=1, steady orbit, GPS fused)
because it is the simplest reliable Lua stack fixture and mode is irrelevant
to the receive path.
"""
import logging
import sys
from pathlib import Path

_SIM_DIR  = Path(__file__).resolve().parents[3]
_SITL_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_SIM_DIR))
sys.path.insert(0, str(_SITL_DIR))

from stack_infra import StackContext, dump_startup_diagnostics  # noqa: E402

# ---------------------------------------------------------------------------
# Test parameters
# ---------------------------------------------------------------------------

_TEST_NAME      = "RAWES_SUB"
_TEST_VALUE     = 99.0
_ACK_TIMEOUT_S  = 20.0   # wall-clock seconds; ack should arrive within one Lua tick (~10 ms)


# ---------------------------------------------------------------------------
# Test
# ---------------------------------------------------------------------------

def test_nvf_receive(acro_armed_lua_full: StackContext):
    """
    rawes.lua receives NAMED_VALUE_FLOAT and echoes it as STATUSTEXT.

    Validates the full GCS -> ArduPilot -> Lua mavlink.receive_chan() path
    before migrating SCR_USER6 substate to use named-float messages.
    """
    ctx = acro_armed_lua_full
    gcs = ctx.gcs
    log = logging.getLogger("test_nvf_receive")

    log.info("Sending NAMED_VALUE_FLOAT %s=%.0f ...", _TEST_NAME, _TEST_VALUE)
    gcs.send_named_float(_TEST_NAME, _TEST_VALUE)

    ack_seen = False
    deadline = gcs.sim_now() + _ACK_TIMEOUT_S

    try:
        while gcs.sim_now() < deadline:
            for name, proc, lp in [
                ("mediator", ctx.mediator_proc, ctx.mediator_log),
                ("SITL",     ctx.sitl_proc,     ctx.sitl_log),
            ]:
                if proc.poll() is not None:
                    txt = lp.read_text(encoding="utf-8", errors="replace") if lp.exists() else "(no log)"
                    raise RuntimeError(
                        f"{name} exited during nvf_receive test (rc={proc.returncode}):\n{txt[-2000:]}"
                    )

            msg = gcs._recv(type=["STATUSTEXT"], blocking=True, timeout=0.2)
            if msg is None:
                continue

            text = msg.text.rstrip("\x00").strip()
            log.info("STATUSTEXT: %s", text)
            ctx.all_statustext.append(text)

            if _TEST_NAME in text and str(int(_TEST_VALUE)) in text:
                ack_seen = True
                log.info("  -> Lua ack received")
                break

        assert ack_seen, (
            f"rawes.lua never echoed '{_TEST_NAME}={int(_TEST_VALUE)}' as STATUSTEXT "
            f"within {_ACK_TIMEOUT_S:.0f} s sim-time. "
            "mavlink.receive_chan() may not be wired up or ArduPilot is not routing "
            "NAMED_VALUE_FLOAT to the scripting engine. "
            f"All STATUSTEXT seen: {ctx.all_statustext}"
        )

        log.info("=== test_nvf_receive PASSED ===")

    except Exception:
        dump_startup_diagnostics(ctx)
        raise
