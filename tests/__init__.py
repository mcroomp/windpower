"""tests — all RAWES pytest suites.

Subpackages (by tier, see AGENTS.md):
    unit      — fast, no-physics tests
    simtests  — full Python physics loop simtests (marker: simtest)
    sitl      — ArduPilot SITL Docker stack tests (marker: sitl)
    hil       — hardware-in-the-loop smoke tests
    oneoff    — one-off diagnostic scripts (not part of standard suites)
    common    — shared test fixtures/helpers (MockArdupilot, etc.)
    envelope  — tests for the envelope/ point-mass sweep package
"""
