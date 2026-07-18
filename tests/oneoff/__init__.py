"""One-off diagnostic scripts.  Not pytest-discoverable.

Per CLAUDE.md "Workflow Rules": any script run during diagnosis that
isn't a regression guard (Bode probes, gain sweeps, phase sweeps,
plant-id experiments, debug traces) lives here instead of being
inlined as ``python -c "..."`` in chat history.
"""
