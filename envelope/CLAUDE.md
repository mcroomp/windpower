# Envelope module guide

This file supplements the repo-level guidance in `AGENTS.md` with the current
state of the `envelope/` subpackage. Keep it focused on live module facts; do
not duplicate the general project workflow here.

## Current module map

| File | Current role |
|---|---|
| `point_mass.py` | single-point along-tether simulation and equilibrium search (`simulate_point`) |
| `compute_map.py` | grid / column sweep driver, parallelized with `ProcessPoolExecutor` |
| `analyse_envelope.py` | CLI for phase sweeps, envelope slices, and equilibrium inspection |
| `rotor_helpers.py` | shared rotor-definition loading for the envelope tools |

## Current aerodynamic model choice

Both `point_mass.py` and `compute_map.py` currently instantiate:

```python
create_aero(rotor, model="quasi_static")
```

That is the current repo-truth for the envelope tooling. If the model changes,
update this file and the envelope tests in the same change.

## Current operating caveat: high-tension cold-start bistability

`tests/envelope/test_30deg_convergence.py` documents and enforces the current
behavior at `30°` tether elevation:

- for `T >= 725 N`, a cold start (`omega_init=5`) can converge to a different
  bistable attractor than the operational continuation branch;
- those cases are beyond the stated `T_hard_max=496 N` operating envelope and
  are skipped rather than treated as the normal operating solution.

`compute_map.py` addresses this by ramping tension through a column instead of
solving each high-tension point from an unrelated cold start.

## Current tests

Envelope tests currently live in `tests/envelope/`:

- `test_compute_map.py`
- `test_30deg_convergence.py`
- `test_collective_sign.py`
- `test_collective_pid_el0.py`
- `test_pid_collective.py`
- `test_tension_continuation.py`
- `test_tension_ramp_30deg.py`

Query the exact current inventory with:

```powershell
uv run python -m pytest --collect-only -q tests\envelope
```

Run the suite with:

```powershell
uv run python -m pytest tests\envelope -q
```

## Current CLI entry points

```powershell
# Fast preview grid
uv run python envelope\compute_map.py --quick --save envelope\map_quick.npz

# Larger grid
uv run python envelope\compute_map.py --full --save envelope\map_full.npz

# Reload a saved grid
uv run python envelope\compute_map.py --load envelope\map_quick.npz

# Inspect a named phase or one operating point
uv run python envelope\analyse_envelope.py --phase reel_out
uv run python envelope\analyse_envelope.py --el 80 --tension 200 --wind 10
```

## Current synchronization rules

When changing the envelope dynamics or CLI:

1. keep `CLAUDE.md` aligned with the actual file set and entry points;
2. update the matching `tests/envelope/` assertions in the same change;
3. prefer describing current behavior over copying historical pass counts or
   deleted test names.
