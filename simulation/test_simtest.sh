#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Run only the simtest-marked tests (full physics simulation loops).
# For fast unit tests only, use test_unit.sh instead.
cd /rawes && .venv/bin/python -m pytest -s -m simtest simulation/tests/unit "$@"
