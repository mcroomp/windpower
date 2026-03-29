#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cd /rawes && .venv/bin/python -m pytest -s simulation/tests/unit "$@"
