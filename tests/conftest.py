"""Shared pytest options for every test tier."""

import pytest


def pytest_addoption(parser: pytest.Parser) -> None:
    parser.addoption(
        "--run-lint",
        action="store_true",
        default=False,
        help="Enable tests marked lint (Ruff and Pyright).",
    )


def pytest_collection_modifyitems(config: pytest.Config, items: list[pytest.Item]) -> None:
    if config.getoption("--run-lint"):
        return
    skip_lint = pytest.mark.skip(reason="Lint tests require --run-lint")
    for item in items:
        if item.get_closest_marker("lint"):
            item.add_marker(skip_lint)
