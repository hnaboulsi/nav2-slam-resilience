"""Shared fixtures: builders for a minimal valid scenario dict.

Tests call `valid_scenario_dict(**overrides)` and mutate just the piece
they're testing, rather than repeating the whole nested structure everywhere.
"""

from __future__ import annotations

from typing import Any

import pytest


def _valid_scenario_dict(**overrides: Any) -> dict:
    base = {
        "schema_version": 1,
        "name": "lidar_noise_sweep",
        "fault": {
            "type": "noise",
            "severities": [0.0, 0.02, 0.05],
            "trials_per_severity": 3,
        },
        "mission": {
            "goals": [
                {"x": 2.0, "y": 0.0, "yaw": 0.0},
                {"x": 2.0, "y": 2.0, "yaw": 1.57},
            ],
            "goal_timeout_sec": 60.0,
        },
        "thresholds": {
            "localization_error_bound_m": 0.3,
            "goal_tolerance_m": 0.25,
            "map_stale_sec": 2.0,
            "recovery_hold_sec": 3.0,
        },
    }
    base.update(overrides)
    return base


@pytest.fixture
def valid_scenario_dict():
    return _valid_scenario_dict
