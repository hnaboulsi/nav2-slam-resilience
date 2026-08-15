"""Strict loader for benchmark scenario YAML files.

A scenario describes one benchmark run: which fault channel is being swept
(LiDAR noise stddev, or LiDAR dropout via a throttled rate), the severities to
test, how many repeated trials per severity, the fixed navigation mission
(goal poses in the world frame), and the pass/fail thresholds used to grade
each trial's metrics.

This is deliberately a plain dataclass + hand-written strict parser — no
schema-validation library, no DSL. Every field is validated explicitly so
unknown keys, wrong types, and out-of-range values fail loudly with a clear
message instead of silently producing a scenario that doesn't mean what the
YAML author thought it meant.
"""

from __future__ import annotations

import enum
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

SUPPORTED_SCHEMA_VERSION = 1


class ScenarioValidationError(ValueError):
    """Raised when a scenario YAML file is malformed or fails validation."""


class FaultType(enum.Enum):
    NOISE = "noise"
    DROPOUT = "dropout"


@dataclass(frozen=True)
class FaultSweep:
    type: FaultType
    severities: tuple[float, ...]
    trials_per_severity: int


@dataclass(frozen=True)
class MissionGoal:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Mission:
    goals: tuple[MissionGoal, ...]
    goal_timeout_sec: float


@dataclass(frozen=True)
class Thresholds:
    localization_error_bound_m: float
    goal_tolerance_m: float
    map_stale_sec: float
    recovery_hold_sec: float


@dataclass(frozen=True)
class ScenarioConfig:
    schema_version: int
    name: str
    fault: FaultSweep
    mission: Mission
    thresholds: Thresholds


def load_scenario(path: str | Path) -> ScenarioConfig:
    """Read and strictly validate a scenario YAML file."""
    text = Path(path).read_text()
    try:
        data = yaml.safe_load(text)
    except yaml.YAMLError as exc:
        raise ScenarioValidationError(f"invalid YAML in {path}: {exc}") from exc
    return parse_scenario(data)


def parse_scenario(data: Any) -> ScenarioConfig:
    if not isinstance(data, dict):
        raise ScenarioValidationError("scenario root must be a mapping")

    _reject_unknown_keys(
        data, {"schema_version", "name", "fault", "mission", "thresholds"}, "scenario"
    )

    schema_version = _require_int(data, "schema_version", "scenario")
    if schema_version != SUPPORTED_SCHEMA_VERSION:
        raise ScenarioValidationError(
            f"unsupported schema_version {schema_version!r}, expected {SUPPORTED_SCHEMA_VERSION}"
        )

    name = _require_str(data, "name", "scenario")
    if not name:
        raise ScenarioValidationError("scenario.name must be a non-empty string")

    fault = _parse_fault(_require_mapping(data, "fault", "scenario"))
    mission = _parse_mission(_require_mapping(data, "mission", "scenario"))
    thresholds = _parse_thresholds(_require_mapping(data, "thresholds", "scenario"))

    return ScenarioConfig(
        schema_version=schema_version,
        name=name,
        fault=fault,
        mission=mission,
        thresholds=thresholds,
    )


def _parse_fault(data: dict) -> FaultSweep:
    _reject_unknown_keys(data, {"type", "severities", "trials_per_severity"}, "fault")

    raw_type = _require_str(data, "type", "fault")
    try:
        fault_type = FaultType(raw_type)
    except ValueError as exc:
        allowed = ", ".join(t.value for t in FaultType)
        raise ScenarioValidationError(
            f"fault.type must be one of [{allowed}], got {raw_type!r}"
        ) from exc

    raw_severities = data.get("severities")
    if not isinstance(raw_severities, list) or not raw_severities:
        raise ScenarioValidationError("fault.severities must be a non-empty list of numbers")
    severities = []
    for i, value in enumerate(raw_severities):
        if not isinstance(value, (int, float)) or isinstance(value, bool):
            raise ScenarioValidationError(f"fault.severities[{i}] must be a number, got {value!r}")
        if value < 0:
            raise ScenarioValidationError(f"fault.severities[{i}] must be >= 0, got {value!r}")
        severities.append(float(value))
    if fault_type is FaultType.DROPOUT and any(v == 0 for v in severities):
        raise ScenarioValidationError(
            "fault.severities for type 'dropout' are throttle rates in Hz and must be > 0 "
            "(use a separate no-fault baseline scenario instead of a 0 Hz entry)"
        )

    trials = _require_int(data, "trials_per_severity", "fault")
    if trials < 1:
        raise ScenarioValidationError("fault.trials_per_severity must be >= 1")

    return FaultSweep(type=fault_type, severities=tuple(severities), trials_per_severity=trials)


def _parse_mission(data: dict) -> Mission:
    _reject_unknown_keys(data, {"goals", "goal_timeout_sec"}, "mission")

    raw_goals = data.get("goals")
    if not isinstance(raw_goals, list) or not raw_goals:
        raise ScenarioValidationError("mission.goals must be a non-empty list")
    goals = []
    for i, raw_goal in enumerate(raw_goals):
        if not isinstance(raw_goal, dict):
            raise ScenarioValidationError(f"mission.goals[{i}] must be a mapping")
        _reject_unknown_keys(raw_goal, {"x", "y", "yaw"}, f"mission.goals[{i}]")
        x = _require_number(raw_goal, "x", f"mission.goals[{i}]")
        y = _require_number(raw_goal, "y", f"mission.goals[{i}]")
        yaw = _require_number(raw_goal, "yaw", f"mission.goals[{i}]")
        goals.append(MissionGoal(x=x, y=y, yaw=yaw))

    timeout = _require_number(data, "goal_timeout_sec", "mission")
    if timeout <= 0:
        raise ScenarioValidationError("mission.goal_timeout_sec must be > 0")

    return Mission(goals=tuple(goals), goal_timeout_sec=timeout)


def _parse_thresholds(data: dict) -> Thresholds:
    keys = {"localization_error_bound_m", "goal_tolerance_m", "map_stale_sec", "recovery_hold_sec"}
    _reject_unknown_keys(data, keys, "thresholds")
    values = {}
    for key in keys:
        value = _require_number(data, key, "thresholds")
        if value <= 0:
            raise ScenarioValidationError(f"thresholds.{key} must be > 0")
        values[key] = value
    return Thresholds(**values)


def _reject_unknown_keys(data: dict, allowed: set[str], where: str) -> None:
    unknown = set(data) - allowed
    if unknown:
        raise ScenarioValidationError(f"unknown key(s) in {where}: {sorted(unknown)}")


def _require_mapping(data: dict, key: str, where: str) -> dict:
    value = data.get(key)
    if not isinstance(value, dict):
        raise ScenarioValidationError(f"{where}.{key} must be a mapping")
    return value


def _require_str(data: dict, key: str, where: str) -> str:
    value = data.get(key)
    if not isinstance(value, str):
        raise ScenarioValidationError(f"{where}.{key} must be a string, got {value!r}")
    return value


def _require_int(data: dict, key: str, where: str) -> int:
    value = data.get(key)
    if not isinstance(value, int) or isinstance(value, bool):
        raise ScenarioValidationError(f"{where}.{key} must be an integer, got {value!r}")
    return value


def _require_number(data: dict, key: str, where: str) -> float:
    value = data.get(key)
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise ScenarioValidationError(f"{where}.{key} must be a number, got {value!r}")
    return float(value)
