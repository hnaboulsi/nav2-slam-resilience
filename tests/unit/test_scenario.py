import copy

import pytest

from nav2_slam_resilience.scenario import (
    FaultType,
    ScenarioValidationError,
    load_scenario,
    parse_scenario,
)


class TestValidScenario:
    def test_parses_a_minimal_valid_scenario(self, valid_scenario_dict):
        scenario = parse_scenario(valid_scenario_dict())
        assert scenario.name == "lidar_noise_sweep"
        assert scenario.fault.type is FaultType.NOISE
        assert scenario.fault.severities == (0.0, 0.02, 0.05)
        assert scenario.fault.trials_per_severity == 3
        assert len(scenario.mission.goals) == 2
        assert scenario.mission.goals[0].x == 2.0
        assert scenario.mission.goal_timeout_sec == 60.0
        assert scenario.thresholds.localization_error_bound_m == 0.3

    def test_dropout_scenario_with_positive_rates(self, valid_scenario_dict):
        data = valid_scenario_dict()
        data["fault"] = {
            "type": "dropout",
            "severities": [10.0, 5.0, 1.0],
            "trials_per_severity": 5,
        }
        scenario = parse_scenario(data)
        assert scenario.fault.type is FaultType.DROPOUT
        assert scenario.fault.severities == (10.0, 5.0, 1.0)


class TestRootValidation:
    def test_root_must_be_a_mapping(self):
        with pytest.raises(ScenarioValidationError, match="mapping"):
            parse_scenario(["not", "a", "dict"])

    def test_rejects_unknown_top_level_key(self, valid_scenario_dict):
        data = valid_scenario_dict()
        data["unexpected"] = 1
        with pytest.raises(ScenarioValidationError, match="unknown key"):
            parse_scenario(data)

    def test_rejects_unsupported_schema_version(self, valid_scenario_dict):
        data = valid_scenario_dict(schema_version=2)
        with pytest.raises(ScenarioValidationError, match="schema_version"):
            parse_scenario(data)

    def test_rejects_non_integer_schema_version(self, valid_scenario_dict):
        data = valid_scenario_dict(schema_version=1.0)
        with pytest.raises(ScenarioValidationError):
            parse_scenario(data)

    def test_rejects_empty_name(self, valid_scenario_dict):
        data = valid_scenario_dict(name="")
        with pytest.raises(ScenarioValidationError, match="non-empty"):
            parse_scenario(data)

    def test_rejects_non_string_name(self, valid_scenario_dict):
        data = valid_scenario_dict(name=123)
        with pytest.raises(ScenarioValidationError):
            parse_scenario(data)


class TestFaultValidation:
    def test_rejects_unknown_fault_key(self, valid_scenario_dict):
        data = valid_scenario_dict()
        data["fault"]["extra"] = True
        with pytest.raises(ScenarioValidationError, match="unknown key"):
            parse_scenario(data)

    def test_rejects_unsupported_fault_type(self, valid_scenario_dict):
        data = valid_scenario_dict()
        data["fault"]["type"] = "teleport"
        with pytest.raises(ScenarioValidationError, match="fault.type"):
            parse_scenario(data)

    def test_rejects_empty_severities(self, valid_scenario_dict):
        data = valid_scenario_dict()
        data["fault"]["severities"] = []
        with pytest.raises(ScenarioValidationError, match="severities"):
            parse_scenario(data)

    def test_rejects_non_list_severities(self, valid_scenario_dict):
        data = valid_scenario_dict()
        data["fault"]["severities"] = 0.05
        with pytest.raises(ScenarioValidationError, match="severities"):
            parse_scenario(data)

    def test_rejects_negative_severity(self, valid_scenario_dict):
        data = valid_scenario_dict()
        data["fault"]["severities"] = [0.01, -0.02]
        with pytest.raises(ScenarioValidationError, match=r"severities\[1\]"):
            parse_scenario(data)

    def test_rejects_non_numeric_severity(self, valid_scenario_dict):
        data = valid_scenario_dict()
        data["fault"]["severities"] = [0.01, "fast"]
        with pytest.raises(ScenarioValidationError, match=r"severities\[1\]"):
            parse_scenario(data)

    def test_rejects_zero_severity_for_dropout(self, valid_scenario_dict):
        data = valid_scenario_dict()
        data["fault"] = {"type": "dropout", "severities": [0.0, 5.0], "trials_per_severity": 2}
        with pytest.raises(ScenarioValidationError, match="dropout"):
            parse_scenario(data)

    def test_zero_severity_is_fine_for_noise(self, valid_scenario_dict):
        # A 0.0 noise stddev is a legitimate no-fault baseline point on the sweep.
        data = valid_scenario_dict()
        data["fault"]["severities"] = [0.0, 0.05]
        scenario = parse_scenario(data)
        assert scenario.fault.severities[0] == 0.0

    def test_rejects_trials_per_severity_below_one(self, valid_scenario_dict):
        data = valid_scenario_dict()
        data["fault"]["trials_per_severity"] = 0
        with pytest.raises(ScenarioValidationError, match="trials_per_severity"):
            parse_scenario(data)

    def test_rejects_non_integer_trials_per_severity(self, valid_scenario_dict):
        data = valid_scenario_dict()
        data["fault"]["trials_per_severity"] = 2.5
        with pytest.raises(ScenarioValidationError):
            parse_scenario(data)


class TestMissionValidation:
    def test_rejects_unknown_mission_key(self, valid_scenario_dict):
        data = valid_scenario_dict()
        data["mission"]["extra"] = True
        with pytest.raises(ScenarioValidationError, match="unknown key"):
            parse_scenario(data)

    def test_rejects_empty_goals(self, valid_scenario_dict):
        data = valid_scenario_dict()
        data["mission"]["goals"] = []
        with pytest.raises(ScenarioValidationError, match="goals"):
            parse_scenario(data)

    def test_rejects_unknown_goal_key(self, valid_scenario_dict):
        data = valid_scenario_dict()
        data["mission"]["goals"][0]["z"] = 1.0
        with pytest.raises(ScenarioValidationError, match="unknown key"):
            parse_scenario(data)

    def test_rejects_non_numeric_goal_field(self, valid_scenario_dict):
        data = valid_scenario_dict()
        data["mission"]["goals"][0]["x"] = "two"
        with pytest.raises(ScenarioValidationError):
            parse_scenario(data)

    def test_rejects_goal_that_is_not_a_mapping(self, valid_scenario_dict):
        data = valid_scenario_dict()
        data["mission"]["goals"] = [[1.0, 2.0, 0.0]]
        with pytest.raises(ScenarioValidationError, match="mapping"):
            parse_scenario(data)

    def test_rejects_nonpositive_goal_timeout(self, valid_scenario_dict):
        data = valid_scenario_dict()
        data["mission"]["goal_timeout_sec"] = 0
        with pytest.raises(ScenarioValidationError, match="goal_timeout_sec"):
            parse_scenario(data)


class TestThresholdsValidation:
    @pytest.mark.parametrize(
        "key",
        [
            "localization_error_bound_m",
            "goal_tolerance_m",
            "map_stale_sec",
            "recovery_hold_sec",
        ],
    )
    def test_rejects_nonpositive_threshold(self, valid_scenario_dict, key):
        data = valid_scenario_dict()
        data["thresholds"][key] = 0
        with pytest.raises(ScenarioValidationError, match=key):
            parse_scenario(data)

    def test_rejects_missing_threshold_key(self, valid_scenario_dict):
        data = valid_scenario_dict()
        del data["thresholds"]["recovery_hold_sec"]
        with pytest.raises(ScenarioValidationError, match="recovery_hold_sec"):
            parse_scenario(data)

    def test_rejects_unknown_threshold_key(self, valid_scenario_dict):
        data = valid_scenario_dict()
        data["thresholds"]["extra"] = 1.0
        with pytest.raises(ScenarioValidationError, match="unknown key"):
            parse_scenario(data)


class TestImmutability:
    def test_parsing_does_not_mutate_input(self, valid_scenario_dict):
        data = valid_scenario_dict()
        snapshot = copy.deepcopy(data)
        parse_scenario(data)
        assert data == snapshot


class TestLoadScenario:
    def test_loads_a_real_file(self, tmp_path, valid_scenario_dict):
        import yaml

        path = tmp_path / "scenario.yaml"
        path.write_text(yaml.safe_dump(valid_scenario_dict()))
        scenario = load_scenario(path)
        assert scenario.name == "lidar_noise_sweep"

    def test_reports_invalid_yaml_clearly(self, tmp_path):
        path = tmp_path / "broken.yaml"
        path.write_text("fault: [unterminated")
        with pytest.raises(ScenarioValidationError, match="invalid YAML"):
            load_scenario(path)
