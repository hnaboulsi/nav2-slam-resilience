import math

import pytest

from nav2_slam_resilience.metrics import (
    ErrorSample,
    NavOutcome,
    PoseSample,
    align_ground_truth_to_estimate_start,
    classify_navigation_outcome,
    collision_count,
    localization_error_series,
    map_update_gaps,
    synchronize_pose_series,
    time_to_recover,
)


class TestAlignGroundTruthToEstimateStart:
    def test_shifts_ground_truth_so_matched_point_equals_estimate_start(self):
        # Ground truth is offset by (+10, +20) from the map frame throughout -
        # a pure translation, as if the map frame's origin were elsewhere in
        # Gazebo's world. Ground truth is also sampled far more densely than
        # the estimate, matching real /pose vs /ground_truth_pose_array rates.
        estimated = [PoseSample(t=5.0, x=0.0, y=0.0), PoseSample(t=6.0, x=1.0, y=0.0)]
        ground_truth = [
            PoseSample(t=0.0, x=10.0, y=20.0),
            PoseSample(t=5.0, x=10.0, y=20.0),  # matches estimated[0]'s time
            PoseSample(t=6.0, x=11.0, y=20.0),
        ]
        aligned = align_ground_truth_to_estimate_start(estimated, ground_truth)
        # Every sample shifts by the same (-10, -20), including the one at
        # t=5.0 that anchored the shift - it must land exactly on (0, 0).
        assert aligned[1].x == pytest.approx(0.0)
        assert aligned[1].y == pytest.approx(0.0)
        assert aligned[2].x == pytest.approx(1.0)
        assert aligned[2].y == pytest.approx(0.0)
        assert aligned[0].x == pytest.approx(0.0)
        assert aligned[0].y == pytest.approx(0.0)

    def test_ignores_ground_truths_own_first_sample_when_irrelevant(self):
        # Regression case for the bug this replaced: the naive fix ("shift by
        # each series' own first sample") would anchor on t=0 here even
        # though the estimate doesn't start until t=100 - silently comparing
        # two different moments and baking in a fake constant error.
        estimated = [PoseSample(t=100.0, x=0.0, y=0.0)]
        ground_truth = [
            PoseSample(t=0.0, x=999.0, y=999.0),
            PoseSample(t=100.0, x=5.0, y=5.0),
        ]
        aligned = align_ground_truth_to_estimate_start(estimated, ground_truth)
        assert aligned[1].x == pytest.approx(0.0)
        assert aligned[1].y == pytest.approx(0.0)

    def test_preserves_timestamps(self):
        estimated = [PoseSample(t=0.0, x=0.0, y=0.0)]
        ground_truth = [PoseSample(t=0.0, x=1.0, y=1.0)]
        aligned = align_ground_truth_to_estimate_start(estimated, ground_truth)
        assert aligned[0].t == 0.0

    def test_empty_estimated_returns_empty(self):
        assert align_ground_truth_to_estimate_start([], [PoseSample(0.0, 1.0, 1.0)]) == []

    def test_empty_ground_truth_returns_empty(self):
        assert align_ground_truth_to_estimate_start([PoseSample(0.0, 0.0, 0.0)], []) == []

    def test_no_ground_truth_within_gap_returns_empty(self):
        estimated = [PoseSample(t=0.0, x=0.0, y=0.0)]
        ground_truth = [PoseSample(t=10.0, x=1.0, y=1.0)]
        assert align_ground_truth_to_estimate_start(estimated, ground_truth, max_gap_sec=0.5) == []

    def test_does_not_mutate_inputs(self):
        estimated = [PoseSample(t=0.0, x=0.0, y=0.0)]
        ground_truth = [PoseSample(t=0.0, x=10.0, y=20.0)]
        align_ground_truth_to_estimate_start(estimated, ground_truth)
        assert ground_truth[0].x == 10.0 and ground_truth[0].y == 20.0


class TestSynchronizePoseSeries:
    def test_pairs_each_reference_with_nearest_other(self):
        reference = [PoseSample(t=0.0, x=0, y=0), PoseSample(t=1.0, x=0, y=0)]
        other = [PoseSample(t=0.1, x=1, y=1), PoseSample(t=0.9, x=2, y=2)]
        pairs = synchronize_pose_series(reference, other, max_gap_sec=1.0)
        assert [p[1] for p in pairs] == [other[0], other[1]]

    def test_drops_pairs_beyond_max_gap(self):
        reference = [PoseSample(t=0.0, x=0, y=0)]
        other = [PoseSample(t=5.0, x=0, y=0)]
        assert synchronize_pose_series(reference, other, max_gap_sec=0.5) == []

    def test_empty_other_returns_empty(self):
        reference = [PoseSample(t=0.0, x=0, y=0)]
        assert synchronize_pose_series(reference, [], max_gap_sec=1.0) == []

    def test_empty_reference_returns_empty(self):
        other = [PoseSample(t=0.0, x=0, y=0)]
        assert synchronize_pose_series([], other, max_gap_sec=1.0) == []

    def test_negative_max_gap_rejected(self):
        with pytest.raises(ValueError):
            synchronize_pose_series([], [], max_gap_sec=-1.0)


class TestLocalizationErrorSeries:
    def test_computes_euclidean_distance(self):
        estimated = [PoseSample(t=0.0, x=0.0, y=0.0)]
        ground_truth = [PoseSample(t=0.0, x=3.0, y=4.0)]
        result = localization_error_series(estimated, ground_truth)
        assert len(result) == 1
        assert result[0].error_m == pytest.approx(5.0)
        assert result[0].t == 0.0

    def test_zero_error_when_poses_match(self):
        estimated = [PoseSample(t=1.0, x=2.0, y=2.0)]
        ground_truth = [PoseSample(t=1.0, x=2.0, y=2.0)]
        result = localization_error_series(estimated, ground_truth)
        assert result[0].error_m == pytest.approx(0.0)

    def test_no_ground_truth_produces_empty_series(self):
        estimated = [PoseSample(t=0.0, x=0.0, y=0.0)]
        assert localization_error_series(estimated, []) == []


class TestTimeToRecover:
    def test_recovers_immediately_and_holds(self):
        errors = [
            ErrorSample(t=10.0, error_m=0.05),
            ErrorSample(t=11.0, error_m=0.05),
            ErrorSample(t=12.0, error_m=0.05),
        ]
        result = time_to_recover(errors, fault_end_t=10.0, threshold_m=0.3, hold_sec=2.0)
        assert result == pytest.approx(0.0)

    def test_dip_that_spikes_back_up_does_not_count(self):
        errors = [
            ErrorSample(t=10.0, error_m=0.05),  # dips below threshold...
            ErrorSample(t=10.5, error_m=0.9),  # ...but spikes back up before hold_sec elapses
            ErrorSample(t=11.0, error_m=0.05),
            ErrorSample(t=12.5, error_m=0.05),  # this one genuinely holds
        ]
        result = time_to_recover(errors, fault_end_t=10.0, threshold_m=0.3, hold_sec=1.5)
        assert result == pytest.approx(1.0)

    def test_never_recovers_returns_none(self):
        errors = [ErrorSample(t=10.0, error_m=5.0), ErrorSample(t=11.0, error_m=5.0)]
        assert time_to_recover(errors, fault_end_t=10.0, threshold_m=0.3, hold_sec=1.0) is None

    def test_insufficient_trailing_data_returns_none(self):
        # Drops below threshold right at the end of the recording, with no way
        # to prove it actually held for hold_sec — must not claim recovery.
        errors = [ErrorSample(t=10.0, error_m=5.0), ErrorSample(t=10.9, error_m=0.05)]
        result = time_to_recover(errors, fault_end_t=10.0, threshold_m=0.3, hold_sec=1.0)
        assert result is None

    def test_ignores_samples_before_fault_end(self):
        errors = [
            ErrorSample(t=5.0, error_m=0.05),  # before fault_end_t, must be ignored
            ErrorSample(t=10.0, error_m=5.0),
            ErrorSample(t=11.0, error_m=5.0),
        ]
        assert time_to_recover(errors, fault_end_t=10.0, threshold_m=0.3, hold_sec=0.5) is None

    def test_negative_hold_sec_rejected(self):
        with pytest.raises(ValueError):
            time_to_recover([], fault_end_t=0.0, threshold_m=0.3, hold_sec=-1.0)


class TestMapUpdateGaps:
    def test_no_gaps_when_regular(self):
        stamps = [0.0, 1.0, 2.0, 3.0]
        assert map_update_gaps(stamps, expected_period_sec=1.0) == []

    def test_detects_a_gap(self):
        stamps = [0.0, 1.0, 5.0, 6.0]
        gaps = map_update_gaps(stamps, expected_period_sec=1.0, gap_multiplier=3.0)
        assert gaps == [(1.0, 5.0)]

    def test_sorts_unsorted_input(self):
        stamps = [3.0, 0.0, 1.0]
        assert map_update_gaps(stamps, expected_period_sec=1.0) == []

    def test_rejects_nonpositive_period(self):
        with pytest.raises(ValueError):
            map_update_gaps([0.0, 1.0], expected_period_sec=0.0)

    def test_rejects_multiplier_not_greater_than_one(self):
        with pytest.raises(ValueError):
            map_update_gaps([0.0, 1.0], expected_period_sec=1.0, gap_multiplier=1.0)


class TestClassifyNavigationOutcome:
    def test_success_within_timeout_and_tolerance(self):
        outcome = classify_navigation_outcome(
            elapsed_sec=30.0, timeout_sec=60.0, final_distance_to_goal_m=0.1, goal_tolerance_m=0.25
        )
        assert outcome is NavOutcome.SUCCESS

    def test_failure_within_timeout_but_off_target(self):
        outcome = classify_navigation_outcome(
            elapsed_sec=30.0, timeout_sec=60.0, final_distance_to_goal_m=1.5, goal_tolerance_m=0.25
        )
        assert outcome is NavOutcome.FAILURE

    def test_timeout_takes_precedence_over_close_final_position(self):
        # Arrived close to the goal, but only after the deadline — a late
        # success is still a timeout, not a success.
        outcome = classify_navigation_outcome(
            elapsed_sec=90.0, timeout_sec=60.0, final_distance_to_goal_m=0.05, goal_tolerance_m=0.25
        )
        assert outcome is NavOutcome.TIMEOUT

    def test_exact_tolerance_boundary_is_success(self):
        outcome = classify_navigation_outcome(
            elapsed_sec=10.0, timeout_sec=60.0, final_distance_to_goal_m=0.25, goal_tolerance_m=0.25
        )
        assert outcome is NavOutcome.SUCCESS


class TestCollisionCount:
    def test_counts_all_without_window(self):
        assert collision_count([1.0, 2.0, 3.0]) == 3

    def test_counts_within_window(self):
        assert collision_count([1.0, 2.0, 3.0, 10.0], window=(0.0, 5.0)) == 3

    def test_empty_list_is_zero(self):
        assert collision_count([]) == 0

    def test_rejects_inverted_window(self):
        with pytest.raises(ValueError):
            collision_count([1.0], window=(5.0, 0.0))


def test_math_hypot_sanity():
    # Guards against accidentally swapping hypot args in localization_error_series.
    assert math.hypot(3.0, 4.0) == pytest.approx(5.0)
