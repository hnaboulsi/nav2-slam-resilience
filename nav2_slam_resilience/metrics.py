"""Pure functions that turn recorded time-series data into benchmark metrics.

Nothing in this module touches ROS, rosbags, or a live process. It takes
plain `PoseSample` sequences and event timestamps (produced elsewhere, in
`scripts/extract_metrics.py`, by reading a recorded `ros2 bag`) and computes:

- localization error over time (SLAM estimate vs. Gazebo ground truth)
- time-to-recover after a fault clears
- map-update staleness gaps
- navigation outcome classification (success / failure / timeout)
- collision counts

Keeping this pure and ROS-free is deliberate: it is the part of the project
that is fast and reliable to unit test, and it is the part that actually
defines what the benchmark measures. Everything else (Gazebo, Nav2,
slam_toolbox, topic_tools) is off-the-shelf; this module is the contribution.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum


@dataclass(frozen=True)
class PoseSample:
    """A single 2D pose observation at a point in time.

    `t` is seconds on whatever single clock the caller has synchronized all
    series to (for a bag-derived series, that's the bag's recorded time).
    """

    t: float
    x: float
    y: float


@dataclass(frozen=True)
class ErrorSample:
    t: float
    error_m: float


class NavOutcome(Enum):
    SUCCESS = "success"
    FAILURE = "failure"
    TIMEOUT = "timeout"


def align_ground_truth_to_estimate_start(
    estimated: list[PoseSample],
    ground_truth: list[PoseSample],
    max_gap_sec: float = 0.5,
) -> list[PoseSample]:
    """Shift `ground_truth` so it coincides with `estimated`'s first sample.

    The SLAM estimate (`/pose`) lives in the `map` frame, which slam_toolbox
    anchors at wherever the robot happened to be when its first pose
    estimate was produced; ground truth (`/ground_truth_pose_array`) is
    Gazebo's world frame, anchored at the world's own origin and sampled far
    more often. The two frames coincide, up to a pure translation, only
    because the robot spawns with zero yaw.

    Naively shifting each series by *its own* first sample (an earlier
    version of this function) silently assumes both series' first samples
    were captured at the same instant - false here: ground truth publishes
    at physics-timestep rate while `/pose` publishes rarely, so "first
    sample" can be seconds apart, and that gap shows up as a constant,
    fake localization error baked into every subsequent measurement. This
    instead finds ground truth's position at the *same timestamp* as
    `estimated`'s first sample (nearest-match, via `synchronize_pose_series`)
    and shifts the whole ground-truth series by the offset needed to make
    that matched point equal `estimated[0]` - tying both series to one real,
    shared moment instead of two independently-chosen ones.

    Returns an empty list (nothing to compare against) if `estimated` is
    empty or no ground-truth sample falls within `max_gap_sec` of its first
    timestamp.
    """
    if not estimated or not ground_truth:
        return []
    matches = synchronize_pose_series([estimated[0]], ground_truth, max_gap_sec)
    if not matches:
        return []
    _, matched_gt = matches[0]
    dx = estimated[0].x - matched_gt.x
    dy = estimated[0].y - matched_gt.y
    return [PoseSample(t=p.t, x=p.x + dx, y=p.y + dy) for p in ground_truth]


def synchronize_pose_series(
    reference: list[PoseSample],
    other: list[PoseSample],
    max_gap_sec: float,
) -> list[tuple[PoseSample, PoseSample]]:
    """Pair each `reference` sample with its nearest-in-time `other` sample.

    Two independently-published streams (e.g. slam_toolbox's pose estimate
    and Gazebo's ground truth) are never sampled at identical timestamps, so
    comparing them requires nearest-neighbor matching, not zip(). A pair is
    dropped if the nearest match is farther than `max_gap_sec` away, so a
    stalled stream produces missing data rather than a misleadingly-matched
    stale error.

    Both inputs are assumed sorted by `t` (true for anything read off a bag
    in recorded order).
    """
    if max_gap_sec < 0:
        raise ValueError("max_gap_sec must be >= 0")
    if not other:
        return []

    pairs = []
    j = 0
    for sample in reference:
        while j + 1 < len(other) and abs(other[j + 1].t - sample.t) <= abs(other[j].t - sample.t):
            j += 1
        candidate = other[j]
        if abs(candidate.t - sample.t) <= max_gap_sec:
            pairs.append((sample, candidate))
    return pairs


def localization_error_series(
    estimated: list[PoseSample],
    ground_truth: list[PoseSample],
    max_gap_sec: float = 0.5,
) -> list[ErrorSample]:
    """Euclidean position error between an estimate and ground truth over time."""
    pairs = synchronize_pose_series(estimated, ground_truth, max_gap_sec)
    return [
        ErrorSample(t=est.t, error_m=math.hypot(est.x - gt.x, est.y - gt.y)) for est, gt in pairs
    ]


def time_to_recover(
    errors: list[ErrorSample],
    fault_end_t: float,
    threshold_m: float,
    hold_sec: float,
) -> float | None:
    """Seconds from `fault_end_t` until error drops under `threshold_m` and holds.

    "Holds" means every sample in [recover_t, recover_t + hold_sec] stays
    under threshold — a single dip below threshold that immediately spikes
    back up does not count as recovery. Returns None if the series never
    recovers (including if there isn't enough data after fault_end_t to prove
    a full hold window), which the caller should treat as a real result
    ("did not recover in the observed window"), not a missing-data error.
    """
    if hold_sec < 0:
        raise ValueError("hold_sec must be >= 0")

    after = sorted((e for e in errors if e.t >= fault_end_t), key=lambda e: e.t)
    for i, candidate in enumerate(after):
        if candidate.error_m >= threshold_m:
            continue
        hold_until = candidate.t + hold_sec
        window = after[i:]
        if not window or window[-1].t < hold_until:
            # Not enough trailing data to prove the hold window was satisfied.
            continue
        if all(e.error_m < threshold_m for e in window if e.t <= hold_until):
            return candidate.t - fault_end_t
    return None


def map_update_gaps(
    map_stamps: list[float],
    expected_period_sec: float,
    gap_multiplier: float = 3.0,
) -> list[tuple[float, float]]:
    """Intervals where consecutive `/map` updates were slower than expected.

    A gap is reported when the interval between two consecutive map
    timestamps exceeds `expected_period_sec * gap_multiplier` — a simple,
    honest staleness check, not a model of what "good" mapping looks like.
    """
    if expected_period_sec <= 0:
        raise ValueError("expected_period_sec must be > 0")
    if gap_multiplier <= 1:
        raise ValueError("gap_multiplier must be > 1")

    threshold = expected_period_sec * gap_multiplier
    stamps = sorted(map_stamps)
    gaps = []
    for prev, curr in zip(stamps, stamps[1:], strict=False):
        if curr - prev > threshold:
            gaps.append((prev, curr))
    return gaps


def classify_navigation_outcome(
    *,
    elapsed_sec: float,
    timeout_sec: float,
    final_distance_to_goal_m: float,
    goal_tolerance_m: float,
) -> NavOutcome:
    """Grade a mission against ground truth, not the navigation stack's own status.

    Timeout takes precedence over a technically-close final position: a
    mission that only reached the goal after the deadline is a timeout, not a
    late success.
    """
    if elapsed_sec > timeout_sec:
        return NavOutcome.TIMEOUT
    if final_distance_to_goal_m <= goal_tolerance_m:
        return NavOutcome.SUCCESS
    return NavOutcome.FAILURE


def collision_count(
    collision_stamps: list[float],
    window: tuple[float, float] | None = None,
) -> int:
    """Count collision events, optionally restricted to a [start, end] window."""
    if window is None:
        return len(collision_stamps)
    start, end = window
    if end < start:
        raise ValueError("window end must be >= start")
    return sum(1 for t in collision_stamps if start <= t <= end)
