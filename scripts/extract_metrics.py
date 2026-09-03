#!/usr/bin/env python3
"""Turn one recorded trial (a bag + its mission-result JSON) into a metrics JSON.

Run inside the dev container (needs rosbag2_py). Reads:
  - `/pose` (slam_toolbox's map-frame estimate, geometry_msgs/PoseWithCovarianceStamped)
  - `/ground_truth_pose_array` (Gazebo's world-frame physics pose, poses[0] is the
    robot - see config/ground_truth_bridge.yaml for why index 0)
  - `/map` (nav_msgs/OccupancyGrid, for map-update staleness)
  - the mission-result JSON written by `nav_mission_node.py --result-out`

`/pose` lives in `map` frame (anchored wherever the robot was when its first
pose estimate was produced); ground truth lives in Gazebo's world frame. The
two coincide only up to a pure translation (the robot spawns with zero
yaw), so ground truth is shifted to match the estimate's first sample at
that same real moment (`align_ground_truth_to_estimate_start`) before
comparing - not shifted independently by its own first sample, which (since
ground truth publishes far more often than `/pose`) would anchor on a
different, earlier moment and bake a fake constant error into every
subsequent measurement.

Navigation outcome is taken from the mission-result JSON (Nav2's own
TaskResult plus this benchmark's wall-clock timeout check) rather than
re-derived from ground truth position - a deliberate simplification, noted
here rather than silently assumed: the independent, ground-truth-graded
signal this script actually produces is localization error, which is the
benchmark's headline measurement.

Usage:
    python3 scripts/extract_metrics.py \\
        --bag media/bags/demo_baseline --result media/bags/demo_baseline/result.json \\
        --out results/demo_baseline_metrics.json
"""

from __future__ import annotations

import argparse
import json
import sys

from nav2_slam_resilience.bagio import read_messages
from nav2_slam_resilience.metrics import (
    PoseSample,
    align_ground_truth_to_estimate_start,
    localization_error_series,
    map_update_gaps,
)


def _load_pose_series(bag_path: str, topic: str, is_pose_array: bool) -> list[PoseSample]:
    series = []
    for bag_msg in read_messages(bag_path, topics=[topic]):
        t = bag_msg.timestamp_ns / 1e9
        if is_pose_array:
            if not bag_msg.msg.poses:
                continue
            p = bag_msg.msg.poses[0].position
        else:
            p = bag_msg.msg.pose.pose.position
        series.append(PoseSample(t=t, x=p.x, y=p.y))
    return sorted(series, key=lambda s: s.t)


def _load_map_stamps(bag_path: str) -> list[float]:
    return sorted(
        bag_msg.timestamp_ns / 1e9 for bag_msg in read_messages(bag_path, topics=["/map"])
    )


def extract(bag_path: str, result_path: str, expected_map_period_sec: float = 1.0) -> dict:
    with open(result_path) as f:
        mission_result = json.load(f)

    estimated = _load_pose_series(bag_path, "/pose", is_pose_array=False)
    ground_truth_raw = _load_pose_series(bag_path, "/ground_truth_pose_array", is_pose_array=True)
    ground_truth = align_ground_truth_to_estimate_start(estimated, ground_truth_raw)

    errors = localization_error_series(estimated, ground_truth)
    map_stamps = _load_map_stamps(bag_path)
    gaps = map_update_gaps(map_stamps, expected_map_period_sec) if len(map_stamps) >= 2 else []

    error_values = [e.error_m for e in errors]
    return {
        "schema_version": 1,
        "scenario_name": mission_result["scenario_name"],
        "mission_ok": mission_result["ok"],
        "goals": mission_result["goals"],
        "localization_error": {
            "count": len(error_values),
            "mean_m": sum(error_values) / len(error_values) if error_values else None,
            "max_m": max(error_values) if error_values else None,
        },
        "map_update_gaps_sec": [
            {"start": start, "end": end, "duration_sec": end - start} for start, end in gaps
        ],
        "sample_counts": {
            "estimated_pose": len(estimated),
            "ground_truth_pose": len(ground_truth),
            "map_updates": len(map_stamps),
        },
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, help="Path to the recorded bag directory")
    parser.add_argument("--result", required=True, help="Path to the mission-result JSON")
    parser.add_argument("--out", required=True, help="Output metrics JSON path")
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])

    metrics = extract(args.bag, args.result)
    with open(args.out, "w") as f:
        json.dump(metrics, f, indent=2)
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
