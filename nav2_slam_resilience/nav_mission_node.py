"""Script a fixed sequence of Nav2 goals for one benchmark trial.

Thin wrapper around `nav2_simple_commander.robot_navigator.BasicNavigator` -
the standard, official way to script Nav2 missions headlessly. The only
project-specific pieces are: waiting on `slam_toolbox` instead of `amcl`
(this stack runs `slam:=True`, so there is no AMCL lifecycle node to wait
on), and grading each goal against a wall-clock timeout in addition to
Nav2's own TaskResult, since a mission that "succeeds" long after its
deadline should not count as a success for this benchmark.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from nav2_slam_resilience.scenario import Mission, MissionGoal, ScenarioConfig, load_scenario


@dataclass(frozen=True)
class GoalOutcome:
    goal: MissionGoal
    nav2_result: TaskResult
    elapsed_sec: float
    timed_out: bool


def _yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """Planar (roll=pitch=0) yaw -> quaternion (x, y, z, w)."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def _goal_to_pose_stamped(navigator: BasicNavigator, goal: MissionGoal) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = goal.x
    pose.pose.position.y = goal.y
    qx, qy, qz, qw = _yaw_to_quaternion(goal.yaw)
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


def run_mission(navigator: BasicNavigator, mission: Mission) -> list[GoalOutcome]:
    """Drive `navigator` through every goal in `mission`, in order.

    Stops at the first goal that fails or times out - a mission is a fixed
    sequence, and continuing past a failed leg would grade a different,
    unplanned mission rather than the one the scenario actually specifies.
    """
    outcomes: list[GoalOutcome] = []
    for goal in mission.goals:
        pose = _goal_to_pose_stamped(navigator, goal)
        navigator.goToPose(pose)

        start = time.monotonic()
        timed_out = False
        while not navigator.isTaskComplete():
            rclpy.spin_once(navigator, timeout_sec=0.1)
            if time.monotonic() - start > mission.goal_timeout_sec:
                timed_out = True
                navigator.cancelTask()
                break

        elapsed = time.monotonic() - start
        result = navigator.getResult() if not timed_out else TaskResult.FAILED
        outcomes.append(
            GoalOutcome(goal=goal, nav2_result=result, elapsed_sec=elapsed, timed_out=timed_out)
        )

        if timed_out or result != TaskResult.SUCCEEDED:
            break

    return outcomes


def _parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", required=True, help="Path to a scenario YAML file")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv if argv is not None else sys.argv[1:])
    scenario: ScenarioConfig = load_scenario(args.scenario)

    rclpy.init()
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active(localizer="slam_toolbox")

    outcomes = run_mission(navigator, scenario.mission)

    ok = len(outcomes) == len(scenario.mission.goals) and all(
        not o.timed_out and o.nav2_result == TaskResult.SUCCEEDED for o in outcomes
    )
    for i, outcome in enumerate(outcomes):
        status = "TIMEOUT" if outcome.timed_out else outcome.nav2_result.name
        navigator.get_logger().info(
            f"goal {i}: ({outcome.goal.x}, {outcome.goal.y}) -> {status} "
            f"in {outcome.elapsed_sec:.1f}s"
        )

    navigator.lifecycleShutdown()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
