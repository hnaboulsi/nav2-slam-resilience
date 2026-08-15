# nav2-slam-resilience

**How much does LiDAR degradation have to worsen before a standard ROS 2 Nav2 + slam_toolbox
stack's localization, mapping, and navigation actually break down — and how well does it
recover once the sensor clears up?**

This project answers that with a repeatable, quantitative benchmark: a TurtleBot3 drives a
fixed course in Gazebo while running SLAM and autonomous navigation, its LiDAR is degraded
(noise or dropout) at swept severity levels, and the benchmark measures localization error,
mapping health, navigation success, collisions, and recovery time against Gazebo's own
ground-truth pose — not the stack's self-reported status.

> **Status: early build.** The benchmark's metrics engine and scenario configuration are
> implemented and tested (below). The Gazebo/Nav2/slam_toolbox simulation stack, fault wiring,
> and the actual sweep results are in progress — this section will be replaced with real
> plots, a demo video, and measured numbers once they exist. No placeholder results are
> published here in the meantime.

## Why standard packages, not a custom framework

Everything under the hood is an off-the-shelf ROS 2 tool: [Gazebo
Harmonic](https://gazebosim.org/) for simulation, [Nav2](https://nav2.org/) for navigation,
[slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) for SLAM, and
[`topic_tools`](https://github.com/ros-tools/topic_tools)'s `throttle` node plus Gazebo's
built-in sensor `<noise>` model for fault injection. Nothing here reinvents any of that. The
contribution is the benchmark itself: a fixed course, a controlled fault sweep, ground-truth
grading, and the resulting robustness curves — a measurement no off-the-shelf tool produces on
its own.

## Architecture

```
                actual /scan (Gazebo, Gaussian-noise sensor model)
                         │
                         ▼
        topic_tools throttle (severity knob: Hz)
                         │
                         ▼
                 /scan_for_slam ───────────────► slam_toolbox (sync mode) ──► /map, map→odom TF
                         │                                                          │
                         │                                                          ▼
                         │                                                  Nav2 (SLAM-mode bringup)
                         │                                                          │
                         ▼                                                          ▼
                Gazebo ground-truth pose ◄──────── independent comparison ──── estimated pose / nav result
                         │
                         ▼
              metrics.py (pure functions) ──► per-trial JSON ──► plots (localization error,
                                                                    nav success rate, time-to-recover
                                                                    vs. severity)
```

Ground truth, mapping health, and navigation outcome are always graded against an independent
signal (Gazebo physics, wall-clock deadlines, ground-truth position) — never against the
stack's own diagnostics.

## What's implemented so far

- [`nav2_slam_resilience/scenario.py`](nav2_slam_resilience/scenario.py) — a strict YAML loader
  for benchmark scenarios (fault type/severity sweep, mission goals, pass/fail thresholds).
  Rejects unknown keys, wrong types, and out-of-range values with a clear message.
- [`nav2_slam_resilience/metrics.py`](nav2_slam_resilience/metrics.py) — the pure functions the
  benchmark is actually built on: pose-series synchronization, localization error, recovery
  time (with a real "hold" requirement, not just a single dip below threshold), map-update
  staleness gaps, navigation outcome classification, and collision counting.
- 61 unit tests, all passing, no ROS required to run them (`pytest tests/unit`).

## What's next

Gazebo Harmonic + TurtleBot3 + slam_toolbox + Nav2 running headless in Docker, LiDAR fault
wiring, ground-truth bridging, a `ros2 bag`-based metrics extraction script, and the full
severity-sweep benchmark producing real, committed results and a demo video. Tracked in
[CHECKPOINT.md](CHECKPOINT.md).

## Running the tests

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -e . pyyaml pytest ruff
pytest tests/unit -q
```

## License

Apache-2.0 — see [LICENSE](LICENSE).
