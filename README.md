# Nav2-SLAM Resilience Benchmark

**How much does LiDAR degradation have to worsen before a standard ROS 2 Nav2 + slam_toolbox
stack's localization, mapping, and navigation actually break down — and how well does it
recover once the sensor clears up?**

This project answers that with a repeatable, quantitative benchmark: a TurtleBot3 drives a
fixed course in Gazebo while running SLAM and autonomous navigation, its LiDAR is degraded
(noise or dropout) at swept severity levels, and the benchmark measures localization error,
mapping health, navigation success, collisions, and recovery time against Gazebo's own
ground-truth pose — not the stack's self-reported status.

## Demo / Results

The repository includes captures from baseline, 10 Hz dropout, severe dropout, and high-noise
conditions under [`media/`](media/). A validated isolated dropout trial measured 5.001 Hz on
the raw scan, 4.030 Hz during the injected fault, and 4.999 Hz after recovery. Its independently
graded localization RMSE was approximately 0.0149 m.

The full severity sweep is not yet a finished result. A failed Nav2 mission remains valid
benchmark data when the bag, fault rate, ground truth, and synchronization checks pass; this
project measures failure boundaries rather than hiding failed trials.

## What I Built

Everything under the hood is an off-the-shelf ROS 2 tool: [Gazebo
Harmonic](https://gazebosim.org/) for simulation, [Nav2](https://nav2.org/) for navigation,
[slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) for SLAM, and
[`topic_tools`](https://github.com/ros-tools/topic_tools)'s `throttle` node plus Gazebo's
built-in sensor `<noise>` model for fault injection. Nothing here reinvents any of that. My
contribution is the benchmark layer: deterministic scan routing, a fixed mission, controlled
fault sweeps, bag extraction, independent ground-truth grading, strict data-quality gates, and
the metrics used to compare runs.

## How It Works

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

## Implemented Scope

- [`nav2_slam_resilience/scenario.py`](nav2_slam_resilience/scenario.py) — a strict YAML loader
  for benchmark scenarios (fault type/severity sweep, mission goals, pass/fail thresholds).
  Rejects unknown keys, wrong types, and out-of-range values with a clear message.
- [`nav2_slam_resilience/metrics.py`](nav2_slam_resilience/metrics.py) — the pure functions the
  benchmark is actually built on: pose-series synchronization, localization error, recovery
  time (with a real "hold" requirement, not just a single dip below threshold), map-update
  staleness gaps, navigation outcome classification, and collision counting.
- 77 unit tests covering scenario validation, model patching, synchronization, localization
  error, recovery windows, collision counts, and data-quality failures (`pytest tests/unit`).

## Known Limitations

The checked-in media demonstrates individual conditions, not a statistically complete sweep.
The current benchmark is single-robot, single-world, and simulation-only. Results should not be
generalized to physical LiDAR failure behavior or hardware safety without separate experiments.

## Running It

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -e . pyyaml pytest ruff
pytest tests/unit -q
```

The full Gazebo/ROS 2 path is containerized. Run `scripts/dev_shell.sh` to enter the environment
and `scripts/run_demo.sh` for the scripted scenario. This path requires Docker and substantially
more time than the pure metrics tests.

## Tech

ROS 2 Jazzy, Nav2, slam_toolbox, Gazebo Harmonic, TurtleBot3, `topic_tools`, Python, rosbag2,
Docker, pytest, and Ruff.

## License

Apache-2.0 — see [LICENSE](LICENSE).
