# Handoff: nav2-slam-resilience

Context for picking up this project cold — written for another agent (or a future session)
with no memory of how this got here. Read this before touching anything.

## What this project is

**Repo:** https://github.com/hnaboulsi/nav2-slam-resilience (private, GitHub user `hnaboulsi`)
**Local path:** `/Users/naboulsi/Documents/nav2-slam-resilience`
**Docker image:** `nav2-slam-resilience:dev` (built locally, ~6.75GB, arm64 native on this Mac)

**The problem it answers:** every real mobile robot's LiDAR degrades in the field (dust, rain,
vibration, dropped packets). Standard demos of ROS 2 Nav2 + slam_toolbox only ever run against
a clean sensor — nobody publishes a clear, quantitative answer to "how bad does the LiDAR have
to get before this stack stops localizing correctly, mapping correctly, or navigating safely —
and if the sensor recovers, does the robot recover too?" This project is a **repeatable
benchmark** that answers that with real, ground-truth-graded numbers, not a demo.

**Why it's built this way (context that shapes every decision below):** this started as a
*different*, more abstract project (`ros2_resilience` — a custom Python fault-injection testing
harness against a toy odometry stream, no simulator, no visual output). The user — a senior
undergrad building a portfolio piece to get hired for robotics SWE *and* general SWE roles —
correctly felt that project read as "distributed-systems test infra wearing a ROS costume," not
applied robotics, and wouldn't demo well to anyone. This repo is the pivot: same intellectual
core (independent evidence, no component grades its own homework, deadline-graded assertions),
retargeted at a **real, standard, recognizable applied stack** (Gazebo + TurtleBot3 + Nav2 +
slam_toolbox) using **only off-the-shelf mechanisms** for fault injection (Gazebo's own SDF
`<noise>` element, `topic_tools throttle`) rather than custom fault-injection code — deliberately,
per explicit user instruction, so the engineering story is "I know how to use the real ecosystem
tools to answer a real question," not "I reinvented a framework." The old repo
(`hnaboulsi/ros2_resilience`) still exists on GitHub but is abandoned; nothing here reuses its
code.

**Full design rationale**, if you need more than this summary, is in the (now-executed) plan at
the top of this conversation's history — but CHECKPOINT.md below is the living source of truth
for what's actually built, since the plan predates several real course-corrections.

## Current status — read CHECKPOINT.md, it's kept honest and current

**M0 through M3 are done, verified with real evidence, not assumptions:**
- **M0**: Gazebo Harmonic + TurtleBot3 + slam_toolbox + Nav2 running headless in Docker, full
  graph reaching `active`.
- **M1**: a scripted mission actually completes end to end; a bag→gif rendering pipeline
  produces real demo videos (`media/demo_baseline.gif` and others).
- **M2**: both fault types (LiDAR noise via a patched model, LiDAR dropout via `topic_tools
  throttle`) verified to *actually change stack behavior* — and they fail in two different,
  genuinely interesting ways: noise degrades gracefully (mission slows 5×, map gets speckled),
  dropout hits a sharp cliff (works at/above the LiDAR's native 5 Hz, fails almost instantly
  below it via Nav2 TF-staleness).
- **M3**: real ground truth (Gazebo's own physics pose, independent of odometry/SLAM) wired in,
  and `scripts/extract_metrics.py` produces real, sanity-checked localization-error numbers
  (1.5cm mean / 2.9cm max on a clean baseline — physically plausible, not a placeholder).

**Not started:** M4 (repeated trials at one severity), M5 (full severity sweep + aggregation +
the three headline plots), M6 (hero demo video — fault degrading mid-mission), M7 (CI sim-smoke
job), M8 (README rewrite around real results + make the repo public).

**Read `CHECKPOINT.md` in full before starting new work.** It documents ~10 real bugs found and
fixed so far, each with root cause — several are the kind of thing that would silently corrupt
results if reintroduced (e.g., two different pose-series time-alignment bugs, a bag storage
format assumption, a signal-handling assumption). Do not re-derive these from scratch; read what's
already there.

## Architecture (what exists, one level of detail)

```
nav2_slam_resilience/          # ament_python package, mostly pure logic + thin ROS glue
  scenario.py                  # strict YAML scenario loader (fault sweep, mission, thresholds)
  metrics.py                   # pure functions: localization error, alignment, recovery time,
                                #   map staleness, nav outcome classification, collisions
  model_patch.py                # validated text-patch of the stock TurtleBot3 LiDAR noise stddev
  bagio.py                     # rosbag2_py-based bag reader, shared by scripts below
  nav_mission_node.py          # scripts a fixed goal sequence via nav2_simple_commander,
                                #   writes a JSON result (--result-out)
launch/bench_headless.launch.py # composes stock package launch files (ros_gz_sim, turtlebot3_gazebo,
                                 #   slam_toolbox, nav2_bringup) + the fault path + ground truth bridge
config/
  ground_truth_bridge.yaml     # bridges Gazebo's physics pose as ground truth
scenarios/                     # two real scenario YAMLs (lidar_noise_sweep, lidar_dropout_sweep)
scripts/
  build.sh, dev_shell.sh       # docker helpers
  run_demo.sh                  # THE pipeline: launch stack, record bag, run mission, render gif,
                                #   extract metrics — end to end, one command
  render_bag_to_video.py       # bag -> demo gif (uniform-time-grid resampling, not raw msg rate)
  extract_metrics.py           # bag + mission result -> metrics JSON
docker/Dockerfile              # ros:jazzy-ros-base-noble + Gazebo Harmonic + Nav2 + slam_toolbox
                                #   + TurtleBot3 + topic_tools, all stock apt packages
tests/unit/                    # 77 passing tests, pure Python, no ROS needed
media/, results/                # demo gifs and metrics JSON produced by real runs
```

**Fault mechanism** (both are standard tools, no custom fault-injection framework):
- **Noise**: `model_patch.patch_lidar_noise_stddev` edits the real vendor TurtleBot3 `model.sdf`'s
  LiDAR `<noise><stddev>` at spawn time (validated regex patch, tested against the actual
  extracted vendor file in `tests/unit/fixtures/`).
- **Dropout**: a `topic_tools throttle` node sits between Gazebo's real LiDAR (`/scan_raw`,
  untouched) and what slam_toolbox/Nav2 actually consume (`/scan`, throttled) — so `/scan_raw`
  stays available the whole time as an independent check on what was really applied.

**Ground truth**: Gazebo's own physics-computed pose for every dynamic entity
(`/world/default/dynamic_pose/info`, published automatically by gz-sim core — no plugin needed),
bridged as `/ground_truth_pose_array` (`geometry_msgs/PoseArray`). The robot is always
`poses[0]` — verified empirically twice (`gz topic -e`), not assumed; see CHECKPOINT.md for why
a `PosePublisher` plugin approach was tried and rejected first.

**Goal poses are relative to the robot's spawn pose**, not Gazebo world coordinates —
slam_toolbox anchors the `map` frame at wherever the robot woke up. This tripped up the very
first navigation test (M0) and is now documented directly on `MissionGoal` in `scenario.py`.

## How to run things

```bash
# One-time (or after Dockerfile changes):
bash /Users/naboulsi/Documents/nav2-slam-resilience/scripts/build.sh

# The whole pipeline, one command - launch, record, mission, render, extract metrics:
bash /Users/naboulsi/Documents/nav2-slam-resilience/scripts/run_demo.sh \
  scenarios/lidar_noise_sweep.yaml demo_baseline 0.01 30.0
#  args:                          scenario_yaml  bag_name  noise_stddev  throttle_rate_hz
# noise_stddev 0.01 = stock/no-fault; throttle_rate_hz 30.0 = faster than native 5Hz = no-op.
# Output: media/<bag_name>.gif, results/<bag_name>_metrics.json, media/bags/<bag_name>/ (bag, gitignored)

# Local (no Docker) fast unit tests:
cd /Users/naboulsi/Documents/nav2-slam-resilience && source .venv/bin/activate
python -m pytest tests/unit -q   # 77 tests, ~0.04s, no ROS needed
ruff check nav2_slam_resilience tests scripts && ruff format --check nav2_slam_resilience tests scripts
```

**A `run_demo.sh` call takes 2-5 real minutes** (stack warmup ~5-10s, mission ~10-90s depending
on severity, bag close bounded at ≤15s, render/extract a few seconds). Always background it
(`run_in_background: true` if you're Claude Code) and wait for the completion notification or
poll the log — don't block a whole turn on it.

## Critical process notes (do not skip)

1. **The shell's cwd resets unpredictably between tool calls in this environment.** This bit us
   twice — once causing a `docker run --mount source="$PWD"` to bind-mount the wrong directory
   entirely (the user's whole `~/Documents`, including personal files, for a few minutes before
   a `FileNotFoundError` surfaced it; caught and the container removed immediately, nothing was
   pushed/shared). **Always use the absolute path
   `/Users/naboulsi/Documents/nav2-slam-resilience` explicitly** — never rely on cwd persistence,
   never bind-mount `$PWD` without first confirming it, in every docker/bash invocation.
2. **`ros2 bag record` does not reliably stop on `SIGINT`** without a controlling tty (which
   `docker exec` doesn't allocate) — it can sit ignoring the signal indefinitely. `run_demo.sh`
   already handles this correctly (`SIGTERM` first, bounded poll, `SIGKILL` last resort) — don't
   revert to plain `kill -INT` if you touch that script.
3. **A mission that *fails* under a fault is often the most interesting trial** (that's the
   whole point of a severity sweep). `run_demo.sh` deliberately does not let `set -e` abort the
   pipeline when the mission step returns non-zero — it still needs its bag rendered and its
   metrics extracted. Don't "fix" this back to aborting on failure.
4. **Always `docker rm -f` test containers when done** and check `docker ps -a` before assuming
   a clean slate — several debugging sessions in this project's history left probe containers
   running (`probe_pose`, `debug_dropout`, etc.) that had to be cleaned up.
5. **Jazzy's `ros2 bag record` defaults to mcap storage, not sqlite3.** If you're reading bags,
   use `rosbag2_py.StorageOptions(uri=path, storage_id="")` (empty string = auto-detect from the
   bag's own metadata) — hardcoding `"sqlite3"` fails with a cryptic "file is not a database"
   error. This is what `nav2_slam_resilience/bagio.py` already does.

## What's next (in order)

1. **M4**: run one scenario at one severity for a handful of repeated trials (reuse
   `scripts/run_demo.sh`'s pattern, or extend it to loop), confirm results are sane and
   trial-to-trial variation looks reasonable — the sanity-check gate before trusting a full sweep.
2. **M5**: the actual severity sweep — multiple severities × N repeated trials per fault type,
   aggregate `results/*.json` outputs, produce the three headline plots (localization error vs.
   severity, navigation success rate vs. severity, time-to-recover vs. severity). Per M2's
   finding, the dropout sweep specifically deserves finer resolution between 5 Hz and 2 Hz, where
   the real success/failure boundary was found to sit.
3. **M6**: re-record the hero demo gif with a fault actively degrading *mid-mission* (not from
   trial start) — this is the flagship artifact for the eventual README.
4. **M7**: a lightweight `gazebo-smoke` GitHub Actions CI job (short headless run verifying the
   graph wires up) — note CI runs on amd64, only arm64 has been verified so far; this is the
   first real test of that.
5. **M8**: rewrite `README.md` around real results (lead with a gif + the three plots, not
   prose), then flip the repo from private to public.

## Known limitations to be upfront about eventually (README material)

- No per-trial process isolation yet (fresh Gazebo/Nav2/slam_toolbox instance per trial) — M4/M5
  need to decide whether that's necessary or whether resetting robot pose within one long-lived
  instance is good enough.
- The Docker image is large (6.75GB) because `ros-dev-tools` pulls in RViz/VTK/Qt5/boost-all-dev
  that isn't strictly needed (no C++ is compiled here). Not urgent, but a legitimate trim later.
- Navigation outcome (success/failure/timeout) is currently taken from Nav2's own `TaskResult`
  plus a wall-clock timeout check, not independently re-derived from ground truth position. The
  genuinely independent, ground-truth-graded signal this pipeline produces is localization
  error — that's the headline measurement, and it's the one to lead with.
