# Checkpoint

Running status against the milestone plan. Updated as work lands, not written in advance.

- [x] **M0-pre** — Repo scaffolded: `ament_python` package skeleton, scenario/metrics core
      modules, 61 passing unit tests, ruff clean, CI unit job, README, LICENSE.
- [x] **M0** — Docker image (`ros:jazzy-ros-base-noble` + Gazebo Harmonic + Nav2 + slam_toolbox
      + TurtleBot3 + topic_tools, all stock apt packages) builds and runs headless. Verified
      live in a running container, not just "no crash":
      - Full graph up: `gzserver` (headless, `-r -s -v2`), TurtleBot3 spawned, `slam_toolbox`
        and every Nav2 node (`bt_navigator`, `controller_server`, `planner_server`, both
        costmaps, both lifecycle managers) reach `active [3]`.
      - Real data flowing: `/scan_raw` (the real LiDAR) at ~5 Hz, `/odom` at ~50 Hz.
      - Fault-injection plumbing confirmed by measurement, not assumption: `/scan_raw` →
        `topic_tools throttle` → `/scan` (what slam_toolbox/Nav2 actually consume), with a
        30 Hz no-op throttle rate correctly passing the native ~5 Hz through unchanged
        (`ros2 topic hz` on both topics).
      - slam_toolbox genuinely mapping: a real 80×102 occupancy grid received off `/map`,
        and `map`→`odom`→`base_link` resolves via `tf2_echo`.
      - **Real bug found and understood, not yet fixed**: a scripted `NavigateToPose` goal at
        Gazebo-world coordinates `(-1.0, -0.5)` was rejected — `"Goal Coordinates ... was
        outside bounds"`. Cause: slam_toolbox anchors the `map` frame at the robot's spawn
        pose (map-frame origin = wherever the robot woke up, not Gazebo world `(0,0)`), so
        goal poses must be specified relative to spawn, not in world coordinates. This is a
        scripting bug in how M1's mission goals get expressed, not a stack problem - fix
        lands with `nav_mission_node.py` in M1.
      - Also fixed along the way: `nav2_bringup`'s `bringup_launch.py` raw-evals `slam` through
        `PythonExpression` and needs the Python-capitalized `"True"`, not `"true"` like every
        other launch arg here - see the comment in `launch/bench_headless.launch.py`.
      - New pure module + tests: `nav2_slam_resilience/model_patch.py` patches the stock
        LiDAR noise stddev in the real vendor `model.sdf` (extracted into
        `tests/unit/fixtures/` and tested against directly, not a hand-written approximation).
- [x] **M1a** — Real, scripted, no-fault mission actually completes. `nav_mission_node.py`
      (thin wrapper around `nav2_simple_commander.BasicNavigator`, waiting on `slam_toolbox`
      instead of `amcl` via `waitUntilNav2Active(localizer="slam_toolbox")`) drove all 3 goals
      in `scenarios/lidar_noise_sweep.yaml` end to end against the real headless stack:
      `goal 0: (1.5, 0.0) -> SUCCEEDED in 9.8s`, `goal 1: (1.5, 1.5) -> SUCCEEDED in 5.9s`,
      `goal 2: (0.0, 1.5) -> SUCCEEDED in 15.0s`, process exit code 0. The map-frame-vs-world-
      frame bug from M0 is fixed by treating scenario goals as spawn-relative (documented on
      `MissionGoal` in `scenario.py`).
- [x] **M1b** — Record→render pipeline works end to end and produced a real demo gif
      (`media/demo_baseline.gif`, 1.2MB, real map + traced path through all 3 waypoints).
      `scripts/run_demo.sh` launches the stack, records `/map /map_metadata /pose /odom
      /scan_raw /scan /tf /tf_static /cmd_vel /diagnostics` via `ros2 bag record`, runs the
      mission, then renders. Three real bugs found and fixed along the way:
      - `nav2_slam_resilience/bagio.py` hardcoded `storage_id="sqlite3"`, but Jazzy's
        `ros2 bag record` defaults to **mcap** - fixed to `storage_id=""` (auto-detect from
        the bag's own metadata) rather than assuming a backend.
      - `ros2 bag record` did not reliably stop on `SIGINT` without a controlling tty (which
        `docker exec` doesn't allocate) - it sat ignoring the signal indefinitely in one run.
        `run_demo.sh` now sends `SIGTERM` first, polls up to 15s, then escalates to `SIGKILL`
        as a bounded last resort, so the pipeline can never hang on this again.
      - `navigator.lifecycleShutdown()` in `nav_mission_node.py` raised
        `rclpy.executors.ExternalShutdownException` after a successful mission - removed
        entirely, since it tears down the *shared* Nav2/slam_toolbox lifecycle nodes that
        this per-trial script doesn't own anyway (that's the launch process's job).
      - `scripts/render_bag_to_video.py` originally emitted one animation frame per `/map`
        message (only 7-14 over a ~30s mission - unwatchably choppy). Rewrote it to resample
        onto a uniform real-time grid (holding the latest map, linearly interpolating pose
        between bracketing samples), so playback speed now matches actual mission duration.
- [x] **M2** — Both fault types verified to actually change stack behavior, with real trial
      gifs as evidence (`media/demo_noise_high.gif`, `demo_dropout_10hz.gif`,
      `demo_dropout_severe.gif`), and they degrade in qualitatively different ways:
      - **Noise (`lidar_noise_sweep.yaml`, stddev 0.2 vs. baseline 0.01)**: graceful
        degradation. Mission still completes, but goal 2 takes **79.7s vs. ~15s baseline**,
        and the map is visibly speckled with false-positive obstacles from noisy range
        readings (compare `demo_baseline.gif` to `demo_noise_high.gif`).
      - **Dropout (`lidar_dropout_sweep.yaml`, throttle rate)**: a sharp cliff, not a gradient.
        10 Hz (faster than the LiDAR's native 5 Hz, effectively pass-through) completes
        normally (all 3 goals SUCCEEDED). 2 Hz and 0.5 Hz both **fail almost instantly**
        (`FAILED in 0.1-0.5s`) - root cause confirmed from the container log, not assumed:
        `[tf_help]: Transform data too old when converting from odom to map` →
        `[controller_server]: Unable to transform robot pose into global plan's frame` →
        goal aborted. Below-native throttling starves slam_toolbox's `map`→`odom` correction
        rate past Nav2's TF staleness tolerance almost immediately - a real, correctly-
        attributed stack failure mode, not a script bug. This means the current dropout
        severities already bracket a real success/failure boundary somewhere between 5 Hz and
        2 Hz - worth a finer-grained sweep there specifically at M5.
      - **Real bug found and fixed along the way**: `run_demo.sh` had `set -euo pipefail`
        wrapping the mission-run step, so a mission that legitimately *fails* under a fault
        (exactly the interesting trials) aborted the whole script before the render step ever
        ran - the two most interesting trials were silently never rendered. Fixed by scoping
        `set +e`/`set -e` around just that step so failure is captured, not fatal.
- [x] **M3** — Ground truth wired and a real, trustworthy metrics-extraction pipeline works
      end to end (`results/demo_metrics_test_metrics.json` is real output, not a fixture).
      - Ground truth is Gazebo's own physics pose (`/world/default/dynamic_pose/info`,
        bridged as `/ground_truth_pose_array`) - independent of the diff-drive plugin's
        odometry integration and of slam_toolbox's estimate. Chose this over adding a
        `PosePublisher` plugin to the robot after testing both directly: the plugin's
        per-link topic makes every link indistinguishable after bridging (same `frame_id`);
        the world-level topic needs no model patching and was empirically verified (twice) to
        always carry the robot as `poses[0]`.
      - Hit and understood a real crash along the way: an invented, non-existent
        `publish_model_pose` plugin parameter caused a `std::length_error` abort inside
        gz-sim itself - fixed by using only the plugin's actual documented parameters,
        checked against gz-sim's own shipped example world rather than guessed.
      - `nav_mission_node.py --result-out` now writes real ROS/sim-time-stamped per-goal
        outcomes to JSON; `scripts/extract_metrics.py` turns one trial (bag + result JSON)
        into a metrics JSON: localization error, map-update gaps, goal outcomes.
      - **Real bug found and fixed before trusting any numbers**: the first alignment
        approach shifted the estimate and ground truth series each to *its own* first
        sample independently. Since ground truth publishes at physics-timestep rate and
        `/pose` publishes rarely, their first samples are seconds apart - baking a constant
        fake error into every measurement (a suspiciously uniform ~0.5m error on a clean
        baseline gave it away). Fixed with `align_ground_truth_to_estimate_start`: finds
        ground truth's position at the *same timestamp* as the estimate's first sample and
        shifts from there. Same baseline run now reports **mean 1.5cm / max 2.9cm**
        localization error - a physically plausible number for a working SLAM system.
- [ ] **M4** — One scenario, one severity, a handful of trials, real metrics produced and
      sanity-checked by hand.
- [ ] **M5** — Full severity sweep, aggregation, and the three headline plots.
- [ ] **M6** — Hero demo video (fault actively degrading mid-mission).
- [ ] **M7** — CI sim-smoke job.
- [ ] **M8** — README rewritten around real results; repo made public.

## Known risks (updated after M0)

- ~~TurtleBot3 + Gazebo Harmonic + Nav2 + slam_toolbox on Jazzy availability~~ — **resolved**:
  all needed packages installed and verified working from stock Jazzy apt repos on arm64.
- ~~Headless-only demo pipeline~~ — **partially resolved**: headless bringup fully works;
  the bag→render side of the pipeline is still unbuilt (M1).
- arm64 (local Docker Desktop, verified) vs. amd64 (GitHub Actions) package availability: still
  not verified on amd64 - the M7 CI sim-smoke job is the first real test of that.
- Image is large (6.75GB) - `ros-dev-tools` pulls in a big transitive dependency tree
  (RViz, VTK, Qt5, boost-all-dev) that isn't strictly needed since we never compile C++ from
  source. Worth trimming later; not urgent for local dev.

## Process note

While testing M1, a `docker run --mount ... source="$PWD"` used the shell's actual cwd at
that moment, which had silently reset to `~/Documents` (not this repo) - bind-mounting the
whole Documents folder into a local container for a few minutes before a `FileNotFoundError`
surfaced it. Caught immediately, container removed right away, nothing pushed or shared
anywhere. Going forward: docker mount sources in this repo's scripts always use the explicit
absolute repo path, never `$PWD`.

The same cwd-reset happened again later the same session (`bash scripts/run_demo.sh` failing
with "No such file or directory" because cwd had silently reverted to `~/Documents`) -
harmless that time (just a failed command), but it's evidently not a one-off. Every command
in this project now uses an absolute path rather than assuming cwd persists between shell
calls.
