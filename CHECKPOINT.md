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
- [ ] **M1b** — Build the record→render pipeline (bag → gif) around this working mission.
- [ ] **M2** — Verify both fault severities (noise stddev sweep, throttle-rate sweep) actually
      change stack behavior, observable from a recorded bag.
- [ ] **M3** — `scripts/extract_metrics.py`: bag → per-trial metrics JSON.
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
