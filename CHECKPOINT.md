# Checkpoint

Running status against the milestone plan. Updated as work lands, not written in advance.

- [x] **M0-pre** — Repo scaffolded: `ament_python` package skeleton, scenario/metrics core
      modules, 61 passing unit tests, ruff clean, CI unit job, README, LICENSE.
- [ ] **M0** — Docker image with ROS 2 Jazzy + Gazebo Harmonic + Nav2 + slam_toolbox +
      TurtleBot3 builds and runs headless; a scripted no-fault mission completes.
- [ ] **M1** — Baseline demo + record→render pipeline (bag → gif), no faults.
- [ ] **M2** — Fault wiring: LiDAR noise (xacro arg) + throttle-based dropout, both verified
      observable from a recorded bag.
- [ ] **M3** — `scripts/extract_metrics.py`: bag → per-trial metrics JSON.
- [ ] **M4** — One scenario, one severity, a handful of trials, real metrics produced and
      sanity-checked by hand.
- [ ] **M5** — Full severity sweep, aggregation, and the three headline plots.
- [ ] **M6** — Hero demo video (fault actively degrading mid-mission).
- [ ] **M7** — CI sim-smoke job.
- [ ] **M8** — README rewritten around real results; repo made public.

## Known risks (from planning, to resolve early)

- TurtleBot3 + Gazebo Harmonic + Nav2 + slam_toolbox on Jazzy: community precedent found
  (westpoint-robotics/wp_turtlebot3, darshmenon/rosnav), not yet verified in *this* repo's own
  Docker image.
- Headless-only demo pipeline (no live GUI capture): bag → post-hoc render, not yet built.
- arm64 (local Docker Desktop) vs. amd64 (GitHub Actions) package availability: not yet
  verified for `ros-gz-*` specifically.
