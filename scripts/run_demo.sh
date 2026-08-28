#!/bin/bash
# Run one no-fault mission inside the dev container, recording a bag of it,
# then render the bag to a demo gif. Requires scripts/build.sh to have been
# run at least once (or run it now - the image is reused if unchanged).
#
# Usage: scripts/run_demo.sh [scenario_yaml] [bag_name]
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCENARIO="${1:-scenarios/lidar_noise_sweep.yaml}"
BAG_NAME="${2:-demo_baseline}"
CONTAINER_NAME="nav2_slam_resilience_demo_$$"

cleanup() {
  docker rm -f "$CONTAINER_NAME" >/dev/null 2>&1 || true
}
trap cleanup EXIT

docker run -d --name "$CONTAINER_NAME" \
  --mount type=bind,source="$REPO_ROOT",target=/workspace/src/nav2_slam_resilience \
  --workdir /workspace \
  nav2-slam-resilience:dev bash -c "
    source /opt/ros/jazzy/setup.bash
    colcon build --symlink-install --packages-select nav2_slam_resilience
    source install/setup.bash
    ros2 launch nav2_slam_resilience bench_headless.launch.py
  " >/dev/null

echo "waiting for the stack to become active..."
for _ in $(seq 1 90); do
  if docker exec "$CONTAINER_NAME" bash -c \
      "source /opt/ros/jazzy/setup.bash && timeout 3 ros2 lifecycle get /bt_navigator 2>/dev/null" \
      | grep -q active; then
    break
  fi
  sleep 1
done

echo "recording bag + running mission..."
docker exec "$CONTAINER_NAME" bash -c "
  source /opt/ros/jazzy/setup.bash
  source /workspace/install/setup.bash
  mkdir -p /workspace/src/nav2_slam_resilience/media/bags
  rm -rf /workspace/src/nav2_slam_resilience/media/bags/$BAG_NAME
  ros2 bag record -o /workspace/src/nav2_slam_resilience/media/bags/$BAG_NAME \
    /map /map_metadata /pose /odom /scan_raw /scan /tf /tf_static \
    /cmd_vel /diagnostics &
  BAGPID=\$!
  sleep 2
  timeout 150 ros2 run nav2_slam_resilience nav_mission_node --scenario /workspace/src/nav2_slam_resilience/$SCENARIO
  MISSION_RC=\$?
  sleep 2
  # ros2 bag record does not reliably stop on SIGINT without a controlling
  # tty (which docker exec doesn't allocate) - observed it sit ignoring
  # SIGINT indefinitely; SIGTERM actually stops it. Escalate to SIGKILL as a
  # bounded last resort so this can never hang the pipeline.
  kill -TERM \$BAGPID 2>/dev/null || true
  for _ in \$(seq 1 15); do
    kill -0 \$BAGPID 2>/dev/null || break
    sleep 1
  done
  kill -KILL \$BAGPID 2>/dev/null || true
  wait \$BAGPID 2>/dev/null || true
  exit \$MISSION_RC
"
MISSION_RC=$?

echo "mission exit code: $MISSION_RC"
echo "bag written to media/bags/$BAG_NAME"

echo "rendering demo gif..."
docker exec "$CONTAINER_NAME" bash -c "
  source /opt/ros/jazzy/setup.bash
  source /workspace/install/setup.bash
  cd /workspace/src/nav2_slam_resilience
  python3 scripts/render_bag_to_video.py \
    --bag media/bags/$BAG_NAME \
    --out media/$BAG_NAME.gif
"

echo "gif written to media/$BAG_NAME.gif"
exit "$MISSION_RC"
