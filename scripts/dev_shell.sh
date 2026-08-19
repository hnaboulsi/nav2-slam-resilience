#!/bin/bash
# Open an interactive shell in the dev container with the repo mounted.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
docker run --rm -it \
  --mount type=bind,source="$PWD",target=/workspace/src/nav2_slam_resilience \
  --workdir /workspace/src/nav2_slam_resilience \
  nav2-slam-resilience:dev bash
