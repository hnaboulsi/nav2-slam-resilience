#!/bin/bash
# Build the dev/CI Docker image. Run from the repo root or anywhere.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
docker build -f docker/Dockerfile -t nav2-slam-resilience:dev .
