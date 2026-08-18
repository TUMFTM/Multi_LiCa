#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." &>/dev/null && pwd)"

docker build \
  --file "$REPO_ROOT/Dockerfile" \
  --tag tum.ftm.multi_lidar_calibration:latest \
  "$REPO_ROOT"
