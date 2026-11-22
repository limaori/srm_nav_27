#!/usr/bin/env bash
set -eo pipefail
SOURCE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SOURCE_DIR/../../.."
source script/lib/srm_lib_path_guard.sh
source install/setup.bash
export ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1 OPENBLAS_NUM_THREADS=1
exec python3 "$SOURCE_DIR/verify_sim.py"
