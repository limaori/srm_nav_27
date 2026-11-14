#!/usr/bin/env bash
# Run the production decision stack in an isolated ROS domain; never start serial hardware.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$ROOT"
DOCKER=0
BUILD=1
NAV2=0
for arg in "$@"; do
  case "$arg" in
    --docker) DOCKER=1 ;;
    --no-build) BUILD=0 ;;
    --nav2) NAV2=1 ;;
    -h|--help)
      echo 'Usage: src/srm27_behavior/scripts/validate_decision.sh [--docker] [--no-build] [--nav2]'
      echo 'Default: build decision + mux, run core and ROS scenario regressions.'
      echo '--docker: create/use a Humble test image; --nav2: also run real Nav2/MINCO closed-loop simulation.'
      exit 0 ;;
    *) echo "Unknown argument: $arg" >&2; exit 2 ;;
  esac
done
if (( DOCKER )); then
  IMAGE="${DECISION_TEST_IMAGE:-srm27-decision:2027-referee}"
  if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    docker build -t "$IMAGE" -f src/srm27_behavior/test/Dockerfile src/srm27_behavior/test
  fi
  ARGS=()
  (( BUILD )) || ARGS+=(--no-build)
  (( ! NAV2 )) || ARGS+=(--nav2)
  exec docker run --rm --user "$(id -u):$(id -g)" \
    -v "$ROOT:/workspaces/srm_nav_27" -w /workspaces/srm_nav_27 \
    --entrypoint /bin/bash "$IMAGE" src/srm27_behavior/scripts/validate_decision.sh "${ARGS[@]}"
fi
# ROS setup files use optional unset shell variables.
set +u
source /opt/ros/humble/setup.bash
set -u
export CMAKE_PREFIX_PATH="/opt/btcpp:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="/opt/btcpp/lib:${LD_LIBRARY_PATH:-}"
BASE="$ROOT/.cache/decision-validation"
mkdir -p "$BASE/results"
export ROS_LOG_DIR="$BASE/ros-log"
export ROS_DOMAIN_ID="${DECISION_TEST_DOMAIN:-$((100 + $$ % 100))}"
export ROS_LOCALHOST_ONLY=1
export PYTHONDONTWRITEBYTECODE=1
export DECISION_TEST_OUTPUT="$BASE/results"
if (( BUILD )); then
  colcon --log-base "$BASE/colcon-log" build --packages-up-to srm27_behavior srm27_chassis_control srm27_nav_protocol \
    --build-base "$BASE/build" --install-base "$BASE/install" --parallel-workers 3 \
    --cmake-args -DBUILD_TESTING=ON 2>&1 | tee "$BASE/build.log"
  if (( NAV2 )); then
    colcon --log-base "$BASE/colcon-log" build \
      --packages-up-to srm27_minco_controller srm27_nav_plugins fake_vel_transform \
      --build-base "$BASE/build" --install-base "$BASE/install" --parallel-workers 3 \
      --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release 2>&1 | tee -a "$BASE/build.log"
    colcon --log-base "$BASE/colcon-log" build --packages-select srm27_nav_plugins \
      --build-base "$BASE/build" --install-base "$BASE/install" --parallel-workers 3 \
      --cmake-args -DBUILD_TESTING=ON -DAMENT_LINT_AUTO_EXCLUDE=ament_cmake_clang_tidy \
      2>&1 | tee -a "$BASE/build.log"
  fi
fi
set +u
source "$BASE/install/setup.bash"
set -u
ctest --test-dir "$BASE/build/srm27_behavior" -R '^test_decision_core$' --output-on-failure
ctest --test-dir "$BASE/build/srm27_nav_protocol" -R '^test_(referee_2027|feedback_protocol)$' --output-on-failure
ctest --test-dir "$BASE/build/srm27_chassis_control" -R '^test_(velocity_mix|twist_watchdog)$' --output-on-failure
python3 -m pytest -v -o "cache_dir=$BASE/pytest-cache" src/srm27_behavior/test/test_decision_ros.py \
  --junitxml="$BASE/results/decision-junit.xml"
if (( NAV2 )); then
  ctest --test-dir "$BASE/build/srm27_nav_plugins" -R '^test_omni_stopped_goal_checker$' --output-on-failure
  python3 -m pytest -v -o "cache_dir=$BASE/pytest-cache" src/srm27_behavior/test/test_decision_nav2.py \
    --junitxml="$BASE/results/nav2-junit.xml"
fi
echo "Decision validation passed. Evidence: $BASE/results"
