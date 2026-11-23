#!/usr/bin/env bash
# Recovery regression + real ROS navigation + Gazebo physics, isolated from hardware.
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
output_dir="$repo_dir/.cache/recovery-validation"
image_name="${SRM_RECOVERY_IMAGE:-srm27-recovery:humble}"
mkdir -p "$output_dir"
if ! docker image inspect "$image_name" >/dev/null 2>&1; then
  docker build -t "$image_name" -f "$repo_dir/tests/recovery/Dockerfile" "$repo_dir/tests/recovery"
fi
docker image inspect "$image_name" --format '{{.Id}}' > "$output_dir/image-id.txt"
git -C "$repo_dir" rev-parse HEAD > "$output_dir/source-head.txt"
docker run --rm -i --network none --shm-size=512m --ulimit core=0 \
  -v "$repo_dir:/repo:ro" -v "$output_dir:/out" \
  -e ROS_HOME=/out/ros-home -e ROS_LOG_DIR=/out/ros-log \
  -e ROS_LOCALHOST_ONLY=1 -e ROS_DOMAIN_ID=92 -e PYTHONDONTWRITEBYTECODE=1 \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/repo/tests/recovery/fastdds.xml \
  -e SRM_RECOVERY_OUT=/out/results -e SRM_RECOVERY_GAZEBO=1 \
  --entrypoint bash "$image_name" -s <<'INNER' 2>&1 | tee "$output_dir/validation.log"
set -eo pipefail
source /opt/ros/humble/setup.bash
cd /repo
packages=(srm27_minco_vendor srm27_qpoases_vendor srm27_minco_core srm27_minco_controller
          srm27_nav_plugins srm27_costmap_plugins srm27_nav_behavior_plugins fake_vel_transform srm27_chassis_control rm_decision_interfaces
          localization_interfaces srm27_robot_description srm27_gazebo_simulator srm27_nav_tools)
# These packages have no runtime dependency on the unrelated LIO/localization implementations.
colcon --log-base /out/colcon-log build \
  --base-paths src/planner src/controller src/simulation src/interfaces src/bringup/srm27_nav_tools \
  --build-base /out/build --install-base /out/install --packages-select "${packages[@]}" \
  --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
source /out/install/local_setup.bash
# Install the production navigation XML/config/launch assets without building unused hardware drivers.
colcon --log-base /out/colcon-log build --paths src/bringup/srm27_nav_bringup --cmake-force-configure \
  --build-base /out/build --install-base /out/install --cmake-args -DBUILD_TESTING=ON
source /out/install/local_setup.bash
for package in srm27_minco_core srm27_minco_controller srm27_costmap_plugins srm27_nav_behavior_plugins \
               fake_vel_transform srm27_chassis_control srm27_nav_bringup; do
  ctest --test-dir "/out/build/$package" -R '^test_' --output-on-failure
done
python3 -m pytest -p no:cacheprovider -q tests/recovery/test_inject_recovery_stuck.py \
  --junitxml=/out/injection.xml
python3 -m pytest -p no:cacheprovider -q \
  src/controller/srm27_minco_controller/test/test_recovery_ros.py \
  --junitxml=/out/scenarios.xml
python3 - <<'PY'
import json
from pathlib import Path
from xml.etree import ElementTree as ET
out = Path('/out')
units = [ET.parse(p).getroot() for p in (out / 'build').glob('*/test_results/*/*.gtest.xml')]
scenarios = ET.parse(out / 'scenarios.xml').getroot()
injection = ET.parse(out / 'injection.xml').getroot()
manifest = {
    'source_head': (out / 'source-head.txt').read_text().strip(),
    'image_id': (out / 'image-id.txt').read_text().strip(),
    'unit_cases': sum(len(list(root.iter('testcase'))) for root in units),
    'scenario_cases': len(list(scenarios.iter('testcase'))),
    'injection_cases': len(list(injection.iter('testcase'))),
    'gazebo_sensors': {p.parent.name: json.loads(p.read_text()) for p in (out / 'results').glob('test_gazebo*/sensors.json')},
    'result': 'passed',
    'boundary': 'isolated Nav2 and Gazebo; ground-truth odometry and known static maps, no hardware or field acceptance',
}
(out / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
print(json.dumps(manifest, ensure_ascii=False, indent=2))
PY
INNER
