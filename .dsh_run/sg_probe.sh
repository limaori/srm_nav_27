#!/usr/bin/env bash
set -u
cd /home/srm/srm_nav_27
set +u; source install/setup.bash >/dev/null 2>&1; set -u
rm -rf .dsh_run/sg_probe
echo "=== configure ==="
time cmake -S src/localization/backend/small_gicp_relocalization -B .dsh_run/sg_probe \
  -DCMAKE_BUILD_TYPE=Release \
  -DFETCHCONTENT_SOURCE_DIR_SMALL_GICP=/home/srm/srm_nav_27/.deps/small_gicp 2>&1 | tail -15
echo "=== compile (-j8) ==="
time cmake --build .dsh_run/sg_probe -j8 2>&1 | tail -25
