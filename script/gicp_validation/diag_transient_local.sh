#!/usr/bin/env bash
set -uo pipefail
WS=/home/srm/srm_nav_27
export ROS_DOMAIN_ID=212
set +u
source "$WS/tmp/gicp_validation/env.sh"
set -u
cd "$WS"
python3 tmp/gicp_validation/tl_test/pub2.py > tmp/gicp_validation/logs/tl_pub2.log 2>&1 &
sleep 8
echo "--- 普通订阅者 depth=1"
python3 tmp/gicp_validation/tl_test/sub3.py 1 2>&1 | tail -2
echo "--- 普通订阅者 depth=100"
python3 tmp/gicp_validation/tl_test/sub3.py 100 2>&1 | tail -2
