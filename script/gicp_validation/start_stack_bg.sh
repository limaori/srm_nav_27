#!/usr/bin/env bash
set -u
WS=/home/srm/srm_nav_27
export GICP_DOMAIN_ID=181
bash "$WS/script/gicp_validation/start_validation_stack.sh"
