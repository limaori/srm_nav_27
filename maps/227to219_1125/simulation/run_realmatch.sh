#!/usr/bin/env bash
# =============================================================
# 227to219_1125 仿真 —— **与实车同链**版
#
# 与 run.sh 的区别只在参数文件：
#   nav2_params_yastar_realmatch.yaml（由 build_realmatch_params.py 生成）
#     * 平滑器 = MinimumSnapSmoother（不再是 SimpleSmoother）
#     * 行为树 = navigate_to_pose_w_smoothing.xml（含 SmoothPath，平滑器真的会被调用）
#     * costmap 外形 0.30 / inflation 0.30（对齐实车）
#     * controller 的 10 个差异项全部 = 实车（corner_speed_ratio 0.45 等）
#
# 用途：在仿真里复现实车的拐弯/净空行为，调出来的参数能直接搬回实车。
# 用法: ./maps/227to219_1125/simulation/run_realmatch.sh [--rebuild]
# =============================================================
set -eo pipefail
SCENE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export SCENE_PARAMS_OVERRIDE="$SCENE_DIR/nav2_params_yastar_realmatch.yaml"
if [ ! -f "$SCENE_PARAMS_OVERRIDE" ]; then
  python3 "$SCENE_DIR/build_realmatch_params.py"
fi
exec "$SCENE_DIR/run.sh" "$@"
