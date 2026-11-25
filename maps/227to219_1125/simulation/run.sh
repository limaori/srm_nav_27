#!/usr/bin/env bash
# =============================================================
# 227to219_1125 仿真导航 —— 专供 **YAstar 规划层** 调参
#
#   图   : maps/227to219_1125/227to219_1125.yaml（今天新建的那张）
#   场景 : 本目录 world.sdf / occupied_cells.stl / sim.yaml（build_scene.py 生成）
#   参数 : 本目录 nav2_params_yastar.yaml —— **YAstar 的唯一来源就在这里**
#          （不要再用 --yastar-planner：那个开关会用 launch 里硬编码的
#            YASTAR_PLANNER_OVERRIDE 整段覆盖掉这个文件，改 yaml 就没用了）
#
# 用法:
#   ./maps/227to219_1125/simulation/run.sh                    # 无头 Gazebo + RViz
#   USE_RVIZ=False ./maps/227to219_1125/simulation/run.sh     # 不开 RViz
#   ./maps/227to219_1125/simulation/run.sh --rebuild          # 重新生成场景再起
#
# 调参只改本目录 nav2_params_yastar.yaml 里 planner_server.GridBased 段，
# 改完 Ctrl-C 再跑一次（插件参数在 configure 时读，热改不生效）。
# =============================================================
set -eo pipefail

SCENE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCENE_DIR/../../.." && pwd)"
# 注意：变量名不能叫 MAP / PARAMS —— env_srm27.sh 会导出同名变量把它们覆盖掉。
SCENE_MAP="$WS_DIR/maps/227to219_1125/227to219_1125.yaml"
SCENE_PARAMS="$SCENE_DIR/nav2_params_yastar.yaml"
SCENE_SIM="$SCENE_DIR/sim.yaml"
cd "$WS_DIR"

if [ "${1:-}" = "--rebuild" ] || [ ! -f "$SCENE_DIR/world.sdf" ] \
   || [ ! -f "$SCENE_PARAMS" ] || [ ! -f "$SCENE_SIM" ]; then
  echo "[scene] 生成仿真场景（world.sdf / sim.yaml / nav2_params_yastar.yaml）"
  python3 "$SCENE_DIR/build_scene.py"
fi

[ -f "$SCENE_MAP" ] || { echo "[错误] 找不到地图: $SCENE_MAP" >&2; exit 1; }

# 顺序很重要：**install/setup.bash 必须在 env_srm27.sh 之前**。
# 本工作区的 setup.bash 是当初在 source 过 srm_auto_sentry 的 shell 里生成的，colcon
# 把那些旧前缀一并记录了进去 —— 每次 source 它都会往 LD_LIBRARY_PATH 里塞回 27 条
# srm_auto_sentry 路径。那些老 install 里也有同名包 `fake_vel_transform`（nav_se2 之前
# 的 legacy 版本），动态库会先命中它，于是 gimbal_yaw_fake 根本没人发，
# nav2 的 costmap 永远等 TF、导航永远不 activate。
# 所以：先 source ROS + 本工作区，最后再用 env_srm27.sh 把残留清干净。
# （env_srm27.sh 里有 `[ -n ... ]` 探测，返回码不一定是 0，这段关掉 -e。）
set +e
source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash"
source scripts/setup/env_srm27.sh
set -e
command -v ros2 >/dev/null || { echo "[错误] source 环境失败，找不到 ros2" >&2; exit 1; }

# ROS 日志落在工作区里（与 scripts/mapping/rebuild_map_from_bag.sh 同口径）。
export ROS_LOG_DIR="${ROS_LOG_DIR:-$WS_DIR/.tmp_roslog}"
mkdir -p "$ROS_LOG_DIR"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-87}"
export ROS_LOCALHOST_ONLY=1
export IGN_PARTITION="${IGN_PARTITION:-srm_227to219_1125_87}"

# 出生点 = map->odom 的静态偏移（真值里程计把出生点当地图原点）
SCENE_SPAWN="$(python3 - "$SCENE_SIM" <<'PY'
import sys, yaml
robot = yaml.safe_load(open(sys.argv[1], encoding='utf-8'))['robot']
print(f"{robot['x']:.4f} {robot['y']:.4f}")
PY
)"
SCENE_SX="${SCENE_SPAWN% *}"
SCENE_SY="${SCENE_SPAWN#* }"

echo "[启动] 地图   $SCENE_MAP"
echo "[启动] 场景   $SCENE_SIM"
echo "[启动] 出生点 ($SCENE_SX, $SCENE_SY)  → map->odom 用同一组数"
echo "[启动] 参数   $SCENE_PARAMS（YAstar 段就在这里改）"

exec ros2 launch srm27_nav_bringup nav_srm_simulation_launch.py \
  namespace:=red_standard_robot1 \
  world:=srm_empty \
  map:="$SCENE_MAP" \
  params_file:="$SCENE_PARAMS" \
  start_simulation:=True \
  sim_config_file:="$SCENE_SIM" \
  map_to_odom_x:="$SCENE_SX" \
  map_to_odom_y:="$SCENE_SY" \
  map_to_odom_yaw:=0.0 \
  gui:=False run_immediately:=True \
  use_rviz:="${USE_RVIZ:-True}" \
  use_composition:=False \
  use_sim_time:=True \
  use_lio_odometry:=False use_pcd_localization:=False \
  odometry_frontend:=point_lio \
  decision_required:=false \
  gimbal_spin_rate:=0.0
