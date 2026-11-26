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
#   ./maps/227to219_1125/simulation/run.sh --no-clean         # 跳过残留进程清理
#   USE_RVIZ=False ./maps/227to219_1125/simulation/run.sh     # 不开 RViz
#   USE_GUI=True  ./maps/227to219_1125/simulation/run.sh      # Gazebo 带窗口（默认无头）
#   ./maps/227to219_1125/simulation/run.sh --rebuild          # 重新生成场景再起
#
# 已经在跑、只想另开窗口看一眼：不用重启，另开终端挂 GUI 客户端（IGN_PARTITION 必须一致）
#   IGN_PARTITION=srm_227to219_1125_87 ign gazebo -g
#
# 默认用 **nav2_params_yastar_realmatch.yaml**（与实车同链：MinimumSnap +
# SmoothPath 行为树 + 实车那套 safety/corner/costmap 参数），
# 这样仿真里调出来的拐角结论能直接搬回实车。
# 想跑旧的「仿真档」（SimpleSmoother、无 SmoothPath、corner 0.8）：
#   ./run.sh --sim-profile
# 调参改 realmatch 那份的 planner_server.GridBased 段；改完 Ctrl-C 重跑
# （插件参数在 configure 时读，热改不生效）。
# =============================================================
set -eo pipefail

# --no-clean 可以从任意位置给，先剥出来；其余参数照旧按 $1 处理
CLEAN_LEFT=1
_ARGS=()
for _a in "$@"; do
  case "$_a" in
    --no-clean) CLEAN_LEFT=0 ;;
    *) _ARGS+=("$_a") ;;
  esac
done
set -- ${_ARGS[@]+"${_ARGS[@]}"}

SCENE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCENE_DIR/../../.." && pwd)"
# 注意：变量名不能叫 MAP / PARAMS —— env_srm27.sh 会导出同名变量把它们覆盖掉。
SCENE_MAP="$WS_DIR/maps/227to219_1125/227to219_1125.yaml"
# 默认「与实车同链」；--sim-profile 才用旧的仿真档。
# SCENE_PARAMS_OVERRIDE 可以指任意一份参数文件（run_realmatch.sh 就是用它）。
if [ "${1:-}" = "--sim-profile" ]; then
  shift
  SCENE_PARAMS_DEFAULT="$SCENE_DIR/nav2_params_yastar.yaml"
else
  SCENE_PARAMS_DEFAULT="$SCENE_DIR/nav2_params_yastar_realmatch.yaml"
fi
SCENE_PARAMS="${SCENE_PARAMS_OVERRIDE:-$SCENE_PARAMS_DEFAULT}"
SCENE_SIM="$SCENE_DIR/sim.yaml"
cd "$WS_DIR"

if [ "${1:-}" = "--rebuild" ] || [ ! -f "$SCENE_DIR/world.sdf" ] \
   || [ ! -f "$SCENE_PARAMS" ] || [ ! -f "$SCENE_SIM" ]; then
  echo "[scene] 生成仿真场景（world.sdf / sim.yaml / nav2_params_yastar.yaml）"
  python3 "$SCENE_DIR/build_scene.py"
fi

[ -f "$SCENE_MAP" ] || { echo "[错误] 找不到地图: $SCENE_MAP" >&2; exit 1; }
if [ ! -f "$SCENE_PARAMS" ]; then
  echo "[scene] 缺少参数文件，先生成: $SCENE_PARAMS"
  python3 "$SCENE_DIR/build_realmatch_params.py"
fi

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

# ---------------------------------------------------------------------------
# 清理残留进程
#   2026-11-26 事故：09:37 起过一套、09:39 又起一套，前一套的 Gazebo/节点没退干净，
#   两套都在 /red_standard_robot1 + ROS_DOMAIN_ID=87 上发 odom->gimbal_yaw，
#   于是 TF 在 (0,0) 和 (23.8,-2.6) 之间跳、规划 0 帧、目标立刻 abort。
#   这里只杀**引用本场景 sim.yaml / world.sdf** 的进程，不碰别的场景/实车栈。
#   要跳过就加 --no-clean。
# ---------------------------------------------------------------------------
clean_leftovers() {
  python3 - "$SCENE_SIM" "$SCENE_DIR/world.sdf" <<'CLEANPY'
import os
import signal
import sys
import time

sim_yaml, world = sys.argv[1], sys.argv[2]
me = os.getpid()
SIG_INT, SIG_KILL = signal.SIGINT, signal.SIGKILL
GRACE_S = 5.0


def cmdline(pid):
    try:
        with open('/proc/%s/cmdline' % pid, 'rb') as fh:
            return fh.read().decode('utf-8', 'replace').replace('\0', ' ').strip()
    except Exception:
        return ''


def ppid_of(pid):
    try:
        with open('/proc/%s/stat' % pid, 'rb') as fh:
            raw = fh.read().decode('utf-8', 'replace')
        return int(raw.rsplit(')', 1)[1].split()[1])
    except Exception:
        return None


children = {}
for pid in os.listdir('/proc'):
    if not pid.isdigit():
        continue
    pp = ppid_of(pid)
    if pp is not None:
        children.setdefault(pp, []).append(int(pid))


def descendants(pid):
    out, stack = [], [pid]
    while stack:
        cur = stack.pop()
        for child in children.get(cur, []):
            out.append(child)
            stack.append(child)
    return out


found = []
for pid in os.listdir('/proc'):
    if not pid.isdigit() or int(pid) == me:
        continue
    cmd = cmdline(pid)
    if not cmd:
        continue
    why = None
    if 'nav_srm_simulation_launch.py' in cmd and sim_yaml in cmd:
        why = '上一套仿真 launch'
    elif 'ign gazebo' in cmd and world in cmd:
        why = '上一套 Gazebo'
    if why:
        found.append((int(pid), why, cmd[:120]))

if not found:
    print('[清理] 没有本场景的残留进程')
    raise SystemExit(0)

# launch 的子进程（nav2/ros_gz/bridge…）在 launch 被强杀后会变孤儿继续发 TF，必须一起收
targets = {}
for pid, why, cmd in found:
    targets[pid] = (why, cmd)
    for kid in descendants(pid):
        targets.setdefault(kid, ('%s 的子进程' % why, cmdline(kid)[:120]))

for pid, (why, cmd) in sorted(targets.items()):
    print('[清理] SIGINT -> %s pid=%d  %s' % (why, pid, cmd))
    try:
        os.kill(pid, SIG_INT)
    except ProcessLookupError:
        pass
    except PermissionError:
        print('[清理] 无权限结束 pid=%d（可用 sudo 或手动处理）' % pid)

deadline = time.time() + GRACE_S
while time.time() < deadline:
    if not any(os.path.exists('/proc/%d' % p) for p in targets):
        break
    time.sleep(0.2)

for pid, (why, cmd) in sorted(targets.items()):
    if not os.path.exists('/proc/%d' % pid):
        continue
    try:
        os.kill(pid, SIG_KILL)
        print('[清理] SIGKILL -> %s pid=%d（SIGINT 后 %.0f s 仍在）' % (why, pid, GRACE_S))
    except ProcessLookupError:
        pass
    except PermissionError:
        pass
CLEANPY
}

if [ "$CLEAN_LEFT" = "1" ]; then
  clean_leftovers
  # 还有别的栈留在同一 domain/namespace 上就提醒（不自动杀：可能是你自己的另一套）
  # 给刚被收掉的节点最多 10 s 退干净，再判断"是不是还有别的栈"
  _others=""
  for _try in 1 2 3 4 5; do
    _others="$(timeout 8 ros2 node list --no-daemon 2>/dev/null | grep '^/red_standard_robot1/' || true)"
    [ -z "$_others" ] && break
    sleep 2
  done
  if [ -n "$_others" ]; then
    _n="$(printf '%s\n' "$_others" | wc -l)"
    echo "⚠ domain $ROS_DOMAIN_ID 上还有 $_n 个 /red_standard_robot1 节点（不是本场景的？）："
    printf '%s\n' "$_others" | head -8 | sed 's/^/    /'
    echo "  两套同时跑会让 odom->gimbal_yaw 互相打架（TF 跳变、规划 0 帧、目标 abort）。"
    echo "  先停掉它，或者给本次换 ROS_DOMAIN_ID / IGN_PARTITION；--no-clean 可跳过本段。"
  fi
fi

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
  gui:=${USE_GUI:-False} run_immediately:=True \
  use_rviz:="${USE_RVIZ:-True}" \
  use_composition:=False \
  use_sim_time:=True \
  use_lio_odometry:=False use_pcd_localization:=False \
  odometry_frontend:=point_lio \
  decision_required:=false \
  gimbal_spin_rate:=0.0
