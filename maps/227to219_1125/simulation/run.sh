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
# 自转 / 小陀螺：两类开关可以单独用，也可以一起用
#   --gimbal-spin [转速]    云台（= 雷达）相对**底盘**匀速自转，仿真器直接驱动关节
#   --chassis-spin [转速]   底盘小陀螺，经 rotation_cmd -> srm_cmd_mux -> cmd_vel_sim
#   --no-gimbal-spin / --no-chassis-spin   显式锁死（两者默认都不转）
#
#   ./maps/227to219_1125/simulation/run.sh --gimbal-spin
#                                            # 只转云台/雷达：缺省 150 °/s = 2.618 rad/s
#   ./maps/227to219_1125/simulation/run.sh --chassis-spin
#                                            # 只转底盘：缺省 150 °/s（雷达跟着底盘转）
#   ./maps/227to219_1125/simulation/run.sh --gimbal-spin --chassis-spin
#                                            # 都转：雷达世界转速 = 300 °/s
#   ./maps/227to219_1125/simulation/run.sh --chassis-spin 150deg/s --gimbal-spin -150deg/s
#                                            # 底盘转、雷达在 map 里不转（互相抵消）
#   GIMBAL_SPIN_RATE=150deg/s CHASSIS_SPIN_RATE=90deg/s ./maps/227to219_1125/simulation/run.sh
#
#   单位：带 deg / deg/s / dps / °/s 的按**度**每秒换算，纯数字或 rad/s 按**弧度**每秒；
#   负值反向。150 °/s ≈ 2.618 rad/s。⚠ 裸 150 是 rad/s，会被上限拦下并提示改单位。
#
#   ⚠ 云台转速是**关节相对底盘**的转速：雷达在 map 里的转速 = 底盘 wz + 云台关节转速。
#   所以"底盘 150、雷达要在世界里 0"就写 --chassis-spin 150deg/s --gimbal-spin -150deg/s；
#   只想让雷达转、底盘不动，就只给 --gimbal-spin（默认场景）。
#
#   本场景是云台系导航（velocity_command_frame=gimbal、真值里程计发 odom->gimbal_yaw）：
#   一开转，导航参考系 gimbal_yaw 的 yaw 就以世界转速持续增长
#   （`ros2 run tf2_ros tf2_echo odom gimbal_yaw` 可见），点云在 map 里跟着转；
#   fake_vel_transform 会按 IMU 实测转速把速度换算回世界方向，正常现象是
#   **航向一直转、轨迹仍贴着全局路径走**。转不动（joint_states 里云台角不变 /
#   cmd_vel_sim.angular.z 恒 0）或车原地画圈，才是链路没接上。
#   底盘自转由本脚本后台发 rotation_cmd（20 Hz）实现，停发 0.25 s mux 自动停转，
#   脚本退出时会把发布器一起收掉。
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

# ---------------------------------------------------------------------------
# 参数解析
#   --no-clean              跳过残留进程清理
#   --gimbal-spin [转速]    云台相对底盘匀速自转（缺省 150deg/s = 2.618 rad/s），雷达跟着转
#   --chassis-spin [转速]   底盘小陀螺（缺省 150deg/s），rotation_cmd -> srm_cmd_mux
#                           单位：150deg/s / 150°/s / 150dps = 度每秒；纯数字 = rad/s
#   --no-gimbal-spin / --no-chassis-spin  显式锁死（默认二者都不转）
#   其余参数照旧按 $1 处理（--sim-profile / --rebuild）
# ---------------------------------------------------------------------------
GIMBAL_SPIN_DEFAULT="150deg/s"
CHASSIS_SPIN_DEFAULT="150deg/s"
# 内部统一用 rad/s（launch/SDF 的单位）；命令行与环境变量都允许带单位。
GIMBAL_SPIN_RATE="${GIMBAL_SPIN_RATE:-0.0}"
CHASSIS_SPIN_RATE="${CHASSIS_SPIN_RATE:-0.0}"
CLEAN_LEFT=1

# 转速文本 -> rad/s（打印到 stdout）；格式不合法返回 1。
# 纯数字按 rad/s（与 launch 参数同义），带 deg/dps 单位才做换算。
_rate_to_rad() {
  local raw="${1// /}" num unit
  raw="${raw//°/deg}"                       # 150°/s -> 150deg/s
  raw="$(printf '%s' "$raw" | tr '[:upper:]' '[:lower:]')"
  [[ "$raw" =~ ^([-+]?[0-9]*\.?[0-9]+)(deg/s|deg|dps|d/s|rad/s|rad)?$ ]] || return 1
  num="${BASH_REMATCH[1]}"
  unit="${BASH_REMATCH[2]}"
  case "$unit" in
    deg|deg/s|dps|d/s)
      awk -v v="$num" 'BEGIN { printf "%.6f", v * 3.14159265358979323846 / 180.0 }' ;;
    *)
      awk -v v="$num" 'BEGIN { printf "%.6f", v }' ;;
  esac
}

# 解析"可带数值"的自转开关：下一个参数是合法转速（含单位）就吃掉它，
# 否则用缺省值（`--gimbal-spin --no-rviz` 不会把 --no-rviz 误吞成转速）。
# 参数: $1=要写入的变量名 $2=缺省值 $3=选项名 $4=下一个参数
# 返回: 0 = 吃掉两个参数；1 = 只吃开关本身（调用方自己 shift）
_take_spin_rate() {
  local var="$1" default="$2" opt="$3" next="${4:-}"
  if [ -n "$next" ] && _rate_to_rad "$next" >/dev/null; then
    printf -v "$var" '%s' "$next"
    return 0
  fi
  # 下一参数既不是合法转速、又不是别的开关：多半是单位写错了，直接报错，
  # 免得静默退回缺省值（`--gimbal-spin 150degss` 这种）。
  if [ -n "$next" ] && [ "${next#-}" = "$next" ]; then
    echo "[错误] ${opt} 的转速格式不对: '$next'（示例: 150deg/s 或 2.618）" >&2
    exit 2
  fi
  printf -v "$var" '%s' "$default"
  return 1
}

_ARGS=()
while [ "$#" -gt 0 ]; do
  case "$1" in
    --no-clean) CLEAN_LEFT=0; shift ;;
    --gimbal-spin)
      if _take_spin_rate GIMBAL_SPIN_RATE "$GIMBAL_SPIN_DEFAULT" --gimbal-spin "${2:-}"; then
        shift 2
      else
        shift
      fi
      ;;
    --chassis-spin)
      if _take_spin_rate CHASSIS_SPIN_RATE "$CHASSIS_SPIN_DEFAULT" --chassis-spin "${2:-}"; then
        shift 2
      else
        shift
      fi
      ;;
    --no-gimbal-spin) GIMBAL_SPIN_RATE="0.0"; shift ;;
    --no-chassis-spin) CHASSIS_SPIN_RATE="0.0"; shift ;;
    *) _ARGS+=("$1"); shift ;;
  esac
done
set -- ${_ARGS[@]+"${_ARGS[@]}"}

if ! GIMBAL_SPIN_RAD="$(_rate_to_rad "$GIMBAL_SPIN_RATE")"; then
  echo "[错误] 云台转速格式不对（示例: 1.0 / 2.618rad/s / 150deg/s），收到: '$GIMBAL_SPIN_RATE'" >&2
  exit 2
fi
if ! CHASSIS_SPIN_RAD="$(_rate_to_rad "$CHASSIS_SPIN_RATE")"; then
  echo "[错误] 底盘转速格式不对（示例: 1.0 / 2.618rad/s / 150deg/s），收到: '$CHASSIS_SPIN_RATE'" >&2
  exit 2
fi
GIMBAL_SPIN_RATE="$GIMBAL_SPIN_RAD"
CHASSIS_SPIN_RATE="$CHASSIS_SPIN_RAD"
GIMBAL_SPIN_DEG="$(awk -v v="$GIMBAL_SPIN_RATE" \
  'BEGIN { printf "%.1f", v * 180.0 / 3.14159265358979323846 }')"
CHASSIS_SPIN_DEG="$(awk -v v="$CHASSIS_SPIN_RATE" \
  'BEGIN { printf "%.1f", v * 180.0 / 3.14159265358979323846 }')"
# 雷达在 map（odom）里的世界转速 = 底盘 wz + 云台关节转速。两路方向相同就相加。
GIMBAL_WORLD_RATE="$(awk -v g="$GIMBAL_SPIN_RATE" -v c="$CHASSIS_SPIN_RATE" \
  'BEGIN { printf "%.6f", g + c }')"
GIMBAL_WORLD_DEG="$(awk -v v="$GIMBAL_WORLD_RATE" \
  'BEGIN { printf "%.1f", v * 180.0 / 3.14159265358979323846 }')"
# 上限：云台关节 8 rad/s（yaw_envelope.max_velocity）；底盘 8.5 rad/s（mux 的 wz_max，
# 见 config/simulation/speed_limits.yaml）。超过多半是单位写错了（把 150 °/s 写成裸 150），
# 直接拦下来，免得仿真里疯转、导航完全失真。
if awk -v v="$GIMBAL_SPIN_RATE" 'BEGIN { a = (v < 0) ? -v : v; exit !(a > 8.0) }'; then
  echo "[错误] 云台转速 ${GIMBAL_SPIN_RATE} rad/s（${GIMBAL_SPIN_DEG} °/s）超过关节上限 8 rad/s。" >&2
  echo "       纯数字按 rad/s 解释：150 °/s 要写 --gimbal-spin 150deg/s（或 2.618rad/s）。" >&2
  exit 2
fi
if awk -v v="$CHASSIS_SPIN_RATE" 'BEGIN { a = (v < 0) ? -v : v; exit !(a > 8.5) }'; then
  echo "[错误] 底盘转速 ${CHASSIS_SPIN_RATE} rad/s（${CHASSIS_SPIN_DEG} °/s）超过 mux 的 wz_max=8.5 rad/s。" >&2
  echo "       纯数字按 rad/s 解释：150 °/s 要写 --chassis-spin 150deg/s（或 2.618rad/s）。" >&2
  exit 2
fi

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
if awk -v v="$GIMBAL_SPIN_RATE" 'BEGIN { exit !(v != 0) }'; then
  echo "[启动] 云台   ${GIMBAL_SPIN_RATE} rad/s（${GIMBAL_SPIN_DEG} °/s）相对底盘自转（雷达跟着转）"
else
  echo "[启动] 云台   锁定（gimbal_spin_rate=0；要转加 --gimbal-spin）"
fi
if awk -v v="$CHASSIS_SPIN_RATE" 'BEGIN { exit !(v != 0) }'; then
  echo "[启动] 底盘   ${CHASSIS_SPIN_RATE} rad/s（${CHASSIS_SPIN_DEG} °/s）小陀螺（rotation_cmd -> srm_cmd_mux）"
  echo "              mux 的 wz_max=8.5、wz_ramp=25，请求在 mux 订阅后开始下发"
else
  echo "[启动] 底盘   不转（要转加 --chassis-spin）"
fi
if awk -v g="$GIMBAL_SPIN_RATE" -v c="$CHASSIS_SPIN_RATE" \
   'BEGIN { exit !(g != 0 || c != 0) }'; then
  echo "[启动] 雷达   世界转速 ≈ ${GIMBAL_WORLD_RATE} rad/s（${GIMBAL_WORLD_DEG} °/s）= 底盘 + 云台"
  echo "        验证: ros2 run tf2_ros tf2_echo odom gimbal_yaw"
  echo "              ros2 topic echo /red_standard_robot1/cmd_vel_sim --field angular.z"
fi

# ---------------------------------------------------------------------------
# 底盘小陀螺：后台向 mux 的 spin_topic（rotation_cmd）持续发 angular.z。
# launch 不能再 exec：放后台后，退出时才能把这个发布器一起收掉。
# ---------------------------------------------------------------------------
CHASSIS_SPIN_TOPIC="/red_standard_robot1/rotation_cmd"
CHASSIS_SPIN_LOG="$ROS_LOG_DIR/chassis_spin.log"
CHASSIS_SPIN_PID=""
LAUNCH_PID=""

# Ctrl-C / TERM 只转给 launch，让它自己收尾；脚本等它退干净再返回（保持旧的手感）。
_forward_int() {
  # 先把自转请求停掉：mux 的 0.25 s 看门狗会让底盘马上停，不必等 launch 收尾。
  # （终端 Ctrl-C 时发布器本来也会收到 SIGINT；这里是显式 kill 单独发给 run.sh 的情况。）
  if [ -n "$CHASSIS_SPIN_PID" ] && kill -0 "$CHASSIS_SPIN_PID" 2>/dev/null; then
    kill "$CHASSIS_SPIN_PID" 2>/dev/null || true
    CHASSIS_SPIN_PID=""
  fi
  if [ -n "$LAUNCH_PID" ] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
  fi
}
_cleanup() {
  local rc=$?
  _forward_int
  return "$rc"
}
trap _cleanup EXIT
trap _forward_int INT TERM

# 上次被 SIGKILL 时 trap 来不及跑：残留的 rotation_cmd 发布器会让新栈一起来就自转。
# 只清本场景这条话题（与 scripts/manual/spin_control.sh 的 stop_publisher 同口径）。
pkill -f "topic pub.*red_standard_robot1/rotation_cmd" 2>/dev/null || true

# 注意：非交互 shell 里用 `&` 起的子进程会继承 SIGINT=SIG_IGN（POSIX 对异步列表的
# 要求），直接 kill -INT 会被忽略 —— Ctrl-C 后 launch 永远不收尾。所以先在子 shell
# 里装一个空 handler 再 exec："被捕获"的信号在 exec 时会重置成默认，launch 才能
# 正常收到 INT 并走退出流程。
(
  trap ':' INT TERM
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
    gimbal_spin_rate:="$GIMBAL_SPIN_RATE"
) &
LAUNCH_PID=$!

if awk -v v="$CHASSIS_SPIN_RATE" 'BEGIN { exit !(v != 0) }'; then
  # -w 1：等 srm_cmd_mux 订阅上再发，避免导航栈没起来空转；停发 0.25 s mux 自动停转。
  ros2 topic pub --rate 20 --print 1000 -w 1 --qos-profile services_default \
    "$CHASSIS_SPIN_TOPIC" geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: ${CHASSIS_SPIN_RATE}}}" \
    > "$CHASSIS_SPIN_LOG" 2>&1 &
  CHASSIS_SPIN_PID=$!
  # 起不来就明说（常见：ros2 环境没 source 好、话题类型写错），否则底盘"参数给了却不转"。
  sleep 1
  if ! kill -0 "$CHASSIS_SPIN_PID" 2>/dev/null; then
    echo "[警告] 底盘自转发布器没起来，底盘不会转。日志: $CHASSIS_SPIN_LOG" >&2
    CHASSIS_SPIN_PID=""
  fi
fi

set +e
wait "$LAUNCH_PID"
rc=$?
# 被信号打断时 launch 可能还在收尾：再等它退干净，避免终端先返回、进程留在后台。
if kill -0 "$LAUNCH_PID" 2>/dev/null; then
  wait "$LAUNCH_PID"
  rc=$?
fi
set -e
exit "$rc"
