#!/usr/bin/env bash
# =============================================================
# SRM 仿真 NAV 导航 + 独立自转测试 (一键启动)
#
#   标签页 1: SRM Gazebo 仿真 (世界 + SRM 圆柱底盘 + 雷达 + 速度执行)
#   标签页 2: 速度合成 + Nav2 导航栈 + RViz (含独立自转测试发送器)
#   标签页 3: 手柄自转 (可选, 默认关闭; 需要手柄, 输出 remap 到 rotation_cmd)
#
# 速度链路 (与实施方案 3.3 一致):
#   Nav2 controller ─cmd_vel_nav─┐
#                                ├─ srm_cmd_mux ─cmd_vel_sim─► SRM 速度适配器 → Gazebo
#   rotation_test_sender ─ rotation_controller ─ rotation_velocity ┘
#
#   导航只产生 vx/vy; mux 丢弃导航输入的 angular.z。自转由独立自转链路给出。
#
# 速度限幅 (控制器 / 平滑器 / mux / 恢复行为 / 自转) 的**唯一来源**:
#   src/srm27_navigation/srm27_nav_bringup/config/simulation/speed_limits.yaml
#   launch 启动时把它合并进 nav2_params_srm*.yaml 与 srm_chassis_control.yaml,
#   参数文件里不再写这些键 (两处都写且值不同 -> 启动直接报错)。
#   改档位只改那一份; 改完用下面的自检看"卡住速度的是哪一层":
#     python3 src/srm27_navigation/srm27_nav_bringup/scripts/srm_speed_limits_check.py
# 详见 docs/导航速度调试指南.md §1/§2。
#
# 模式说明 (本脚本只做导航):
#   slam:=False  use_pcd_localization:=False
#     -> 由 map_server 加载现成的 PGM/YAML 地图, map->odom 为静态 TF,
#        odom->base_link 由 simulation_ground_truth_odometry 给出;
#        不启动 slam_toolbox / Point-LIO / small_gicp_relocalization。
#   如果需要建图, 请另起 launch (slam:=True), 不要与本脚本同时运行,
#   否则多个节点会争抢 map->odom。
#
#   初始重定位 (GICP):
#     --gicp 会把定位方式切成 use_pcd_localization:=True + use_lio_odometry:=False:
#       * 先验 PCD 由 --prior-pcd 给出, 可由 --prior-pcd-manifest 自动推出初值;
#       * odom->base_link 交给 sensor_scan_generation (launch 会覆盖参数文件里的
#         publish_tf: False), 真值里程计适配器不再启动;
#       * 本脚本不再发静态 map->odom, map->odom 只由 small_gicp_relocalization 发布。
#     这样地图原点与先验点云原点一致时, 机器人出生即"已知位置"; 若已知先验点云相对
#     出生点有偏移, 用 --init-pose 直接给 map->odom 初值 (方案 §5 的 T_M_O_init)。
#
# 用法:
#   ./script/start_sim_nav.sh                                  # rmuc_2025 + 隧道地图(默认), 不自转
#   ./script/start_sim_nav.sh -m rmuc_2025                     # 换成普通场地地图
#   ./script/start_sim_nav.sh -w srm_empty                     # 空场 (Gazebo 起来自动开始运行)
#   ./script/start_sim_nav.sh --controller minco               # 局部控制器换 MINCO+MPC (自动挑 minco 参数文件)
#   ./script/start_sim_nav.sh --controller omni                # 显式回到全向 PID 控制器 (历史默认)
#   ./script/start_sim_nav.sh --no-run                         # Gazebo 保持暂停, 手动点播放
#   ./script/start_sim_nav.sh --rotation-mode constant --rotation-speed 1.0
#   ./script/start_sim_nav.sh --rotation-mode periodic \
#       --rotation-offset 1.0 --rotation-amplitude 0.5 --rotation-period 4.0
#   ./script/start_sim_nav.sh --no-rviz --no-rotation          # 只跑仿真 + 导航
#   ./script/start_sim_nav.sh --gicp --prior-pcd /abs/prior.pcd \
#       --map /abs/prior.yaml --no-rotation                    # 用 GICP 初始重定位
#   ./script/start_sim_nav.sh --gicp --prior-pcd /abs/prior.pcd \
#       --init-pose "0,0,0,0,0,0.5236"                         # 手动给 map->odom 初值
#   DRY_RUN=1 ./script/start_sim_nav.sh                        # 只打印将执行的命令
#
# 参数:
#   -w, --world   <name>      rmuc_2025 / rmuc_2024 / rmul_2024 / rmul_2025 / srm_empty
#   -m, --map     <名字|绝对路径>  地图名(自动补 .yaml)或 YAML 绝对路径
#   -p, --params  <绝对路径>  Nav2 参数文件; 不给时由 --controller 决定 (不给 -c 就是 Omni 那份)
#   -c, --controller <auto|omni|minco>
#                            局部控制器 = 用哪份 Nav2 参数文件 (与 start_real_nav.sh 同名同义):
#                              minco -> config/simulation/nav2_params_srm_minco.yaml
#                                       (FollowPath = srm27_minco_controller::MincoMpcController)
#                              omni  -> config/simulation/nav2_params_srm.yaml
#                                       (FollowPath = srm27_omni_pid_controller::OmniPidPursuitController)
#                              auto  -> 默认值: 按参数文件里 FollowPath 的插件判定。
#                            注意默认是 auto, 所以"什么都不给"仍然是 Omni 那份, 行为与以前一致;
#                            只给 -p 时也按 auto 判定 (老命令 `-p .../nav2_params_srm_minco.yaml`
#                            照样工作, 并且会认出是 MINCO)。-c 与 -p 同时给出时以 -p 加载,
#                            但会校验两者不矛盾: 选错直接报错, 不让"名字是 MINCO、跑的是 Omni"溜过去。
#       --minco / --omni     等价于 --controller minco / --controller omni
#       --gicp / --no-gicp       是否用 GICP 初始重定位 (默认关闭 = 原来的静态地图模式)
#       --prior-pcd <绝对路径>   先验点云 PCD (--gicp 时必填)
#       --prior-pcd-manifest <绝对路径>
#                                先验点云清单 (重建工具生成的 *_manifest.yaml);
#                                给出时会按其中的 T_M_W 与 srm_sim.yaml 的出生点自动
#                                推出 init_pose, 并在两者不一致时告警
#       --init-pose "<x,y,z,roll,pitch,yaw>"
#                                map->odom 初值, 等价于 nav2 参数的 init_pose;
#                                给了就以它为准 (不再从 manifest 推导)
#       --rviz / --no-rviz        是否启动 RViz (默认启动)
#       --gui / --no-gui          Gazebo 是否带 GUI (默认带)
#       --run / --no-run          Gazebo 是否直接开始运行 (默认直接运行, 即 gz sim -r;
#                                 --no-run 则保持暂停等你点播放)
#       --smoother / --no-smoother velocity_smoother 是否串联 (默认串联)
#       --rotation-mode <stop|constant|periodic>   自转模式 (默认 stop)
#       --rotation-speed <rad/s>  恒速模式角速度
#       --rotation-offset <rad/s> 周期模式平均角速度
#       --rotation-amplitude <rad/s> 周期模式变化幅度
#       --rotation-period <s>     周期
#       --rotation-phase <rad>    初相位
#       --rotation-wave <sine|square> 周期波形
#       --rotation / --no-rotation 是否启动自转测试发送器 (默认启动, stop 模式)
#       --teleop / --no-teleop    手柄自转标签页 (默认关闭)
#   -h, --help                显示本帮助
#
# 环境变量:
#   WORLD MAP MAP_NAME PARAMS_FILE CONTROLLER USE_RVIZ USE_GUI RUN_IMMEDIATELY
#   USE_GICP PRIOR_PCD_FILE PRIOR_PCD_MANIFEST INIT_POSE GICP_RUN_ID
#   USE_VELOCITY_SMOOTHER ROTATION_MODE ROTATION_SPEED ROTATION_OFFSET
#   ROTATION_AMPLITUDE ROTATION_PERIOD ROTATION_PHASE ROTATION_SINE_WAVE
#   START_ROTATION_SENDER ENABLE_TELEOP
#   OPEN_MODE(tab|window) TERMINAL ROBOT_NS USE_COMPOSITION DRY_RUN SRM27_WS_DIR PREKILL
#
# 说明:
#   - 启动前会自动依次跑 script/kill_nav.sh、script/kill_gzb.sh、script/kill_rviz.sh,
#     清掉上次遗留的 Nav2 导航栈 / Gazebo / RViz。注意: 上一次仿真还在跑时也会被清掉
#     (而不是像以前那样 "[跳过] 已检测到正在运行的 ..."), 目的是保证每次都是干净的一套
#     仿真 + 导航。**导航栈必须一起重启**: costmap 的膨胀半径等参数只在节点 configure
#     时读一次 (Humble 的 InflationLayer 没有动态回调), 只重启 Gazebo 会留下"仿真新、
#     导航旧"的错配, 表现就是改了参数却看到旧值。
#     不想清就用 PREKILL=0, 那时恢复"检测到在跑就跳过"的老行为。清理失败只告警不阻断。
#   - Gazebo 世界、SRM 初始位姿和速度参数统一由
#     srm27_gazebo_simulator/config/srm_sim.yaml 给出; -w 会覆盖其中的 world。
#   - 地图与参数文件一律使用绝对路径; 地图可直接给名字, 依次在
#     src/srm27_navigation/srm27_nav_bringup/map/simulation/ 与工作空间根目录 maps/
#     (支持 maps/<名字>.yaml 与 maps/<名字>/<名字>.yaml) 下解析, 找不到会列出全部可用地图。
#   - 每个标签页都会先 source 工作空间的 install/setup.bash。
#   - 控制清单：导航速度 cmd_vel_nav、自转请求 rotation_cmd、自转输出
#     rotation_velocity、合成命令 cmd_vel_sim; 诊断在 /<ns>/diagnostics。
# =============================================================

set -euo pipefail

# ---------- 路径 ----------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${SRM27_WS_DIR:-$(cd "$SCRIPT_DIR/.." && pwd)}"
PKG_SRC="$WS_DIR/src/srm27_navigation/srm27_nav_bringup"
SIM_SRC="$WS_DIR/src/srm27_gazebo_simulator"

# ---------- 可配置项 ----------
OPEN_MODE="${OPEN_MODE:-tab}"                # tab: 同一窗口多标签页; window: 每命令独立窗口
TERMINAL="${TERMINAL:-gnome-terminal}"
ROBOT_NS="${ROBOT_NS:-red_standard_robot1}"  # 与 nav_srm_simulation_launch.py 的 namespace 默认值一致
USE_COMPOSITION="${USE_COMPOSITION:-False}"
USE_RVIZ="${USE_RVIZ:-True}"
USE_GUI="${USE_GUI:-true}"
# Gazebo 默认"起来就跑"(gz sim -r): 仿真时间立即推进, 导航栈不用等手动点播放。
# 想保持旧的"暂停等点播放"行为: --no-run 或 RUN_IMMEDIATELY=false。
RUN_IMMEDIATELY="${RUN_IMMEDIATELY:-true}"
USE_VELOCITY_SMOOTHER="${USE_VELOCITY_SMOOTHER:-True}"
ENABLE_TELEOP="${ENABLE_TELEOP:-0}"
DRY_RUN="${DRY_RUN:-0}"

START_ROTATION_SENDER="${START_ROTATION_SENDER:-1}"
ROTATION_MODE="${ROTATION_MODE:-stop}"
ROTATION_SPEED="${ROTATION_SPEED:-0.0}"
ROTATION_OFFSET="${ROTATION_OFFSET:-0.0}"
ROTATION_AMPLITUDE="${ROTATION_AMPLITUDE:-0.0}"
ROTATION_PERIOD="${ROTATION_PERIOD:-4.0}"
ROTATION_PHASE="${ROTATION_PHASE:-0.0}"
ROTATION_SINE_WAVE="${ROTATION_SINE_WAVE:-true}"

MAP_NAME="${MAP_NAME:-}"
MAP_FILE="${MAP:-}"
PARAMS_FILE="${PARAMS_FILE:-}"
PARAMS_FILE_GIVEN=0

# 局部控制器 (决定用哪份 Nav2 参数文件): auto 按参数文件里的 FollowPath 插件判定,
# omni / minco 显式指定。默认 auto 是刻意选的 —— 不指定时仍然落在 Omni 那份文件上,
# 与历史行为完全一致; 只有 -c minco / --minco 才切到 MINCO+MPC。
CONTROLLER="${CONTROLLER:-auto}"
CONTROLLER_CLI_GIVEN=0

# 初始重定位 (GICP)
USE_GICP="${USE_GICP:-0}"                    # 1 = use_pcd_localization:=True
PRIOR_PCD_FILE="${PRIOR_PCD_FILE:-}"         # 先验点云 PCD 绝对路径
PRIOR_PCD_MANIFEST="${PRIOR_PCD_MANIFEST:-}" # 先验点云清单 (可选, 用于推导 init_pose)
PRIOR_PCD_SEARCH_DIRS="${PRIOR_PCD_SEARCH_DIRS:-}"  # 额外搜索目录 (冒号分隔)
INIT_POSE="${INIT_POSE:-}"                   # map->odom 初值 [x,y,z,roll,pitch,yaw]
GICP_RUN_ID="${GICP_RUN_ID:-}"               # 写入逐次诊断的试验编号
WORLD_FROM_USER="${WORLD:-}"
WORLD=""

# ---------- 帮助 ----------
usage() {
  # 打印文件头注释块 (跳过 shebang 与首尾分隔线), 避免帮助文本与代码不同步。
  awk 'NR == 1 { next }
       /^# =+[[:space:]]*$/ { sep++; if (sep == 2) exit; next }
       { sub(/^# ?/, ""); print }' "${BASH_SOURCE[0]}"
}

# ---------- 命令行参数 ----------
while [ $# -gt 0 ]; do
  case "$1" in
    -w|--world)   WORLD_FROM_USER="${2:?--world 需要一个世界名}"; shift 2 ;;
    -m|--map)     MAP_FILE="${2:?--map 需要地图名或绝对路径}"; shift 2 ;;
    -p|--params)  PARAMS_FILE="${2:?--params 需要绝对路径}"; PARAMS_FILE_GIVEN=1; shift 2 ;;
    -c|--controller) CONTROLLER="${2:?--controller 需要 auto|omni|minco}"
                     CONTROLLER_CLI_GIVEN=1; shift 2 ;;
    --minco)      CONTROLLER="minco"; CONTROLLER_CLI_GIVEN=1; shift ;;
    --omni)       CONTROLLER="omni";  CONTROLLER_CLI_GIVEN=1; shift ;;
    --gicp)       USE_GICP="1"; shift ;;
    --no-gicp)    USE_GICP="0"; shift ;;
    --prior-pcd)          PRIOR_PCD_FILE="${2:?--prior-pcd 需要绝对路径}"; shift 2 ;;
    --prior-pcd-manifest) PRIOR_PCD_MANIFEST="${2:?--prior-pcd-manifest 需要绝对路径}"; shift 2 ;;
    --init-pose)          INIT_POSE="${2:?--init-pose 需要 x,y,z,roll,pitch,yaw}"; shift 2 ;;
    --rviz)       USE_RVIZ="True"; shift ;;
    --no-rviz)    USE_RVIZ="False"; shift ;;
    --gui)        USE_GUI="true"; shift ;;
    --no-gui)     USE_GUI="false"; shift ;;
    --run)        RUN_IMMEDIATELY="true"; shift ;;
    --no-run)     RUN_IMMEDIATELY="false"; shift ;;
    --smoother)   USE_VELOCITY_SMOOTHER="True"; shift ;;
    --no-smoother) USE_VELOCITY_SMOOTHER="False"; shift ;;
    --rotation-mode)      ROTATION_MODE="${2:?--rotation-mode 需要 stop|constant|periodic}"; shift 2 ;;
    --rotation-speed)     ROTATION_SPEED="${2:?--rotation-speed 需要 rad/s}"; shift 2 ;;
    --rotation-offset)    ROTATION_OFFSET="${2:?--rotation-offset 需要 rad/s}"; shift 2 ;;
    --rotation-amplitude) ROTATION_AMPLITUDE="${2:?--rotation-amplitude 需要 rad/s}"; shift 2 ;;
    --rotation-period)    ROTATION_PERIOD="${2:?--rotation-period 需要秒}"; shift 2 ;;
    --rotation-phase)     ROTATION_PHASE="${2:?--rotation-phase 需要弧度}"; shift 2 ;;
    --rotation-wave)      case "${2:?--rotation-wave 需要 sine|square}" in
                            sine)   ROTATION_SINE_WAVE="true" ;;
                            square) ROTATION_SINE_WAVE="false" ;;
                            *) echo "[错误] --rotation-wave 只支持 sine 或 square" >&2; exit 2 ;;
                          esac
                          shift 2 ;;
    --rotation)     START_ROTATION_SENDER="1"; shift ;;
    --no-rotation)  START_ROTATION_SENDER="0"; shift ;;
    --teleop)     ENABLE_TELEOP="1"; shift ;;
    --no-teleop)  ENABLE_TELEOP="0"; shift ;;
    -h|--help)    usage; exit 0 ;;
    *) echo "[错误] 未知参数: $1 (用 -h 查看用法)" >&2; exit 2 ;;
  esac
done

# ---------- 环境检查 ----------
if [ ! -f "$WS_DIR/install/setup.bash" ]; then
  echo "[错误] 未找到 $WS_DIR/install/setup.bash" >&2
  echo "[提示] 先构建工作空间, 或用 SRM27_WS_DIR=/path/to/ws 指定工作空间。" >&2
  exit 1
fi

if [ "$DRY_RUN" != "1" ] && ! command -v "$TERMINAL" >/dev/null 2>&1; then
  echo "[错误] 未找到终端程序: $TERMINAL" >&2
  echo "[提示] 可用 TERMINAL=konsole 或 TERMINAL=xfce4-terminal 指定。" >&2
  exit 1
fi

if [ "$DRY_RUN" != "1" ] && [ "$USE_GUI" = "true" ] \
   && [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
  echo "[警告] 未检测到 DISPLAY / WAYLAND_DISPLAY, Gazebo 与 RViz 的图形界面可能起不来。" >&2
  echo "[警告] 可以改用 --no-gui --no-rviz。" >&2
fi

# ---------- 工作区一致性检查 ----------
# 若当前 shell 还 source 了同机的另一份工作区（例如 ~/srm_nav_27test），同名包会优先解析到
# 那边：底盘限速、参数文件、行为树可能全部来自另一个工作区，而 MINCO 插件只存在于本工作区
# 于是出现"一半新一半旧"的混合环境——现场表现为"改了 v_max=1.5 却仍然被限到 0.5"。
# 这里在启动前逐个核对关键包的解析路径，不一致就直接失败，而不是让混合环境跑起来。
require_pkg_from_ws() {
  local pkg="$1" prefix
  prefix="$(bash -c "source '$WS_DIR/install/setup.bash' >/dev/null 2>&1; ros2 pkg prefix '$pkg' 2>/dev/null" || true)"
  if [ -z "$prefix" ]; then
    echo "[错误] 找不到包 $pkg，请确认 $WS_DIR/install/setup.bash 可用。" >&2
    return 1
  fi
  case "$prefix" in
    "$WS_DIR"/*) return 0 ;;
    *)
      echo "[错误] 包 $pkg 解析到了其它工作区: $prefix" >&2
      echo "       期望前缀: $WS_DIR" >&2
      echo "       当前环境里还 source 了别的工作区（检查 ~/.bashrc 与当前 shell 的 AMENT_PREFIX_PATH），" >&2
      echo "       否则同名包（含底盘限速、Nav2 参数、行为树）会从那边加载。请开新终端只 source 本工作区。" >&2
      return 1
      ;;
  esac
}

if [ "$DRY_RUN" != "1" ]; then
  for _pkg in srm27_nav_bringup srm27_chassis_control srm27_minco_controller; do
    require_pkg_from_ws "$_pkg" || exit 1
  done
fi

if [ "$DRY_RUN" != "1" ] && ! command -v flock >/dev/null 2>&1; then
  echo "[错误] 未找到 flock, 无法保证 Gazebo 单实例启动。" >&2
  exit 1
fi

# ---------- 地图 / 参数文件解析 ----------
# 优先用源码目录 (symlink-install 下与 install 内容一致), 退化到 install。
MAP_DIR="$PKG_SRC/map/simulation"
if [ ! -d "$MAP_DIR" ]; then
  MAP_DIR="$WS_DIR/install/srm27_nav_bringup/share/srm27_nav_bringup/map/simulation"
fi
if [ ! -d "$MAP_DIR" ]; then
  echo "[错误] 未找到仿真地图目录: $MAP_DIR" >&2
  exit 1
fi

# 工作空间根目录 maps/ 也纳入搜索范围: 那里放现场采集 / SLAM 新出的地图
# (与 start_real_nav.sh 的搜索规则保持一致), 这样新图不必先拷进包内 map/simulation/。
# 两个目录同名时以包内 map/simulation/ 优先, 保证仿真默认地图不被意外顶掉。
MAPS_DIR="$WS_DIR/maps"

# 两份仿真 Nav2 参数文件: Omni (历史默认) 与 MINCO+MPC。优先源码目录 (symlink-install 下与
# install 一致), 退化到 install。选中哪份由 --controller 决定, 只给 -p 则按文件内容判定。
OMNI_PARAMS_DEFAULT="$PKG_SRC/config/simulation/nav2_params_srm.yaml"
if [ ! -f "$OMNI_PARAMS_DEFAULT" ]; then
  OMNI_PARAMS_DEFAULT="$WS_DIR/install/srm27_nav_bringup/share/srm27_nav_bringup/config/simulation/nav2_params_srm.yaml"
fi
MINCO_PARAMS_DEFAULT="$PKG_SRC/config/simulation/nav2_params_srm_minco.yaml"
if [ ! -f "$MINCO_PARAMS_DEFAULT" ]; then
  MINCO_PARAMS_DEFAULT="$WS_DIR/install/srm27_nav_bringup/share/srm27_nav_bringup/config/simulation/nav2_params_srm_minco.yaml"
fi
MINCO_PLUGIN_NAME="srm27_minco_controller::MincoMpcController"

# 只给了 -p、没显式选控制器: 交给下面的 auto 判定, 免得"-p 指到 Omni 文件"被当成
# "选了 minco 却加载 Omni 文件"拦下来 (与 start_real_nav.sh 同一套规则)。
if [ "$CONTROLLER_CLI_GIVEN" = "0" ] && [ "$PARAMS_FILE_GIVEN" = "1" ]; then
  CONTROLLER="auto"
fi

case "$CONTROLLER" in
  auto|omni|minco) ;;
  *) echo "[错误] --controller 只支持 auto / omni / minco, 收到: $CONTROLLER" >&2; exit 2 ;;
esac

if [ -z "$PARAMS_FILE" ]; then
  case "$CONTROLLER" in
    omni)  PARAMS_FILE="$OMNI_PARAMS_DEFAULT" ;;
    minco) PARAMS_FILE="$MINCO_PARAMS_DEFAULT" ;;
    auto)  PARAMS_FILE="$OMNI_PARAMS_DEFAULT" ;;   # 不给控制器时保持历史默认 (Omni 那份)
  esac
fi

# Gazebo 世界与 SRM 初始位姿由 srm27_gazebo_simulator/config/srm_sim.yaml 给出。
SIM_CONFIG="$SIM_SRC/config/srm_sim.yaml"
if [ ! -f "$SIM_CONFIG" ]; then
  SIM_CONFIG="$WS_DIR/install/srm27_gazebo_simulator/share/srm27_gazebo_simulator/config/srm_sim.yaml"
fi
GZ_WORLD=""
if [ -f "$SIM_CONFIG" ]; then
  GZ_WORLD="$(sed -n 's/^world:[[:space:]]*//p' "$SIM_CONFIG" | head -n 1)"
  GZ_WORLD="${GZ_WORLD%\"}"; GZ_WORLD="${GZ_WORLD#\"}"
  GZ_WORLD="${GZ_WORLD%\'}"; GZ_WORLD="${GZ_WORLD#\'}"
fi

WORLD="${WORLD_FROM_USER:-${GZ_WORLD:-rmuc_2025}}"

# 地图候选文件: map/simulation/*.yaml、maps/*.yaml、maps/<名字>/<名字>.yaml。
# 最后一种布局只认"目录名与 yaml 同名"的那份, 否则 rosbag 之类的
# maps/<bag>/metadata.yaml 会被误当成地图。
list_map_candidates() {
  local f
  for f in "$MAP_DIR"/*.yaml "$MAPS_DIR"/*.yaml "$MAPS_DIR"/*/*.yaml; do
    [ -f "$f" ] || continue
    case "$f" in
      "$MAPS_DIR"/*/*)
        [ "$(basename "$(dirname "$f")")" = "$(basename "$f" .yaml)" ] || continue
        ;;
    esac
    printf '%s\n' "$f"
  done
}

list_maps() {
  echo "[提示] 可用的地图 (左侧名称即为 -m 的取值, 右侧为实际会加载的文件):" >&2
  local f name seen="" found=0
  while IFS= read -r f; do
    [ -n "$f" ] || continue
    name="$(basename "$f" .yaml)"
    case " $seen " in *" $name "*) continue ;; esac
    seen="$seen $name"
    printf '  - %-22s %s\n' "$name" "$f" >&2
    found=1
  done < <(list_map_candidates)
  if [ "$found" = "0" ]; then
    echo "  (未找到任何 .yaml 地图)" >&2
  fi
  echo "[提示] 搜索目录: $MAP_DIR" >&2
  if [ -d "$MAPS_DIR" ]; then
    echo "[提示]           $MAPS_DIR  (支持 maps/<名字>.yaml 与 maps/<名字>/<名字>.yaml)" >&2
  fi
}

# 地图名 -> 绝对路径; 也接受绝对路径。
# 搜索顺序: 包内 map/simulation/<名字>.yaml, 再 maps/<名字>/<名字>.yaml、maps/<名字>.yaml。
resolve_map() {
  local value="$1" name cand
  if [ -z "$value" ]; then
    return 1
  fi
  if [ "${value#/}" != "$value" ]; then
    if [ -f "$value" ]; then
      printf '%s\n' "$value"
      return 0
    fi
    echo "[错误] 地图文件不存在: $value" >&2
    return 1
  fi
  if [ "${value#*/}" != "$value" ]; then
    echo "[错误] 相对路径地图不受支持, 请用绝对路径: $value" >&2
    return 1
  fi
  name="$value"
  case "$name" in
    *.yaml) ;;
    *) name="${name}.yaml" ;;
  esac
  for cand in \
    "$MAP_DIR/$name" \
    "$MAPS_DIR/${name%.yaml}/${name}" \
    "$MAPS_DIR/$name"; do
    if [ -f "$cand" ]; then
      printf '%s\n' "$cand"
      return 0
    fi
  done
  echo "[错误] 未找到地图: $name" >&2
  list_maps
  return 1
}

if [ -z "$MAP_FILE" ]; then
  if [ -z "$MAP_NAME" ]; then
    for candidate in "${WORLD}_tunnel" "$WORLD"; do
      if resolve_map "$candidate" >/dev/null 2>&1; then
        MAP_NAME="$candidate"
        break
      fi
    done
  fi
  if [ -z "$MAP_NAME" ]; then
    echo "[错误] 无法为世界 $WORLD 推断默认地图, 请用 -m 指定。" >&2
    list_maps
    exit 1
  fi
  MAP_FILE="$MAP_NAME"
fi

MAP_FILE="$(resolve_map "$MAP_FILE")" || exit 1

if [ ! -f "$PARAMS_FILE" ]; then
  echo "[错误] 参数文件不存在: $PARAMS_FILE" >&2
  exit 1
fi
if [ "${PARAMS_FILE#/}" = "$PARAMS_FILE" ]; then
  echo "[错误] 参数文件必须是绝对路径: $PARAMS_FILE" >&2
  exit 1
fi

# ---------- 局部控制器判定 (auto 由文件内容认, 显式指定则校验文件是否对得上) ----------
# 判据与 start_real_nav.sh 一致: 参数文件里有没有 MINCO 的 FollowPath 插件。
if [ "$CONTROLLER" = "auto" ]; then
  if grep -q "$MINCO_PLUGIN_NAME" "$PARAMS_FILE"; then
    CONTROLLER="minco"
  else
    CONTROLLER="omni"
  fi
fi

# 显式选 minco 但文件里没有 MINCO 插件 -> 直接失败, 不让"名字是 MINCO、跑的是 Omni"溜过去。
if [ "$CONTROLLER" = "minco" ] && ! grep -q "$MINCO_PLUGIN_NAME" "$PARAMS_FILE"; then
  echo "[错误] --controller minco, 但参数文件里没有 $MINCO_PLUGIN_NAME:" >&2
  echo "[错误]   $PARAMS_FILE" >&2
  echo "[提示] MINCO 仿真参数文件: $MINCO_PARAMS_DEFAULT" >&2
  exit 1
fi

# 反向也拦一下: 显式选 omni 却加载了 MINCO 那份, 表现同样是"选的与实际跑的不一致"。
if [ "$CONTROLLER" = "omni" ] && grep -q "$MINCO_PLUGIN_NAME" "$PARAMS_FILE"; then
  echo "[错误] --controller omni, 但参数文件里是 $MINCO_PLUGIN_NAME:" >&2
  echo "[错误]   $PARAMS_FILE" >&2
  echo "[提示] Omni 仿真参数文件: $OMNI_PARAMS_DEFAULT" >&2
  echo "[提示] 想跑 MINCO 就用 --controller minco (不给 -p 时会自动选对文件)。" >&2
  exit 1
fi

# ---------- 布尔值归一化 ----------
norm_bool() {
  case "$(printf '%s' "$1" | tr '[:upper:]' '[:lower:]')" in
    1|true|yes|on)  echo "True" ;;
    0|false|no|off) echo "False" ;;
    *)              echo "$1" ;;
  esac
}
USE_RVIZ="$(norm_bool "$USE_RVIZ")"

norm_switch() {
  case "$(printf '%s' "$1" | tr '[:upper:]' '[:lower:]')" in
    1|true|yes|on)  echo "true" ;;
    0|false|no|off) echo "false" ;;
    *)              echo "$1" ;;
  esac
}
USE_GUI="$(norm_switch "$USE_GUI")"
RUN_IMMEDIATELY="$(norm_switch "$RUN_IMMEDIATELY")"
ROTATION_SINE_WAVE="$(norm_switch "$ROTATION_SINE_WAVE")"

# ---------- 初始重定位 (GICP) 前置解析 ----------
USE_GICP="$(norm_switch "${USE_GICP:-0}")"
NAV_EXTRA_ARGS=""

# 先验 PCD: 接受绝对路径, 或文件名(在若干常用目录里找)
resolve_prior_pcd() {
  local name="$1" dir
  case "$name" in
    /*) [ -f "$name" ] && { printf '%s' "$name"; return 0; }
        echo "[错误] --prior-pcd 指定的文件不存在: $name" >&2; return 1 ;;
  esac
  for dir in ${PRIOR_PCD_SEARCH_DIRS//:/ } "$WS_DIR/pcd/simulation" \
             "$WS_DIR/src/srm27_navigation/srm27_nav_bringup/pcd/simulation"; do
    [ -n "$dir" ] || continue
    if [ -f "$dir/$name" ]; then printf '%s' "$dir/$name"; return 0; fi
    if [ -f "$dir/$name.pcd" ]; then printf '%s' "$dir/$name.pcd"; return 0; fi
  done
  echo "[错误] 找不到先验点云 '$name'。请用 --prior-pcd 给绝对路径。" >&2
  return 1
}

if [ "$USE_GICP" = "true" ]; then
  if [ -z "$PRIOR_PCD_FILE" ]; then
    echo "[错误] --gicp 需要 --prior-pcd <先验点云 PCD>" >&2
    echo "[提示] 仓库里 pcd/simulation 目前只有 readme, 需要先建图 (slam:=True), 或用" >&2
    echo "       script/pcd_to_mapframe.py 把建图产物处理成 GICP 要的坐标系后放进该目录。" >&2
    exit 1
  fi
  PRIOR_PCD_FILE="$(resolve_prior_pcd "$PRIOR_PCD_FILE")" || exit 1
  [ -z "$PRIOR_PCD_MANIFEST" ] && [ -f "${PRIOR_PCD_FILE%.pcd}_manifest.yaml" ] && \
    PRIOR_PCD_MANIFEST="${PRIOR_PCD_FILE%.pcd}_manifest.yaml"
  printf -v PRIOR_PCD_Q '%q' "$PRIOR_PCD_FILE"
  NAV_EXTRA_ARGS=" use_pcd_localization:=True use_lio_odometry:=False prior_pcd_file:=${PRIOR_PCD_Q}"
  # 栅格图 (map_server 用) 与先验点云必须同原点, 但它不是定位输入;
  # 若同目录正好有一张同名栅格图, 提示一下, 不自动替换 (避免顺序耦合)。
  if [ -z "${MAP:-}" ] && [ -f "${PRIOR_PCD_FILE%.pcd}.yaml" ]; then
    echo "[提示] 未显式 -m/--map, 当前用默认地图; 若先验点云是同源重建的, 建议:"
    echo "       -m ${PRIOR_PCD_FILE%.pcd}.yaml"
  fi
else
  NAV_EXTRA_ARGS=" use_pcd_localization:=False"
fi

# ---------- GICP: 由建图清单推导 map->odom 初值 ----------
# init_pose 是 small_gicp_relocalization 的节点参数, 语义是 map->odom 初值,
# 与"机器人在 map 里的位姿"不是同一个量 (方案 §2)。
#   T_M_O_init = T_M_W · T_W_B(spawn)
# odom 与地图在出生点同原点, 因此建图起点 == 出生点 时上面结果就是恒等矩阵;
# 两者不同原点时, 这个式子正好给出让机器人出现在正确位置所需的初值。
if [ "$USE_GICP" = "true" ]; then
  read_spawn() {
    python3 - "$SIM_CONFIG" "${1:-x}" <<'PYEOF'
import sys
import yaml
cfg = yaml.safe_load(open(sys.argv[1], "r", encoding="utf-8"))
print(cfg.get("robot", {}).get(sys.argv[2], ""))
PYEOF
  }
  SPAWN_X="$(read_spawn x)"; SPAWN_Y="$(read_spawn y)"; SPAWN_YAW="$(read_spawn yaw)"

  if [ -n "$PRIOR_PCD_MANIFEST" ] && [ -f "$PRIOR_PCD_MANIFEST" ]; then
    if [ -z "$INIT_POSE" ]; then
      INIT_POSE="$(python3 - "$PRIOR_PCD_MANIFEST" "$SPAWN_X" "$SPAWN_Y" "$SPAWN_YAW" <<'PYEOF'
import math
import sys
import yaml

manifest, sx, sy, syaw = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
rows = yaml.safe_load(open(manifest, "r", encoding="utf-8")).get("map_origin", {}).get("T_M_W")
if not rows:
    sys.exit("manifest 缺少 map_origin.T_M_W")


def inverse(t):
    out = [[t[j][i] for j in range(3)] + [0.0] for i in range(3)]
    for i in range(3):
        out[i][3] = -sum(t[j][i] * t[j][3] for j in range(3))
    out.append([0.0, 0.0, 0.0, 1.0])
    return out


def mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


t_m_w = [[float(v) for v in row] for row in rows]
c, s = math.cos(syaw), math.sin(syaw)
t_w_b = [[c, -s, 0.0, sx], [s, c, 0.0, sy], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]
# odom 原点就在出生点 (odom:=T_W_B(spawn)), 因此
#   T_M_O_init = T_M_W · T_W_B(spawn) = inverse(T_W_B(spawn)) · T_W_B(spawn) = I
# 建图起点与出生点一致时结果就是恒等; 不一致时正好给出需要的初值。
t_m_o = mul(t_m_w, t_w_b)
print("%.6f,%.6f,%.6f,0.0,0.0,%.6f" % (t_m_o[0][3], t_m_o[1][3], t_m_o[2][3],
                                       math.atan2(t_m_o[1][0], t_m_o[0][0])))
PYEOF
)" || { echo "[错误] 从 $PRIOR_PCD_MANIFEST 推导 init_pose 失败。" >&2; exit 1; }
      echo "[提示] 由 $(basename "$PRIOR_PCD_MANIFEST") 的 T_M_W 与出生点 (${SPAWN_X}, ${SPAWN_Y}, yaw ${SPAWN_YAW}) 推出 init_pose=${INIT_POSE}"
    fi

    MANIFEST_SPAWN="$(python3 - "$PRIOR_PCD_MANIFEST" <<'PYEOF'
import sys
import yaml
spawn = yaml.safe_load(open(sys.argv[1], "r", encoding="utf-8")) \
    .get("map_origin", {}).get("spawn_pose_world", {})
print("%s|%s|%s" % (spawn.get("x", ""), spawn.get("y", ""), spawn.get("yaw", "")))
PYEOF
)"
    MS_X="${MANIFEST_SPAWN%%|*}"
    MS_REST="${MANIFEST_SPAWN#*|}"
    MS_Y="${MS_REST%%|*}"
    MS_YAW="${MS_REST#*|}"
    if [ -n "$MS_X" ] && [ "$MS_X" != "$SPAWN_X" ]; then
      echo "[警告] 先验点云清单的建图出生点是 (${MS_X}, ${MS_Y}, yaw ${MS_YAW}), 与 srm_sim.yaml 的" >&2
      echo "       (${SPAWN_X}, ${SPAWN_Y}, yaw ${SPAWN_YAW}) 不一致: 两张图不是同一原点," >&2
      echo "       RViz 里先验点云会与机器人错位。请把 srm_sim.yaml 的 robot.x/y/yaw 改回建图时的" >&2
      echo "       位置, 或用 --init-pose 显式给 map->odom 初值。" >&2
    fi
  else
    echo "[提示] 未找到先验点云清单 (${PRIOR_PCD_FILE%.pcd}_manifest.yaml), 无法自动推导 init_pose。"
    echo "       将使用 init_pose=${INIT_POSE:-0,0,0,0,0,0}; 若先验点云原点不是出生点, 请在" >&2
    echo "       RViz 用 2D Pose Estimate 给初值, 或用 --init-pose 指定。" >&2
  fi

  [ -z "$INIT_POSE" ] && INIT_POSE="0,0,0,0,0,0"
  GICP_RUN_ID="${GICP_RUN_ID:-$(date +%Y%m%dT%H%M%S)}"
fi



if [ -n "$WORLD_FROM_USER" ] && [ -n "$GZ_WORLD" ] && [ "$GZ_WORLD" != "$WORLD" ]; then
  echo "[提示] 世界由 srm_sim.yaml 的 '$GZ_WORLD' 覆盖为 '$WORLD'。"
fi

case "$ROTATION_MODE" in
  stop|constant|periodic) ;;
  *) echo "[错误] --rotation-mode 只支持 stop / constant / periodic" >&2; exit 2 ;;
esac

if [ "$START_ROTATION_SENDER" = "0" ] || [ "$START_ROTATION_SENDER" = "false" ] || [ "$START_ROTATION_SENDER" = "no" ]; then
  START_ROTATION_SENDER="False"
else
  START_ROTATION_SENDER="True"
fi

if [ "$ENABLE_TELEOP" = "0" ] || [ "$ENABLE_TELEOP" = "false" ] || [ "$ENABLE_TELEOP" = "no" ]; then
  ENABLE_TELEOP=0
else
  ENABLE_TELEOP=1
fi

# ---------- 进程检测 / 清理 ----------
# 方括号包住首字符, 避免 pgrep 匹配到自身命令行。
GAZEBO_PATTERN='(^|/)(gzserver|gzclient|gazebo)([[:space:]]|$)|[i]gn[[:space:]]+gazebo|[g]z[[:space:]]+sim|[r]os2[[:space:]]+launch[[:space:]]+srm27_gazebo_simulator'
GAZEBO_RESIDUAL_PATTERN='[/]ros_gz_bridge/parameter_bridge[[:space:]]+/clock@rosgraph_msgs/msg/Clock\[gz.msgs.Clock|[s]rm_velocity_adapter|[r]obot_state_publisher.*__ns:=/red_standard_robot1'
NAV_PATTERN='[r]os2[[:space:]]+launch[[:space:]]+srm27_nav_bringup[[:space:]]+(nav_srm_simulation_launch|nav_simulation_launch)\.py'
TELEOP_PATTERN='[s]rm27_teleop_twist_joy_node'

GAZEBO_LOCK_FILE="${SRM27_GAZEBO_LOCK_FILE:-${XDG_RUNTIME_DIR:-/tmp}/srm27_sentry_gazebo.lock}"

process_running() {
  pgrep -u "$(id -u)" -f "$1" >/dev/null 2>&1
}

gazebo_lock_held() {
  local lock_fd
  exec {lock_fd}>"$GAZEBO_LOCK_FILE"
  if flock -n "$lock_fd"; then
    flock -u "$lock_fd"
    exec {lock_fd}>&-
    return 1
  fi
  exec {lock_fd}>&-
  return 0
}

# 上次 launch 异常退出时, /clock bridge 与机器人节点可能留在后台;
# 没有 Gazebo 本体却有这些残留时先清掉, 避免新旧时钟并存。
cleanup_orphaned_gazebo() {
  process_running "$GAZEBO_PATTERN" && return 0
  process_running "$GAZEBO_RESIDUAL_PATTERN" || return 0

  echo "[提示] 检测到上次仿真的残留进程, 正在清理..."
  WAIT_SECONDS="${GAZEBO_CLEANUP_WAIT_SECONDS:-5}" "$SCRIPT_DIR/kill_gzb.sh"
}

# ---------- 启动前清理残留 ----------
# 每次启动前依次跑 kill_nav.sh / kill_gzb.sh / kill_rviz.sh: 上一次异常退出留下的
# Gazebo (以及它的 /clock bridge) 和 RViz 不清掉, 会和这次启动的仿真抢 /clock、抢
# RViz 窗口; Nav2 导航栈不清掉则更隐蔽 —— launch 会被下面的 start_once 判为"已在运行"
# 而跳过, 于是节点继续用**旧参数**(膨胀半径/限速都是 configure 期读的)跑。
# 跳过本步骤: PREKILL=0 ./script/start_sim_nav.sh
prekill_leftovers() {
  if [ "${DRY_RUN:-0}" = "1" ]; then
    echo "[dry-run] 跳过启动前清理 (kill_nav.sh / kill_gzb.sh / kill_rviz.sh)。"
    return 0
  fi
  if [ "${PREKILL:-1}" = "0" ]; then
    echo "[提示] PREKILL=0, 跳过启动前清理 (kill_nav.sh / kill_gzb.sh / kill_rviz.sh)。"
    return 0
  fi

  echo "[清理] 启动前先结束残留的 Nav2 导航栈 / Gazebo / RViz 进程..."
  local name
  for name in kill_nav.sh kill_gzb.sh kill_rviz.sh; do
    if [ ! -x "$SCRIPT_DIR/$name" ]; then
      echo "[警告] 未找到可执行脚本: $SCRIPT_DIR/$name (跳过)" >&2
      continue
    fi
    if ! "$SCRIPT_DIR/$name"; then
      # 退出码非 0 只代表"还有进程没死透", 两个 kill 脚本该 SIGINT/SIGKILL 的都发过了;
      # 这里只告警不阻断, 免得清理失败反而连启动都做不了。
      echo "[警告] $SCRIPT_DIR/$name 退出码非 0: 可能有进程未能结束, 继续启动。" >&2
    fi
  done
}

# ---------- 终端 ----------
open_term() {
  local title="$1"
  local cmd="$2"
  # 先 cd 到工作空间并 source 环境; 命令结束后保留 shell 便于查看输出。
  local full_cmd="cd '$WS_DIR' && source install/setup.bash && ${cmd}; exec bash"

  if [ "$DRY_RUN" = "1" ]; then
    echo "--- [dry-run] 标签页: $title"
    echo "$full_cmd"
    return 0
  fi

  if [ "$OPEN_MODE" = "tab" ]; then
    "$TERMINAL" --tab --title="$title" -- bash -c "$full_cmd"
  else
    "$TERMINAL" --title="$title" -- bash -c "$full_cmd"
  fi
}

start_once() {
  local title="$1"
  local process_pattern="$2"
  local cmd="$3"

  if process_running "$process_pattern"; then
    echo "[跳过] 已检测到正在运行的进程: $title"
    return 0
  fi

  open_term "$title" "$cmd"
}

# ---------- 启动 ----------
printf '========== SRM27 仿真 NAV 导航 (世界: %s) ==========\n' "$WORLD"
printf '工作空间  : %s\n' "$WS_DIR"
printf '地图      : %s\n' "$MAP_FILE"
printf '参数文件  : %s\n' "$PARAMS_FILE"
# FollowPath 实际用的插件直接从参数文件里读出来打印 —— 比按名字猜可靠, 选错文件时一眼可见。
FOLLOWPATH_PLUGIN="$(grep -A3 -E '^[[:space:]]*FollowPath:' "$PARAMS_FILE" 2>/dev/null \
  | grep -m1 -E '^[[:space:]]*plugin:' \
  | sed -e 's/^[^"]*"//' -e 's/".*$//')"
printf '控制器    : %s%s\n' "$CONTROLLER" \
  "$([ -n "$FOLLOWPATH_PLUGIN" ] && printf '  (FollowPath = %s)' "$FOLLOWPATH_PLUGIN")"
if [ "$USE_GICP" = "true" ]; then
  printf '定位方式  : GICP 初始重定位 (slam:=False, use_pcd_localization:=True, use_lio_odometry:=False)\n'
  printf '先验点云  : %s\n' "$PRIOR_PCD_FILE"
  [ -n "$PRIOR_PCD_MANIFEST" ] && printf '点云清单  : %s\n' "$PRIOR_PCD_MANIFEST"
  printf 'map->odom 初值: [%s]\n' "$INIT_POSE"
  printf '试验编号  : %s (写入逐次诊断 run_id)\n' "$GICP_RUN_ID"
else
  printf '定位方式  : 静态地图 + 仿真真值里程计 (slam:=False, use_pcd_localization:=False)\n'
fi
printf 'RViz      : %s    Gazebo GUI: %s    直接运行: %s\n' "$USE_RVIZ" "$USE_GUI" "$RUN_IMMEDIATELY"
printf '平滑器    : %s\n' "$USE_VELOCITY_SMOOTHER"
printf '自转测试  : 发送器 %s  模式 %s' "$START_ROTATION_SENDER" "$ROTATION_MODE"
case "$ROTATION_MODE" in
  constant) printf '  角速度 %s rad/s' "$ROTATION_SPEED" ;;
  periodic) printf '  offset %s + amplitude %s, period %s s, phase %s rad, %s' \
      "$ROTATION_OFFSET" "$ROTATION_AMPLITUDE" "$ROTATION_PERIOD" "$ROTATION_PHASE" \
      "$([ "$ROTATION_SINE_WAVE" = "true" ] && echo 正弦 || echo 方波)" ;;
esac
printf '\n终端模式  : %s (%s)\n' "$OPEN_MODE" "$TERMINAL"
echo

prekill_leftovers

SIM_CMD="ros2 launch srm27_gazebo_simulator srm_sim.launch.py world:=${WORLD} gui:=${USE_GUI} run_immediately:=${RUN_IMMEDIATELY}"

if [ "$DRY_RUN" != "1" ]; then
  mkdir -p "$(dirname "$GAZEBO_LOCK_FILE")"
  echo "[1/3] 启动 SRM Gazebo 仿真..."
  cleanup_orphaned_gazebo

  if process_running "$GAZEBO_PATTERN" || gazebo_lock_held; then
    echo "[跳过] 已检测到正在运行或正在启动的 Gazebo 仿真。"
  else
    GAZEBO_LOCK_FILE_Q="$(printf '%q' "$GAZEBO_LOCK_FILE")"
    # 释放 FD 9 后终端才会进入保留 shell, 否则锁会被一直持有,
    # 导致旧 Gazebo 退出后下一次仿真仍起不来。
    GAZEBO_CMD="exec 9>${GAZEBO_LOCK_FILE_Q}; if ! flock -n 9; then echo '[跳过] 另一个启动脚本已占用 Gazebo 锁。'; else ${SIM_CMD}; fi; flock -u 9; exec 9>&-"
    open_term "SRM27 Gazebo 仿真" "$GAZEBO_CMD"
  fi
else
  echo "[1/3] SRM Gazebo 仿真 (dry-run)..."
  open_term "SRM27 Gazebo 仿真" "$SIM_CMD"
fi

echo "[2/3] 启动速度合成 + 导航栈 + RViz..."
MAP_FILE_Q="$(printf '%q' "$MAP_FILE")"
PARAMS_FILE_Q="$(printf '%q' "$PARAMS_FILE")"

# GICP 模式下把 init_pose 写进一份临时参数副本: init_pose 是 small_gicp_relocalization
# 的节点参数, launch 入口没有为它开独立 launch 参数, 直接改副本最省事且可追溯。
if [ "$USE_GICP" = "true" ]; then
  GICP_PARAMS_FILE="${TMPDIR:-/tmp}/gicp_nav2_params_$$.yaml"
  cp "$PARAMS_FILE" "$GICP_PARAMS_FILE"
  python3 - "$GICP_PARAMS_FILE" "$INIT_POSE" "$GICP_RUN_ID" <<'PYEOF'
import sys
import yaml

path, init_pose, run_id = sys.argv[1], sys.argv[2], sys.argv[3]
values = [float(v) for v in init_pose.split(",")]
if len(values) != 6:
    sys.exit("init_pose 需要 6 个数: x,y,z,roll,pitch,yaw")
text = open(path, "r", encoding="utf-8").read()
data = yaml.safe_load(text)
node = None
for key in data:
    if key.split("/")[-1] == "small_gicp_relocalization":
        node = data[key]
        break
if node is None:
    sys.exit("参数文件里找不到 small_gicp_relocalization 段")
node.setdefault("ros__parameters", {})["init_pose"] = values
node["ros__parameters"]["run_id"] = run_id
with open(path, "w", encoding="utf-8") as stream:
    yaml.safe_dump(data, stream, allow_unicode=True, sort_keys=False)
print(f"[提示] init_pose={values} run_id={run_id} 已写入临时参数副本 {path}")
PYEOF
  PARAMS_FILE_Q="$(printf '%q' "$GICP_PARAMS_FILE")"
  echo "[提示] GICP 参数副本: $GICP_PARAMS_FILE"
fi
NAV_CMD="ros2 launch srm27_nav_bringup nav_srm_simulation_launch.py namespace:=${ROBOT_NS} world:=${WORLD} map:=${MAP_FILE_Q} params_file:=${PARAMS_FILE_Q} slam:=False${NAV_EXTRA_ARGS} use_composition:=${USE_COMPOSITION} use_sim_time:=True use_rviz:=${USE_RVIZ} use_velocity_smoother:=${USE_VELOCITY_SMOOTHER} start_rotation_sender:=${START_ROTATION_SENDER} rotation_mode:=${ROTATION_MODE} rotation_speed:=${ROTATION_SPEED} rotation_offset:=${ROTATION_OFFSET} rotation_amplitude:=${ROTATION_AMPLITUDE} rotation_period:=${ROTATION_PERIOD} rotation_phase:=${ROTATION_PHASE} rotation_sine_wave:=${ROTATION_SINE_WAVE}"
start_once "SRM27 导航 + 速度合成 + RViz" "$NAV_PATTERN" "$NAV_CMD"

if [ "$ENABLE_TELEOP" = "1" ]; then
  echo "[3/3] 启动手柄自转 (输出 remap 到 rotation_cmd)..."
  start_once \
    "SRM27 手柄自转" \
    "$TELEOP_PATTERN" \
    "ros2 run srm27_teleop_twist_joy srm27_teleop_twist_joy_node --ros-args -r __ns:=/${ROBOT_NS} -r cmd_vel:=rotation_cmd -p use_sim_time:=true"
else
  echo "[3/3] 跳过手柄自转 (--no-teleop)。自转请用 --rotation-mode 选择模式。"
fi

cat <<EOF

启动流程处理完成。
  - 导航栈刚起时打印 "waiting for clock" 属正常, 等 Gazebo 起来后会自行继续。
  - Gazebo $( [ "$RUN_IMMEDIATELY" = "true" ] && echo "已直接开始运行(gz sim -r), 不用点播放。" || echo "以暂停启动(--no-run), 需要点播放后仿真才推进。" )
  - Gazebo 里车出现后, 在 RViz 用 "2D Goal Pose" / "Nav2 Goal" 下发目标点。

速度链路检查 (新终端):
  cd '$WS_DIR' && source install/setup.bash
  ros2 topic echo /${ROBOT_NS}/cmd_vel_nav --once         # 导航平移速度 (angular.z 应为 0)
  ros2 topic echo /${ROBOT_NS}/rotation_velocity --once   # 独立自转速度
  ros2 topic echo /${ROBOT_NS}/cmd_vel_sim --once         # 合成后的最终执行命令
  ros2 topic hz /${ROBOT_NS}/cmd_vel_sim                  # 应为 200 Hz 左右
  ros2 topic echo /${ROBOT_NS}/odometry --once            # 真值里程计 (含实际 yaw/wz)
  ros2 run tf2_ros tf2_echo map base_link                 # 完整 TF 链: map->odom->base_link
  ros2 topic echo /${ROBOT_NS}/diagnostics --once         # 合成与超时状态

切换自转模式 (无需重启导航):
  ros2 service call /${ROBOT_NS}/rotation_test_sender/disable std_srvs/srv/Trigger
  ros2 param set /${ROBOT_NS}/rotation_test_sender rotation_mode periodic
  ros2 param set /${ROBOT_NS}/rotation_test_sender offset 1.0
  ros2 param set /${ROBOT_NS}/rotation_test_sender amplitude 0.5
  ros2 param set /${ROBOT_NS}/rotation_test_sender period 4.0
  ros2 service call /${ROBOT_NS}/rotation_test_sender/enable std_srvs/srv/Trigger

急停 (清零并保持零速):
  ros2 service call /${ROBOT_NS}/srm_cmd_mux/stop_all std_srvs/srv/Trigger
  ros2 service call /${ROBOT_NS}/srm_cmd_mux/resume_all std_srvs/srv/Trigger

清理残留:
  ./script/kill_gzb.sh && ./script/kill_rviz.sh
EOF

# 只有真的跑 MINCO 才打印这段自检: 换控制器最容易出的错是"以为换了, 其实参数文件没选对"。
if [ "$CONTROLLER" = "minco" ]; then
  cat <<EOF

MINCO 自检 (仿真, 按顺序确认):
  1) 插件真的被加载 (不是"launch 没报错"就算):
       ros2 param get /${ROBOT_NS}/controller_server FollowPath.plugin
       期望: srm27_minco_controller::MincoMpcController
       若显示 Omni, 说明参数文件选错了 (--controller minco 会自动选对文件; 但若旧进程还在,
       也会看到旧值 —— 那种情况先 ./script/kill_nav.sh)。
  2) 轨迹真的算出来了:
       ros2 topic hz /${ROBOT_NS}/FollowPath/minco_trajectory
       ros2 topic hz /${ROBOT_NS}/FollowPath/mpc_prediction
  3) 诊断健康 (判"卡在哪"的第一手证据, 失败路径也会发布):
       ros2 topic echo /${ROBOT_NS}/FollowPath/diagnostics --once
       关注: planning_result=success、minimum_clearance>0、missed_deadline_count 不持续增长。
  4) 阶段一 MPC 没有自转权限: /${ROBOT_NS}/cmd_vel_nav 的 angular.z 应恒为 0 (自转仍由自转链路负责)。
EOF
fi
