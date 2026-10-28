#!/usr/bin/env bash
# =============================================================
# SRM 仿真 NAV 导航 (一键启动)
#
#   标签页 1: SRM Gazebo 仿真 (世界 + SRM 圆柱底盘 + 雷达 + 速度执行)
#   标签页 2: 速度出口 + Nav2 导航栈 + RViz
#
# 速度链路 (见 docs/wz控制迁移导航与fakeyaw仿真实施方案.md):
#
#   MINCO/Nav2(base_link_fake) ─cmd_vel_nav_fake─► fake_vel_transform
#      ─cmd_vel_nav_body─► srm_cmd_mux ─cmd_vel_sim─► 速度适配器
#
#   **wz 由上位机(导航栈)拥有**: 底盘真实 yaw 由导航输出的 wz 积分得到。
#   2026-10-19 独立自转链 (rotation_test_sender / rotation_controller /
#   rotation_velocity) 已删除, 不存在第二路 wz 来源, 也没有 --wz-source/
#   --no-fakeyaw/--rotation-* 这些开关。
#   小陀螺(持续自转)用 --yaw-mode spin + script/spin_control.sh: 请求直接进 MINCO,
#   wz 仍由导航输出 (不再经 rotation_controller)。
#
# 速度限幅 (控制器 / 平滑器 / mux / 恢复行为) 的**唯一来源**:
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
#   ./script/start_sim_nav.sh                                  # 默认: MINCO + fakeyaw (导航拥有 wz)
#   ./script/start_sim_nav.sh --yaw-mode spin                   # 小陀螺 (脚本手动控制转速)
#   ./script/start_sim_nav.sh --verify-spin                     # 一步验证小陀螺导航 (无头 + 自动断言 + 清理)
#   ./script/start_sim_nav.sh --no-run                          # Gazebo 保持暂停, 手动点播放
#   ./script/start_sim_nav.sh --no-rviz                         # 只跑仿真 + 导航
#   ./script/start_sim_nav.sh --gicp --prior-pcd /abs/prior.pcd \
#       --map /abs/prior.yaml                                   # 用 GICP 初始重定位
#   ./script/start_sim_nav.sh --gicp --prior-pcd /abs/prior.pcd \
#       --init-pose "0,0,0,0,0,0.5236"                          # 手动给 map->odom 初值
#   DRY_RUN=1 ./script/start_sim_nav.sh                         # 只打印将执行的命令
#
# 参数:
#   -w, --world   <name>      rmuc_2025 / rmuc_2024 / rmul_2024 / rmul_2025 / srm_empty
#   -m, --map     <名字|绝对路径>  地图名(自动补 .yaml)或 YAML 绝对路径
#   -p, --params  <绝对路径>  Nav2 参数文件; 不给时固定用
#                             config/simulation/nav2_params_srm_fakeyaw.yaml
#                             (MincoMpcController + se2 + base_link_fake, 限幅由
#                              speed_limits.yaml 合并)。必须是 MINCO 那一套 ——
#                             Omni/旧链参数文件在本入口已经不可用。
#   -c, --controller <auto|minco>
#                            局部控制器 = 用哪份 Nav2 参数文件 (与 start_real_nav.sh 同名同义):
#                              minco -> MINCO+MPC (FollowPath = srm27_minco_controller::MincoMpcController)
#                              auto  -> 默认值: 按参数文件里 FollowPath 的插件判定。
#                            注意: 只给 -p 时按 auto 判定; -c 与 -p 同时给出时以 -p 加载,
#                            但会校验两者不矛盾: 选错直接报错, 不让"名字是 MINCO、跑的是 Omni"溜过去。
#       --minco             等价于 --controller minco
#                            (--omni / -c omni 已随独立自转链一起删除, 见下)
#       --gicp / --no-gicp       是否用 GICP 初始重定位 (默认关闭 = 原来的静态地图模式)
#       --prior-pcd <绝对路径>   先验点云 PCD (--gicp 时必填)
#       --prior-pcd-manifest <绝对路径>
#                                先验点云清单 (重建工具生成的 *_manifest.yaml);
#                                给出时会按其中的 T_M_W 与 srm_sim.yaml 的出生点自动
#                                推出 init_pose, 并在两者不一致时告警
#       --init-pose "<x,y,z,roll,pitch,yaw>"
#                                map->odom 初值, 等价于 nav2 参数的 init_pose;
#                                给了就以它为准 (不再从 manifest 推导)
#       --yaw-mode <auto|se2|spin>  MINCO 航向策略 (默认 auto=se2, 导航拥有 wz)。
#                                   spin = 小陀螺: 自转请求经 rotation_cmd 直接进
#                                   MINCO, 判停自动换 OmniStoppedGoalChecker,
#                                   转速用 script/spin_control.sh
#       (--wz-source / --fakeyaw / --no-fakeyaw / --yaw-mode xy_only 已删除:
#        独立自转链于 2026-10-19 删除, wz 只由导航产生, 没有旧链可退回)
#       --verify-spin       一步验证小陀螺导航: 强制无头 (--no-rviz --no-gui, direct 模式),
#                           启动后自动发自转请求 + 导航目标, 断言
#                           "wz 归导航 / fake->body 逐条透传 / 边转边走 / 到点成功 / 撤回即停",
#                           最后清理并返回退出码 (0 = 全通过, 1 = 有用例失败, 3 = 启动超时)。
#                           可用环境变量: VERIFY_SPIN_WZ(2.0) VERIFY_GOAL_DIST(1.5)
#                           VERIFY_KEEP(1 = 不清理) VERIFY_READY_TIMEOUT(180) VERIFY_GOAL_TIMEOUT(180)
#       --rviz / --no-rviz        是否启动 RViz (默认启动)
#       --rviz-config <路径>      用自定义 .rviz 配置 (默认包内 rviz/nav2_default_view.rviz)。
#                                 例: --rviz-config viz/cloud_only.rviz —— 只看 3D 点云,
#                                 不显示二维地图与代价地图
#       --gui / --no-gui          Gazebo 是否带 GUI (默认带)
#       --run / --no-run          Gazebo 是否直接开始运行 (默认直接运行, 即 gz sim -r;
#                                 --no-run 则保持暂停等你点播放)
#       --smoother / --no-smoother velocity_smoother 是否串联 (默认串联)
#   -h, --help                显示本帮助
#
# 环境变量:
#   WORLD MAP MAP_NAME PARAMS_FILE CONTROLLER USE_RVIZ USE_GUI RUN_IMMEDIATELY
#   USE_GICP PRIOR_PCD_FILE PRIOR_PCD_MANIFEST INIT_POSE GICP_RUN_ID
#   USE_VELOCITY_SMOOTHER YAW_MODE
#   VERIFY_SPIN_WZ VERIFY_GOAL_DIST VERIFY_KEEP VERIFY_READY_TIMEOUT VERIFY_GOAL_TIMEOUT
#   OPEN_MODE(tab|window|direct) TERMINAL ROBOT_NS USE_COMPOSITION DRY_RUN SRM27_WS_DIR PREKILL
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
#   - 控制清单：导航速度 cmd_vel_nav_fake → cmd_vel_nav_body、执行命令 cmd_vel_sim、
#     小陀螺请求 rotation_cmd; 诊断在 /<ns>/diagnostics。
# =============================================================

set -euo pipefail

# ---------- 路径 ----------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${SRM27_WS_DIR:-$(cd "$SCRIPT_DIR/.." && pwd)}"
PKG_SRC="$WS_DIR/src/srm27_navigation/srm27_nav_bringup"
SIM_SRC="$WS_DIR/src/srm27_gazebo_simulator"

# ---------- 可配置项 ----------
OPEN_MODE="${OPEN_MODE:-tab}"                # tab: 同一窗口多标签页; window: 每命令独立窗口; direct: 无终端后台直启(日志在 .sim_run/logs/)
TERMINAL="${TERMINAL:-gnome-terminal}"
ROBOT_NS="${ROBOT_NS:-red_standard_robot1}"  # 与 nav_srm_simulation_launch.py 的 namespace 默认值一致
USE_COMPOSITION="${USE_COMPOSITION:-False}"
USE_RVIZ="${USE_RVIZ:-True}"
USE_GUI="${USE_GUI:-true}"
# Gazebo 默认"起来就跑"(gz sim -r): 仿真时间立即推进, 导航栈不用等手动点播放。
# 想保持旧的"暂停等点播放"行为: --no-run 或 RUN_IMMEDIATELY=false。
RUN_IMMEDIATELY="${RUN_IMMEDIATELY:-true}"
USE_VELOCITY_SMOOTHER="${USE_VELOCITY_SMOOTHER:-True}"
DRY_RUN="${DRY_RUN:-0}"                      # 1 = 只打印将执行的命令, 不真的启动
# 一步验证小陀螺导航 (--verify-spin): 无头起栈 -> 发自转请求 -> 发导航目标 ->
# 断言 wz 链路与"边转边走" -> 清理。参数用环境变量覆盖。
VERIFY_SPIN="${VERIFY_SPIN:-0}"              # 1 = 跑完验证流程 (由 --verify-spin 置位)
VERIFY_SPIN_WZ="${VERIFY_SPIN_WZ:-2.0}"      # 自转请求 rad/s
VERIFY_GOAL_DIST="${VERIFY_GOAL_DIST:-1.5}"  # 首选导航目标距离 (米), 不可达会自动换候选
VERIFY_READY_TIMEOUT="${VERIFY_READY_TIMEOUT:-180}"
VERIFY_GOAL_TIMEOUT="${VERIFY_GOAL_TIMEOUT:-180}"
VERIFY_KEEP="${VERIFY_KEEP:-0}"              # 1 = 验证完不清理, 留着看 RViz/调试
#: MINCO 航向策略: auto=se2 (导航拥有 wz); spin=小陀螺 (外部自转请求进线)。
YAW_MODE="${YAW_MODE:-auto}"

# 自定义 RViz 配置（留空 = 用包内 rviz/nav2_default_view.rviz；--rviz-config 覆盖）
RVIZ_CONFIG="${RVIZ_CONFIG:-}"
MAP_NAME="${MAP_NAME:-}"
MAP_FILE="${MAP:-}"
PARAMS_FILE="${PARAMS_FILE:-}"
PARAMS_FILE_GIVEN=0

# 局部控制器 (决定用哪份 Nav2 参数文件): auto 按参数文件里的 FollowPath 插件判定,
# minco 显式指定。默认 auto, 缺省参数文件就是 fakeyaw 那份。
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
    -c|--controller) CONTROLLER="${2:?--controller 需要 auto|minco}"
                     CONTROLLER_CLI_GIVEN=1; shift 2 ;;
    --minco)      CONTROLLER="minco"; CONTROLLER_CLI_GIVEN=1; shift ;;
    --omni)       echo "[错误] --omni 已删除: 独立自转链删除后 Omni 参数文件(base_link/旧链)" >&2
                  echo "       无法在仿真链上工作, 唯一链路是 MINCO + fakeyaw (--minco 或默认)。" >&2
                  exit 2 ;;
    --gicp)       USE_GICP="1"; shift ;;
    --no-gicp)    USE_GICP="0"; shift ;;
    --prior-pcd)          PRIOR_PCD_FILE="${2:?--prior-pcd 需要绝对路径}"; shift 2 ;;
    --prior-pcd-manifest) PRIOR_PCD_MANIFEST="${2:?--prior-pcd-manifest 需要绝对路径}"; shift 2 ;;
    --init-pose)          INIT_POSE="${2:?--init-pose 需要 x,y,z,roll,pitch,yaw}"; shift 2 ;;
    --rviz)       USE_RVIZ="True"; shift ;;
    --no-rviz)    USE_RVIZ="False"; shift ;;
    --rviz-config) RVIZ_CONFIG="${2:?--rviz-config 需要 .rviz 文件绝对路径}"; shift 2 ;;
    --gui)        USE_GUI="true"; shift ;;
    --no-gui)     USE_GUI="false"; shift ;;
    --run)        RUN_IMMEDIATELY="true"; shift ;;
    --no-run)     RUN_IMMEDIATELY="false"; shift ;;
    --smoother)   USE_VELOCITY_SMOOTHER="True"; shift ;;
    --no-smoother) USE_VELOCITY_SMOOTHER="False"; shift ;;
    --yaw-mode)   YAW_MODE="${2:?--yaw-mode 需要 auto|se2|spin}"; shift 2 ;;
    --verify-spin) VERIFY_SPIN=1; YAW_MODE="spin"; shift ;;
    -h|--help)    usage; exit 0 ;;
    *) echo "[错误] 未知参数: $1 (用 -h 查看用法)" >&2; exit 2 ;;
  esac
done

# --verify-spin 必须无头 + 无终端 (direct)：验证过程不需要 GUI，也不能依赖终端窗口。
if [ "$VERIFY_SPIN" = "1" ]; then
  OPEN_MODE="direct"
  USE_RVIZ="False"
  USE_GUI="false"
fi

# ---------- 航向策略 ----------
# 唯一链路: wz 由导航(上位机)拥有。yaw_mode=auto 与 se2 等价；spin = 小陀螺，
# 外部自转请求经 rotation_cmd 直接进 MINCO（独立自转链已删除，没有中间节点，
# 也没有 rotation_velocity）。
case "$YAW_MODE" in
  auto|se2|spin) ;;
  xy_only)
    echo "[错误] --yaw-mode xy_only 属于已删除的旧链路（导航不控自转）。" >&2
    echo "       独立自转链已删除, wz 只由导航产生: 请用 auto/se2（车头跟切线）或 spin。" >&2
    exit 2
    ;;
  *) echo "[错误] --yaw-mode 只支持 auto / se2 / spin" >&2; exit 2 ;;
esac
if [ "$VERIFY_SPIN" = "1" ] && [ "$YAW_MODE" != "spin" ]; then
  echo "[错误] --verify-spin 需要 --yaw-mode spin（小陀螺验证），当前是 $YAW_MODE" >&2
  exit 2
fi
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

# 默认 Nav2 参数文件: 唯一链路 = MINCO + fakeyaw。优先源码目录 (symlink-install 下与
# install 一致), 退化到 install。只给 -p 时按参数文件内容判定控制器。
PARAMS_DEFAULT_NAME="nav2_params_srm_fakeyaw.yaml"
MINCO_PLUGIN_NAME="srm27_minco_controller::MincoMpcController"

# 只给了 -p、没显式选控制器: 交给下面的 auto 判定 (与 start_real_nav.sh 同一套规则)。
if [ "$CONTROLLER_CLI_GIVEN" = "0" ] && [ "$PARAMS_FILE_GIVEN" = "1" ]; then
  CONTROLLER="auto"
fi

case "$CONTROLLER" in
  auto|minco) ;;
  omni)
    echo "[错误] -c omni 已删除: 独立自转链删除后 Omni 参数文件(base_link/旧链)无法在仿真链上" >&2
    echo "       工作 (它的角速度不归导航)。唯一链路是 MINCO + fakeyaw: 用默认或 -c minco。" >&2
    echo "       需要 Omni 基线做对照只能回 main 分支。" >&2
    exit 2
    ;;
  *) echo "[错误] --controller 只支持 auto / minco, 收到: $CONTROLLER" >&2; exit 2 ;;
esac

if [ -z "$PARAMS_FILE" ]; then
  PARAMS_FILE="$PKG_SRC/config/simulation/$PARAMS_DEFAULT_NAME"
  if [ ! -f "$PARAMS_FILE" ]; then
    PARAMS_FILE="$WS_DIR/install/srm27_nav_bringup/share/srm27_nav_bringup/config/simulation/$PARAMS_DEFAULT_NAME"
  fi
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

# 唯一链路要求 MINCO 插件 + fakeyaw 参数文件: 两者缺一就直接失败, 不跑"半条链"。
if ! grep -q "$MINCO_PLUGIN_NAME" "$PARAMS_FILE"; then
  echo "[错误] 参数文件里没有 $MINCO_PLUGIN_NAME:" >&2
  echo "[错误]   $PARAMS_FILE" >&2
  echo "[提示] 唯一链路是 MINCO + fakeyaw: 用 -p .../nav2_params_srm_fakeyaw.yaml" >&2
  echo "       (不给 -p 时默认就是它)。Omni/旧链参数文件已随独立自转链一起删除。" >&2
  exit 1
fi
if ! grep -q "base_link_fake" "$PARAMS_FILE"; then
  echo "[错误] 参数文件不是 fakeyaw 变体 (找不到 base_link_fake): $PARAMS_FILE" >&2
  echo "[提示] wz 由导航拥有, 导航参考系必须是 base_link_fake; 用默认参数文件即可。" >&2
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

# 唯一链路: fakeyaw 迁移 (导航拥有 wz), 命令系 base_link_fake。
USE_FAKEYAW_ARG="True"
COMMAND_FRAME_ARG="base_link_fake"

# ---------- 进程检测 / 清理 ----------
# 方括号包住首字符, 避免 pgrep 匹配到自身命令行。
GAZEBO_PATTERN='(^|/)(gzserver|gzclient|gazebo)([[:space:]]|$)|[i]gn[[:space:]]+gazebo|[g]z[[:space:]]+sim|[r]os2[[:space:]]+launch[[:space:]]+srm27_gazebo_simulator'
GAZEBO_RESIDUAL_PATTERN='[/]ros_gz_bridge/parameter_bridge[[:space:]]+/clock@rosgraph_msgs/msg/Clock\[gz.msgs.Clock|[s]rm_velocity_adapter|[r]obot_state_publisher.*__ns:=/red_standard_robot1'
NAV_PATTERN='[r]os2[[:space:]]+launch[[:space:]]+srm27_nav_bringup[[:space:]]+(nav_srm_simulation_launch|nav_simulation_launch)\.py'

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

  if [ "$OPEN_MODE" = "direct" ]; then
    # 无终端环境（无头沙箱 / CI / 无桌面的 SSH）：不弹终端，后台运行，日志落盘。
    # 脚本末尾会 wait 托管这些子进程——direct 模式下本脚本就是宿主，
    # 结束仿真 = 结束本脚本（或其进程组）。
    local direct_log_dir="$WS_DIR/.sim_run/logs"
    mkdir -p "$direct_log_dir"
    local direct_log_file="$direct_log_dir/$(printf '%s' "$title" | tr -c 'A-Za-z0-9_.-' '_').log"
    echo "[direct] $title -> $direct_log_file"
    bash -c "cd '$WS_DIR' && source install/setup.bash && ${cmd}" >"$direct_log_file" 2>&1 &
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
printf 'wz 归属   : 导航(上位机)  (fake_vel_transform 链路: 底盘 yaw 由导航 wz 积分; 独立自转链已删除)\n'
printf '速度链    : Nav2[cmd_vel_nav_fake] -> fake_vel_transform[cmd_vel_nav_body] -> srm_cmd_mux -> cmd_vel_sim -> Gazebo\n'
printf 'TF owner  : odom->base_link = 定位/真值里程计;  base_link->base_link_fake = fake_vel_transform\n'
printf '航向策略  : %s' "$YAW_MODE"
case "$YAW_MODE" in
  auto) echo '  (推导: se2, 导航拥有 wz)' ;;
  se2)  echo '  (导航拥有 wz: 车头跟路径切线)' ;;
  spin) echo '  (小陀螺: script/spin_control.sh on <rad/s> / off / status)' ;;
esac
if [ "$VERIFY_SPIN" = "1" ]; then
  printf '验证模式  : --verify-spin (自转 %s rad/s, 目标 %s m, 跑完%s)\n' \
      "$VERIFY_SPIN_WZ" "$VERIFY_GOAL_DIST" \
      "$([ "$VERIFY_KEEP" = "1" ] && echo '保留栈' || echo '自动清理')"
fi
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
# 自定义 RViz 配置 -> 传给 nav_srm_simulation_launch.py 的 rviz_config_file
RVIZ_CONFIG_ARG=""
if [ -n "$RVIZ_CONFIG" ]; then
  if [ ! -f "$RVIZ_CONFIG" ]; then
    echo "[错误] --rviz-config 指定的文件不存在: $RVIZ_CONFIG" >&2
    exit 1
  fi
  # 转成绝对路径: nav 那个标签页内部会先 cd 到工作空间, 相对路径容易失配
  RVIZ_CONFIG="$(readlink -f "$RVIZ_CONFIG")"
  RVIZ_CONFIG_ARG=" rviz_config_file:=$(printf '%q' "$RVIZ_CONFIG")"
fi

# 定位方式由 NAV_EXTRA_ARGS 统一给出 (use_pcd_localization / use_lio_odometry /
# prior_pcd_file); fakeyaw 链路的两个开关跟在它后面, 与定位方式正交。
NAV_CMD="ros2 launch srm27_nav_bringup nav_srm_simulation_launch.py namespace:=${ROBOT_NS} world:=${WORLD} map:=${MAP_FILE_Q} params_file:=${PARAMS_FILE_Q} slam:=False${NAV_EXTRA_ARGS} use_fakeyaw_transform:=${USE_FAKEYAW_ARG} command_frame:=${COMMAND_FRAME_ARG} use_composition:=${USE_COMPOSITION} use_sim_time:=True use_rviz:=${USE_RVIZ}${RVIZ_CONFIG_ARG} use_velocity_smoother:=${USE_VELOCITY_SMOOTHER} yaw_mode:=${YAW_MODE}"
start_once "SRM27 导航 + 速度出口 + RViz" "$NAV_PATTERN" "$NAV_CMD"

if [ "$VERIFY_SPIN" = "1" ]; then
  BANNER_DONE=1
fi
if [ "${BANNER_DONE:-0}" = "0" ]; then

cat <<EOF

启动流程处理完成。
  - 导航栈刚起时打印 "waiting for clock" 属正常, 等 Gazebo 起来后会自行继续。
  - Gazebo $( [ "$RUN_IMMEDIATELY" = "true" ] && echo "已直接开始运行(gz sim -r), 不用点播放。" || echo "以暂停启动(--no-run), 需要点播放后仿真才推进。" )
  - Gazebo 里车出现后, 在 RViz 用 "2D Goal Pose" / "Nav2 Goal" 下发目标点。

小陀螺 (需以 --yaw-mode spin 启动; 请求经 rotation_cmd 直接进 MINCO, wz 仍由导航输出):
  ./script/spin_control.sh on 2.0      # 恒速自转 2.0 rad/s
  ./script/spin_control.sh status      # 查看请求/实际转速
  ./script/spin_control.sh off         # 撤回请求 (停转)

急停 (清零并保持零速):
  ros2 service call /${ROBOT_NS}/srm_cmd_mux/stop_all std_srvs/srv/Trigger
  ros2 service call /${ROBOT_NS}/srm_cmd_mux/resume_all std_srvs/srv/Trigger

清理残留:
  ./script/kill_gzb.sh && ./script/kill_rviz.sh
EOF

cat <<EOF

速度链路检查 (新终端):
  cd '$WS_DIR' && source install/setup.bash
  ros2 topic echo /${ROBOT_NS}/cmd_vel_nav_fake --once    # 导航请求 (fake 系, 含 wz)
  ros2 topic echo /${ROBOT_NS}/cmd_vel_nav_body --once    # fake 转换后 (车体系, wz 应逐条相同)
  ros2 topic echo /${ROBOT_NS}/rotation_cmd --once         # 小陀螺请求 (仅 spin 模式有数据)
  ros2 topic echo /${ROBOT_NS}/cmd_vel_sim --once         # mux 最终输出 (含导航 wz)
  ros2 topic hz /${ROBOT_NS}/cmd_vel_sim                  # 应为 200 Hz 左右
  ros2 topic echo /${ROBOT_NS}/odometry --once            # 真值里程计 (含实际 yaw/wz)
  ros2 run tf2_ros tf2_echo base_link base_link_fake      # fakeyaw TF: yaw 应等于 -底盘 yaw
  ros2 run tf2_ros tf2_echo map base_link                 # 完整 TF 链
  ros2 topic echo /${ROBOT_NS}/diagnostics --once         # mux: wz_source=mavigation
  ros2 topic echo /${ROBOT_NS}/fake_vel_transform/diagnostics --once  # wz 透传误差/过期 yaw 计数

记录要求 (方案 §6): 必须区分"导航请求 wz"(cmd_vel_nav_fake.angular.z)、
"转换器输出 wz"(cmd_vel_nav_body.angular.z)、"mux 最终 wz"(cmd_vel_sim.angular.z)、
"里程计实际 wz"(odometry.twist.angular.z), 不能只看 Gazebo 模型是否转动。

P0 自检 (不需要 Gazebo):
  python3 script/check_wz_fakeyaw_chain.py
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
  4) MPC 拥有自转权限 (wz 归导航): /${ROBOT_NS}/cmd_vel_nav_fake 的 angular.z 在弯道应为非零,
     且与 cmd_vel_nav_body / cmd_vel_sim 的 angular.z 逐条相同 (fake 转换只转平移)。
     yaw_policy.mode 若不是 se2/spin, 这里会恒为 0 —— 那说明航向权限没开。

MINCO 脱困自检 (脱困判定 / 脱困路径规划 / 脱困成功判定):
  a) 参数真的生效 (configure 期读一次; 改完 yaml 必须重启导航栈):
       ros2 param get /${ROBOT_NS}/controller_server FollowPath.recovery.fail_threshold
       ros2 param get /${ROBOT_NS}/controller_server FollowPath.recovery.escape_speed
       ros2 param get /${ROBOT_NS}/controller_server FollowPath.recovery.stuck_clearance
     期望 5 / 0.5 / 0.25。启动日志里还有一行 "MincoMpcController 脱困参数: ...",
     打印的是夹取后实际生效的值, 必须与这里一致 (不一致说明参数文件选错了)。
  b) 脱困可视化话题存在 (平时是空的, 只有真脱困才有内容):
       ros2 topic list | grep recovery_markers
     期望 /${ROBOT_NS}/FollowPath/recovery_markers。
     RViz 里已预置显示项 "Recovery (MINCO)", 不用手动加:
       绿线 = 脱困路径规划结果, 蓝箭头 = 本周期下发的那一步, 球 = 当前净空。
  c) 实时看脱困状态 (只筛脱困字段, 不刷屏):
       ros2 topic echo /${ROBOT_NS}/FollowPath/diagnostics | grep -E "recovery_(active|trigger|exit|clearance)"
     正常导航 recovery_active=false; 进脱困为 true 且 recovery_trigger=gradient|circular_search;
     退出时 recovery_exit=escaped|timeout。
  d) 怎么把它逼进脱困 (两个条件必须同时成立, 最容易白等的地方):
       条件 1: 重规划连续失败 fail_threshold(5) 次;
       条件 2: 机器人自身净空 < stuck_clearance(0.25 m)。
     ⚠ 条件 2 的净空是相对原始致命栅格算的 (膨胀层不计入), 所以
       "被膨胀层包住、规划失败"不等于"卡死" —— 那种情况判为"前方路障", 只绕路不脱困。
     想稳定复现: 用 RViz 的 "2D Goal Pose" 把目标设在死角/墙后, 把车引进退无可退的位置。
  e) 想先看链路通不通, 把"更容易判卡死"调出来:
       ⚠ **必须改 yaml 后重启导航栈, 不能用 ros2 param set。**
       MincoMpcController 只在 configure 期 (loadParameters) 读一次参数, 且**没有注册
       onSetParameters 回调** (参数只在 configure 期读一次)。
       对 controller_server 用 ros2 param set 会"成功返回但毫无效果"—— 这是最费时间的一种坑。
       做法: 改 config/simulation/nav2_params_srm_minco.yaml 里
         FollowPath.recovery.fail_threshold: 2      # 更容易进脱困
         FollowPath.recovery.stuck_clearance: 0.60   # 不必贴到 0.25 m 才算卡死
       然后 ./script/kill_nav.sh && ./script/start_sim_nav.sh --controller minco
       (本脚本默认启动前就会自己杀一遍导航栈, 直接重跑也行。)
       看到链路通了再把这两个值改回 5 / 0.25 复验一遍 —— 正式值才是交付状态。
  f) 两条脱困链路的分工 (不是互相打架, 是先后兜底):
       控制器内部脱困 (recovery.window_sec=3 s, 上面那些日志) 先试;
       3 s 没成功 -> 回重规划; 仍失败 -> 等 build_grace_period(1 s) 抛给行为树
       -> ClearEntireCostmap x2 + BackUpFreeSpace (现在也走 ESDF 梯度选方向),
       后者日志在 behavior_server 终端, 关键字 "backing up ... towards free space".
EOF
fi

fi  # if [ "${BANNER_DONE:-0}" = "0" ] —— --verify-spin 不打印交互提示

# =============================================================
# 一步验证小陀螺导航（--verify-spin）
#
# 启动后自动：等栈就绪 -> 上位机发自转请求 -> 用 ComputePathToPose 探测可达目标 ->
# 下发 NavigateToPose -> 断言「wz 归导航 / fake->body 逐条透传 / 边转边走 / 到点成功 /
# 撤回即停」-> 清理。退出码: 0 = 全通过, 1 = 有用例失败, 3 = 启动超时。
# 只验证不清理: VERIFY_KEEP=1 ./script/start_sim_nav.sh --verify-spin
# =============================================================
if [ "$VERIFY_SPIN" = "1" ] && [ "$DRY_RUN" = "1" ]; then
  echo "[dry-run] --verify-spin 需要真实启动 Stack，dry-run 下跳过断言。"
  VERIFY_SPIN=0
fi
if [ "$VERIFY_SPIN" = "1" ]; then
  vlog() { printf '%s\n' "$*"; }

  topic_has_data() {
    local topic="$1" window="${2:-2}"
    timeout "$((window + 4))" ros2 topic echo --once "$topic" >/dev/null 2>&1
  }

  action_ready() {
    timeout 10 ros2 action list 2>/dev/null | grep -q "/${ROBOT_NS}/navigate_to_pose"
  }

  wait_ready() {
    local what="$1" deadline
    shift
    deadline=$(( $(date +%s) + VERIFY_READY_TIMEOUT ))
    while [ "$(date +%s)" -lt "$deadline" ]; do
      if "$@" >/dev/null 2>&1; then
        vlog "      ✓ $what"
        return 0
      fi
      sleep 3
    done
    vlog "      ✗ 等待超时（${VERIFY_READY_TIMEOUT}s）：$what"
    return 3
  }

  verify_cleanup() {
    vlog "[清理] 撤回自转请求 + 结束导航栈与仿真..."
    timeout 5 ros2 topic pub --once "/${ROBOT_NS}/rotation_cmd" geometry_msgs/msg/Twist \
      "{angular: {z: 0.0}}" >/dev/null 2>&1 || true
    timeout 40 "$SCRIPT_DIR/kill_nav.sh" >/dev/null 2>&1 || true
    timeout 40 "$SCRIPT_DIR/kill_gzb.sh" >/dev/null 2>&1 || true
  }

  echo
  echo "[验证] 等待栈就绪（最长 ${VERIFY_READY_TIMEOUT}s）..."
  if ! wait_ready "/clock 在发布（Gazebo 在跑）" topic_has_data /clock 2 \
     || ! wait_ready "cmd_vel_sim 在发布（mux 在跑）" topic_has_data "/${ROBOT_NS}/cmd_vel_sim" 2 \
     || ! wait_ready "navigate_to_pose action 可用（Nav2 已激活）" action_ready; then
    verify_cleanup
    exit 3
  fi

  echo "[验证] 发自转请求 + 下发导航目标，开始断言（最长 ${VERIFY_GOAL_TIMEOUT}s）..."
  set +e
  python3 - "$ROBOT_NS" "$VERIFY_SPIN_WZ" "$VERIFY_GOAL_DIST" "$VERIFY_GOAL_TIMEOUT" <<'PYCHECK'
import math
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node

NS, SPIN_WZ, GOAL_DIST, GOAL_TIMEOUT = (
    sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
)


def topic(name):
    return f"/{NS}/{name}"


failures = []


def check(ok, message, detail=""):
    tail = "  " + detail if detail else ""
    print(("[PASS] " if ok else "[FAIL] ") + "  " + message + tail)
    if not ok:
        failures.append(message)


def note(message):
    print("  [观察] " + message)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class Sampler(Node):
    """采样速度链三层 + 里程计（只在导航目标激活期间采）。"""

    def __init__(self):
        super().__init__("verify_spin_nav")
        self.lock = threading.Lock()
        self.samples = {"fake": [], "body": [], "sim": [], "odom_wz": []}
        self.odom_pose = None
        for name, attr in (
            ("cmd_vel_nav_fake", "fake"),
            ("cmd_vel_nav_body", "body"),
            ("cmd_vel_sim", "sim"),
        ):
            self.create_subscription(
                Twist, topic(name), lambda msg, a=attr: self._on_twist(a, msg), 20
            )
        self.create_subscription(Odometry, topic("odometry"), self._on_odom, 20)

    def reset_window(self):
        with self.lock:
            self.samples = {"fake": [], "body": [], "sim": [], "odom_wz": []}

    def _on_twist(self, attr, msg):
        with self.lock:
            self.samples[attr].append((msg.linear.x, msg.linear.y, msg.angular.z))

    def _on_odom(self, msg):
        with self.lock:
            self.samples["odom_wz"].append(msg.twist.twist.angular.z)
            p = msg.pose.pose
            self.odom_pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def mean(self, key):
        with self.lock:
            data = list(self.samples[key])
        if not data:
            return None
        if key == "odom_wz":
            return sum(data) / len(data)
        return tuple(sum(v[i] for v in data) / len(data) for i in range(3))

    def median_nonzero(self, key):
        """非零样本的中位数（带符号）。

        均值会被"目标刚发出去、控制器还没开始输出"的零值样本稀释，短行程尤其明显；
        中位数对这类空档不敏感，用来判断"导航是否持续输出该量级"。
        """
        with self.lock:
            data = list(self.samples[key])
        if not data:
            return None
        if key == "odom_wz":
            values = [v for v in data if abs(v) > 0.05]
        else:
            values = [v[2] for v in data if abs(v[2]) > 0.05]
        if not values:
            return 0.0
        values.sort()
        mid = len(values) // 2
        return values[mid] if len(values) % 2 else 0.5 * (values[mid - 1] + values[mid])

    def counts(self):
        with self.lock:
            return {k: len(v) for k, v in self.samples.items()}

    def pose(self):
        with self.lock:
            return self.odom_pose


rclpy.init()
node = Sampler()
spin_pub = node.create_publisher(Twist, topic("rotation_cmd"), 10)
stop_spin = threading.Event()


def spin_loop():
    msg = Twist()
    msg.angular.z = SPIN_WZ
    while not stop_spin.is_set():
        spin_pub.publish(msg)
        time.sleep(0.05)


print("  提示: 控制器只在有导航目标时发布 cmd_vel_*（wz 归导航），")
print("        所以先探测可达目标、发目标，再在目标激活期间采速度链断言。")
threading.Thread(target=spin_loop, daemon=True).start()
time.sleep(0.5)

pose = node.pose()
deadline = time.time() + 3.0
while pose is None and time.time() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
    pose = node.pose()
if pose is None:
    check(False, "能读到里程计位姿", "没有 /odometry")
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(1)

x0, y0, yaw0 = pose
print("  起点 (map=odom 系): (%.2f, %.2f, yaw %+.2f)" % (x0, y0, yaw0))

client = ActionClient(node, NavigateToPose, topic("navigate_to_pose"))
if not client.wait_for_server(timeout_sec=15.0):
    check(False, "NavigateToPose action server 可用", "15 s 内没等到")
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(1)


def path_reachable(gx_try, gy_try):
    """用 ComputePathToPose 探测可达性：拿到非空路径才算可走。

    直接发 NavigateToPose 再"看车有没有动"不可靠 —— 规划失败时行为树会做
    BackUpFreeSpace 恢复（车会后退 ~0.4 m），看起来像"动了"。
    """
    planner = ActionClient(node, ComputePathToPose, topic("compute_path_to_pose"))
    if not planner.wait_for_server(timeout_sec=10.0):
        return None
    probe = ComputePathToPose.Goal()
    probe.goal.header.frame_id = "map"
    probe.goal.header.stamp = node.get_clock().now().to_msg()
    probe.goal.pose.position.x = gx_try
    probe.goal.pose.position.y = gy_try
    probe.goal.pose.orientation.w = 1.0
    probe.use_start = False
    future = planner.send_goal_async(probe)
    rclpy.spin_until_future_complete(node, future, timeout_sec=8.0)
    handle_probe = future.result() if future.done() else None
    if handle_probe is None or not handle_probe.accepted:
        return False
    result_future = handle_probe.get_result_async()
    t_end = time.time() + 10.0
    while time.time() < t_end and not result_future.done():
        rclpy.spin_once(node, timeout_sec=0.05)
    if not result_future.done():
        return False
    result = result_future.result()
    if result.status != 4:
        return False
    return len(result.result.path.poses) > 1


# 静态地图里"前方 N 米"未必可通行（如 227_1011 出生点前方 2.5 m 就有障碍），
# 所以按候选清单探测，用第一个真能规划的目标。
probe_plan = [(GOAL_DIST, 0.0), (1.5, 0.0), (2.5, 0.0), (0.8, 0.0), (0.8, 180.0),
              (1.5, 180.0), (0.8, 90.0), (0.8, -90.0), (1.2, 45.0), (1.2, -45.0)]
candidates = []
for dist_try, deg in probe_plan:
    ang = yaw0 + math.radians(deg)
    candidates.append((x0 + dist_try * math.cos(ang), y0 + dist_try * math.sin(ang), dist_try))

gx = gy = 0.0
chosen_dist = 0.0
for cx, cy, cdist in candidates:
    ok_path = path_reachable(cx, cy)
    if ok_path is None:
        check(False, "compute_path_to_pose action 可用", "10 s 内没等到 planner")
        break
    print("    探测目标 (%.2f, %.2f) 距离 %.2f m -> %s"
          % (cx, cy, cdist, "可通行" if ok_path else "不可达"))
    if ok_path:
        gx, gy, chosen_dist = cx, cy, cdist
        break

check(chosen_dist > 0.0, "该地图/出生点至少有一个可达目标", "试了 %d 个候选" % len(candidates))

handle = None
status = None
yaw_travel = 0.0
last_yaw = yaw0
min_dist = chosen_dist if chosen_dist > 0.0 else GOAL_DIST
motion_seconds = 0.0
if chosen_dist > 0.0:
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = "map"
    goal.pose.header.stamp = node.get_clock().now().to_msg()
    goal.pose.pose.position.x = gx
    goal.pose.pose.position.y = gy
    goal.pose.pose.orientation.w = 1.0
    print("    下发 NavigateToPose -> (%.2f, %.2f)，距离 %.2f m" % (gx, gy, chosen_dist))
    node.reset_window()
    send_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future, timeout_sec=10.0)
    handle = send_future.result() if send_future.done() else None
    accepted = bool(handle is not None and handle.accepted)
    check(accepted, "导航目标被接受", "accepted=%s" % accepted)
    if accepted:
        result_future = handle.get_result_async()
        t_start = time.time()
        t_end = t_start + GOAL_TIMEOUT
        while time.time() < t_end and not result_future.done():
            rclpy.spin_once(node, timeout_sec=0.05)
            p = node.pose()
            if p is not None:
                dyaw = abs(wrap(p[2] - last_yaw))
                if dyaw < 1.0:
                    yaw_travel += dyaw
                last_yaw = p[2]
                min_dist = min(min_dist, math.hypot(gx - p[0], gy - p[1]))
        motion_seconds = time.time() - t_start
        if result_future.done():
            status = result_future.result().status

counts = node.counts()
fake = node.mean("fake")
body = node.mean("body")
sim = node.mean("sim")
odom_wz = node.mean("odom_wz")
final = node.pose()
dist = math.hypot(gx - final[0], gy - final[1]) if final else float("nan")
print("  采样条数: %s" % counts)
print("  终态: 距目标 %.2f m（过程中最近 %.2f m），累计转角 %.2fπ，用时 %.1f s，action status=%s"
      % (dist, min_dist, yaw_travel / math.pi, motion_seconds, status))

if not fake or not body or not sim:
    check(False, "速度链三层都有数据（导航在输出）", str(counts))
else:
    fwz = node.median_nonzero("fake")
    bwz = node.median_nonzero("body")
    swz = node.median_nonzero("sim")
    print("  wz 统计（非零样本中位数 / 全窗口均值）: fake %+.3f / %+.3f, "
          "body %+.3f / %+.3f, sim %+.3f / %+.3f"
          % (fwz, fake[2], bwz, body[2], swz, sim[2]))
    tol = max(0.30 * abs(SPIN_WZ), 0.25)
    check(abs(fwz - SPIN_WZ) <= tol,
          "导航输出 wz 跟随上位机自转请求（wz 归导航）",
          "fake_wz(中位)=%+.3f 请求=%+.3f" % (fwz, SPIN_WZ))
    check(abs(bwz - fwz) <= 0.10 * abs(SPIN_WZ) + 0.05,
          "fake -> body 的 wz 逐条透传（不叠加、不衰减）", "body_wz(中位)=%+.3f" % bwz)
    check(abs(swz - bwz) <= 0.10 * abs(SPIN_WZ) + 0.05,
          "mux 输出 wz = body（未触发 wz_max 限幅）", "sim_wz(中位)=%+.3f" % swz)
    check(abs(fake[0]) > 0.05 or abs(fake[1]) > 0.05,
          "平移也在动（不是纯原地自转）", "fake_v=(%+.3f, %+.3f)" % (fake[0], fake[1]))

odom_med = node.median_nonzero("odom_wz")
if odom_med is None:
    check(False, "里程计有角速度", "odom.twist.angular.z 没采到")
else:
    check(odom_med * SPIN_WZ > 0.0, "里程计实际转向与请求同号",
          "odom_wz(中位)=%+.3f" % odom_med)
    ratio = abs(odom_med) / abs(SPIN_WZ) if SPIN_WZ else 0.0
    note("底盘自转执行比例 ≈ %.2f×（仿真底盘不可靠，仅记录）" % ratio)

check(status == 4, "导航到点成功（action SUCCEEDED）", "status=%s" % status)
check(dist < max(0.6, 0.45 * chosen_dist) if chosen_dist > 0 else False,
      "终点位置误差在容差内", "%.2f m" % dist)
expected_yaw = abs(SPIN_WZ) * max(motion_seconds, 0.1)
check(yaw_travel > max(0.15 * expected_yaw, 0.30),
      "导航途中持续自转（位姿 yaw 持续累计）",
      "%.2f rad vs 请求推算 %.2f rad" % (yaw_travel, expected_yaw))

stop_spin.set()
time.sleep(0.6)
node.reset_window()
deadline = time.time() + 2.0
while time.time() < deadline:
    rclpy.spin_once(node, timeout_sec=0.05)
tail = node.mean("odom_wz")
if tail is not None:
    check(abs(tail) < 0.3, "撤回请求后车停转（MINCO 0.25 s 超时）", "尾段 odom_wz=%+.3f" % tail)

print()
if failures:
    print("结论: FAIL（%d 项）" % len(failures))
    for item in failures:
        print("  - " + item)
else:
    print("结论: PASS（wz 归导航、链路逐条透传、边转边走、到点成功、撤回即停）")

node.destroy_node()
rclpy.shutdown()
sys.exit(1 if failures else 0)
PYCHECK
  CHECK_RC=$?
  set -e

  echo
  if [ "$CHECK_RC" -eq 0 ]; then
    echo "[验证] 结果: PASS ✅"
  else
    echo "[验证] 结果: FAIL ❌（看上面 [FAIL] 项与 .sim_run/ 日志）"
  fi
  if [ "$VERIFY_KEEP" = "1" ]; then
    echo "[验证] VERIFY_KEEP=1：仿真与导航栈继续运行；手动清理: ./script/kill_nav.sh && ./script/kill_gzb.sh"
  else
    verify_cleanup
  fi
  exit "$CHECK_RC"
fi

# ---------- direct 模式：脚本作为宿主托管全部子进程 ----------
if [ "$OPEN_MODE" = "direct" ] && [ "$DRY_RUN" != "1" ]; then
  echo "[direct] 宿主模式：本脚本保持运行以托管子进程；结束仿真 = 结束本脚本（或其进程组）。"
  wait
fi
