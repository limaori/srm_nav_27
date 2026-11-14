#!/usr/bin/env bash
# =============================================================
# SRM 仿真 NAV 导航 (一键启动)
#
#   标签页 1: SRM Gazebo 仿真 (世界 + SRM 圆柱底盘 + 雷达 + 速度执行)
#   标签页 2: 速度出口 + Nav2 导航栈 + RViz
#
# 速度链路 (2026-11-12 仿真云台系, 与实车链对齐):
#
#   MINCO/Nav2(gimbal_yaw_fake, 只出 vx/vy) ─cmd_vel_nav_fake─► fake_vel_transform
#      ─cmd_vel_nav_body─► srm_cmd_mux ◄─ rotation_cmd(决策层底盘 wz)
#                             └─cmd_vel_sim─► 速度适配器 ─► Gazebo
#                                (插件按云台关节角把 vx/vy 旋到底盘执行)
#
#   **wz 与导航完全解耦**: 导航只出平移 (yaw_policy.mode=xy_only)。底盘真实 yaw
#   由决策层发 rotation_cmd, 在 srm_cmd_mux 里合成 —— 不经过 MINCO, 也不经过平滑器。
#   小陀螺用 --chassis-spin [rad/s] (缺省 2.0): 脚本自动把请求发到 rotation_cmd,
#   由 mux 消费; 停发 0.25 s 即停转。也可以直接 scripts/manual/spin_control.sh on <rad/s>。
#   导航的"车"是云台: odom->gimbal_yaw 只依赖固定外参, TF 树反过来
#   (URDF gimbal_is_root:=true, 关节轴反号以匹配 Gazebo /joint_states)。
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
#        odom->gimbal_yaw 由 simulation_ground_truth_odometry 给出;
#        不启动 slam_toolbox / Point-LIO / small_gicp_relocalization。
#   如果需要建图, 请另起 launch (slam:=True), 不要与本脚本同时运行,
#   否则多个节点会争抢 map->odom。
#
#   初始重定位 (GICP):
#   ./scripts/bringup/start_sim_nav.sh                                    # 默认: Gazebo 真值里程计 (纯控制链路调试)
#   ./scripts/bringup/start_sim_nav.sh --lio                              # 纯 LIO 里程计, 默认前端 = ZLIO
#   ./scripts/bringup/start_sim_nav.sh --lio --lio-frontend point_lio \
#       -w rmuc_2025 -m rmuc_2025                                # 回退: 前端换回 Point-LIO (A/B 对照)
#     --lio 与 --gicp 互斥: --lio 用 LIO 出 odom->gimbal_yaw, map->odom 保持静态;
#     --gicp 用 LIO + 先验点云重定位 (需要 PCD)。
#     --gicp 会把定位方式切成 use_pcd_localization:=True + use_lio_odometry:=False:
#       * 先验 PCD 由 --prior-pcd 给出, 可由 --prior-pcd-manifest 自动推出初值;
#       * odom->gimbal_yaw 交给 sensor_scan_generation (launch 会覆盖参数文件里的
#         publish_tf: False), 真值里程计适配器不再启动;
#       * 本脚本不再发静态 map->odom, map->odom 只由 small_gicp_relocalization 发布。
#     这样地图原点与先验点云原点一致时, 机器人出生即"已知位置"; 若已知先验点云相对
#     出生点有偏移, 用 --init-pose 直接给 map->odom 初值 (方案 §5 的 T_M_O_init)。
#
# 用法:
#   ./scripts/bringup/start_sim_nav.sh                                  # rmuc_2025 + 隧道地图(默认)
#   ./scripts/bringup/start_sim_nav.sh -m rmuc_2025                     # 换成普通场地地图
#   ./scripts/bringup/start_sim_nav.sh -w srm_empty                     # 空场 (Gazebo 起来自动开始运行)
#   ./scripts/bringup/start_sim_nav.sh                                  # 默认: 云台系导航, 底盘与云台都不转
#   ./scripts/bringup/start_sim_nav.sh --chassis-spin                   # 底盘小陀螺 2.0 rad/s (mux 消费)
#   ./scripts/bringup/start_sim_nav.sh --gimbal-spin                    # 云台自转 -1.0 rad/s (底盘不转)
#   ./scripts/bringup/start_sim_nav.sh --chassis-spin --gimbal-spin     # 两个一起: 底盘 2.0 + 云台 -1.0
#   ./scripts/bringup/start_sim_nav.sh --chassis-spin 3.0 --gimbal-spin 1.5   # 自己给转速
#   ./scripts/bringup/start_sim_nav.sh --yaw-mode xy_only                # 显式导航只出平移 (默认)
#   ./scripts/bringup/start_sim_nav.sh --verify-spin                     # 一步验证小陀螺导航 (无头 + 自动断言 + 清理)
#   ./scripts/bringup/start_sim_nav.sh --no-run                          # Gazebo 保持暂停, 手动点播放
#   ./scripts/bringup/start_sim_nav.sh --no-rviz                         # 只跑仿真 + 导航
#   ./scripts/bringup/start_sim_nav.sh --gicp --prior-pcd /abs/prior.pcd \
#       --map /abs/prior.yaml                                   # 用 GICP 初始重定位
#   ./scripts/bringup/start_sim_nav.sh --gicp --prior-pcd /abs/prior.pcd \
#       --init-pose "0,0,0,0,0,0.5236"                          # 手动给 map->odom 初值
#   DRY_RUN=1 ./scripts/bringup/start_sim_nav.sh                         # 只打印将执行的命令
#
# 参数:
#   -w, --world   <name>      rmuc_2025 / rmuc_2024 / rmul_2024 / rmul_2025 / srm_empty
#   -m, --map     <名字|绝对路径>  地图名(自动补 .yaml)或 YAML 绝对路径
#   -p, --params  <绝对路径>  Nav2 参数文件; 不给时固定用
#                             config/simulation/nav2_params_srm_fakeyaw.yaml
#                             (MincoMpcController + xy_only + gimbal_yaw_fake, 限幅由
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
#       --lio / --no-lio         是否用 Point-LIO 出里程计 (**默认关闭** = Gazebo 真值)。
#                                LIO 需要场景有几何结构: 空场(srm_empty)里 x/y 不可观, 会跑飞。
#                                最接近实车: odom->base_link 由 sensor_scan_generation 从 LIO 反推,
#                                点云会被 LIO 去畸变。与 --gicp 互斥。
#       --lio-frontend <point_lio|zlio>
#                                --lio 时由哪个前端产生里程计。**默认 zlio**(2026-11-05 切换);
#                                point_lio = 回退到迁移前的前端(方案 §9 第 1 级回退)。
#                                没构建 frontend 包时默认值会自动退回 point_lio 并告警。
#                                zlio = z-localization 的 frontend 包 (ZLIO):
#                                  * 第 ① 层换成 ZLIO, 第 ② 层(坐标系改写)由 zlio_bridge
#                                    承担, loam_interface 不再启动;
#                                  * 需要先构建 frontend 与 localization_interfaces:
#                                    colcon build --symlink-install \
#                                      --packages-select localization_interfaces frontend
#                                  * 方案文档: docs/仿真LIO前端替换Z-LIO实施方案(ai).md
#                                  * 需要 --lio; --gicp 暂不接入 ZLIO。
#                                  环境变量: LIO_FRONTEND
#       --gicp / --no-gicp       是否用 GICP 初始重定位 (默认关闭 = 原来的静态地图模式)
#       --prior-pcd <绝对路径>   先验点云 PCD (--gicp 时必填)
#       --prior-pcd-manifest <绝对路径>
#                                先验点云清单 (重建工具生成的 *_manifest.yaml);
#                                给出时会按其中的 T_M_W 与 srm_sim.yaml 的出生点自动
#                                推出 init_pose, 并在两者不一致时告警
#       --init-pose "<x,y,z,roll,pitch,yaw>"
#                                map->odom 初值, 等价于 nav2 参数的 init_pose;
#                                给了就以它为准 (不再从 manifest 推导)
#       --yaw-mode <auto|xy_only|spin|se2>
#                                   MINCO 航向策略 (云台系默认 xy_only = 导航只出平移)。
#                                   auto/xy_only -> xy_only; spin 按 xy_only 跑（保留参数面）;
#                                   se2 直接拒绝（导航的 yaw 是云台、wz 转的是底盘，不成立）。
#                                   底盘自转请用 --chassis-spin 或决策层的 rotation_cmd。
#       --chassis-spin [rad/s]      底盘小陀螺: 给了就以 rad/s 恒速自转, 不给数值时
#                                   **默认 2.0 rad/s**(负值=反向)。请求走
#                                   rotation_cmd -> srm_cmd_mux（spin_topic）, **不进 MINCO**,
#                                   与导航完全解耦: 没有导航目标时底盘也会转。
#                                   **不加这个参数 = 底盘不转**(默认行为)。
#                                   脚本在导航栈就绪后自动调 scripts/manual/spin_control.sh on；
#                                   停转: scripts/manual/spin_control.sh off（停发 0.25 s 即停）。
#       --no-chassis-spin           等价于 --chassis-spin 0 (显式关掉, 覆盖环境变量)。
#       --gimbal-spin [rad/s]       云台自转角速度: 给了就让云台连同其上的雷达恒速自转,
#                                   不给数值时**默认 -1.0 rad/s**(负值=反向)。
#                                   **不加这个参数 = 云台不转**(旧版"默认 2.0 一直转"已取消)。
#                                   由仿真器 SrmVelocitySystem 直接施加, 不经 ROS 话题。
#                                   也可用环境变量 GIMBAL_SPIN_WZ 覆盖。
#       --no-gimbal-spin            等价于 --gimbal-spin 0。
#       (--wz-source / --fakeyaw / --no-fakeyaw 已删除:
#        2026-11-12 云台系后 wz 归决策层, 导航只出平移)
#       --verify-spin       一步验证小陀螺导航: 强制无头 (--no-rviz --no-gui, direct 模式),
#                           启动后自动发自转请求 + 导航目标, 断言
#                           "导航只出平移 / wz 走 mux / 边转边走 / 到点成功 / 撤回即停",
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
#   USE_VELOCITY_SMOOTHER YAW_MODE CHASSIS_SPIN_WZ GIMBAL_SPIN_WZ
#   CHASSIS_SPIN_READY_TIMEOUT(120)
#   VERIFY_SPIN_WZ VERIFY_GOAL_DIST VERIFY_KEEP VERIFY_READY_TIMEOUT VERIFY_GOAL_TIMEOUT
#   USE_LIO_ODOMETRY(0/1) LIO_FRONTEND(point_lio|zlio)
#   OPEN_MODE(tab|window|direct) TERMINAL ROBOT_NS USE_COMPOSITION DRY_RUN SRM27_WS_DIR PREKILL
#
# 说明:
#   - 启动前会自动依次跑 scripts/bringup/kill_nav.sh、scripts/bringup/kill_gzb.sh、scripts/bringup/kill_rviz.sh,
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
SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
WS_DIR="${SRM27_WS_DIR:-$(cd "$SCRIPT_DIR/../.." && pwd)}"
DATA_DIR="${SRM27_DATA_DIR:-$HOME/srm_nav_27_data}"
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
#: MINCO 航向策略: 云台系默认 xy_only（导航只出平移，底盘 wz 走 mux）；
#: spin 仅保留参数面（按 xy_only 跑）；se2 在云台系下直接拒绝。
YAW_MODE="${YAW_MODE:-auto}"
YAW_MODE_CLI_GIVEN="${YAW_MODE_CLI_GIVEN:-0}"  # 1 = 命令行显式给过 --yaw-mode

# 底盘自转 (小陀螺) 角速度 rad/s：留空 = 不自转；--chassis-spin 不带数值时用 2.0。
# 请求走 rotation_cmd -> srm_cmd_mux（spin_topic）-> cmd_vel_sim，
# 与手动的 scripts/manual/spin_control.sh 完全同一条链路（同一话题、同一 PID 文件）。
CHASSIS_SPIN_WZ="${CHASSIS_SPIN_WZ:-}"
if [ -n "$CHASSIS_SPIN_WZ" ]; then CHASSIS_SPIN_GIVEN=1; fi
CHASSIS_SPIN_GIVEN="${CHASSIS_SPIN_GIVEN:-0}"  # 1 = 用户显式指定过（含环境变量）
CHASSIS_SPIN_DEFAULT="2.0"                     # --chassis-spin 不带数值时的转速
# Nav2 就绪后再下发自转请求，等多久（秒）。等不到也照常启动发布器，只是先告警。
CHASSIS_SPIN_READY_TIMEOUT="${CHASSIS_SPIN_READY_TIMEOUT:-120}"

# 云台自转角速度 (rad/s)：留空 = 不自转；--gimbal-spin 不带数值时用 -1.0（负值 = 反向）。
# 由仿真器 SrmVelocitySystem 直接施加，不经 ROS 话题/桥接。
GIMBAL_SPIN_WZ="${GIMBAL_SPIN_WZ:-}"
if [ -n "$GIMBAL_SPIN_WZ" ]; then GIMBAL_SPIN_GIVEN=1; fi
GIMBAL_SPIN_GIVEN="${GIMBAL_SPIN_GIVEN:-0}"    # 1 = 用户显式指定过（含环境变量）
GIMBAL_SPIN_DEFAULT="-1.0"                     # --gimbal-spin 不带数值时的转速

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
# 默认 Gazebo 真值里程计；要用 Point-LIO 出里程计(最接近实车链路)显式加 --lio / USE_LIO_ODOMETRY=1。
USE_LIO_ODOMETRY="${USE_LIO_ODOMETRY:-0}"    # 1 = use_lio_odometry:=True（Point-LIO 出里程计）
LIO_CLI_GIVEN="${LIO_CLI_GIVEN:-0}"          # 1 = 命令行显式给过 --lio/--no-lio
# --lio 用哪个前端（方案 §6.2 / §8.0）：**默认 zlio**（2026-11-05 切换），point_lio = 回退。
# 回退有三级，随时可用：
#   1) 运行期：--lio-frontend point_lio
#   2) 构建期：不构建 frontend/localization_interfaces —— 本脚本会自动退回 point_lio 并告警
#   3) 代码期：把下面这行的缺省改回 point_lio（四个 launch 的默认值同理）
LIO_FRONTEND="${LIO_FRONTEND:-zlio}"
LIO_FRONTEND_CLI_GIVEN="${LIO_FRONTEND_CLI_GIVEN:-0}"  # 1 = 命令行显式给过 --lio-frontend
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
# --chassis-spin / --gimbal-spin 是"可带数值"的开关: 下一个参数是数字就当转速,
# 否则用各自的默认值。这样 `--gimbal-spin --no-rviz` 不会被误吞成转速。
is_spin_rate() {
  case "${1:-}" in
    ''|*[!0-9.eE+-]*) return 1 ;;
  esac
  awk -v v="$1" 'BEGIN{ exit !(v == v + 0) }'
}

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
    --lio)        USE_LIO_ODOMETRY="1"; LIO_CLI_GIVEN=1; shift ;;
    --no-lio)     USE_LIO_ODOMETRY="0"; LIO_CLI_GIVEN=1; shift ;;
    --lio-frontend) LIO_FRONTEND="${2:?--lio-frontend 需要 point_lio|zlio}"
                    LIO_FRONTEND_CLI_GIVEN=1; shift 2 ;;
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
    --yaw-mode)   YAW_MODE="${2:?--yaw-mode 需要 auto|xy_only|spin|se2}"; YAW_MODE_CLI_GIVEN=1; shift 2 ;;
    --chassis-spin)
                  CHASSIS_SPIN_GIVEN=1
                  if [ $# -ge 2 ] && is_spin_rate "$2"; then
                    CHASSIS_SPIN_WZ="$2"; shift 2
                  else
                    CHASSIS_SPIN_WZ="$CHASSIS_SPIN_DEFAULT"; shift
                  fi ;;
    --no-chassis-spin) CHASSIS_SPIN_WZ="0"; CHASSIS_SPIN_GIVEN=1; shift ;;
    --gimbal-spin)
                  GIMBAL_SPIN_GIVEN=1
                  if [ $# -ge 2 ] && is_spin_rate "$2"; then
                    GIMBAL_SPIN_WZ="$2"; shift 2
                  else
                    GIMBAL_SPIN_WZ="$GIMBAL_SPIN_DEFAULT"; shift
                  fi ;;
    --no-gimbal-spin) GIMBAL_SPIN_WZ="0"; GIMBAL_SPIN_GIVEN=1; shift ;;
    --verify-spin) VERIFY_SPIN=1; shift ;;
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

# ---------- 航向策略（2026-11-12 仿真云台系）----------
# 导航**只出平移**（xy_only）：底盘 wz 由决策层发 rotation_cmd，在 srm_cmd_mux 里
# 合成，**完全和导航解耦**（与实车 start_real_nav.sh 同一套语义）。
#   auto / xy_only -> xy_only
#   spin           -> 按 xy_only 跑（保留参数面；自转请求由 mux 消费，不进 MINCO）
#   se2            -> 直接拒绝：云台系下导航的 yaw 是云台、wz 转的是底盘，没有意义
case "$YAW_MODE" in
  auto)   YAW_MODE="xy_only" ;;
  xy_only) ;;
  spin)
    echo "[提示] --yaw-mode spin：云台系下导航只出平移，已按 xy_only 跑。" >&2
    echo "       自转请求走 rotation_cmd -> srm_cmd_mux（不进 MINCO）。" >&2
    YAW_MODE="xy_only"
    ;;
  se2)
    echo "[错误] 云台系下不再支持 --yaw-mode se2：" >&2
    echo "       导航的 yaw 是云台，wz 转的是底盘，se2（导航用 wz 追切线）在语义上不成立。" >&2
    echo "[提示] 底盘自转请改由决策层发 rotation_cmd（mux 消费）：" >&2
    echo "         ./scripts/bringup/start_sim_nav.sh --chassis-spin 0.5" >&2
    echo "         scripts/manual/spin_control.sh on 0.5     # 运行中手动改转速" >&2
    exit 2
    ;;
  *) echo "[错误] --yaw-mode 只支持 auto / xy_only / spin / se2" >&2; exit 2 ;;
esac

# ---------- 底盘 / 云台自转（默认都不转）----------
# 约定（2026-11-05 起）：**不加参数 = 不转**。旧版本"云台默认 2.0 rad/s 一直转"已取消。
# 显式给参数但不带数值时用各自的默认转速：
#   --chassis-spin -> 2.0 rad/s（负值反向）；--gimbal-spin -> -1.0 rad/s（负值反向）。
if [ "$CHASSIS_SPIN_GIVEN" = "1" ]; then
  [ -n "$CHASSIS_SPIN_WZ" ] || CHASSIS_SPIN_WZ="$CHASSIS_SPIN_DEFAULT"
else
  CHASSIS_SPIN_WZ="0"
fi
if [ "$GIMBAL_SPIN_GIVEN" = "1" ]; then
  [ -n "$GIMBAL_SPIN_WZ" ] || GIMBAL_SPIN_WZ="$GIMBAL_SPIN_DEFAULT"
else
  GIMBAL_SPIN_WZ="0"
fi

check_spin_rate() {
  local label="$1" value="$2"
  case "$value" in
    ''|*[!0-9.eE+-]*) echo "[错误] ${label}必须是数字 (rad/s): $value" >&2; return 1 ;;
  esac
  if ! awk -v v="$value" 'BEGIN{ exit !(v == v + 0) }'; then
    echo "[错误] ${label}必须是数字 (rad/s): $value" >&2
    return 1
  fi
  return 0
}
check_spin_rate "底盘自转速度" "$CHASSIS_SPIN_WZ" || exit 2
check_spin_rate "云台自转速度" "$GIMBAL_SPIN_WZ" || exit 2

# 底盘自转**不需要**改航向策略：请求直接进 mux（spin_topic），与 MINCO 无关。
if [ "$CHASSIS_SPIN_GIVEN" = "1" ] && \
   ! awk -v v="$CHASSIS_SPIN_WZ" 'BEGIN{ exit (v == 0) ? 0 : 1 }'; then
  echo "[提示] 底盘自转 ${CHASSIS_SPIN_WZ} rad/s：rotation_cmd -> srm_cmd_mux（不进 MINCO）。"
fi

# --verify-spin 自己发 VERIFY_SPIN_WZ 的请求；若同时给了 --chassis-spin，用它的转速，
# 让"脚本说转多少"与"验证请求多少"是同一个数。
if [ "$VERIFY_SPIN" = "1" ] && [ "$CHASSIS_SPIN_GIVEN" = "1" ]; then
  VERIFY_SPIN_WZ="$CHASSIS_SPIN_WZ"
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

read_spawn() {
  python3 - "$SIM_CONFIG" "${1:-x}" <<'PYEOF'
import sys
import yaml
cfg = yaml.safe_load(open(sys.argv[1], "r", encoding="utf-8"))
print(cfg.get("robot", {}).get(sys.argv[2], ""))
PYEOF
}


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
if ! grep -q "gimbal_yaw_fake" "$PARAMS_FILE"; then
  echo "[错误] 参数文件不是云台系 fakeyaw 变体 (找不到 gimbal_yaw_fake): $PARAMS_FILE" >&2
  echo "[提示] 2026-11-12 起仿真导航参考系是 gimbal_yaw_fake; 用默认参数文件即可。" >&2
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
USE_LIO_ODOMETRY="$(norm_switch "${USE_LIO_ODOMETRY:-0}")"
# map->odom 的平移/航向（真值模式下恒为 0；LIO 模式下 = 出生点）
MAP2ODOM_X="${MAP2ODOM_X:-0}"; MAP2ODOM_Y="${MAP2ODOM_Y:-0}"; MAP2ODOM_YAW="${MAP2ODOM_YAW:-0}"
if [ "$USE_GICP" = "true" ] && [ "$USE_LIO_ODOMETRY" = "true" ] && [ "$LIO_CLI_GIVEN" = "1" ]; then
  echo "[错误] --gicp 与 --lio 互斥: --gicp 本身就用 LIO + 先验点云重定位。" >&2
  echo "       想要纯 LIO 就只给 --lio; 想要重定位就只给 --gicp --prior-pcd <pcd>。" >&2
  exit 2
fi
# 默认是 LIO，但 --gicp 自带 LIO 重定位：没有显式 --lio 时静默让位，不报错。
if [ "$USE_GICP" = "true" ] && [ "$LIO_CLI_GIVEN" != "1" ]; then
  USE_LIO_ODOMETRY="0"
fi

# ---------- 里程计前端（--lio-frontend） ----------
# 放在 --gicp 归一化之后：--gicp 会把 USE_LIO_ODOMETRY 静默改成 false，
# 那时再判 "zlio 需要 --lio" 才准确。
case "$LIO_FRONTEND" in
  point_lio|zlio) ;;
  *)
    echo "[错误] --lio-frontend 只支持 point_lio / zlio，当前是: $LIO_FRONTEND" >&2
    exit 2
    ;;
esac
# 非 LIO 链路（真值里程计 / --gicp）下"前端"没有意义：
#   * 显式要了 zlio 又没给 --lio，说明命令写错了 → 直接报错；
#   * 只是用了默认值 → 静默让位给 point_lio（与上面 --gicp 对 USE_LIO_ODOMETRY 的处理一致），
#     否则 `--gicp` 会因为"默认 zlio"而莫名报错。
if [ "$LIO_FRONTEND" = "zlio" ] && [ "$USE_LIO_ODOMETRY" != "true" ]; then
  if [ "$LIO_FRONTEND_CLI_GIVEN" = "1" ]; then
    echo "[错误] --lio-frontend zlio 需要 --lio: ZLIO 只在纯 LIO 链路启用。" >&2
    echo "       --gicp 暂不接入 ZLIO（方案 §10：small_gicp 那一侧只差放宽条件，尚未做）。" >&2
    echo "       用法: ./scripts/bringup/start_sim_nav.sh --lio" >&2
    exit 2
  fi
  LIO_FRONTEND="point_lio"
fi

# ZLIO 前端需要 frontend 包。默认值没构建时**自动退回** Point-LIO 并告警（方案 §9 第 2 级回退：
# 不装那两个包，其余照旧应该还能跑）；显式要求 zlio 时则直接失败，不让人以为在跑 ZLIO。
if [ "$LIO_FRONTEND" = "zlio" ] && [ "$DRY_RUN" != "1" ]; then
  if ! require_pkg_from_ws frontend >/dev/null 2>&1; then
    if [ "$LIO_FRONTEND_CLI_GIVEN" = "1" ]; then
      echo "[错误] --lio-frontend zlio 需要 frontend 包（z-localization 的 LIO 前端），当前未找到。" >&2
      echo "       构建方法: colcon build --symlink-install --packages-select localization_interfaces frontend" >&2
      echo "       或者用 Point-LIO 前端: --lio-frontend point_lio" >&2
      exit 1
    fi
    echo "[警告] 默认前端 zlio 需要 frontend 包，但当前工作空间里没有构建它 ——" >&2
    echo "       本次自动退回 Point-LIO（方案 §9 第 2 级回退）。要试 ZLIO 请先:" >&2
    echo "         colcon build --symlink-install --packages-select localization_interfaces frontend" >&2
    LIO_FRONTEND="point_lio"
  fi
fi

# ---------- 云台驱动方式：谁来让云台转 ----------
# system  = 仿真插件按 gimbal_spin_rate 直接驱动（真值里程计模式，起来就转）
# external = 插件不驱动，改由 ROS 话题 + gz JointController；LIO 模式下由
#            gimbal_spin_gate 等 Point-LIO 初始化稳定之后才发转速。
#
# 为什么 LIO 要 external：Point-LIO 把"雷达 + IMU 所在刚体"当机体系（本工程里
# 就是云台系，见 loam_interface 的 base_frame: gimbal_yaw；车体位姿由
# sensor_scan_generation 每帧查 TF 换到 base_link），所以云台**可以**转；
# 但 Point-LIO 初始化要求传感器静止几秒，一上电就 2 rad/s 会直接发散
# （实测：RViz 里车飞出去、Gazebo 里不动）。所以只在初始化稳定后才开始转。
GIMBAL_DRIVER="system"
if [ "$USE_LIO_ODOMETRY" = "true" ]; then
  GIMBAL_DRIVER="external"
  # LIO 下云台转太快会让 Point-LIO 的里程计抖（每帧转角随转速线性变大）：
  # 2 rad/s @10 Hz 时每帧 11.5°，实测会抖；1.0 rad/s 稳得多。这里不擅自改用户的
  # 数值（默认已是 -1.0），只提醒一句。
  if awk -v v="$GIMBAL_SPIN_WZ" 'BEGIN{ a = (v < 0) ? -v : v; exit !(a > 1.0) }'; then
    echo "[警告] LIO 模式下云台转速 ${GIMBAL_SPIN_WZ} rad/s 偏快（|wz| > 1.0）：" >&2
    echo "       Point-LIO 每帧转角变大，里程计容易抖。建议 --gimbal-spin 1.0 或更小。" >&2
  fi
  GIMBAL_SPIN_NOTE_LIO=1
fi
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
    echo "       scripts/mapping/pcd_to_mapframe.py 把建图产物处理成 GICP 要的坐标系后放进该目录。" >&2
    exit 1
  fi
  PRIOR_PCD_FILE="$(resolve_prior_pcd "$PRIOR_PCD_FILE")" || exit 1
  [ -z "$PRIOR_PCD_MANIFEST" ] && [ -f "${PRIOR_PCD_FILE%.pcd}_manifest.yaml" ] && \
    PRIOR_PCD_MANIFEST="${PRIOR_PCD_FILE%.pcd}_manifest.yaml"
  printf -v PRIOR_PCD_Q '%q' "$PRIOR_PCD_FILE"
  # odometry_frontend 必须**每条链路都显式传**：nav_srm_simulation_launch.py 的
  # 声明缺省是 zlio（2026-11-05 切换），而 nav2_stack_launch.py 校验
  # "zlio 必须配 use_lio_odometry:=True"。不传就会让 launch 拿到 zlio + lio=False
  # → 启动直接抛异常（现象：整条导航栈没起来，连带 RViz 也不出现）。
  # 这里传的是已经解析过的 LIO_FRONTEND（非 LIO 链路上面已退回 point_lio）。
  NAV_EXTRA_ARGS=" use_pcd_localization:=True use_lio_odometry:=False odometry_frontend:=${LIO_FRONTEND} prior_pcd_file:=${PRIOR_PCD_Q}"
  # 栅格图 (map_server 用) 与先验点云必须同原点, 但它不是定位输入;
  # 若同目录正好有一张同名栅格图, 提示一下, 不自动替换 (避免顺序耦合)。
  if [ -z "${MAP:-}" ] && [ -f "${PRIOR_PCD_FILE%.pcd}.yaml" ]; then
    echo "[提示] 未显式 -m/--map, 当前用默认地图; 若先验点云是同源重建的, 建议:"
    echo "       -m ${PRIOR_PCD_FILE%.pcd}.yaml"
  fi
else
  NAV_EXTRA_ARGS=" use_pcd_localization:=False"
  if [ "$USE_LIO_ODOMETRY" = "true" ]; then
    # 空场世界只有地面(平面)，点云在 x/y 上不可观：LIO 只能靠 IMU 推算，必然漂 → 会"跑飞"。
    if [ "$WORLD" = "srm_empty" ]; then
      echo "[警告] -w srm_empty 是空场世界(只有地面，没有任何墙面/障碍)。" >&2
      echo "       Point-LIO 在这种场景下 x/y 不可观，位姿只能靠 IMU 推算，很快就会漂，" >&2
      echo "       表现为导航'跑飞'、净空/轨迹校验失败、到点 ABORT。建议二选一：" >&2
      echo "         · 换有结构的世界(如 -w rmuc_2025 -m rmuc_2025)再跑 LIO；" >&2
      echo "         · 或加 --no-lio 用真值里程计做控制/决策验收。" >&2
      [ "${ALLOW_LIO_EMPTY_WORLD:-0}" = "1" ] || { echo "[退出] 如确认要这么跑，加 ALLOW_LIO_EMPTY_WORLD=1。" >&2; exit 2; }
    fi
    # 纯 LIO：odom->base_link 由 sensor_scan_generation 从 LIO 反推。
    # ⚠ LIO 的 odom 原点 = 出生点（不是地图原点），而 map->odom 在本链路是静态发布，
    #   所以必须把它设成**出生点在地图里的位姿**，否则机器人会被投影到离真实位置
    #   一个出生点远的地方（默认真值模式下出生点是 (3.4, 9.5)，错位 10 m 后
    #   净空/规划全废：实测 minimum_clearance=0.349 < 0.38 → validation_failed → ABORT）。
    # 地图是以**出生点为原点**建的（GT 模式 map≡odom 才能正常导航），而 LIO 的 odom
    # 原点同样是出生点 → 所以 map->odom 必须保持恒等，**不要**加出生点偏移。
    #
    # odometry_frontend 决定第 ① 层由谁产生：point_lio（旧行为）或 zlio。
    # 第 ② 层（坐标系改写）永远是"非 point_lio 就用 zlio_bridge"，由 launch 里的
    # 条件统一处理，脚本只负责把选择传下去（方案 §6.2）。
    NAV_EXTRA_ARGS="$NAV_EXTRA_ARGS use_lio_odometry:=True odometry_frontend:=${LIO_FRONTEND}"
  else
    # 同上：即使不用 LIO 也要显式给前端，否则 launch 会用到它自己的缺省 zlio，
    # 与 use_lio_odometry:=False 冲突 → 导航栈启动即失败（RViz 也跟着不出现）。
    NAV_EXTRA_ARGS="$NAV_EXTRA_ARGS use_lio_odometry:=False odometry_frontend:=${LIO_FRONTEND}"
  fi
fi

# ---------- GICP: 由建图清单推导 map->odom 初值 ----------
# init_pose 是 small_gicp_relocalization 的节点参数, 语义是 map->odom 初值,
# 与"机器人在 map 里的位姿"不是同一个量 (方案 §2)。
#   T_M_O_init = T_M_W · T_W_B(spawn)
# odom 与地图在出生点同原点, 因此建图起点 == 出生点 时上面结果就是恒等矩阵;
# 两者不同原点时, 这个式子正好给出让机器人出现在正确位置所需的初值。
if [ "$USE_GICP" = "true" ]; then
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

# 唯一链路: fakeyaw 迁移 + 云台系 (导航只出平移, wz 归决策层)，命令系 gimbal_yaw_fake。
USE_FAKEYAW_ARG="True"
COMMAND_FRAME_ARG="gimbal_yaw_fake"

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
# 跳过本步骤: PREKILL=0 ./scripts/bringup/start_sim_nav.sh
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
elif [ "$USE_LIO_ODOMETRY" = "true" ]; then
  if [ "$LIO_FRONTEND" = "zlio" ]; then
    printf '定位方式  : ZLIO 里程计 (slam:=False, use_pcd_localization:=False, use_lio_odometry:=True, odometry_frontend:=zlio)\n'
    printf '            前端 = frontend/zlio_node; 坐标系改写 + 换代锚定 = zlio_bridge\n'
    printf '            loam_interface 不启动; registered_scan/lidar_odometry 由 bridge 唯一发布\n'
    printf '算法配置  : %s\n' "$PKG_SRC/config/simulation/zlio_mid360.yaml"
  else
    printf '定位方式  : Point-LIO 里程计 (slam:=False, use_pcd_localization:=False, use_lio_odometry:=True, odometry_frontend:=point_lio)\n'
  fi
  printf '            odom->gimbal_yaw = sensor_scan_generation 从 LIO 反推; map->odom 静态\n'
  printf '            点云由 LIO 去畸变; 云台 yaw = LIO 传感器 yaw（固定外参, 不含关节角）\n'
  printf '对照源    : /%s/chassis_odometry_gt (Gazebo 真值, 仿真的"下位机 yaw")\n' "$ROBOT_NS"
else
  printf '定位方式  : 静态地图 + 仿真真值里程计 (slam:=False, use_pcd_localization:=False, use_lio_odometry:=False)\n'
  printf '            默认值; 需要 LIO 加 --lio (最接近实车, 但要有几何结构的世界)\n'
fi
printf 'RViz      : %s    Gazebo GUI: %s    直接运行: %s\n' "$USE_RVIZ" "$USE_GUI" "$RUN_IMMEDIATELY"
printf '平滑器    : %s\n' "$USE_VELOCITY_SMOOTHER"
printf 'wz 归属   : 决策层 rotation_cmd -> srm_cmd_mux（导航只出平移, 完全解耦）\n'
printf '速度链    : Nav2[cmd_vel_nav_fake] -> fake_vel_transform[cmd_vel_nav_body] -> srm_cmd_mux -> cmd_vel_sim -> Gazebo\n'
printf '            srm_cmd_mux 第二路: rotation_cmd (只取 angular.z, 停发 0.25 s 停转)\n'
printf 'TF owner  : odom->gimbal_yaw = 定位/真值里程计;  gimbal_yaw->gimbal_yaw_fake = fake_vel_transform\n'
printf '航向策略  : %s' "$YAW_MODE"
case "$YAW_MODE" in
  xy_only) echo '  (云台系: 导航只出平移, 底盘 wz 归决策层)' ;;
  *)       echo '' ;;
esac
# 底盘 / 云台自转：默认都是"关"，要转必须显式给参数。
if awk -v v="$CHASSIS_SPIN_WZ" 'BEGIN{ exit (v == 0) ? 0 : 1 }'; then
  printf '底盘自转  : 关 (默认; 要转加 --chassis-spin [rad/s], 缺省 2.0)\n'
else
  printf '底盘自转  : %s rad/s (rotation_cmd -> srm_cmd_mux; 脚本在导航栈就绪后自动下发)\n' "$CHASSIS_SPIN_WZ"
  printf '            与导航解耦: 没有导航目标时底盘也会转\n'
fi
if awk -v v="$GIMBAL_SPIN_WZ" 'BEGIN{ exit (v == 0) ? 0 : 1 }'; then
  printf '云台自转  : 关 (默认; 要转加 --gimbal-spin [rad/s], 缺省 -1.0)\n'
else
  if [ "$GIMBAL_DRIVER" = "external" ]; then
    printf '云台自转  : %s rad/s (LIO 模式：等 Point-LIO 稳定后由 gimbal_spin_gate 启动；此前锁定)\n' "$GIMBAL_SPIN_WZ"
  else
    printf '云台自转  : %s rad/s (由仿真器直接施加; 云台上的雷达随之转动)\n' "$GIMBAL_SPIN_WZ"
  fi
fi
if [ "$VERIFY_SPIN" = "1" ]; then
  printf '验证模式  : --verify-spin (自转 %s rad/s, 目标 %s m, 跑完%s)\n' \
      "$VERIFY_SPIN_WZ" "$VERIFY_GOAL_DIST" \
      "$([ "$VERIFY_KEEP" = "1" ] && echo '保留栈' || echo '自动清理')"
fi
printf '\n终端模式  : %s (%s)\n' "$OPEN_MODE" "$TERMINAL"
echo

prekill_leftovers

# 云台自转必须传给**这里**：Gazebo 是这条单独命令拉起的（导航 launch 的
# start_simulation 默认 False，不走那条路），漏了就永远是 0 = 云台锁死。
SIM_CMD="ros2 launch srm27_gazebo_simulator srm_sim.launch.py world:=${WORLD} gui:=${USE_GUI} run_immediately:=${RUN_IMMEDIATELY} gimbal_spin_rate:=${GIMBAL_SPIN_WZ} gimbal_driver:=${GIMBAL_DRIVER} velocity_command_frame:=gimbal odom_link:=gimbal_yaw"

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
NAV_CMD="ros2 launch srm27_nav_bringup nav_srm_simulation_launch.py namespace:=${ROBOT_NS} world:=${WORLD} map:=${MAP_FILE_Q} params_file:=${PARAMS_FILE_Q} slam:=False${NAV_EXTRA_ARGS} use_fakeyaw_transform:=${USE_FAKEYAW_ARG} command_frame:=${COMMAND_FRAME_ARG} use_composition:=${USE_COMPOSITION} use_sim_time:=True use_rviz:=${USE_RVIZ}${RVIZ_CONFIG_ARG} use_velocity_smoother:=${USE_VELOCITY_SMOOTHER} yaw_mode:=${YAW_MODE} gimbal_spin_rate:=${GIMBAL_SPIN_WZ}"
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

小陀螺 (wz 与导航解耦: rotation_cmd -> srm_cmd_mux, 不进 MINCO):
  启动时加 --chassis-spin [rad/s]      # 脚本自动下发 (缺省 2.0), 无需手动
  只在启动后手动改转速/停转:
  ./scripts/manual/spin_control.sh on 2.0      # 恒速自转 2.0 rad/s
  ./scripts/manual/spin_control.sh status      # 查看请求/实际转速
  ./scripts/manual/spin_control.sh off         # 撤回请求 (停发 0.25 s 即停转)

急停 (清零并保持零速):
  ros2 service call /${ROBOT_NS}/srm_cmd_mux/stop_all std_srvs/srv/Trigger
  ros2 service call /${ROBOT_NS}/srm_cmd_mux/resume_all std_srvs/srv/Trigger

清理残留:
  ./scripts/bringup/kill_gzb.sh && ./scripts/bringup/kill_rviz.sh
EOF

cat <<EOF

速度链路检查 (新终端):
  cd '$WS_DIR' && source install/setup.bash
  ros2 topic echo /${ROBOT_NS}/cmd_vel_nav_fake --once    # 导航请求 (fake 系, wz 应恒 0)
  ros2 topic echo /${ROBOT_NS}/cmd_vel_nav_body --once    # fake 转换后 (云台系平移, wz 仍 0)
  ros2 topic echo /${ROBOT_NS}/rotation_cmd --once         # 决策层小陀螺请求 (只取 angular.z)
  ros2 topic echo /${ROBOT_NS}/cmd_vel_sim --once         # mux 最终输出 (wz 来自 rotation_cmd)
  ros2 topic hz /${ROBOT_NS}/cmd_vel_sim                  # 应为 200 Hz 左右
  ros2 topic echo /${ROBOT_NS}/odometry --once            # 真值里程计 (child = gimbal_yaw)
  ros2 run tf2_ros tf2_echo gimbal_yaw gimbal_yaw_fake    # fakeyaw TF: yaw 应等于 -云台 yaw
  ros2 run tf2_ros tf2_echo map gimbal_yaw                # 完整 TF 链
  ros2 run tf2_ros tf2_echo gimbal_yaw base_link          # 云台->底盘 (关节角, 只给 RViz)
  ros2 topic echo /${ROBOT_NS}/diagnostics --once         # mux: 双输入/wz 来源计数
  ros2 topic echo /${ROBOT_NS}/fake_vel_transform/diagnostics --once  # yaw 外推/过期计数

记录要求 (方案 §6): 必须区分"导航请求 wz"(cmd_vel_nav_fake.angular.z, 应恒 0)、
"决策层请求 wz"(rotation_cmd.angular.z)、"mux 最终 wz"(cmd_vel_sim.angular.z)、
"里程计实际 wz"(odometry.twist.angular.z), 不能只看 Gazebo 模型是否转动。

P0 自检 (不需要 Gazebo): 用 ros2 topic echo 确认
  cmd_vel_nav_fake.angular.z 恒 0、rotation_cmd / cmd_vel_sim 的 angular.z
  与 odometry.twist.angular.z 一致。
EOF

# 只有真的跑 MINCO 才打印这段自检: 换控制器最容易出的错是"以为换了, 其实参数文件没选对"。
if [ "$CONTROLLER" = "minco" ]; then
  cat <<EOF

MINCO 自检 (仿真, 按顺序确认):
  1) 插件真的被加载 (不是"launch 没报错"就算):
       ros2 param get /${ROBOT_NS}/controller_server FollowPath.plugin
       期望: srm27_minco_controller::MincoMpcController
       若显示 Omni, 说明参数文件选错了 (--controller minco 会自动选对文件; 但若旧进程还在,
       也会看到旧值 —— 那种情况先 ./scripts/bringup/kill_nav.sh)。
  2) 轨迹真的算出来了:
       ros2 topic hz /${ROBOT_NS}/FollowPath/minco_trajectory
       ros2 topic hz /${ROBOT_NS}/FollowPath/mpc_prediction
  3) 诊断健康 (判"卡在哪"的第一手证据, 失败路径也会发布):
       ros2 topic echo /${ROBOT_NS}/FollowPath/diagnostics --once
       关注: planning_result=success、minimum_clearance>0、missed_deadline_count 不持续增长。
  4) 导航只出平移 (xy_only): /${ROBOT_NS}/cmd_vel_nav_fake 的 angular.z 应恒为 0,
     cmd_vel_nav_body 也恒为 0; 自转请求被 MINCO 拒绝是预期
     (诊断里 rejected_spin_request_count 增长)。
     底盘 wz 只看 cmd_vel_sim 的 angular.z (来自 rotation_cmd)。

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
       ros2 topic echo /${ROBOT_NS}/FollowPath/diagnostics | grep -E "recovery_(active|trigger|exit|clearance|command_count|repeat_suppressed)"
     正常导航 recovery_active=false; 进脱困为 true 且 recovery_trigger=gradient|circular_search;
     退出时 recovery_exit=escaped|timeout。
     两条"看得懂"的辅助判据:
       * recovery_command_count 非 0 才算真的推了车 (日志里的"输出 N 个周期"已修正,
         2026-11-03 之前那行永远打 0, 是因为 leaveRecovery() 先把计数清零再打印);
       * recovery_repeat_suppressed_count 增长而 recovery_enter_count 不增长 = 车被夹在
         "推得动一点但推不出困境"的窄带里, 脱困对它没用 —— 该查通道宽度/机器人半径,
         而不是继续调脱困参数。
  d) 怎么把它逼进脱困 (两个条件必须同时成立, 最容易白等的地方):
       条件 1: 重规划连续失败 fail_threshold(10) 次;
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
       然后 ./scripts/bringup/kill_nav.sh && ./scripts/bringup/start_sim_nav.sh --controller minco
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
# 底盘自转（小陀螺）自动下发 —— 只有给了 --chassis-spin 才走这里
#
# 用法契约与手动的 scripts/manual/spin_control.sh 完全一致（同一话题、同一 PID 文件）：
#
#   rotation_cmd --20 Hz--> srm_cmd_mux(spin_topic, 只取 angular.z) --> cmd_vel_sim --> Gazebo
#
# 2026-11-12 起 wz 与导航**完全解耦**：
#   * 请求不进 MINCO（xy_only 会把 spin_request 拒掉并计数），由 mux 直接合成；
#   * 所以**没有导航目标时底盘也照样转**（与实车决策层小陀螺一致）；
#   * 停发 0.25 s（mux 的 spin_timeout）即视为停转。
#
# 所以：
#   * 停转: ./scripts/manual/spin_control.sh off      查看: ./scripts/manual/spin_control.sh status
#   * 请求靠 20 Hz 持续发布保持"新鲜"，mux 超时即撤销自转；
# 没给 --chassis-spin 时这里什么都不做（默认不转）。
# =============================================================
if [ "${VERIFY_SPIN:-0}" = "1" ]; then
  : # --verify-spin 自己发 VERIFY_SPIN_WZ 的请求，这里不重复起发布器
elif awk -v v="$CHASSIS_SPIN_WZ" 'BEGIN{ exit (v == 0) ? 0 : 1 }'; then
  : # 底盘不转（未给参数或显式 --no-chassis-spin）
else
  CHASSIS_SPIN_TOPIC="/${ROBOT_NS}/rotation_cmd"
  CHASSIS_SPIN_ABS="$(awk -v v="$CHASSIS_SPIN_WZ" 'BEGIN{ printf "%.3f", (v < 0) ? -v : v }')"
  # 本地钳位（spin_control.sh 的 SPIN_MAX_WZ）按本次请求放行，否则 --chassis-spin 3
  # 会被静默默认钳回 2.0；真正的上限仍由 mux 的 wz_max 兜底。
  if awk -v v="$CHASSIS_SPIN_ABS" 'BEGIN{ exit !(v > 6.0) }'; then
    echo "[提示] 请求 ${CHASSIS_SPIN_WZ} rad/s 超过 mux 的 wz_max（speed_limits.yaml 当前 6.0），实际执行会被限幅。" >&2
  fi
  if [ "$DRY_RUN" = "1" ]; then
    echo "--- [dry-run] 底盘自转: 等 ${CHASSIS_SPIN_TOPIC} 有订阅者(srm_cmd_mux)后执行"
    echo "SRM27_WS_DIR='$WS_DIR' SPIN_MAX_WZ='$CHASSIS_SPIN_ABS' '$SCRIPT_DIR/../manual/spin_control.sh' on '$CHASSIS_SPIN_WZ'"
  else
    if [ "$RUN_IMMEDIATELY" != "true" ]; then
      # Gazebo 是暂停起的 (--no-run)：仿真时钟不推进，Nav2 永远不会 configure，
      # 等下去只会白耗 CHASSIS_SPIN_READY_TIMEOUT 秒。直接起发布器，点播放后自然生效。
      echo "[3/3] 底盘自转: Gazebo 以暂停启动(--no-run)，不等 Nav2，直接启动发布器；点播放后请求会被消费。"
    else
      echo "[3/3] 底盘自转: 等 Nav2 就绪（${CHASSIS_SPIN_TOPIC} 有订阅者，最长 ${CHASSIS_SPIN_READY_TIMEOUT}s）..."
      _spin_deadline=$(( $(date +%s) + CHASSIS_SPIN_READY_TIMEOUT ))
      _spin_subs=""
      while [ "$(date +%s)" -lt "$_spin_deadline" ]; do
        _spin_subs="$(bash -c "source '$WS_DIR/install/setup.bash' >/dev/null 2>&1; timeout 6 ros2 topic info '$CHASSIS_SPIN_TOPIC' 2>/dev/null" 2>/dev/null \
          | sed -n 's/^Subscription count: //p' | head -n 1)" || true
        if [ -n "$_spin_subs" ] && [ "$_spin_subs" != "0" ]; then
          break
        fi
        sleep 2
      done
      if [ -z "$_spin_subs" ] || [ "$_spin_subs" = "0" ]; then
        echo "[警告] ${CHASSIS_SPIN_READY_TIMEOUT}s 内没等到 ${CHASSIS_SPIN_TOPIC} 的订阅者（srm_cmd_mux 还没起来？）" >&2
        echo "       仍会启动发布器：等导航栈上线后请求会自动被消费。" >&2
      fi
    fi
    if SRM27_WS_DIR="$WS_DIR" SPIN_MAX_WZ="$CHASSIS_SPIN_ABS" \
       "$SCRIPT_DIR/../manual/spin_control.sh" on "$CHASSIS_SPIN_WZ"; then
      printf '底盘自转  : %s rad/s 已下发 (20 Hz)；停转: %s/spin_control.sh off\n' \
        "$CHASSIS_SPIN_WZ" "$SCRIPT_DIR"
    else
      echo "[警告] spin_control.sh on 失败，底盘不会自转。" >&2
    fi
  fi
fi

# =============================================================
# 一步验证小陀螺导航（--verify-spin）
#
# 启动后自动：等栈就绪 -> 上位机发自转请求 -> 用 ComputePathToPose 探测可达目标 ->
# 下发 NavigateToPose -> 断言「导航只出平移(fake/body wz=0) / mux 的 wz 跟随请求 /
# 边转边走 / 到点成功 / 撤回即停」-> 清理。退出码: 0 = 全通过, 1 = 有用例失败, 3 = 启动超时。
# 只验证不清理: VERIFY_KEEP=1 ./scripts/bringup/start_sim_nav.sh --verify-spin
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
  python3 - "$ROBOT_NS" "$VERIFY_SPIN_WZ" "$VERIFY_GOAL_DIST" "$VERIFY_GOAL_TIMEOUT" \
  "$MAP2ODOM_X" "$MAP2ODOM_Y" "$MAP2ODOM_YAW" <<'PYCHECK'
import math
import sys
import threading
import time

import rclpy
import tf2_ros
from geometry_msgs.msg import Twist
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node

NS, SPIN_WZ, GOAL_DIST, GOAL_TIMEOUT = (
    sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
)
# 真值模式恒为 0；LIO 模式下 odom 原点在出生点，所以 map 位姿 = odom 位姿 + 该偏移。
MAP2ODOM_X, MAP2ODOM_Y, MAP2ODOM_YAW = (
    float(sys.argv[5]), float(sys.argv[6]), float(sys.argv[7])
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
        # LIO 模式下 odom 原点在出生点、map->odom 不为恒等，所以发目标前必须把
        # odom 位姿换算到 map 系，否则目标会落在离车一个出生点远的地方（全不可达）。
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
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


print("  提示: wz 与导航解耦（xy_only）：自转请求由 srm_cmd_mux 消费，导航只出平移。")
print("        先探测可达目标、发目标，再在目标激活期间采速度链断言。")
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
# 换到 map 系（目标必须用 map 系下发）；LIO 模式下 map->odom 是出生点偏移。
cos_off, sin_off = math.cos(MAP2ODOM_YAW), math.sin(MAP2ODOM_YAW)
x_odom, y_odom = x0, y0
x0 = MAP2ODOM_X + cos_off * x_odom - sin_off * y_odom
y0 = MAP2ODOM_Y + sin_off * x_odom + cos_off * y_odom
yaw0 = wrap(yaw0 + MAP2ODOM_YAW)
print("  起点 (map 系): (%.2f, %.2f, yaw %+.2f)   [map->odom 偏移 x=%.2f y=%.2f yaw=%+.2f]"
      % (x0, y0, yaw0, MAP2ODOM_X, MAP2ODOM_Y, MAP2ODOM_YAW))

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
    check(abs(fwz) <= 0.15,
          "导航只出平移（fake 的 wz 恒 0，wz 已与导航解耦）",
          "fake_wz(中位)=%+.3f" % fwz)
    check(abs(bwz) <= 0.15,
          "fake -> body 不引入 wz（body 的 wz 也恒 0）", "body_wz(中位)=%+.3f" % bwz)
    check(abs(swz - SPIN_WZ) <= tol,
          "mux 输出的 wz 跟随决策层自转请求（rotation_cmd -> mux）",
          "sim_wz(中位)=%+.3f 请求=%+.3f" % (swz, SPIN_WZ))
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
    check(abs(tail) < 0.3, "撤回请求后车停转（mux spin_timeout）", "尾段 odom_wz=%+.3f" % tail)

print()
if failures:
    print("结论: FAIL（%d 项）" % len(failures))
    for item in failures:
        print("  - " + item)
else:
    print("结论: PASS（导航只出平移、wz 走 mux、边转边走、到点成功、撤回即停）")

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
    echo "[验证] VERIFY_KEEP=1：仿真与导航栈继续运行；手动清理: ./scripts/bringup/kill_nav.sh && ./scripts/bringup/kill_gzb.sh"
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
