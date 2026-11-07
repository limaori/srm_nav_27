#!/usr/bin/env bash
# =============================================================
# SRM 实车 NAV 导航测试 (一键启动)
#
#   步骤 1: Livox Mid-360 雷达驱动
#   步骤 2: SRM 车体 TF (robot_state_publisher: base_link / livox_* 等 frame)
#   步骤 3: 底盘串口 srm27_nav_protocol (/cmd_vel_chassis 的唯一消费者)
#   步骤 4: 导航栈 Nav2 (定位方式见下)
#   步骤 5: RViz
#   步骤 6: 手柄遥控 (可选, 默认关闭)
#
# 定位方式 (四者互斥, 区别只在 map->odom 由谁发布):
#   --lio     (默认) Point-LIO 只提供里程计, map->odom 用静态 TF —— 实车推荐
#   --reloc   small_gicp 与先验 PCD 做 GICP 配准, 由它发布 map->odom (需要 --prior-pcd)
#   --slam    slam_toolbox 边跑边建图, 由它发布 map->odom (不需要先验地图)
#   --static  只加载先验栅格图 + 静态 map->odom, 没有任何里程计来源
#             (仅当车体模块自己发 odom->base_link 时可用; 直接用本脚本起车不会动)
#
# 实车速度链路（2026-11-06 起与仿真同构；见 docs/仿真链迁移实车实施记录(ai).md）:
#
#   MINCO 迁移链（默认）:
#     controller_server ─cmd_vel_controller─► velocity_smoother ─cmd_vel_nav_fake─►
#       fake_vel_transform ─cmd_vel_nav_body─► srm_cmd_mux ─cmd_vel_chassis─►
#         srm27_nav_protocol ─► 串口 ─► C 板
#
#   Omni 旧链（-c omni）:
#     controller_server ─► velocity_smoother ─cmd_vel_nav2_result─►
#       fake_vel_transform（legacy）─cmd_vel_chassis─► srm27_nav_protocol ─► 串口
#
# 迁移链的四个要点（与仿真的 `start_sim_nav.sh --chassis-spin` 完全一致）:
#   1) 导航参考系 = base_link_fake（与 odom 平行、朝向稳定），代价地图/行为树也用它；
#   2) MINCO 的 command_frame = base_link_fake：平移的 R(-yaw) **只**在 fake_vel_transform
#      里做一次。旧实车链路关掉 fake 的理由（"重复旋转"、"依赖 local_plan 同步"）是针对
#      legacy 模式 + command_frame=base_link 的，nav_se2 链上不成立；
#   3) wz 由导航拥有（yaw_policy.mode=se2；`--yaw-mode spin` 改成小陀螺），
#      mux 是 cmd_vel_chassis 的唯一发布者，并带 stop_all/resume_all 急停服务；
#   4) 速度限幅仍只有一处：config/real/speed_limits.yaml（`@fakeyaw` 段负责放开 smoother
#      的 yaw 界，`srm_cmd_mux` 段给 mux 的 wz_max —— 首跑 1.5 rad/s，验过再抬）。
#
# ⚠ 首跑必读（wz 与云台）
#   * C 板固件现在**执行 wz**（用户确认，2026-11-06）。但"符号/增益/死区"只有转起来才知道：
#     首次务必架起车辆（轮子离地）或留人守物理急停，用 `--chassis-spin 0.3` 这种小速度先看方向。
#   * 云台自转（仿真里的 `--gimbal-spin`）**实车暂不支持**：实车云台由下位机驱动，而当前协议
#     （上→下 12 字节 vx/vy/wz、下→上只有比赛状态）**不回传云台关节角**。雷达在云台上，
#     没有关节角时 `base_link` yaw 会差一个云台角 → 导航看到的车"自己在转"。
#     所以本脚本拒绝 `--gimbal-spin`，只保留"协议补齐回传后用 --gimbal-joint-topic 接进来"的接口。
#
# 速度限幅 (控制器 / 平滑器 / mux / 恢复行为) 的**唯一来源**:
#   src/srm27_navigation/srm27_nav_bringup/config/real/speed_limits.yaml
#   launch 启动时把它合并进 nav2_params_srm*.yaml 与 srm_chassis_control.yaml
#   (参数文件里不再写这些键; 两处都写且值不同 -> 启动直接报错)。
#   串口限幅仍在 srm27_nav_protocol 自己的 config 里 (而且限幅作用在 linear_velocity_scale
#   缩放**之后**)。
#   改档位只改那一份; 改完先跑自检看"卡住速度的是哪一层", 再按下面的预检清单复核:
#     python3 src/srm27_navigation/srm27_nav_bringup/scripts/srm_speed_limits_check.py
# 详见 docs/导航速度调试指南.md §1/§2。
#
# ⚠ 安全须知 (务必先读):
#   1) srm27_nav_protocol 的发送线程按 send_rate_hz (默认 100 Hz) 重发"最近一次"收到的速度,
#      并带 0.5 s 看门狗 (cmd_timeout_sec): cmd_vel_chassis 超过 0.5 s 没更新就把控制量归零。
#      **但看门狗只在节点活着的时候有效**: 进程被 kill -9 / 主机崩溃 / 串口掉线时它没机会跑,
#      而 C 板固件自己**没有超时保护**, 会一直用最后一帧的速度。所以任何"停节点"操作前
#      先发零速 (本脚本的 --stop 会先发), 物理急停按钮始终是第一手段。
#   2) srm27_nav_protocol 没有任何急停/使能 service, 软件层面唯一的停车手段是在
#      /cmd_vel_chassis 上发零速度 Twist。实车急停只能靠物理按钮或断电。
#   3) 底盘能动之后, 先在 RViz 用很小的目标点试一次, 确认车头方向/正负号都对。
#
# 用法:
#   ./script/start_real_nav.sh                       # 默认 --reloc + MINCO 迁移链, 启动全部
#   ./script/start_real_nav.sh --controller minco    # MINCO + MPC (默认, fakeyaw 迁移链)
#   ./script/start_real_nav.sh --chassis-spin        # 小陀螺 2.0 rad/s (首跑建议给 0.3)
#   ./script/start_real_nav.sh --chassis-spin --gimbal-spin   # 与仿真同一条命令（云台参数只告警）
#   ./script/start_real_nav.sh --yaw-mode xy_only    # A/B 对照: 导航不控自转
#   ./script/start_real_nav.sh --no-mux              # 不经 mux (出口 = cmd_vel_nav_body, 需自行重映射)
#   ./script/start_real_nav.sh --lio-frontend zlio   # 定位前端换 ZLIO (实车参数在 config/real)
#   ./script/start_real_nav.sh -m xjl0914            # 指定地图 (maps/ 下的名字)
#   ./script/start_real_nav.sh --prior-pcd /abs/map.pcd      # 指定重定位先验点云
#   ./script/start_real_nav.sh --no-reloc                    # 不用重定位: LIO + 静态 map->odom, 不需要先验 PCD
#   ./script/start_real_nav.sh -c omni                       # 换回 Omni 旧链 (fake_vel_transform legacy)
#   ./script/start_real_nav.sh --slam                # 边建图边导航
#   ./script/start_real_nav.sh --map-to-odom 0 0 0   # 指定起步位姿 (map 系, 弧度)
#   ./script/start_real_nav.sh --no-rviz --no-joy    # 只起链路, 不开图形界面
#   ./script/start_real_nav.sh --list-maps           # 列出可用地图与先验 PCD
#   ./script/start_real_nav.sh -n                    # 只解析并打印将执行的命令, 不启动
#   ./script/start_real_nav.sh --stop                # 先发零速, 再结束实车链路节点
#
# 参数（参数面与 script/start_sim_nav.sh **对齐**：同名的参数两边同义、同一条命令两边都能跑）:
#
#   ── 与 start_sim_nav.sh 同名同义 ─────────────────────────────────────────────
#   -m, --map <名字|绝对路径>   地图名(在 maps/ 下解析)或 YAML 绝对路径; 缺省自动选择
#       --prior-pcd <名字|路径> 重定位先验点云, 默认与地图同名
#   -p, --params <绝对路径>     Nav2 参数文件（实车默认 = 迁移链那份, 见 -c）
#   -c, --controller <auto|minco>   控制器（实车另有 omni 可选, 见下）
#       --minco                 等价于 --controller minco
#       --gicp / --no-gicp      先验点云重定位（= --reloc）/ 关掉它（= --no-reloc, LIO+静态图）
#       --lio / --no-lio        LIO 出里程计 + 静态 map->odom / 关掉 LIO（实车=--static）
#       --lio-frontend <point_lio|zlio>   里程计前端（**默认 zlio**；point_lio = 回退）
#       --init-pose "<x,y,z,roll,pitch,yaw>"   map->odom 初值（实车只取 x/y/yaw）
#       --prior-pcd-manifest <路径>   仿真专用, 实车只提示忽略
#       --yaw-mode <auto|se2|spin>    MINCO 航向策略（实车另有 xy_only 对照档）
#       --chassis-spin [rad/s] / --no-chassis-spin   底盘小陀螺（默认 2.0 rad/s）
#       --gimbal-spin [rad/s] / --no-gimbal-spin
#                               **实车不执行**：云台由下位机驱动、上位机无指令通道;
#                               参数被接受只为命令行一致（详见下方"云台"说明）。
#       --rviz / --no-rviz, --rviz-config <路径>, --smoother / --no-smoother
#       --gui / --no-gui, --run / --no-run, -w / --world <名字>
#                               Gazebo 专用; 实车接受并忽略（会给一行提示）
#       --verify-spin           仿真专用的自主验证; 实车接受但**拒绝执行**（会自主驱动车辆），
#                               并打印人工分步验收清单
#
#   ── 实车专用 ───────────────────────────────────────────────────────────────
#       --map-to-odom X Y YAW   静态 map->odom 初值 (m, m, rad), 即"车现在停的位置
#                               在地图坐标系里的位姿"; 默认 0 0 0 (车在 map 原点)
#       --slam / --reloc / --static   其余定位方式（与 --lio/--gicp 同一组互斥开关）
#       --no-reloc              等价于 --lio
#       --gimbal-joint-topic <话题>   云台关节角回传话题（中继成 /joint_states）
#       --mux / --no-mux        是否串 srm_cmd_mux（默认串）
#       --speed-limits <路径>   速度限幅覆盖层（默认 config/real/speed_limits.yaml）
#       --lidar-xyz / --lidar-rpy / --lidar-config / --namespace
#       --robot-state-pub / --no-robot-state-pub, --chassis / --no-chassis, --joy / --no-joy
#       --fake-vel-transform / --no-fake-vel-transform, --composition / --no-composition
#       --log-level <级别>, -n/--check, --list-maps, --stop
#   -c, --controller <auto|omni|minco>   控制器（实车扩展）:
#                                minco = config/real/nav2_params_srm_minco_fakeyaw.yaml
#                                        (**迁移链**: base_link_fake + fakeyaw + mux)
#                                omni  = config/real/nav2_params_srm.yaml (旧链, 无 mux)
#                                auto  = 读参数文件里的 FollowPath.plugin 自动判断;
#                                        **只给 -p 而没给 -c 时也走这条路**。
#                              选 minco 时自动走 nav_srm_real_launch.py（迁移链 + mux）、
#                              把导航出口对准 cmd_vel_nav_fake，并跑一轮 MINCO 预检。
#       --minco / --omni        等价于 --controller minco / --controller omni
#       --yaw-mode <auto|se2|spin|xy_only>
#                               MINCO 航向策略 (默认 auto=se2, 导航拥有 wz):
#                                se2 / spin / xy_only 含义同上; 仿真只到 spin。
#                               只给 --yaw-mode spin 车**还不会转**, 要转加 --chassis-spin。
#       --chassis-spin [rad/s]  底盘小陀螺（默认 2.0, 负值反向）:
#                               ⚠ 首跑请架起车辆或用 0.3 这种小转速先确认方向；
#                               mux 的 wz_max 首跑限在 1.5 rad/s。
#       --gimbal-joint-topic <话题>
#                               云台关节角回传话题 (std_msgs/Float32 或 JointState)。
#                               gimbal_joint_state_relay 常驻 (TF 树需要)，给出话题时走
#                               float32 模式转成 /joint_states；默认 gimbal_joint_angle
#                               （srm27_nav_protocol 由 0x0010 第 4 个字段换算而来，
#                               见 docs/导航通信协议-下位机接口说明.md）。
#                               留空 = constant 暂位 0 度，此时底盘相对云台转 = base_link yaw 必错。
#       --no-gimbal-joint-topic  等价于 --gimbal-joint-topic ""（退回暂位 0 度）。
#       --zlio-config <路径>     ZLIO 算法配置, 默认 config/real/zlio_mid360.yaml
#                               (--lio-frontend zlio 与 --reloc/--slam 互斥)
#       --mux / --no-mux         是否串 srm_cmd_mux (默认串)。关掉时导航出口变成
#                               cmd_vel_nav_body, 而串口节点订阅 cmd_vel_chassis —— 车不会动。
#       --slam / --reloc / --lio / --static   定位方式, 四者互斥 (默认 --reloc)
#       --no-reloc              等价于 --lio: LIO 里程计 + 静态 map->odom, 不需要先验 PCD。
#       --lidar-xyz "X Y Z"     临时覆盖雷达安装位置, 米 (默认空 = 用 URDF 模型里的值)
#       --lidar-rpy "R P Y"     临时覆盖雷达安装角, 弧度 (默认空 = 用 URDF 模型里的值)
#       --lidar-config <路径>   雷达驱动网络配置, 默认 config/real/mid360_user_config.json
#       --namespace <ns>        命名空间 (默认空 = 根命名空间; 环境变量 ROBOT_NS 亦可)
#   -n, --check                 只解析并打印将执行的命令 (等同 DRY_RUN=1)
#       --list-maps             列出可选地图与先验 PCD
#       --stop                  发零速后结束本链路进程 (不起新节点)
#   -h, --help                  显示本帮助
#
# 环境变量 (同名参数均可由环境变量给默认值):
#   MAP MAP_NAME PRIOR_PCD_FILE PARAMS_FILE CONTROLLER LIDAR_CONFIG_FILE NAMESPACE
#   START_SLAM START_RELOC START_LIO START_STATIC NO_RELOC
#   USE_RVIZ USE_ROBOT_STATE_PUB USE_CHASSIS USE_JOY USE_COMPOSITION
#   USE_VELOCITY_SMOOTHER USE_FAKE_VEL_TRANSFORM USE_CHASSIS_MUX
#   YAW_MODE CHASSIS_SPIN_WZ CHASSIS_SPIN_READY_TIMEOUT GIMBAL_JOINT_TOPIC
#   LIO_FRONTEND ZLIO_CONFIG
#   LIDAR_XYZ LIDAR_RPY MAP_TO_ODOM_X MAP_TO_ODOM_Y MAP_TO_ODOM_YAW LOG_LEVEL
#   OPEN_MODE(tab|window) TERMINAL LOG_LEVEL DRY_RUN SRM27_WS_DIR PREKILL
#
# 说明:
#   - 启动前会自动先跑一遍 script/kill_gzb.sh 和 script/kill_rviz.sh, 清掉上次
#     遗留的 Gazebo / RViz (不清会抢 /clock、抢 TF、抢 RViz 窗口)。清理失败只告警
#     不阻断; 想完全跳过这一步用 PREKILL=0。
#   - MINCO（迁移链）走 launch/nav_srm_real_launch.py：它在 nav2_stack 之上多做了三件事
#     —— 把 yaw_policy/判停器按 --yaw-mode 注入、起 srm_cmd_mux（出口 cmd_vel_chassis）、
#     启动时核对整条速度链。Omni 旧链仍走 nav2_stack_launch.py。
#     不使用 nav_real_launch.py：它没有透传 use_pcd_localization / use_lio_odometry /
#     use_fake_vel_transform，--reloc / --lio / 迁移链都会失效。
#     MINCO 实车首次上电的分阶段验收清单见 docs/minco实车迁移实施记录(ai).md，
#     迁移链（小陀螺/云台缺口）见 docs/仿真链迁移实车实施记录(ai).md。
#   - 雷达驱动单独启动: 驱动代码里的节点名是 livox_driver_node, 而参数文件顶层键是
#     livox_ros_driver2, ROS 2 按节点名匹配 --params-file, 对不上整段参数被忽略,
#     因此这里显式 -r __node:=livox_ros_driver2; 参数文件里的 user_config_path 用的是
#     launch 专有的 $(find-pkg-share ...) 语法, ros2 run 不会展开, 故再用
#     -p user_config_path:=<绝对路径> 覆盖一次。
#   - map / prior_pcd_file / params_file 一律传绝对路径。
#   - 先验点云按以下顺序找 (--prior-pcd 给名字时; 给绝对路径则直接用):
#       maps/<名字>_mapframe.pcd            已转地图系 (旧布局, 仍兼容)
#       maps/pcd/<名字>_mapframe.pcd        已转地图系 (现行产品: build_pcd_from_bag.sh
#                                          从录包重建出 maps/pcd/<名字>.pcd, 再用
#                                          script/pcd_to_mapframe.py 转出 _mapframe 版)
#       maps/pcd/<名字>.pcd                 Point-LIO camera_init 系原始件 (退路, 会告警)
#       maps/<名字>.pcd
#       <包>/pcd/real/<名字>.pcd
#     **_mapframe 版必须排在原始件前面**: 两者同名只差后缀, 顺序反了就会静默挑到
#     camera_init 系那份, 车被定位在差 90° 的姿态上而 GICP 不报错。
#     原始件被选中时下面那条"头部未标注坐标系"的告警会响 (它头部没有 frame 注释)。
#   - 每个标签页都会先 source 工作空间的 install/setup.bash。
#   - 想建图请只用 --slam; 建图与导航不要各起一套, 否则多个节点争抢 map->odom。
# =============================================================

set -euo pipefail

# ---------- 路径 ----------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${SRM27_WS_DIR:-$(cd "$SCRIPT_DIR/.." && pwd)}"

PKG_SRC="$WS_DIR/src/srm27_navigation/srm27_nav_bringup"
PKG_INSTALL="$WS_DIR/install/srm27_nav_bringup/share/srm27_nav_bringup"
# 优先用安装后的 share 目录 (launch 运行时 get_package_share_directory 返回的就是它),
# 找不到再退回源码目录 (未编译时也能做 --check / --list-maps)。
pick_dir() {
  local d
  for d in "$@"; do
    if [ -d "$d" ]; then
      printf '%s' "$d"
      return 0
    fi
  done
  printf '%s' "$1"
}
PKG_DIR="$(pick_dir "$PKG_INSTALL" "$PKG_SRC")"
MAP_REAL_DIR="$PKG_DIR/map/real"
MAP_SIM_DIR="$PKG_DIR/map/simulation"
PCD_REAL_DIR="$PKG_DIR/pcd/real"
# 实车先验点云的**主要落地点**: 由 script/build_pcd_from_bag.sh 从录包重建, 再用
# script/pcd_to_mapframe.py 就地转成地图系 (每张图就这一份 <名字>.pcd, 名字即地图名)。
PCD_MAPS_DIR="$WS_DIR/maps/pcd"
SRC_MAP_REAL_DIR="$PKG_SRC/map/real"

# ---------- 可配置项 ----------
OPEN_MODE="${OPEN_MODE:-tab}"                # tab: 同一窗口多标签页; window: 每个命令独立窗口
TERMINAL="${TERMINAL:-gnome-terminal}"
NAMESPACE="${NAMESPACE:-}"
LOG_LEVEL="${LOG_LEVEL:-info}"
DRY_RUN="${DRY_RUN:-0}"

USE_RVIZ="${USE_RVIZ:-1}"
USE_ROBOT_STATE_PUB="${USE_ROBOT_STATE_PUB:-1}"
USE_CHASSIS="${USE_CHASSIS:-1}"
USE_JOY="${USE_JOY:-0}"
USE_COMPOSITION="${USE_COMPOSITION:-1}"
USE_VELOCITY_SMOOTHER="${USE_VELOCITY_SMOOTHER:-1}"
# fake_vel_transform 的默认值取决于控制器: Omni 沿用旧链路 (启用 legacy), MINCO 迁移链
# **必须**启用 nav_se2 (它就是链路的一部分)。这里先记住"环境变量有没有显式给过", 解析
# 参数文件之后再定默认值 —— 与定位方式的 MODE_GIVEN 是同一个思路。
if [ -n "${USE_FAKE_VEL_TRANSFORM:-}" ]; then
  USE_FAKE_VEL_TRANSFORM_GIVEN=1
else
  USE_FAKE_VEL_TRANSFORM=1
  USE_FAKE_VEL_TRANSFORM_GIVEN=0
fi
# 同上: 命令行是否显式给过 --fake-vel-transform / --no-fake-vel-transform。
FAKE_VEL_CLI_GIVEN=0

# 速度出口 mux (srm_cmd_mux): MINCO 迁移链默认串联, 它是 cmd_vel_chassis 的唯一发布者。
USE_CHASSIS_MUX="${USE_CHASSIS_MUX:-1}"
MUX_CLI_GIVEN=0

# MINCO 航向策略: auto=按链路推导 (迁移链 se2); spin=小陀螺; xy_only=导航不控自转。
YAW_MODE="${YAW_MODE:-auto}"
YAW_MODE_CLI_GIVEN=0
# 底盘小陀螺角速度 (rad/s): 留空 = 不自转; --chassis-spin 不带数值时用 2.0。
CHASSIS_SPIN_WZ="${CHASSIS_SPIN_WZ:-}"
CHASSIS_SPIN_DEFAULT="2.0"
CHASSIS_SPIN_CLI_GIVEN=0
if [ -n "${CHASSIS_SPIN_WZ}" ]; then CHASSIS_SPIN_CLI_GIVEN=1; fi
# Nav2 就绪后再下发自转请求, 等多久 (秒)。
CHASSIS_SPIN_READY_TIMEOUT="${CHASSIS_SPIN_READY_TIMEOUT:-120}"

# 云台自转: 与 start_sim_nav.sh 同名同义（便于同一条命令两边用）。
# ⚠ 实车**不执行**这个参数：云台由下位机驱动，上位机没有指令通道（见文件头）。
# 保留它是为了参数面一致；真正影响 TF 的是 --gimbal-joint-topic。
GIMBAL_SPIN_WZ="${GIMBAL_SPIN_WZ:-}"
GIMBAL_SPIN_DEFAULT="-1.0"
GIMBAL_SPIN_CLI_GIVEN="${GIMBAL_SPIN_CLI_GIVEN:-0}"
if [ -n "$GIMBAL_SPIN_WZ" ]; then GIMBAL_SPIN_CLI_GIVEN=1; fi

# 一步验证（仿真专用）：实车会自主驱动车辆，因此只接受参数、不执行（给出人工步骤）。
VERIFY_SPIN="${VERIFY_SPIN:-0}"

# 先验点云清单（仿真用它推导 map->odom 初值；实车没有"出生点"概念，只提示忽略）。
PRIOR_PCD_MANIFEST="${PRIOR_PCD_MANIFEST:-}"
# --world 是仿真世界名；实车只接受并提示忽略（地图仍用 -m）。
WORLD_ARG="${WORLD:-}"
# 自定义 RViz 配置（与 start_sim_nav.sh 同名）。
RVIZ_CONFIG="${RVIZ_CONFIG:-}"

# 云台关节角回传话题：0x0010 第 4 个字段已改为 chassis_to_gimbal_yaw，
# srm27_nav_protocol 会据此发出 gimbal_joint_angle（云台相对底盘、弧度）。
# 默认直接接上 —— 不接的话 base_link 的 yaw 等于云台 yaw，底盘相对云台一转，
# 速度方向就错一个关节角（导航正常转向时关节角一直在变，不是边角情况）。
# 想退回"暂位 0 度"：--no-gimbal-joint-topic 或 GIMBAL_JOINT_TOPIC= 启动。
# ⚠ 数据断了 relay 会在 timeout_sec(0.3 s) 后停发 /joint_states，TF 停在最后一帧。
# 注意用 ${VAR-default} 而不是 ${VAR:-default}：后者会把"显式设为空"也当成未设置，
# 那样 GIMBAL_JOINT_TOPIC= 就没法关掉回传了。
GIMBAL_JOINT_TOPIC="${GIMBAL_JOINT_TOPIC-gimbal_joint_angle}"

# 里程计前端 (point_lio / zlio) 与 ZLIO 算法配置。
# **默认 zlio**（2026-11-08 切换，与 start_sim_nav.sh 一致）；point_lio = 回退选项。
LIO_FRONTEND="${LIO_FRONTEND:-zlio}"
LIO_FRONTEND_CLI_GIVEN=0
ZLIO_CONFIG="${ZLIO_CONFIG:-}"

# 局部控制器: auto (按参数文件内容判断) / omni / minco。
CONTROLLER="${CONTROLLER:-minco}"
CONTROLLER_CLI_GIVEN=0

START_SLAM="${START_SLAM:-0}"
START_RELOC="${START_RELOC:-0}"
START_LIO="${START_LIO:-0}"
START_STATIC="${START_STATIC:-0}"

# ---- 与 start_sim_nav.sh 共用的环境变量别名 ----
# 仿真脚本用 USE_GICP / USE_LIO_ODOMETRY / ROBOT_NS / INIT_POSE 这些名字。
# 这里做**别名**而不是改名：两侧脚本"同一条命令、同一批环境变量"都能跑。
# 注意这段在 is_true() 定义之前，所以只用 case 判断（不调函数）。
_alias_true() {
  case "$(printf '%s' "$1" | tr '[:upper:]' '[:lower:]')" in
    1|true|yes|on) return 0 ;;
    *)             return 1 ;;
  esac
}
if _alias_true "${USE_GICP:-0}"; then
  START_RELOC=1
fi
if _alias_true "${USE_LIO_ODOMETRY:-0}"; then
  START_LIO=1
fi
if [ -z "${NAMESPACE:-}" ] && [ -n "${ROBOT_NS:-}" ]; then
  NAMESPACE="$ROBOT_NS"
fi
if [ -n "${INIT_POSE:-}" ]; then
  FIRST_INIT_POSE="$INIT_POSE"     # 解析阶段统一处理（--init-pose 也写这里）
else
  FIRST_INIT_POSE=""
fi
unset -f _alias_true

# --no-reloc: 显式不用重定位 (此时不需要先验 PCD), 退回 LIO 里程计 + 静态 map->odom。
# 单独一个变量是为了区分"没提过重定位"和"明确关掉重定位"——后者要压过默认值 --reloc。
NO_RELOC="${NO_RELOC:-0}"
# --no-lio（与 sim 同名）：实车没有真值里程计，"关闭 LIO"等于没有里程计来源。
LIO_DISABLED="${LIO_DISABLED:-0}"
# 命令行是否显式给过定位方式。没给才在解析之后套用默认 --reloc
# (默认值不能在这里就写进 START_LIO, 否则 --slam 会叠成"两个模式同时开")。
MODE_GIVEN=0

MAP_NAME="${MAP_NAME:-${MAP:-}}"
PRIOR_PCD_FILE="${PRIOR_PCD_FILE:-}"
PARAMS_FILE="${PARAMS_FILE:-}"
# 参数文件是用户给的还是本脚本的默认值: -c 只在自己选默认文件时才去覆盖它。
if [ -n "$PARAMS_FILE" ]; then PARAMS_FILE_GIVEN=1; else PARAMS_FILE_GIVEN=0; fi
LIDAR_CONFIG_FILE="${LIDAR_CONFIG_FILE:-}"
# 速度限幅覆盖层（控制器/平滑器/mux/恢复行为的唯一来源）。
SPEED_LIMITS_FILE="${SPEED_LIMITS_FILE:-}"
MAP_YAML=""
PRIOR_PCD=""

# SRM 实车雷达安装位姿: **唯一来源是 URDF**。这里默认留空 = 不给 launch 传
# lidar_xyz/lidar_rpy, real_robot_state_publisher_launch.py 就用模型里的值,
# 与建图链路 (start_real_slam.sh) 完全一致, 换 URDF 不用改这两个脚本。
# 只有现场临时改安装角才显式传; 用 ${VAR-} 而不是 ${VAR:-} 才能保留"显式空串"。
LIDAR_XYZ="${LIDAR_XYZ-}"
LIDAR_RPY="${LIDAR_RPY-}"
# 起步位姿: map->odom 静态变换。仅 --lio / --static 模式下有意义,
# 默认 0 0 0 = 认为 Point-LIO 的起点就在地图原点。
# 若车停在别处, 必须改成"该点在 map 系下的位姿", 否则 RViz 里车不在真实位置,
# 全局代价地图也会错位。
MAP_TO_ODOM_X="${MAP_TO_ODOM_X:-0.0}"
MAP_TO_ODOM_Y="${MAP_TO_ODOM_Y:-0.0}"
MAP_TO_ODOM_YAW="${MAP_TO_ODOM_YAW:-0.0}"

ROBOT_STATE_PUB_PATTERN='[r]eal_robot_state_publisher_launch\.py'
LIVOX_PATTERN='[l]ivox_ros_driver2'
CHASSIS_PATTERN='[s]rm27_nav_protocol'
# 导航入口：迁移链走 nav_srm_real_launch.py，旧链走 nav2_stack_launch.py。
NAV_PATTERN='[n]av_(srm_real|2_stack)_launch\.py'
RVIZ_PATTERN='[r]viz_launch\.py'
JOY_PATTERN='[j]oy_teleop_launch\.py'

# ---------- 帮助 ----------
usage() {
  # 打印文件头注释块 (跳过 shebang 与首尾分隔线), 避免帮助文本与代码不同步。
  awk 'NR == 1 { next }
       /^# =+[[:space:]]*$/ { sep++; if (sep == 2) exit; next }
       { sub(/^# ?/, ""); print }' "${BASH_SOURCE[0]}"
}

# ---------- 小工具 ----------
is_true() {
  case "$(printf '%s' "$1" | tr '[:upper:]' '[:lower:]')" in
    1|true|yes|on) return 0 ;;
    *)             return 1 ;;
  esac
}

# 归一化为 launch 需要的 True/False
bool_arg() {
  if is_true "$1"; then printf 'True'; else printf 'False'; fi
}

q() { printf '%q' "$1"; }

process_running() {
  pgrep -u "$(id -u)" -f "$1" >/dev/null 2>&1
}

# 地图候选: maps/<名>/<名>.yaml 与 maps/<名>.yaml (现场采集的实车地图)
list_map_candidates() {
  local f
  for f in "$WS_DIR"/maps/*/*.yaml "$WS_DIR"/maps/*.yaml "$MAP_REAL_DIR"/*.yaml; do
    [ -f "$f" ] || continue
    # maps/<名字>/<名字>.yaml 这种布局只认"目录名与 yaml 同名"的那份,
    # 否则 rosbag 的 maps/<bag>/metadata.yaml 会被误当成地图。
    case "$f" in
      "$WS_DIR"/maps/*/*)
        [ "$(basename "$(dirname "$f")")" = "$(basename "$f" .yaml)" ] || continue
        ;;
    esac
    printf '%s\n' "$f"
  done
}

resolve_map() {
  local name="$1" cand
  if [ "${name#/}" != "$name" ]; then          # 绝对路径
    [ -f "$name" ] && { readlink -f "$name"; return 0; }
    return 1
  fi
  if [ "${name#*/}" != "$name" ]; then         # 相对路径不受支持
    return 1
  fi
  for cand in \
    "$WS_DIR/maps/$name/$name.yaml" \
    "$WS_DIR/maps/$name.yaml" \
    "$MAP_REAL_DIR/$name.yaml" \
    "$MAP_SIM_DIR/$name.yaml"; do
    [ -f "$cand" ] && { readlink -f "$cand"; return 0; }
  done
  return 1
}

resolve_prior_pcd() {
  local name="$1" cand
  if [ "${name#/}" != "$name" ]; then
    [ -f "$name" ] && { readlink -f "$name"; return 0; }
    return 1
  fi
  for cand in \
    "$WS_DIR/maps/${name}_mapframe.pcd" \
    "$PCD_MAPS_DIR/${name}_mapframe.pcd" \
    "$PCD_MAPS_DIR/$name.pcd" \
    "$WS_DIR/maps/$name.pcd" \
    "$PCD_REAL_DIR/$name.pcd"; do
    [ -f "$cand" ] && { readlink -f "$cand"; return 0; }
  done
  return 1
}

list_maps() {
  echo "可选地图 (名称去掉 .yaml 后作为 --map 的值):"
  echo
  local f name pcd seen="" found=0
  while IFS= read -r f; do
    [ -n "$f" ] || continue
    name="$(basename "$f" .yaml)"
    case " $seen " in *" $name "*) continue ;; esac
    seen="$seen $name"
    pcd="无"
    if [ -f "$WS_DIR/maps/${name}_mapframe.pcd" ] || [ -f "$PCD_MAPS_DIR/${name}_mapframe.pcd" ] \
       || [ -f "$PCD_MAPS_DIR/$name.pcd" ] \
       || [ -f "$WS_DIR/maps/$name.pcd" ] || [ -f "$PCD_REAL_DIR/$name.pcd" ]; then
      pcd="有"
    fi
    printf '  %-22s 先验PCD: %-3s %s\n' "$name" "$pcd" "$f"
    found=1
  done < <(list_map_candidates)
  if [ "$found" = "0" ]; then
    echo "  (未找到任何 .yaml 地图)"
  fi
  echo
  echo "地图目录  : $WS_DIR/maps  (支持 maps/<名字>.yaml 与 maps/<名字>/<名字>.yaml)"
  echo "          : $MAP_REAL_DIR"
  echo "先验PCD   : $PCD_MAPS_DIR/<名字>_mapframe.pcd   (已转地图系, --reloc 用这份)"
  echo "          : $PCD_MAPS_DIR/<名字>.pcd            (Point-LIO camera_init 系原始件)"
  echo "          : $WS_DIR/maps/<名字>_mapframe.pcd"
  echo "          : $PCD_REAL_DIR/<名字>.pcd"
  echo "注意: --reloc 需要先验 PCD, 且该 PCD 必须已经在地图坐标系下。"
}

# ---------- 停止 ----------
# 先发零速再停进程: srm27_nav_protocol 没有超时清零, 直接杀节点车会保持最后速度。
stop_real_nav() {
  echo "[停止] 先发零速到 cmd_vel_chassis (防止底盘保持最后速度)..."
  if command -v ros2 >/dev/null 2>&1 && [ -f "$WS_DIR/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    ( set +u; source "$WS_DIR/install/setup.bash" >/dev/null 2>&1
      timeout 2 ros2 topic pub -r 20 /cmd_vel_chassis geometry_msgs/msg/Twist "{}" >/dev/null 2>&1 ) || true
  else
    echo "[警告] 无法发送零速 (ros2 不可用或工作空间未编译), 请手动确认底盘已停。" >&2
  fi

  local patterns=(
    "$LIVOX_PATTERN"
    "$ROBOT_STATE_PUB_PATTERN"
    "$CHASSIS_PATTERN"
    "$NAV_PATTERN"
    "$RVIZ_PATTERN"
    "$JOY_PATTERN"
    '[n]av2_container'
    '[p]oint_lio'
    '[l]oam_interface'
    '[s]ensor_scan_generation'
    '[s]mall_gicp_relocalization'
    '[t]errain_analysis'
    '[l]ifecycle_manager'
    '[f]ake_vel_transform'
    '[c]md_mux'
    '[z]lio_node'
    '[z]lio_bridge'
    '[i]mu_saturation_guard'
    '[g]imbal_joint_state_relay'
    '[c]ontroller_server'
    '[p]lanner_server'
    '[b]t_navigator'
    '[v]elocity_smoother'
    '[b]ehavior_server'
    '[w]aypoint_follower'
    '[m]ap_server'
    '[r]obot_state_publisher'
  )
  local p
  for p in "${patterns[@]}"; do
    pkill -INT -u "$(id -u)" -f "$p" 2>/dev/null || true
  done
  sleep 5
  local killed=0
  for p in "${patterns[@]}"; do
    if pgrep -u "$(id -u)" -f "$p" >/dev/null 2>&1; then
      pkill -KILL -u "$(id -u)" -f "$p" 2>/dev/null || true
      killed=1
    fi
  done
  if [ "$killed" = "1" ]; then
    echo "[停止] 部分节点未响应 SIGINT, 已强制结束。"
  fi
  echo "[停止] 完成。请确认终端里的标签页也已退出 (必要时手动 Ctrl-C)。"
}

# ---------- 命令行参数 ----------
# --init-pose "<x,y,z,roll,pitch,yaw>"（与 start_sim_nav.sh 同名）：只取 x/y/yaw，
# 写进静态 map->odom。z/roll/pitch 在实车是二维静态 TF，会被忽略（并告警）。
# 返回非 0 表示格式不对。
parse_init_pose() {
  printf '%s' "$1" | tr ',' ' ' | awk '
    NF != 6 { exit 1 }
    { printf "%.6f %.6f %.6f", $1, $2, $6 }'
}

apply_init_pose() {
  local parsed
  if ! parsed="$(parse_init_pose "$1")"; then
    echo "[错误] --init-pose 需要 6 个数 x,y,z,roll,pitch,yaw: $1" >&2
    exit 2
  fi
  # shellcheck disable=SC2086
  set -- $parsed
  MAP_TO_ODOM_X="$1"; MAP_TO_ODOM_Y="$2"; MAP_TO_ODOM_YAW="$3"
  echo "[提示] --init-pose 已转成 map->odom: x=$1 y=$2 yaw=$3" \
       "（z/roll/pitch 在实车是二维静态 TF，已忽略）"
}

# 环境变量 INIT_POSE（仿真脚本用的名字）在命令行之前生效，CLI 可覆盖。
if [ -n "${FIRST_INIT_POSE:-}" ]; then
  apply_init_pose "$FIRST_INIT_POSE"
fi

# --chassis-spin 是可带数值的开关: 下一个参数是数字就当转速, 否则用默认值
# (这样 `--chassis-spin --no-rviz` 不会被误吞成转速)。与 start_sim_nav.sh 同一套实现。
is_spin_rate() {
  case "${1:-}" in
    ''|*[!0-9.eE+-]*) return 1 ;;
  esac
  awk -v v="$1" 'BEGIN{ exit !(v == v + 0) }'
}

while [ $# -gt 0 ]; do
  case "$1" in
    -m|--map)         MAP_NAME="${2:?--map 需要地图名或绝对路径}"; shift 2 ;;
    --prior-pcd)      PRIOR_PCD_FILE="${2:?--prior-pcd 需要名字或路径}"; shift 2 ;;
    -p|--params)      PARAMS_FILE="${2:?--params 需要绝对路径}"; PARAMS_FILE_GIVEN=1; shift 2 ;;
    -c|--controller)  CONTROLLER="${2:?--controller 需要 auto|omni|minco}"
                      CONTROLLER_CLI_GIVEN=1; shift 2 ;;
    --minco)          CONTROLLER="minco"; CONTROLLER_CLI_GIVEN=1; shift ;;
    --omni)           CONTROLLER="omni"; CONTROLLER_CLI_GIVEN=1; shift ;;
    --map-to-odom)
      MAP_TO_ODOM_X="${2:?--map-to-odom 需要 X Y YAW}"
      MAP_TO_ODOM_Y="${3:?--map-to-odom 需要 X Y YAW}"
      MAP_TO_ODOM_YAW="${4:?--map-to-odom 需要 X Y YAW}"
      shift 4 ;;
    # 模式开关各自独立置位, 不互相覆盖: 同时给两个会被下面的互斥检查明确拒绝,
    # 比"后者悄悄覆盖前者"更适合实车 (起错定位方式 = 车不动或 TF 打架)。
    --slam)           START_SLAM=1; MODE_GIVEN=1; shift ;;
    --reloc|--relocalization) START_RELOC=1; NO_RELOC=0; MODE_GIVEN=1; shift ;;
    --no-reloc)       START_RELOC=0; NO_RELOC=1; MODE_GIVEN=1; shift ;;
    --lio|--lio-only) START_LIO=1; MODE_GIVEN=1; shift ;;
    --static)         START_STATIC=1; MODE_GIVEN=1; shift ;;
    # ${2?} 而不是 ${2:?}: 允许显式给空串, 含义是"这一项跟随 URDF 模型"。
    --lidar-xyz)      LIDAR_XYZ="${2?--lidar-xyz 需要 \"X Y Z\"}"; shift 2 ;;
    --lidar-rpy)      LIDAR_RPY="${2?--lidar-rpy 需要 \"R P Y\"}"; shift 2 ;;
    --lidar-config)   LIDAR_CONFIG_FILE="${2:?--lidar-config 需要路径}"; shift 2 ;;
    --namespace)      NAMESPACE="${2:?--namespace 需要命名空间}"; shift 2 ;;
    --rviz)           USE_RVIZ=1; shift ;;
    --no-rviz)        USE_RVIZ=0; shift ;;
    --robot-state-pub)    USE_ROBOT_STATE_PUB=1; shift ;;
    --no-robot-state-pub) USE_ROBOT_STATE_PUB=0; shift ;;
    --chassis)        USE_CHASSIS=1; shift ;;
    --no-chassis)     USE_CHASSIS=0; shift ;;
    --joy)            USE_JOY=1; shift ;;
    --no-joy)         USE_JOY=0; shift ;;
    --smoother)       USE_VELOCITY_SMOOTHER=1; shift ;;
    --no-smoother)    USE_VELOCITY_SMOOTHER=0; shift ;;
    --fake-vel-transform)    USE_FAKE_VEL_TRANSFORM=1; FAKE_VEL_CLI_GIVEN=1; shift ;;
    --no-fake-vel-transform) USE_FAKE_VEL_TRANSFORM=0; FAKE_VEL_CLI_GIVEN=1; shift ;;
    --mux)            USE_CHASSIS_MUX=1; MUX_CLI_GIVEN=1; shift ;;
    --no-mux)         USE_CHASSIS_MUX=0; MUX_CLI_GIVEN=1; shift ;;
    --yaw-mode)       YAW_MODE="${2:?--yaw-mode 需要 auto|se2|spin|xy_only}"
                      YAW_MODE_CLI_GIVEN=1; shift 2 ;;
    # --chassis-spin 是可带数值的开关: 下一个参数是数字就当转速, 否则用默认 2.0。
    --chassis-spin)
                      CHASSIS_SPIN_CLI_GIVEN=1
                      if [ $# -ge 2 ] && is_spin_rate "$2"; then
                        CHASSIS_SPIN_WZ="$2"; shift 2
                      else
                        CHASSIS_SPIN_WZ="$CHASSIS_SPIN_DEFAULT"; shift
                      fi ;;
    --no-chassis-spin) CHASSIS_SPIN_WZ="0"; CHASSIS_SPIN_CLI_GIVEN=1; shift ;;
    # 与 start_sim_nav.sh 同名同义（可带数值）。⚠ 实车不执行，见文件头与下方提示。
    --gimbal-spin)
                      GIMBAL_SPIN_CLI_GIVEN=1
                      if [ $# -ge 2 ] && is_spin_rate "$2"; then
                        GIMBAL_SPIN_WZ="$2"; shift 2
                      else
                        GIMBAL_SPIN_WZ="$GIMBAL_SPIN_DEFAULT"; shift
                      fi ;;
    --no-gimbal-spin) GIMBAL_SPIN_WZ="0"; GIMBAL_SPIN_CLI_GIVEN=1; shift ;;
    --gimbal-joint-topic) GIMBAL_JOINT_TOPIC="${2:?--gimbal-joint-topic 需要话题}"; shift 2 ;;
    --no-gimbal-joint-topic) GIMBAL_JOINT_TOPIC=""; shift ;;
    --lio-frontend)   LIO_FRONTEND="${2:?--lio-frontend 需要 point_lio|zlio}"
                      LIO_FRONTEND_CLI_GIVEN=1; shift 2 ;;
    # --- 与 start_sim_nav.sh 对齐的参数面（同一条命令两边通用）---
    -w|--world)       WORLD_ARG="${2:?--world 需要一个世界名}"; shift 2 ;;   # 实车忽略
    --gicp)           START_RELOC=1; NO_RELOC=0; MODE_GIVEN=1; shift ;;     # = --reloc
    --no-gicp)        START_RELOC=0; NO_RELOC=1; MODE_GIVEN=1; shift ;;     # = --no-reloc(LIO)
    --no-lio)         LIO_DISABLED=1; MODE_GIVEN=1; shift ;;                # 无里程计来源=--static
    --init-pose)      apply_init_pose "${2:?--init-pose 需要 x,y,z,roll,pitch,yaw}"; shift 2 ;;
    --prior-pcd-manifest) PRIOR_PCD_MANIFEST="${2:?--prior-pcd-manifest 需要绝对路径}"; shift 2 ;;
    --rviz-config)    RVIZ_CONFIG="${2:?--rviz-config 需要 .rviz 文件绝对路径}"; shift 2 ;;
    --gui|--no-gui)   shift ;;                                              # Gazebo 专用，忽略
    --run|--no-run)   shift ;;                                              # Gazebo 专用，忽略
    --verify-spin)    VERIFY_SPIN=1; shift ;;
    --zlio-config)    ZLIO_CONFIG="${2:?--zlio-config 需要路径}"; shift 2 ;;
    --speed-limits)   SPEED_LIMITS_FILE="${2:?--speed-limits 需要路径}"; shift 2 ;;
    --composition)    USE_COMPOSITION=1; shift ;;
    --no-composition) USE_COMPOSITION=0; shift ;;
    --log-level)      LOG_LEVEL="${2:?--log-level 需要级别}"; shift 2 ;;
    -n|--check)       DRY_RUN=1; shift ;;
    --list-maps)      list_maps; exit 0 ;;
    --stop)           stop_real_nav; exit 0 ;;
    -h|--help)        usage; exit 0 ;;
    *) echo "[错误] 未知参数: $1 (用 -h 查看用法)" >&2; exit 2 ;;
  esac
done

# ---------- 航向策略 / 小陀螺 ----------
case "$YAW_MODE" in
  auto|se2|spin|xy_only) ;;
  *) echo "[错误] --yaw-mode 只支持 auto / se2 / spin / xy_only, 收到: $YAW_MODE" >&2; exit 2 ;;
esac

# 没给 --chassis-spin 就是不自转（默认行为）。
if [ "$CHASSIS_SPIN_CLI_GIVEN" = "1" ] && [ -z "$CHASSIS_SPIN_WZ" ]; then
  CHASSIS_SPIN_WZ="$CHASSIS_SPIN_DEFAULT"
fi
CHASSIS_SPIN_WZ="${CHASSIS_SPIN_WZ:-0}"
case "$CHASSIS_SPIN_WZ" in
  ''|*[!0-9.eE+-]*) echo "[错误] 底盘自转速度必须是数字 (rad/s): $CHASSIS_SPIN_WZ" >&2; exit 2 ;;
esac
if ! awk -v v="$CHASSIS_SPIN_WZ" 'BEGIN{ exit !(v == v + 0) }'; then
  echo "[错误] 底盘自转速度必须是数字 (rad/s): $CHASSIS_SPIN_WZ" >&2; exit 2
fi
# 底盘自转必须让 MINCO 进入 spin 航向策略，否则请求会被航向策略忽略（车不转）。
if ! awk -v v="$CHASSIS_SPIN_WZ" 'BEGIN{ exit (v == 0) ? 0 : 1 }'; then
  if [ "$YAW_MODE_CLI_GIVEN" = "1" ] && [ "$YAW_MODE" != "spin" ]; then
    echo "[错误] --chassis-spin 与 --yaw-mode ${YAW_MODE} 矛盾：" >&2
    echo "       底盘自转请求只有在 yaw_mode=spin 时才会被 MINCO 消费。" >&2
    echo "       去掉 --yaw-mode，或改成 --yaw-mode spin（只加 --chassis-spin 会自动切）。" >&2
    exit 2
  fi
  YAW_MODE="spin"
fi
if awk -v v="$CHASSIS_SPIN_WZ" 'BEGIN{ a=(v<0)?-v:v; exit !(a > 0) }'; then
  echo "[提示] 底盘自转 ${CHASSIS_SPIN_WZ} rad/s 已请求：首跑请架起车辆或守物理急停，先确认方向与增益。"
  echo "[提示] 实际 wz 还受两层钳位：velocity_smoother 的 yaw 界与 srm_cmd_mux.wz_max"
  echo "[提示] （config/real/speed_limits.yaml，首跑都是 1.5 rad/s）。"
fi

# ---------- --no-lio（与 sim 同名）----------
# 仿真里 --no-lio 的意思是"不用 LIO 出里程计"，那时还有 Gazebo 真值兜底。
# 实车没有真值：关掉 LIO 就真的没有 odometry 来源了，等价于 --static（车不会动）。
if [ "$LIO_DISABLED" = "1" ]; then
  if [ "$START_RELOC" = "1" ] || [ "$START_SLAM" = "1" ]; then
    echo "[提示] --no-lio + 重定位/建图: 里程计仍由 LIO 提供（small_gicp / slam_toolbox 只发布 map->odom），"
    echo "       实车上没有「不用 LIO 的里程计」，参数按给定模式继续。"
  else
    START_STATIC=1
    cat >&2 <<'EOF'
[警告] --no-lio: 实车没有仿真真值里程计，关掉 LIO 就没有任何 odom->base_link 来源。
[警告]        已按 --static 处理（只加载栅格图 + 静态 map->odom），除非车体模块自己发里程计，
[警告]        否则导航起不来、车不会动。仿真脚本里这个组合靠 Gazebo 真值兜底，实车没有那层。
EOF
  fi
fi

# ---------- 与 start_sim_nav.sh 对齐的参数：实车侧的语义说明 ----------
# 云台自转：仿真里由 Gazebo 插件直接施加；实车由下位机驱动，上位机没有指令通道。
# 为了"同一条命令两边都能跑"，这里接受参数但**不产生任何指令**，并明确告警。
if [ "$GIMBAL_SPIN_CLI_GIVEN" = "1" ]; then
  case "$GIMBAL_SPIN_WZ" in
    ''|*[!0-9.eE+-]*) echo "[错误] 云台自转速度必须是数字 (rad/s): $GIMBAL_SPIN_WZ" >&2; exit 2 ;;
  esac
  if ! awk -v v="$GIMBAL_SPIN_WZ" 'BEGIN{ exit !(v == v + 0) }'; then
    echo "[错误] 云台自转速度必须是数字 (rad/s): $GIMBAL_SPIN_WZ" >&2; exit 2
  fi
  cat >&2 <<EOF
[警告] 实车不执行 --gimbal-spin ${GIMBAL_SPIN_WZ} rad/s：云台由下位机驱动，上位机没有指令通道
[警告]   （协议上→下只有 vx/vy/wz 12 字节，下→上只有比赛状态，没有云台转速/关节角字段）。
[警告]   参数被接受只是为了与 ./script/start_sim_nav.sh 保持同一套命令行。
[提示] 真正要处理的是"云台在转、上位机不知道角度"：
[提示]   · 云台保持固定 → 什么都不用做（默认）；
[提示]   · 下位机自己转云台且能回传关节角 → 加 --gimbal-joint-topic <话题>；
[提示]     中继节点会把它转成 /joint_states，robot_state_publisher 用 revolute 关节接进 TF。
[提示]   · 否则 base_link yaw 会差一个云台角，导航"看到车自己在转"且不报错。
EOF
fi

# 一步验证：仿真里它会自主下发目标/自转并断言；实车上那是"车自己动"，不能默默执行。
if [ "$VERIFY_SPIN" = "1" ]; then
  cat >&2 <<'EOF'
[错误] 实车不支持 --verify-spin：它会自动下发自转请求与导航目标、让车自己动起来，
[错误]   而实车没有"无头沙箱"这层保护（仿真里它还会顺手清掉 Gazebo/Nav2）。
[提示] 参数被接受只是为了命令行一致；实车请按文档人工分步验收：
[提示]   ./script/start_real_nav.sh --yaw-mode xy_only                     # S1 平移链路
[提示]   ./script/start_real_nav.sh --yaw-mode spin --chassis-spin 0.3     # S2 方向/增益（架车）
[提示]   ./script/start_real_nav.sh                                       # S3 SE(2) 闭环
[提示]   ./script/start_real_nav.sh --chassis-spin 1.0                    # S4 小陀螺导航
[提示]   完整清单见 docs/仿真链迁移实车实施记录(ai).md §6。
EOF
  exit 2
fi

if [ -n "$WORLD_ARG" ]; then
  echo "[提示] --world $WORLD_ARG 是仿真专用参数，实车忽略；地图仍用 -m <名字> 指定。" >&2
fi
if [ -n "$PRIOR_PCD_MANIFEST" ]; then
  echo "[提示] --prior-pcd-manifest 是仿真专用参数（用建图出生点推导 map->odom 初值）。" >&2
  echo "[提示]   实车的起步位置每场不同，请用 --map-to-odom X Y YAW 或 --init-pose 指定。" >&2
fi

# ---------- 定位方式互斥 ----------
to01() { if is_true "$1"; then printf '1'; else printf '0'; fi; }
SLAM_ON="$(to01 "$START_SLAM")"
RELOC_ON="$(to01 "$START_RELOC")"
LIO_ON="$(to01 "$START_LIO")"
STATIC_ON="$(to01 "$START_STATIC")"

# --no-reloc: 把重定位关掉 (即使 START_RELOC 被环境变量打开), 于是**不需要先验 PCD**。
if is_true "$NO_RELOC"; then
  RELOC_ON=0
  START_RELOC=0
fi

# 一个模式都没给 (命令行没给、环境变量也没给) -> 默认 --reloc, 实车默认。
# 前提是这张地图有先验点云 (maps/pcd/<地图名>_mapframe.pcd); 没有的话下面 resolve 会
# 报错并告诉你缺哪份、怎么生成。想回到"静态 map->odom"用 --no-reloc 或 --lio。
if [ $((SLAM_ON + RELOC_ON + LIO_ON + STATIC_ON)) -eq 0 ] && [ "$MODE_GIVEN" = "0" ]; then
  if [ "$LIO_FRONTEND" = "zlio" ]; then
    # ZLIO 只接在纯 LIO 链上（与 --reloc/--slam 互斥），所以默认方式跟着前端走。
    LIO_ON=1
    START_LIO=1
    NO_RELOC=1
    echo "[提示] 未指定定位方式, 默认按 --lio 启动 (ZLIO 里程计 + 静态 map->odom)。"
    echo "[提示] ZLIO 只接纯 LIO 链; 要用 GICP 重定位请: --lio-frontend point_lio --reloc。"
    echo "[提示] 车必须停在 map 原点, 或用 --map-to-odom X Y YAW 给初值。"
  else
    RELOC_ON=1
    START_RELOC=1
    echo "[提示] 未指定定位方式, 默认按 --reloc 启动 (small_gicp 与先验 PCD 配准后发布 map->odom)。"
    echo "[提示] 需要该地图的先验点云 maps/pcd/<地图名>_mapframe.pcd。"
    echo "[提示] 不用重定位 (也就不需要 PCD): --no-reloc; 其他: --slam / --static, 见 --help。"
  fi
fi

# --no-reloc 单独给 (没搭别的模式): 就是 LIO 里程计 + 静态 map->odom, map->odom 取
# --map-to-odom 的值 (默认 0 0 0, 即车停在 map 原点朝 +x)。
if is_true "$NO_RELOC" && [ $((SLAM_ON + RELOC_ON + LIO_ON + STATIC_ON)) -eq 0 ]; then
  LIO_ON=1
  START_LIO=1
  echo "[提示] --no-reloc: $LIO_FRONTEND 里程计 + 静态 map->odom (不需要先验 PCD;"
  echo "[提示]             map->odom 默认 0 0 0, 用 --map-to-odom X Y YAW 改)。"
fi

if [ $((SLAM_ON + RELOC_ON + LIO_ON + STATIC_ON)) -gt 1 ]; then
  cat >&2 <<'EOF'
[错误] --slam / --reloc / --lio / --static 互斥, 因为它们都决定 map->odom 由谁发布:
[错误]   --slam  : slam_toolbox 发布 map->odom (边跑边建图)
[错误]   --reloc : small_gicp 与先验 PCD 配准后发布 map->odom
[错误]   --lio   : Point-LIO 只给里程计, map->odom 用静态 TF
[错误]   --static: 只加载栅格图, map->odom 用静态 TF, 且没有里程计来源
[错误] 同时开启会导致 TF 冲突。
[错误] 只选一个; 若没在命令行给多个, 检查环境变量 START_SLAM / START_RELOC /
[错误] START_LIO / START_STATIC 是否被导出过。
EOF
  exit 1
fi

if [ "$STATIC_ON" = "1" ]; then
  echo "[警告] --static 模式下没有任何里程计来源, odom->base_link 不会有人发布;" >&2
  echo "[警告] 除非车体模块自己发里程计, 否则导航起不来, 车也不会动。" >&2
fi

SLAM_ARG="$(bool_arg "$SLAM_ON")"
RELOC_ARG="$(bool_arg "$RELOC_ON")"
LIO_ARG="$(bool_arg "$LIO_ON")"
COMPOSITION_ARG="$(bool_arg "$USE_COMPOSITION")"
RVIZ_ARG="$(bool_arg "$USE_RVIZ")"
SMOOTHER_ARG="$(bool_arg "$USE_VELOCITY_SMOOTHER")"

case "$LOG_LEVEL" in
  debug|info|warn|error|fatal) ;;
  *) echo "[错误] --log-level 只支持 debug/info/warn/error/fatal" >&2; exit 2 ;;
esac

# ---------- 局部控制器: 先定控制器, 再由它决定用哪份参数文件 ----------
# 迁移 MINCO 时最容易出错的不是算法, 而是"以为加载了 MINCO 其实还是 Omni":
# 控制器和参数文件必须一致 (靠 FollowPath.plugin 区分), 所以顺序是
# 控制器 -> 参数文件 -> 一致性核对, 而不是先定文件再猜控制器。
# MINCO 有两份：迁移链（fakeyaw + base_link_fake，默认）与"不串 fake"的旧版。
# 默认选迁移链那份 —— 2026-11-06 起实车与仿真 `start_sim_nav.sh` 同构。
MINCO_PARAMS_DEFAULT="$PKG_DIR/config/real/nav2_params_srm_minco_fakeyaw.yaml"
if [ ! -f "$MINCO_PARAMS_DEFAULT" ]; then
  MINCO_PARAMS_DEFAULT="$PKG_SRC/config/real/nav2_params_srm_minco_fakeyaw.yaml"
fi
MINCO_PARAMS_NOFAKE="$PKG_DIR/config/real/nav2_params_srm_minco.yaml"
if [ ! -f "$MINCO_PARAMS_NOFAKE" ]; then
  MINCO_PARAMS_NOFAKE="$PKG_SRC/config/real/nav2_params_srm_minco.yaml"
fi
OMNI_PARAMS_DEFAULT="$PKG_DIR/config/real/nav2_params_srm.yaml"
if [ ! -f "$OMNI_PARAMS_DEFAULT" ]; then
  OMNI_PARAMS_DEFAULT="$PKG_SRC/config/real/nav2_params_srm.yaml"
fi
MINCO_PLUGIN_NAME="srm27_minco_controller::MincoMpcController"

# 只给了 -p、没显式选控制器: 交给下面按文件里的插件自动判定。
# 否则默认值 minco 会把"-p 指到 Omni 文件"这种老用法按"minco 却加载 Omni 文件"拦下来,
# 而用户其实只是想把参数文件指过去。
if [ "$CONTROLLER" = "minco" ] && [ "$CONTROLLER_CLI_GIVEN" = "0" ] && [ "$PARAMS_FILE_GIVEN" = "1" ]; then
  CONTROLLER="auto"
fi

case "$CONTROLLER" in
  auto|omni|minco) ;;
  *) echo "[错误] --controller 只支持 auto / omni / minco, 收到: $CONTROLLER" >&2; exit 2 ;;
esac

# ---------- 默认路径 ----------
# 默认控制器是 minco, 所以默认参数文件也是 MINCO 那份; -c omni 才用 Omni 那份。
# -p 显式给了就以 -p 为准 (auto 判定与后面的一致性检查再兜底)。
if [ -z "$PARAMS_FILE" ]; then
  if [ "$CONTROLLER" = "omni" ]; then
    PARAMS_FILE="$OMNI_PARAMS_DEFAULT"
  else
    PARAMS_FILE="$MINCO_PARAMS_DEFAULT"
  fi
  if [ ! -f "$PARAMS_FILE" ]; then
    echo "[错误] 默认参数文件不存在: $PARAMS_FILE" >&2
    exit 1
  fi
fi
if [ -z "$LIDAR_CONFIG_FILE" ]; then
  LIDAR_CONFIG_FILE="$PKG_DIR/config/real/mid360_user_config.json"
  if [ ! -f "$LIDAR_CONFIG_FILE" ]; then
    LIDAR_CONFIG_FILE="$PKG_SRC/config/real/mid360_user_config.json"
  fi
fi
if [ -z "$SPEED_LIMITS_FILE" ]; then
  SPEED_LIMITS_FILE="$PKG_DIR/config/real/speed_limits.yaml"
  if [ ! -f "$SPEED_LIMITS_FILE" ]; then
    SPEED_LIMITS_FILE="$PKG_SRC/config/real/speed_limits.yaml"
  fi
fi
if [ ! -f "$SPEED_LIMITS_FILE" ]; then
  echo "[错误] 速度限幅覆盖层不存在: $SPEED_LIMITS_FILE" >&2
  echo "[提示] 用 --speed-limits <绝对路径> 指定，或不用空串关闭合并（不推荐）。" >&2
  exit 1
fi

# ---------- 局部控制器: 速度链路与默认值 ----------
# (MINCO/OMNI 默认文件与控制器校验已在前面完成, 这里只处理从控制器派生出来的默认值。)
if [ "$CONTROLLER_CLI_GIVEN" = "1" ] && [ "$PARAMS_FILE_GIVEN" = "0" ]; then
  case "$CONTROLLER" in
    minco) PARAMS_FILE="$MINCO_PARAMS_DEFAULT" ;;
    omni)  PARAMS_FILE="$OMNI_PARAMS_DEFAULT" ;;
  esac
fi

if [ "$CONTROLLER" = "auto" ]; then
  if [ -f "$PARAMS_FILE" ] && grep -q "$MINCO_PLUGIN_NAME" "$PARAMS_FILE"; then
    CONTROLLER="minco"
  else
    CONTROLLER="omni"
  fi
fi

# 显式选 minco 但文件里不是 MINCO 插件 -> 直接失败, 不让"名字是 MINCO、跑的是 Omni"溜过去。
if [ "$CONTROLLER" = "minco" ] && [ -f "$PARAMS_FILE" ] \
   && ! grep -q "$MINCO_PLUGIN_NAME" "$PARAMS_FILE"; then
  echo "[错误] --controller minco, 但参数文件里没有 $MINCO_PLUGIN_NAME:" >&2
  echo "[错误]   $PARAMS_FILE" >&2
  echo "[提示] MINCO 实车参数文件: $MINCO_PARAMS_DEFAULT" >&2
  exit 1
fi

# fake_vel_transform 默认值跟着控制器走: MINCO 迁移链**必须启用**（它是链路的一部分）,
# Omni 旧链沿用"启用 legacy 模式"; 显式指定优先。
if [ "$CONTROLLER" = "minco" ]; then
  if [ "$FAKE_VEL_CLI_GIVEN" = "0" ] && [ "$USE_FAKE_VEL_TRANSFORM_GIVEN" = "0" ]; then
    USE_FAKE_VEL_TRANSFORM=1
    echo "[提示] MINCO 迁移链: fake_vel_transform 默认启用 (nav_se2, 与仿真 start_sim_nav.sh 一致)。"
  elif [ "$USE_FAKE_VEL_TRANSFORM" = "0" ]; then
    cat >&2 <<'EOF'
[错误] MINCO 迁移链必须启用 fake_vel_transform：
[错误]   参数文件把命令系设成了 base_link_fake（导航参考系不随底盘自转），
[错误]   平移速度的 R(-yaw) 必须由 fake_vel_transform 做那一次；
[错误]   关掉它 = 导航输出永远在"假车头方向"的系里，车会往错方向走且不报错。
[提示] 确实想用"导航不控自转、命令系 base_link"的旧链，请显式指定参数文件：
[提示]   ./script/start_real_nav.sh -p <绝对路径>/config/real/nav2_params_srm_minco.yaml
[提示]   （或用 -c omni 回到 Omni 旧链。）
EOF
    exit 1
  fi
fi

FAKE_ARG="$(bool_arg "$USE_FAKE_VEL_TRANSFORM")"

# 迁移链要求参数文件是 base_link_fake 变体（否则 fakeyaw 链本身就是错的）。
MINCO_FAKEYAW_CHAIN=0
if [ "$CONTROLLER" = "minco" ] && [ "$USE_FAKE_VEL_TRANSFORM" = "1" ] \
   && grep -q "base_link_fake" "$PARAMS_FILE" 2>/dev/null; then
  MINCO_FAKEYAW_CHAIN=1
fi

# 速度出口：迁移链的出口话题是 fake_vel_transform 的输入 cmd_vel_nav_fake；
# Omni 旧链保持原样（fake -> cmd_vel_nav2_result，无 fake -> 直接 cmd_vel_chassis）。
if [ "$MINCO_FAKEYAW_CHAIN" = "1" ]; then
  CMD_VEL_NAV_TOPIC="cmd_vel_nav_fake"
elif [ "$USE_FAKE_VEL_TRANSFORM" = "1" ]; then
  CMD_VEL_NAV_TOPIC="cmd_vel_nav2_result"
else
  CMD_VEL_NAV_TOPIC="cmd_vel_chassis"
fi

# 迁移链必须串 mux：cmd_vel_chassis 的唯一发布者是 srm_cmd_mux。
if [ "$MINCO_FAKEYAW_CHAIN" = "1" ] && [ "$USE_CHASSIS_MUX" = "0" ]; then
  echo "[警告] --no-mux: 导航出口是 cmd_vel_nav_body，而串口节点订阅 cmd_vel_chassis；" >&2
  echo "[警告]           不做额外重映射时车不会动。仅用于排查链路。" >&2
fi
if [ "$MINCO_FAKEYAW_CHAIN" = "0" ] && [ "$USE_CHASSIS_MUX" = "1" ] \
   && [ "$CONTROLLER" = "omni" ]; then
  # Omni 旧链不进 mux（navigation_launch 的出口就是 fake 的输入）。显式 --mux 会被忽略。
  if [ "$MUX_CLI_GIVEN" = "1" ]; then
    echo "[警告] Omni 旧链不支持 mux（旧链出口 cmd_vel_nav2_result），已忽略 --mux。" >&2
  fi
  USE_CHASSIS_MUX=0
fi

# 航向策略只在迁移链上有意义；旧链强制 xy_only（导航不控自转）。
if [ "$MINCO_FAKEYAW_CHAIN" = "0" ] && [ "$CONTROLLER" = "minco" ]; then
  echo "[提示] 参数文件不是 base_link_fake 变体：按旧链处理（导航不控自转，yaw_mode 忽略）。"
  YAW_MODE="xy_only"
fi
if [ "$CONTROLLER" = "omni" ] && [ "$CHASSIS_SPIN_CLI_GIVEN" = "1" ]; then
  cat >&2 <<'EOF'
[错误] Omni 旧链不支持小陀螺（--chassis-spin）：
[错误]   Omni 参数文件里 yaw_policy/ spin_request 都按"导航不控自转"配置，
[错误]   没有 MINCO 的 setSpinRequest 进线，请求发出去没人消费。
[提示] 小陀螺请用 MINCO 迁移链（默认）：./script/start_real_nav.sh --chassis-spin
EOF
  exit 2
fi

# ZLIO 前端与 GICP 重定位暂不兼容（方案 §10：small_gicp 那一侧尚未放宽条件）。
case "$LIO_FRONTEND" in
  point_lio|zlio) ;;
  *) echo "[错误] --lio-frontend 只支持 point_lio / zlio, 收到: $LIO_FRONTEND" >&2; exit 2 ;;
esac
if [ "$LIO_FRONTEND" = "zlio" ]; then
  if [ "$RELOC_ON" = "1" ]; then
    echo "[错误] --lio-frontend zlio 与 --reloc 不兼容：ZLIO 只接在纯 LIO 链路上" >&2
    echo "       （--reloc 用 small_gicp + point_lio）。请加 --no-reloc / --lio, 或 --lio-frontend point_lio。" >&2
    exit 2
  fi
  if [ "$SLAM_ON" = "1" ]; then
    echo "[错误] --lio-frontend zlio 与 --slam 不兼容：建图链路不经过 localization_launch。" >&2
    echo "       请加 --lio-frontend point_lio。" >&2
    exit 2
  fi
fi
case "$LIO_FRONTEND" in
  zlio)      LIO_LABEL="ZLIO" ;;
  *)         LIO_LABEL="Point-LIO" ;;
esac
if [ -z "$ZLIO_CONFIG" ]; then
  ZLIO_CONFIG="$PKG_DIR/config/real/zlio_mid360.yaml"
  if [ ! -f "$ZLIO_CONFIG" ]; then
    ZLIO_CONFIG="$PKG_SRC/config/real/zlio_mid360.yaml"
  fi
fi

# ---------- 环境检查 ----------
if [ "$DRY_RUN" != "1" ] && [ ! -f "$WS_DIR/install/setup.bash" ]; then
  echo "[错误] 未找到 $WS_DIR/install/setup.bash" >&2
  echo "[提示] 先构建工作空间:" >&2
  echo "[提示]   cd $WS_DIR && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 2" >&2
  echo "[提示] 或用 SRM27_WS_DIR=/path/to/ws 指定工作空间。" >&2
  exit 1
fi
if [ ! -f "$WS_DIR/install/setup.bash" ]; then
  echo "[警告] 未找到 $WS_DIR/install/setup.bash (仅 --check 可继续)。" >&2
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[警告] 当前 shell 里找不到 ros2; 每个标签页都会 source install/setup.bash, 正常可继续。" >&2
fi

if [ "$DRY_RUN" != "1" ] && ! command -v "$TERMINAL" >/dev/null 2>&1; then
  echo "[错误] 未找到终端程序: $TERMINAL" >&2
  echo "[提示] 可用 TERMINAL=konsole 或 TERMINAL=xfce4-terminal 指定。" >&2
  exit 1
fi

if [ "$DRY_RUN" != "1" ] && [ "$USE_RVIZ" = "1" ] \
   && [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
  echo "[警告] 未检测到 DISPLAY / WAYLAND_DISPLAY, RViz 可能起不来 (可加 --no-rviz)。" >&2
fi

# ---------- 参数 / 模型 / 地图 校验 ----------
# 先查"是不是绝对路径": nav2 的参数文件会被 launch 多次重写落盘, 相对路径在不同 cwd 下
# 会解析成不同文件, 报错也更难懂, 所以这里先给规则明确的那条错误。
if [ "${PARAMS_FILE#/}" = "$PARAMS_FILE" ]; then
  echo "[错误] --params 必须是绝对路径: $PARAMS_FILE" >&2
  exit 1
fi
if [ ! -f "$PARAMS_FILE" ]; then
  echo "[错误] Nav2 参数文件不存在: $PARAMS_FILE" >&2
  exit 1
fi

if [ ! -f "$LIDAR_CONFIG_FILE" ]; then
  echo "[错误] 雷达驱动配置不存在: $LIDAR_CONFIG_FILE" >&2
  echo "[提示] 用 --lidar-config <路径> 指定。" >&2
  exit 1
fi

# 驱动外参必须全零: 非零时驱动自己转一次点云、URDF 再转一次,
# 而且 real_robot_state_publisher_launch.py 会直接拒绝启动 —— 这里提前报错更清楚。
# 自检脚本把"文件坏了/缺字段/外参非零"分别说明, 避免都归到"外参非零"上误导排查。
if command -v python3 >/dev/null 2>&1; then
  EXTRINSIC_PROBLEM="$(python3 -c '
import json, sys

try:
    with open(sys.argv[1], encoding="utf-8") as stream:
        config = json.load(stream)
except Exception as ex:
    print("读取或解析失败: %s" % ex)
    sys.exit(0)

lidars = config.get("lidar_configs")
if not isinstance(lidars, list) or not lidars:
    print("缺少可用的 lidar_configs 列表")
    sys.exit(0)

bad = [str(item.get("ip", "?")) for item in lidars
       if any(item.get("extrinsic_parameter", {}).values())]
if bad:
    print("extrinsic_parameter 不是全零, 受影响雷达: %s" % ", ".join(bad))
' "$LIDAR_CONFIG_FILE" 2>&1 || true)"
  if [ -n "$EXTRINSIC_PROBLEM" ]; then
    echo "[错误] 雷达驱动配置不可用: $LIDAR_CONFIG_FILE" >&2
    echo "[错误] $EXTRINSIC_PROBLEM" >&2
    echo "[错误] 驱动外参必须全零, 安装位姿默认由 URDF 模型承担," >&2
    echo "[错误] (现场临时改才用 --lidar-xyz / --lidar-rpy), 否则点云会被旋转两次。" >&2
    exit 1
  fi
else
  echo "[警告] 未找到 python3, 跳过雷达驱动外参自检。" >&2
fi

# 参数文件与车体模型必须配套: 默认 SRM 模型发 base_link / livox_*;
# 上游那份 nav2_params_upstream.yaml 要的是 base_footprint / gimbal_yaw / front_mid360,
# 混用会满屏 TF 报错且重定位卡死。
if [ "$USE_ROBOT_STATE_PUB" = "1" ] && ! grep -qE "livox_imu|base_link" "$PARAMS_FILE"; then
  cat >&2 <<EOF
[错误] 参数文件与车体模型不配套:
[错误]   参数文件 $PARAMS_FILE
[错误]   里面没有 base_link / livox_imu, 看起来是 nav2_params_upstream.yaml 那份
[错误]   (需要 base_footprint / gimbal_yaw / front_mid360), 而本脚本启动的
[错误]   SRM 模型只发 base_link / livox_frame / livox_imu / livox_scan。
[错误] 请用默认的 config/real/nav2_params_srm.yaml。
EOF
  exit 1
fi

# ---------- 工作区一致性检查 ----------
# 与 start_sim_nav.sh 同一个理由: 若当前环境还 source 了同机的另一份工作区
# (例如 ~/srm_nav_27test), 同名包会优先解析到那边 —— 底盘限速、参数文件、行为树可能
# 全部来自另一个工作区, 而 MINCO 插件只存在于本工作区, 于是形成"一半新一半旧"的混合环境。
# 现场表现正是"改了参数却仍按旧值跑"。这里在启动前逐个核对关键包的解析路径。
require_pkg_from_ws() {
  local pkg="$1" prefix
  prefix="$(bash -c "source '$WS_DIR/install/setup.bash' >/dev/null 2>&1; ros2 pkg prefix '$pkg' 2>/dev/null" || true)"
  if [ -z "$prefix" ]; then
    echo "[错误] 找不到包 $pkg, 请确认 $WS_DIR/install/setup.bash 可用。" >&2
    return 1
  fi
  case "$prefix" in
    "$WS_DIR"/*) return 0 ;;
    *)
      echo "[错误] 包 $pkg 解析到了其它工作区: $prefix" >&2
      echo "       期望前缀: $WS_DIR" >&2
      echo "       当前环境里还 source 了别的工作区 (检查 ~/.bashrc 与 AMENT_PREFIX_PATH)。" >&2
      echo "       否则同名包 (含底盘限速、Nav2 参数、行为树) 会从那边加载。请开新终端只 source 本工作区。" >&2
      return 1
      ;;
  esac
}

if [ "$DRY_RUN" != "1" ]; then
  WS_CHECK_PKGS=(srm27_nav_bringup srm27_nav_protocol)
  if [ "$CONTROLLER" = "minco" ]; then
    WS_CHECK_PKGS+=(srm27_minco_controller srm27_minco_core)
  fi
  if [ "$USE_CHASSIS_MUX" = "1" ]; then
    # 迁移链的出口节点；缺了它 cmd_vel_chassis 没有任何发布者（车不动）。
    WS_CHECK_PKGS+=(srm27_chassis_control)
  fi
  if [ "$LIO_FRONTEND" = "zlio" ]; then
    WS_CHECK_PKGS+=(frontend localization_interfaces)
  fi
  for _pkg in "${WS_CHECK_PKGS[@]}"; do
    require_pkg_from_ws "$_pkg" || exit 1
  done
fi

# ---------- MINCO 专属预检 ----------
# 换控制器最容易出的错不是算法, 而是"名字是 MINCO、跑的其实还是 Omni"或"参数按仿真标定、
# 到实车就周期性失效"。预检脚本把这些只能靠读配置发现的问题集中起来, 只读、不需要 ROS,
# 因此放在上电之前跑。致命项直接拒绝启动; 待辨识项只告警。
if [ "$CONTROLLER" = "minco" ]; then
  PREFLIGHT_PY="$PKG_SRC/scripts/srm_minco_real_preflight.py"
  [ -f "$PREFLIGHT_PY" ] || PREFLIGHT_PY="$PKG_DIR/scripts/srm_minco_real_preflight.py"
  if [ -f "$PREFLIGHT_PY" ] && command -v python3 >/dev/null 2>&1; then
    PROTOCOL_CFG_FOR_PREFLIGHT="$WS_DIR/src/srm27_nav_protocol/config/srm27_nav_protocol.yaml"
    PREFLIGHT_ARGS=(--params "$PARAMS_FILE" --expect-minco)
    [ -f "$PROTOCOL_CFG_FOR_PREFLIGHT" ] && PREFLIGHT_ARGS+=(--protocol "$PROTOCOL_CFG_FOR_PREFLIGHT")
    if ! python3 "$PREFLIGHT_PY" "${PREFLIGHT_ARGS[@]}"; then
      echo >&2
      echo "[错误] MINCO 实车参数预检未通过 (见上)。" >&2
      echo "[提示] 预检把\"阻塞问题\"和\"待辨识警告\"分开列出; 带 [阻塞] 的必须先处理, 没有跳过开关 ——" >&2
      echo "[提示] 那些都是能让车不动或走错的硬性问题 (use_sim_time 为真、state_timeout 不大于" >&2
      echo "[提示] 里程计周期、制动走廊不可能满足、坐标系/到点几何矛盾), 绕过它们等于带着已知故障上电。" >&2
      echo "[提示] 请先修参数, 或在验收清单里记录这次为什么可以带着该问题启动。" >&2
      exit 1
    fi
    echo "[提示] MINCO 预检通过 (警告项仍需在验收清单里逐条确认)。"
  else
    echo "[警告] 未找到 MINCO 预检脚本或 python3, 跳过预检:" >&2
    echo "[警告]   $PREFLIGHT_PY" >&2
  fi

  # 插件必须真的能被 pluginlib 解析, 否则 controller_server 只会在运行时才报错。
  if [ "$DRY_RUN" != "1" ]; then
    PLUGIN_XML="$(bash -c "source '$WS_DIR/install/setup.bash' >/dev/null 2>&1; \
      ros2 pkg prefix srm27_minco_controller 2>/dev/null" || true)"
    if [ -n "$PLUGIN_XML" ] && [ ! -f "$PLUGIN_XML/share/srm27_minco_controller/srm27_minco_controller.xml" ]; then
      echo "[错误] 缺少 pluginlib 描述文件: $PLUGIN_XML/share/srm27_minco_controller/srm27_minco_controller.xml" >&2
      echo "[错误] controller_server 会在加载 FollowPath 时失败。" >&2
      exit 1
    fi
  fi
fi

# ---------- 多目标航点(途径点)行为树的部署检查 ----------
# 本仓库的 params 里 default_nav_through_poses_bt_xml 指向 behavior_trees/
# navigate_through_poses_route_aware.xml, 它用自研 BT 节点 RemovePassedGoalsByRoute
# 代替 nav2 的 RemovePassedGoals(nav2 那个在这套部署上实测不删已过的途径点)。
# 树由 srm27_nav_bringup 提供、节点由 srm27_nav_plugins 的 .so 提供, 两者都是 **启动时**
# 由 bt_navigator 按 plugin_lib_names 里的库名 dlopen 的 —— 只改参数文件而没重新编译部署
# 这两个包时, 报错要到 bt_navigator 启动时才出现(现场表现: 多目标导航直接不可用/加载报错)。
# 所以这里提前拦住, 并写清楚要重编哪两个包。
if [ "$DRY_RUN" != "1" ]; then
  BT_REL="$(sed -n 's|.*default_nav_through_poses_bt_xml:.*behavior_trees/\([^ ]*\).*|\1|p' \
    "$PARAMS_FILE" | head -1)"
  if [ -n "$BT_REL" ]; then
    BT_PATH=""
    for _cand in "$PKG_DIR/share/srm27_nav_bringup/behavior_trees/$BT_REL" \
                 "$PKG_SRC/behavior_trees/$BT_REL"; do
      if [ -f "$_cand" ]; then BT_PATH="$_cand"; break; fi
    done
    if [ -z "$BT_PATH" ]; then
      echo "[错误] 参数指向的行为树不存在: behavior_trees/$BT_REL" >&2
      echo "       参数文件: $PARAMS_FILE" >&2
      echo "       先重新编译部署: colcon build --packages-select srm27_nav_bringup" >&2
      exit 1
    fi
    if grep -q "srm27_remove_passed_goals_bt_node" "$PARAMS_FILE"; then
      BT_LIB_DIR="$(bash -c "source '$WS_DIR/install/setup.bash' >/dev/null 2>&1; \
        ros2 pkg prefix srm27_nav_plugins 2>/dev/null" || true)"
      if [ -z "$BT_LIB_DIR" ] || [ ! -f "$BT_LIB_DIR/lib/libsrm27_remove_passed_goals_bt_node.so" ]; then
        echo "[错误] 参数里要求自研 BT 节点库 libsrm27_remove_passed_goals_bt_node.so," >&2
        echo "       但在本工作区没找到 (srm27_nav_plugins 解析到: ${BT_LIB_DIR:-未找到})。" >&2
        echo "       先重新编译部署: colcon build --packages-select srm27_nav_plugins" >&2
        echo "       (缺这个库时 bt_navigator 会在启动时报找不到节点, 多目标导航不可用。)" >&2
        exit 1
      fi
    fi
  fi
fi

# 地图: --slam 仍要求该参数存在, 但不加载栅格图
if [ -z "$MAP_NAME" ]; then
  # 自动选择: 只有一个候选就用它, 多个则取最近修改的一份, 并明确打印出来。
  best=""; best_time=-1; count=0
  while IFS= read -r f; do
    [ -n "$f" ] || continue
    count=$((count + 1))
    t="$(stat -c %Y "$f" 2>/dev/null || echo 0)"
    if [ "$t" -gt "$best_time" ]; then best_time="$t"; best="$f"; fi
  done < <(list_map_candidates)
  if [ "$count" = "0" ]; then
    echo "[错误] 没有自动找到任何实车地图, 请用 -m 指定。" >&2
    list_maps >&2
    exit 1
  fi
  MAP_NAME="$(basename "$best" .yaml)"
  if [ "$count" = "1" ]; then
    echo "[提示] 自动选择唯一的地图: $MAP_NAME"
  else
    echo "[提示] 找到 $count 份地图, 自动选择最近修改的: $MAP_NAME ($best)"
    echo "[提示] 要用其他地图请加 -m <名字>, 用 --list-maps 查看全部。"
  fi
fi

if MAP_YAML="$(resolve_map "$MAP_NAME")"; then
  case "$MAP_YAML" in
    "$MAP_SIM_DIR"/*)
      echo "[警告] 选中的是仿真地图: $MAP_YAML" >&2
      echo "[警告] 实车请用 maps/ 下的场地地图 (-m <名字>, 或 --list-maps 查看)。" >&2
      ;;
  esac
else
  if [ "$SLAM_ON" = "1" ]; then
    MAP_YAML="$SRC_MAP_REAL_DIR/$MAP_NAME.yaml"   # 占位: slam 不加载栅格图
    echo "[提示] SLAM 模式不加载先验栅格图, 未找到 $MAP_NAME.yaml 属正常。"
  else
    echo "[错误] 未找到地图: $MAP_NAME" >&2
    list_maps >&2
    exit 1
  fi
fi

# 先验点云: 仅重定位需要
if [ "$RELOC_ON" = "1" ]; then
  [ -n "$PRIOR_PCD_FILE" ] || PRIOR_PCD_FILE="$MAP_NAME"
  if ! PRIOR_PCD="$(resolve_prior_pcd "$PRIOR_PCD_FILE")"; then
    echo "[错误] 重定位需要先验点云, 但未找到: $PRIOR_PCD_FILE" >&2
    echo "[错误] 已查找: $WS_DIR/maps/${PRIOR_PCD_FILE}_mapframe.pcd" >&2
    echo "[错误]           $PCD_MAPS_DIR/${PRIOR_PCD_FILE}_mapframe.pcd" >&2
    echo "[错误]           $PCD_MAPS_DIR/$PRIOR_PCD_FILE.pcd" >&2
    echo "[错误]           $WS_DIR/maps/$PRIOR_PCD_FILE.pcd" >&2
    echo "[错误]           $PCD_REAL_DIR/$PRIOR_PCD_FILE.pcd" >&2
    echo "[错误] 用 --prior-pcd <路径> 指定。" >&2
    echo "[错误] 注意: PCD 必须在地图坐标系下, Point-LIO camera_init 系的原始点云不能直接用。" >&2
    echo "[错误] 有录包的话可以先重建再转系:" >&2
    echo "[错误]   ./script/build_pcd_from_bag.sh --name <名字>" >&2
    echo "[错误]   python3 script/pcd_to_mapframe.py --pcd maps/pcd/<名字>.pcd --map maps/<名字>/<名字>.yaml" >&2
    echo "[提示] 只是想先把链路跑起来 (用静态 map->odom): 加 --lio。" >&2
    exit 1
  fi
  # 坐标系自检: mapframe 版本头部有标注, camera_init 系的原始点云通常没有。
  if ! head -c 1024 "$PRIOR_PCD" 2>/dev/null | grep -aqiE "map frame|map_frame|地图坐标系"; then
    echo "[警告] $PRIOR_PCD 头部未标注坐标系;" >&2
    echo "[警告] 若它是 Point-LIO camera_init 系的原始点云, GICP 初值会错, 请先转到地图坐标系。" >&2
  fi
else
  PRIOR_PCD="${PRIOR_PCD_FILE:-$PCD_REAL_DIR/$MAP_NAME.pcd}"   # 仅供 launch 声明
fi

# 底盘串口: 设备存在性与权限, 只告警不阻断 (车可能还没插上 / 只做链路验证)
if [ "$USE_CHASSIS" = "1" ]; then
  PROTOCOL_CFG="$WS_DIR/src/srm27_nav_protocol/config/srm27_nav_protocol.yaml"
  [ -f "$PROTOCOL_CFG" ] || PROTOCOL_CFG="$WS_DIR/install/srm27_nav_protocol/share/srm27_nav_protocol/config/srm27_nav_protocol.yaml"
  SERIAL_DEV=""
  if [ -f "$PROTOCOL_CFG" ]; then
    SERIAL_DEV="$(sed -n 's/^[[:space:]]*device_name:[[:space:]]*//p' "$PROTOCOL_CFG" | head -n 1)"
    SERIAL_DEV="${SERIAL_DEV%\"}"; SERIAL_DEV="${SERIAL_DEV#\"}"
  fi
  SERIAL_DEV="${SERIAL_DEV:-/dev/ttyACM0}"
  if [ ! -e "$SERIAL_DEV" ]; then
    echo "[警告] 底盘串口设备不存在: $SERIAL_DEV (插好下位机 USB 后再启动底盘标签页)" >&2
    # 物理接口在云台板 USART1（2026-11-08 现场确认设备号仍是 /dev/ttyACM0，不用改）。
    # 万一以后换转接板导致编号变化，这里只列出当前可用的候选，真正的值要改 yaml 的 device_name。
    # set -e + pipefail 下, 匹配不到文件时 ls 返回非 0 会直接退出脚本, 必须兜底。
    CANDS="$(ls /dev/serial/by-id/* /dev/ttyACM* /dev/ttyUSB* 2>/dev/null | tr '\n' ' ' || true)"
    if [ -n "$CANDS" ]; then
      echo "[警告] 当前检测到的串口: $CANDS" >&2
      echo "[警告] 若底盘/云台板就是其中一个, 把 yaml 的 device_name 改过去 (推荐 by-id 路径)" >&2
    else
      echo "[警告] 当前没有任何 ttyACM*/ttyUSB*, 先查 USB 线/供电/驱动 (dmesg | tail)" >&2
    fi
  elif [ ! -r "$SERIAL_DEV" ] || [ ! -w "$SERIAL_DEV" ]; then
    echo "[警告] 无读写权限: $SERIAL_DEV" >&2
    echo "[警告] 处理: sudo usermod -aG dialout \$USER 后重新登录, 或临时 sudo chmod 666 $SERIAL_DEV" >&2
  fi
fi

if [ "$USE_CHASSIS" = "0" ]; then
  echo "[警告] 未启动底盘串口节点: /cmd_vel_chassis 将没有消费者, 车不会动。" >&2
fi

# srm27_nav_protocol.launch.py 里 namespace 写死为空, 且订阅的是全局 cmd_vel_chassis。
# 一旦导航栈加了命名空间, 速度出口变成 /<ns>/cmd_vel_chassis, 底盘就收不到了。
if [ -n "$NAMESPACE" ] && [ "$USE_CHASSIS" = "1" ]; then
  echo "[警告] 指定了 --namespace=$NAMESPACE, 但底盘串口节点固定在根命名空间下," >&2
  echo "[警告] 它订阅 /cmd_vel_chassis, 而导航出口会变成 /$NAMESPACE/cmd_vel_chassis," >&2
  echo "[警告] 除非另有转发, 否则车不会动。实车建议不加 --namespace。" >&2
fi

# ---------- 组装命令 ----------
NS_ARG=""
[ -n "$NAMESPACE" ] && NS_ARG="namespace:=$NAMESPACE"

# 1) 雷达驱动: nav2_stack_launch.py 不负责起驱动, 必须单独启动。
DRIVER_CMD="ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args"
DRIVER_CMD="$DRIVER_CMD -r __node:=livox_ros_driver2"
DRIVER_CMD="$DRIVER_CMD --params-file $(q "$PARAMS_FILE")"
DRIVER_CMD="$DRIVER_CMD -p user_config_path:=$(q "$LIDAR_CONFIG_FILE")"
if [ -n "$NAMESPACE" ]; then
  DRIVER_CMD="$DRIVER_CMD -r __ns:=$(q "$NAMESPACE")"
fi

# 2) 车体 TF: 提供 base_link / livox_frame / livox_imu / livox_scan。
#    缺了这些 frame, small_gicp 会在构造时一直等 TF 而卡住整个 launch。
RSP_CMD="ros2 launch srm27_nav_bringup real_robot_state_publisher_launch.py"
RSP_CMD="$RSP_CMD use_sim_time:=False"
RSP_CMD="$RSP_CMD params_file:=$(q "$PARAMS_FILE")"
RSP_CMD="$RSP_CMD lidar_config:=$(q "$LIDAR_CONFIG_FILE")"
# 外参为空 = 干脆不传这个参数, 让 launch 走模型里的值。
# 不能改传空串: ros2 launch 会直接报 "malformed launch argument 'lidar_xyz:='"。
if [ -n "$LIDAR_XYZ" ]; then
  RSP_CMD="$RSP_CMD lidar_xyz:=$(q "$LIDAR_XYZ")"
fi
if [ -n "$LIDAR_RPY" ]; then
  RSP_CMD="$RSP_CMD lidar_rpy:=$(q "$LIDAR_RPY")"
fi
# 云台按**真关节**处理 (与仿真一致): gimbal_joint_type:=revolute。
# 前提是 /joint_states 有发布者 —— nav_srm_real_launch.py 里的 gimbal_joint_state_relay
# 一定会起: 给了 --gimbal-joint-topic 就用真实回传角, 没给就用 constant 模式暂位 0 度
# (TF 与 fixed 等价, 但结构就绪, 协议补上回传字段后只需加这个参数)。
# ⚠ 千万别在没有 relay 的情况下单独传 revolute —— 那会让整棵 TF 树断开。
RSP_CMD="$RSP_CMD gimbal_joint_type:=revolute"

# 3) 底盘串口: cmd_vel_chassis 的唯一消费者, 把速度写进串口发给 C 板。
CHASSIS_CMD="ros2 launch srm27_nav_protocol srm27_nav_protocol.launch.py"

# 4) 导航栈
#   * MINCO 迁移链：走 nav_srm_real_launch.py（nav2_stack + srm_cmd_mux + 链核对）。
#     它按 --yaw-mode 注入 yaw_policy/判停器，并把 speed_limits 合并进 mux 配置。
#   * Omni 旧链 / 非 fakeyaw 的 MINCO：仍走 nav2_stack_launch.py（与迁移前一致）。
Z_MUX_ARG="$(bool_arg "$USE_CHASSIS_MUX")"
if [ "$MINCO_FAKEYAW_CHAIN" = "1" ]; then
  NAV_CMD="ros2 launch srm27_nav_bringup nav_srm_real_launch.py"
  [ -n "$NS_ARG" ] && NAV_CMD="$NAV_CMD $NS_ARG"
  NAV_CMD="$NAV_CMD slam:=$SLAM_ARG"
  NAV_CMD="$NAV_CMD use_pcd_localization:=$RELOC_ARG"
  # 迁移链必须有里程计来源：--reloc 由 small_gicp 提供 map->odom、LIO 提供 odom->base_link；
  # 其余模式下用 launch 的默认 True。
  # --static 语义是"没有任何里程计来源"（只加载栅格图 + 静态 map->odom）；
  # --reloc / --slam 由 small_gicp / slam_toolbox 提供 map->odom、LIO 只做里程计，
  # 它们的 use_lio_odometry 必须是 False。其余（--lio / --no-reloc）才是 True。
  if [ "$SLAM_ON" = "1" ] || [ "$RELOC_ON" = "1" ] || [ "$STATIC_ON" = "1" ]; then
    NAV_CMD="$NAV_CMD use_lio_odometry:=False"
  else
    NAV_CMD="$NAV_CMD use_lio_odometry:=True"
  fi
  NAV_CMD="$NAV_CMD odometry_frontend:=$LIO_FRONTEND"
  NAV_CMD="$NAV_CMD zlio_config:=$(q "$ZLIO_CONFIG")"
  NAV_CMD="$NAV_CMD map:=$(q "$MAP_YAML")"
  NAV_CMD="$NAV_CMD prior_pcd_file:=$(q "$PRIOR_PCD")"
  NAV_CMD="$NAV_CMD params_file:=$(q "$PARAMS_FILE")"
  NAV_CMD="$NAV_CMD speed_limits_file:=$(q "$SPEED_LIMITS_FILE")"
  NAV_CMD="$NAV_CMD use_sim_time:=False"
  NAV_CMD="$NAV_CMD autostart:=true"
  NAV_CMD="$NAV_CMD use_composition:=$COMPOSITION_ARG"
  NAV_CMD="$NAV_CMD use_respawn:=false"
  NAV_CMD="$NAV_CMD use_fake_vel_transform:=$FAKE_ARG"
  NAV_CMD="$NAV_CMD command_frame:=base_link_fake"
  NAV_CMD="$NAV_CMD use_chassis_mux:=$Z_MUX_ARG"
  NAV_CMD="$NAV_CMD mux_nav_topic:=cmd_vel_nav_body"
  NAV_CMD="$NAV_CMD mux_output_topic:=cmd_vel_chassis"
  # 空值绝对不能传: ros2 launch 会报 malformed launch argument 'gimbal_joint_topic:='
  # 并直接退出 —— 表现是整个 nav2 栈一个节点都不起、launch.log 是 0 字节。
  # 没接云台角回传时干脆不传这个参数, 让 launch 用自己的默认值 ('')。
  if [ -n "$GIMBAL_JOINT_TOPIC" ]; then
    NAV_CMD="$NAV_CMD gimbal_joint_topic:=$(q "$GIMBAL_JOINT_TOPIC")"
  fi
  NAV_CMD="$NAV_CMD use_velocity_smoother:=$SMOOTHER_ARG"
  NAV_CMD="$NAV_CMD cmd_vel_nav_topic:=$CMD_VEL_NAV_TOPIC"
  NAV_CMD="$NAV_CMD yaw_mode:=$YAW_MODE"
  NAV_CMD="$NAV_CMD map_to_odom_x:=$MAP_TO_ODOM_X"
  NAV_CMD="$NAV_CMD map_to_odom_y:=$MAP_TO_ODOM_Y"
  NAV_CMD="$NAV_CMD map_to_odom_yaw:=$MAP_TO_ODOM_YAW"
  NAV_CMD="$NAV_CMD log_level:=$LOG_LEVEL"
  NAV_CMD="$NAV_CMD use_rviz:=False"   # RViz 由第 5 步单独拉（与旧链一致）
  # RViz 不需要两份：本入口的 rviz 由下面第 5 步起。
else
  NAV_CMD="ros2 launch srm27_nav_bringup nav2_stack_launch.py"
  [ -n "$NS_ARG" ] && NAV_CMD="$NAV_CMD $NS_ARG"
  NAV_CMD="$NAV_CMD slam:=$SLAM_ARG"
  NAV_CMD="$NAV_CMD use_pcd_localization:=$RELOC_ARG"
  NAV_CMD="$NAV_CMD use_lio_odometry:=$LIO_ARG"
  NAV_CMD="$NAV_CMD odometry_frontend:=$LIO_FRONTEND"
  NAV_CMD="$NAV_CMD zlio_config:=$(q "$ZLIO_CONFIG")"
  NAV_CMD="$NAV_CMD map:=$(q "$MAP_YAML")"
  NAV_CMD="$NAV_CMD prior_pcd_file:=$(q "$PRIOR_PCD")"
  NAV_CMD="$NAV_CMD params_file:=$(q "$PARAMS_FILE")"
  NAV_CMD="$NAV_CMD use_sim_time:=False"
  NAV_CMD="$NAV_CMD autostart:=true"
  NAV_CMD="$NAV_CMD use_composition:=$COMPOSITION_ARG"
  NAV_CMD="$NAV_CMD use_respawn:=false"
  NAV_CMD="$NAV_CMD use_fake_vel_transform:=$FAKE_ARG"
  NAV_CMD="$NAV_CMD use_velocity_smoother:=$SMOOTHER_ARG"
  NAV_CMD="$NAV_CMD cmd_vel_nav_topic:=$CMD_VEL_NAV_TOPIC"
  NAV_CMD="$NAV_CMD map_to_odom_x:=$MAP_TO_ODOM_X"
  NAV_CMD="$NAV_CMD map_to_odom_y:=$MAP_TO_ODOM_Y"
  NAV_CMD="$NAV_CMD map_to_odom_yaw:=$MAP_TO_ODOM_YAW"
  NAV_CMD="$NAV_CMD log_level:=$LOG_LEVEL"
fi

# 5) RViz
RVIZ_CONFIG_PATH="$PKG_DIR/rviz/nav2_default_view.rviz"
if [ -n "$RVIZ_CONFIG" ]; then
  if [ ! -f "$RVIZ_CONFIG" ]; then
    echo "[错误] --rviz-config 指定的文件不存在: $RVIZ_CONFIG" >&2
    exit 1
  fi
  RVIZ_CONFIG_PATH="$(readlink -f "$RVIZ_CONFIG")"
fi
RVIZ_CMD="ros2 launch srm27_nav_bringup rviz_launch.py"
[ -n "$NS_ARG" ] && RVIZ_CMD="$RVIZ_CMD $NS_ARG"
RVIZ_CMD="$RVIZ_CMD use_sim_time:=False"
RVIZ_CMD="$RVIZ_CMD rviz_config:=$(q "$RVIZ_CONFIG_PATH")"

# 6) 手柄遥控: 底盘订阅的是 cmd_vel_chassis, 这里显式指过去, 否则推杆车不动。
JOY_CMD="ros2 launch srm27_nav_bringup joy_teleop_launch.py"
[ -n "$NS_ARG" ] && JOY_CMD="$JOY_CMD $NS_ARG"
JOY_CMD="$JOY_CMD use_sim_time:=False"
JOY_CMD="$JOY_CMD joy_vel:=cmd_vel_chassis joy_config_file:=$(q "$PARAMS_FILE")"

# ---------- 配置摘要 ----------
if [ "$SLAM_ON" = "1" ]; then
  MODE_DESC="SLAM 建图 (slam_toolbox 发布 map->odom)"
elif [ "$RELOC_ON" = "1" ]; then
  MODE_DESC="先验 PCD 重定位 (small_gicp 发布 map->odom)"
elif [ "$LIO_ON" = "1" ]; then
  MODE_DESC="$LIO_LABEL 里程计 + 静态 map->odom"
else
  MODE_DESC="静态地图, 无里程计来源"
fi

if [ "$USE_VELOCITY_SMOOTHER" = "1" ]; then
  VEL_CHAIN="controller_server → cmd_vel_controller → velocity_smoother"
else
  VEL_CHAIN="controller_server (跳过 velocity_smoother)"
fi
if [ "$MINCO_FAKEYAW_CHAIN" = "1" ]; then
  # 迁移链：与仿真 start_sim_nav.sh --chassis-spin 完全同构，只是执行端换成串口。
  VEL_CHAIN="$VEL_CHAIN → cmd_vel_nav_fake → fake_vel_transform → cmd_vel_nav_body"
  if [ "$USE_CHASSIS_MUX" = "1" ]; then
    VEL_CHAIN="$VEL_CHAIN → srm_cmd_mux → cmd_vel_chassis → srm27_nav_protocol → 串口"
  else
    VEL_CHAIN="$VEL_CHAIN （不经 mux！串口节点订阅的是 cmd_vel_chassis，车不会动）"
  fi
elif [ "$USE_FAKE_VEL_TRANSFORM" = "1" ]; then
  VEL_CHAIN="$VEL_CHAIN → cmd_vel_nav2_result → fake_vel_transform(legacy) → cmd_vel_chassis"
else
  VEL_CHAIN="$VEL_CHAIN → cmd_vel_chassis (不经 fake_vel_transform)"
fi

if [ "$MINCO_FAKEYAW_CHAIN" = "1" ]; then
  case "$YAW_MODE" in
    auto|se2)  YAW_DESC="se2  (导航拥有 wz：车头跟路径切线)" ;;
    spin)      YAW_DESC="spin (小陀螺：rotation_cmd → MINCO)" ;;
    xy_only)   YAW_DESC="xy_only (导航不控自转，A/B 对照)" ;;
    *)         YAW_DESC="$YAW_MODE" ;;
  esac
fi

echo "================= SRM 实车 NAV 导航测试 ================="
echo "  工作空间    : $WS_DIR"
echo "  定位方式    : $MODE_DESC"
echo "  slam        : $SLAM_ARG   use_pcd_localization: $RELOC_ARG   use_lio_odometry: $LIO_ARG$([ "$MINCO_FAKEYAW_CHAIN" = "1" ] && echo '   (迁移链按定位方式自动定 use_lio_odometry)')"
echo "  地图 (yaml) : $MAP_YAML$([ "$SLAM_ON" = "1" ] && echo '   (SLAM 模式未加载)')"
if [ "$RELOC_ON" = "1" ]; then
  echo "  先验 PCD    : $PRIOR_PCD"
else
  echo "  先验 PCD    : $PRIOR_PCD   (未使用)"
fi
echo "  参数文件    : $PARAMS_FILE"
if [ "$CONTROLLER" = "minco" ]; then
  echo "  局部控制器  : MINCO + MPC (srm27_minco_controller::MincoMpcController)"
else
  echo "  局部控制器  : Omni (srm27_omni_pid_controller::OmniPidPursuitController)"
fi
echo "  雷达配置    : $LIDAR_CONFIG_FILE"
if [ "$USE_ROBOT_STATE_PUB" = "1" ]; then
  echo "  车体 TF     : real_robot_state_publisher_launch.py (SRM 模型)"
  if [ -z "$LIDAR_XYZ" ] && [ -z "$LIDAR_RPY" ]; then
    echo "  雷达安装    : 跟随 URDF 模型 (不传 lidar_xyz/lidar_rpy; 核对: tf2_echo base_link livox_frame)"
  else
    echo "  雷达安装    : xyz=[$LIDAR_XYZ]  rpy=[$LIDAR_RPY]  (脚本覆盖模型)"
  fi
else
  echo "  车体 TF     : 不启动 (假定其他模块已发布 URDF/TF)"
fi
echo "  底盘串口    : $([ "$USE_CHASSIS" = "1" ] && echo 'srm27_nav_protocol.launch.py' || echo '不启动')"
echo "  命名空间    : ${NAMESPACE:-<根命名空间>}"
echo "  组合节点    : $COMPOSITION_ARG    RViz: $RVIZ_ARG    手柄: $USE_JOY"
if [ "$USE_RVIZ" = "1" ]; then
  echo "  RViz 配置   : ${RVIZ_CONFIG:-<包内 nav2_default_view.rviz>}"
fi
if [ "$RELOC_ON" != "1" ] && [ "$SLAM_ON" != "1" ]; then
  echo "  map->odom   : x=$MAP_TO_ODOM_X y=$MAP_TO_ODOM_Y yaw=$MAP_TO_ODOM_YAW (静态)"
  if [ "$LIO_ON" = "1" ] && [ "$MAP_TO_ODOM_X" = "0.0" ] && [ "$MAP_TO_ODOM_Y" = "0.0" ] \
     && [ "$MAP_TO_ODOM_YAW" = "0.0" ]; then
    echo "  [提示] map->odom 取的是原点。若车不是停在地图原点, 请用"
    echo "  [提示] --map-to-odom X Y YAW 指定「起步点在 map 系下的位姿」, 否则 RViz 里车的位置是错的。"
  fi
fi
echo "  速度链路    : $VEL_CHAIN"
if [ "$MINCO_FAKEYAW_CHAIN" = "1" ]; then
  echo "  wz 归属     : 导航(上位机)    航向策略: $YAW_DESC"
  echo "  速度限幅    : $SPEED_LIMITS_FILE"
  if awk -v v="$CHASSIS_SPIN_WZ" 'BEGIN{ exit (v == 0) ? 0 : 1 }'; then
    echo "  底盘自转    : 关 (要转加 --chassis-spin [rad/s], 缺省 2.0; 首跑建议 0.3)"
  else
    echo "  底盘自转    : $CHASSIS_SPIN_WZ rad/s (Nav2 就绪后自动下发 rotation_cmd)"
    echo "                mux wz_max 会在这一层钳住 (首跑 1.5 rad/s, 见 speed_limits.yaml)"
  fi
  echo "  云台关节    : revolute (与仿真一致), base_link->gimbal_yaw 来自 /joint_states"
  if [ -n "$GIMBAL_JOINT_TOPIC" ]; then
    echo "                $GIMBAL_JOINT_TOPIC → /joint_states (真实回传角)"
    echo "                首次上车核对: 云台不动、手转底盘, RViz 里 base_link 要跟着转;"
    echo "                反了改 srm27_nav_protocol.yaml 的 gimbal_joint_sign (+1.0/-1.0)"
  else
    echo "                暂位 0 度 (已显式关闭回传)。底盘相对云台转时 base_link yaw 会错,"
    echo "                正常跑导航请去掉 --no-gimbal-joint-topic"
  fi
  if [ "$GIMBAL_SPIN_CLI_GIVEN" = "1" ]; then
    if awk -v v="$GIMBAL_SPIN_WZ" 'BEGIN{ exit (v == 0) ? 0 : 1 }'; then
      echo "  云台自转    : 关 (--gimbal-spin 0)"
    else
      echo "  云台自转    : 请求 $GIMBAL_SPIN_WZ rad/s —— 实车不执行（下位机驱动，无指令通道）"
    fi
  fi
  if [ "$LIO_FRONTEND" = "zlio" ]; then
    echo "  里程计前端  : ZLIO ($ZLIO_CONFIG)"
  else
    echo "  里程计前端  : Point-LIO"
  fi
else
  echo "  wz 归属     : 不由导航控制 (旧链)"
fi
echo "========================================================="

# ---------- 启动 ----------
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

# ---------- 启动前清理残留 ----------
# 每次启动前先跑一遍 kill_gzb.sh / kill_rviz.sh: 上一次异常退出留下的 Gazebo
# (以及它的 /clock bridge) 和 RViz 不清掉, 会和这次启动的节点抢 /clock、抢 TF、
# 抢 RViz 窗口。跳过本步骤: PREKILL=0 ./script/start_real_nav.sh
prekill_leftovers() {
  if [ "${DRY_RUN:-0}" = "1" ]; then
    echo "[dry-run] 跳过启动前清理 (kill_gzb.sh / kill_rviz.sh)。"
    return 0
  fi
  if [ "${PREKILL:-1}" = "0" ]; then
    echo "[提示] PREKILL=0, 跳过启动前清理 (kill_gzb.sh / kill_rviz.sh)。"
    return 0
  fi

  echo "[清理] 启动前先结束残留的 Gazebo / RViz 进程..."
  local name
  for name in kill_gzb.sh kill_rviz.sh; do
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

STEP_TITLES=()
STEP_PATTERNS=()
STEP_CMDS=()
add_step() {
  STEP_TITLES+=("$1")
  STEP_PATTERNS+=("$2")
  STEP_CMDS+=("$3")
}

add_step "SRM27 雷达驱动 (Livox MID360)" "$LIVOX_PATTERN" "$DRIVER_CMD"
if [ "$USE_ROBOT_STATE_PUB" = "1" ]; then
  add_step "SRM27 车体 TF" "$ROBOT_STATE_PUB_PATTERN" "$RSP_CMD"
fi
if [ "$USE_CHASSIS" = "1" ]; then
  add_step "SRM27 底盘串口" "$CHASSIS_PATTERN" "$CHASSIS_CMD"
fi
add_step "SRM27 导航栈 (Nav2)" "$NAV_PATTERN" "$NAV_CMD"
if [ "$USE_RVIZ" = "1" ]; then
  add_step "SRM27 RViz" "$RVIZ_PATTERN" "$RVIZ_CMD"
fi
if [ "$USE_JOY" = "1" ]; then
  add_step "SRM27 手柄遥控" "$JOY_PATTERN" "$JOY_CMD"
fi

TOTAL=${#STEP_TITLES[@]}

if [ "$DRY_RUN" = "1" ]; then
  echo
  echo "[--check] 仅解析, 不启动。将要执行的命令:"
  for i in "${!STEP_TITLES[@]}"; do
    printf '\n[%d/%d] %s\n  %s\n' "$((i + 1))" "$TOTAL" "${STEP_TITLES[$i]}" "${STEP_CMDS[$i]}"
  done
  echo
  echo "解析完成, 未启动任何进程。"
  exit 0
fi

prekill_leftovers

for i in "${!STEP_TITLES[@]}"; do
  title="${STEP_TITLES[$i]}"
  pattern="${STEP_PATTERNS[$i]}"
  cmd="${STEP_CMDS[$i]}"
  printf '[%d/%d] 启动 %s...\n' "$((i + 1))" "$TOTAL" "$title"

  if process_running "$pattern"; then
    echo "[跳过] 已检测到正在运行的进程: $title"
    continue
  fi

  open_term "$title" "$cmd"
done

# ---------- 底盘小陀螺（迁移链）：Nav2 就绪后自动下发 ----------
# 契约与手动的 script/spin_control.sh 完全一致（同一话题、同一 PID 文件）：
#   rotation_cmd --20 Hz--> MINCO(yaw_policy.mode=spin) --cmd_vel_nav_fake-->
#   fake_vel_transform --> srm_cmd_mux --> cmd_vel_chassis --> 串口
# 所以：停转 ./script/spin_control.sh off，查看 ./script/spin_control.sh status。
# ⚠ wz 归导航：控制器只在**跟随路径时**输出速度，没有导航目标时底盘不会转 —— 这不是故障。
if [ "$MINCO_FAKEYAW_CHAIN" = "1" ] \
   && awk -v v="$CHASSIS_SPIN_WZ" 'BEGIN{ exit (v == 0) ? 1 : 0 }'; then
  if [ -n "$NAMESPACE" ]; then
    SPIN_TOPIC="/$NAMESPACE/rotation_cmd"
  else
    SPIN_TOPIC="/rotation_cmd"
  fi
  SPIN_ABS="$(awk -v v="$CHASSIS_SPIN_WZ" 'BEGIN{ printf "%.3f", (v < 0) ? -v : v }')"
  echo "[底盘自转] 等 Nav2 就绪（${SPIN_TOPIC} 有订阅者，最长 ${CHASSIS_SPIN_READY_TIMEOUT}s）..."
  _spin_deadline=$(( $(date +%s) + CHASSIS_SPIN_READY_TIMEOUT ))
  _spin_subs=""
  while [ "$(date +%s)" -lt "$_spin_deadline" ]; do
    _spin_subs="$(bash -c "source '$WS_DIR/install/setup.bash' >/dev/null 2>&1; \
      timeout 6 ros2 topic info '$SPIN_TOPIC' 2>/dev/null" 2>/dev/null \
      | sed -n 's/^Subscription count: //p' | head -n 1)" || true
    [ -n "$_spin_subs" ] && [ "$_spin_subs" != "0" ] && break
    sleep 2
  done
  if [ -z "$_spin_subs" ] || [ "$_spin_subs" = "0" ]; then
    echo "[警告] ${CHASSIS_SPIN_READY_TIMEOUT}s 内没等到 ${SPIN_TOPIC} 的订阅者（MINCO 还没起来？）" >&2
    echo "       仍会启动发布器：等 Nav2 上线后请求会自动被消费。" >&2
  fi
  if ROBOT_NS="$NAMESPACE" SPIN_MAX_WZ="$SPIN_ABS" \
     "$SCRIPT_DIR/spin_control.sh" on "$CHASSIS_SPIN_WZ"; then
    printf '底盘自转  : %s rad/s 已下发 (20 Hz)；停转: %s/spin_control.sh off\n' \
      "$CHASSIS_SPIN_WZ" "$SCRIPT_DIR"
    if awk -v a="$SPIN_ABS" -v m="1.5" 'BEGIN{ exit !(a > m) }'; then
      echo "[提示] 请求 ${CHASSIS_SPIN_WZ} rad/s 超过 mux 的 wz_max(首跑 1.5)，实际会被限幅。"
    fi
  else
    echo "[警告] spin_control.sh on 失败，底盘不会自转。" >&2
  fi
fi

cat <<EOF

启动流程处理完成 (终端模式: $OPEN_MODE)。

安全提醒:
  - C 板固件自己**没有**超时保护: srm27_nav_protocol 的 0.5 s 看门狗只在它活着时有效,
    进程被 kill -9 / 主机崩溃 / 串口掉线时它没机会归零, 底盘会保持最后一帧的速度继续跑。
    停任何节点前先发零速, 或直接按物理急停:
      ros2 topic pub -r 20 /cmd_vel_chassis geometry_msgs/msg/Twist "{}"
  - srm27_nav_protocol 没有任何急停 service, 软件层面唯一的停车手段就是上面那条零速。
  - 一键停整条链路 (先发零速再结束进程):
      ./script/start_real_nav.sh --stop
EOF

if [ "$MINCO_FAKEYAW_CHAIN" = "1" ]; then
cat <<'EOF'
  - 迁移链多了一道软件出口闸（mux），可以直接清零并保持零速：
      ros2 service call /srm_cmd_mux/stop_all std_srvs/srv/Trigger
      ros2 service call /srm_cmd_mux/resume_all std_srvs/srv/Trigger
    ⚠ 它是**上位机侧**的闸，不是急停：串口节点仍在按看门狗重发零速，但 C 板固件没有
      超时保护，进程被 kill -9 / 掉电时它不生效。物理急停永远是第一手段。

  小陀螺 (请求经 rotation_cmd 直接进 MINCO，wz 仍由导航输出):
    ./script/start_real_nav.sh --chassis-spin 0.3    # 首跑：小转速先确认方向
    ./script/spin_control.sh on 1.0                  # 运行中改转速（需以 spin 模式启动）
    ./script/spin_control.sh status                  # 查看请求/实际转速/三层速度
    ./script/spin_control.sh off                     # 撤回请求（停转）

  云台: 实车由下位机驱动，协议暂无关节角回传 → 保持固定，或按文档接
    --gimbal-joint-topic <话题>（见 docs/仿真链迁移实车实施记录(ai).md）。
EOF
fi
cat <<EOF

链路自检 (新终端):
  cd '$WS_DIR' && source install/setup.bash
  ros2 topic hz /livox/lidar                                # 雷达点云 ~10 Hz
  ros2 topic hz /livox/imu                                  # 雷达 IMU ~200 Hz
  ros2 run tf2_ros tf2_echo odom base_link                   # LIO 里程计 TF (--lio/--reloc)
  ros2 run tf2_ros tf2_echo map base_link                    # 完整 TF 链
  ros2 topic echo /cmd_vel_controller --once                 # 控制器原始输出
EOF

if [ "$MINCO_FAKEYAW_CHAIN" = "1" ]; then
cat <<'EOF'
  ros2 topic echo /cmd_vel_nav_fake --once                   # 导航请求 (fake 系, 含 wz)
  ros2 topic echo /cmd_vel_nav_body --once                   # fake 转换后 (车体系, wz 应逐条相同)
  ros2 topic echo /cmd_vel_chassis --once                    # mux 输出 = 底盘实际收到的速度
  ros2 topic echo /chassis_speed --once                       # 底盘**实际**速度反馈 (0x0010)
  ros2 topic echo /chassis_to_gimbal_yaw_deg --once           # 底盘相对云台 (度, 0x0010 第 4 字段)
  ros2 topic echo /gimbal_joint_angle --once                  # 同上换成 gimbal_yaw_joint 弧度 (-π, π]
  ros2 topic hz /cmd_vel_chassis                             # 应为 200 Hz 左右 (mux 合成频率)
  ros2 run tf2_ros tf2_echo base_link base_link_fake         # fakeyaw TF: yaw 应等于 -底盘 yaw
  ros2 topic echo /srm_cmd_mux/diagnostics --once            # mux 诊断 (钳位/超时计数)
  ros2 topic echo /fake_vel_transform/diagnostics --once     # fake 诊断 (wz 透传/过期 yaw)
EOF
else
cat <<'EOF'
  ros2 topic echo /cmd_vel_chassis --once                    # 底盘实际收到的速度 (最终出口)
  ros2 topic hz /cmd_vel_chassis                             # 应有持续输出
EOF
fi
cat <<EOF
  ros2 topic hz /odometry                                    # 实车约 10 Hz (跟随雷达帧率, 不是 50 Hz)
  ros2 node list                                             # 各节点是否都起来了
EOF

if [ "$CONTROLLER" = "minco" ]; then
  cat <<EOF

MINCO 专属自检 (按顺序确认, 任一步不符就不要放开跑):
  1) 插件真的被加载 (不是"launch 没报错"就算):
       ros2 param get /controller_server FollowPath.plugin
       期望: srm27_minco_controller::MincoMpcController
       若显示 Omni, 说明参数文件选错了 —— 本脚本会用 --controller minco 选对文件,
       但如果 --stop 没清干净、旧进程还在, 也会看到旧值。
  3) 轨迹真的算出来了:
       ros2 topic hz /FollowPath/minco_trajectory
       ros2 topic hz /FollowPath/mpc_prediction
  4) 诊断健康 (这是判"卡在哪"的第一手证据, 失败路径也会发布):
       ros2 topic echo /FollowPath/diagnostics --once
       关注: planning_result=success、planning_time_ms、minimum_clearance>0、
             state_age 与 map_age 都很小、missed_deadline_count 不持续增长。
       出现 planning_result 非 success 时, 先看它的 reason 文本再动权重。
EOF
if [ "$MINCO_FAKEYAW_CHAIN" = "1" ]; then
cat <<'EOF'
  2) 迁移链的接线（fake 是链的一环，必须**在**，而不是"不在"）:
       ros2 node list | grep -i -E 'fake_vel_transform|srm_cmd_mux'   # 两个都应在
       ros2 topic info /cmd_vel_chassis -v                            # 发布者应**只有** srm_cmd_mux
       ros2 run tf2_ros tf2_echo base_link base_link_fake             # yaw ≈ -底盘 yaw
       ros2 param get /controller_server FollowPath.yaw_policy.mode    # se2 / spin / xy_only
  5) 导航拥有 wz（se2/spin 模式）: 跟随路径时 /cmd_vel_nav_fake 的 wz 应非零，
     且与 /cmd_vel_nav_body、/cmd_vel_chassis 的 wz **逐层相同**
     （fake 只转平移不碰 wz；mux 只做钳位）。xy_only 模式下三层 wz 恒为 0 才是对的。
     若请求了自转而 /cmd_vel_chassis 的 wz 仍是 0：先看 mux 的 wz_max、再看串口节点。
  6) 速度没有被任何一层静默钳住: /cmd_vel_chassis 的合速度能达到
     min(controllers limits, velocity_smoother, mux v_max, 串口 max_vx) 的 90% 以上。
     想知道"是哪一层卡住"，跑:
       python3 src/srm27_navigation/srm27_nav_bringup/scripts/srm_speed_limits_check.py
EOF
else
cat <<'EOF'
  2) 只有一份速度出口, 且没有 fake_vel_transform 抢:
       ros2 node list | grep -i fake_vel_transform      # 应当没有任何输出
       ros2 topic info /cmd_vel_chassis -v              # 发布者应只有 velocity_smoother
  5) 旧链 MPC 没有自转权限: /cmd_vel_controller 与 /cmd_vel_chassis 的 wz 应恒为 0。
  6) 速度没有被任何一层静默钳住: 实际 /cmd_vel_chassis 的合速度应能达到
     limits.max_linear_speed 的 90% 以上, 且不长期顶在某个更小的值上。
EOF
fi
cat <<EOF

  曲线对照 (判断"路径不对"还是"跟踪不对"):
     RViz 里同时显示全局路径 (map)、/FollowPath/planning_input_path (odom)、
     /FollowPath/minco_trajectory (odom)。三者形状应一致;
     若车不沿曲线走, 才是跟踪/执行层的问题 (限速不一致、command_lookahead 未标定)。

  分阶段上电的完整验收清单与回退步骤见:
     docs/minco实车迁移实施记录(ai).md

MINCO 脱困自检 (实车; 与仿真同源但多了"车真的会动"这一层风险):
  ⚠ 前置硬条件: **人在场、手放在物理急停上**。
     实车没有任何软件急停 —— start_real_nav.sh 顶部已说明: srm27_nav_protocol 没有
     急停/使能 service, 软件层唯一手段是在 /cmd_vel_chassis 发零速; 而它 0.5 s 看门狗
     只在进程活着时有效 (kill -9 / 串口掉线时 C 板固件没有超时保护, 会用最后一帧速度)。
     脱困是一个**主动给 0.5 m/s 平移**的行为, 所以急停只能靠物理按钮或断电。
  a) 参数真的生效 (configure 期读一次; 改完 yaml 必须重启导航栈):
       ros2 param get /controller_server FollowPath.recovery.fail_threshold
       ros2 param get /controller_server FollowPath.recovery.escape_speed
       ros2 param get /controller_server FollowPath.recovery.stuck_clearance
       ros2 param get /controller_server FollowPath.recovery.escaped_clearance
     期望 5 / 0.5 / 0.40 / 0.50。启动日志里还有一行 "MincoMpcController 脱困参数: ...",
     打印的是夹取后**实际生效**的值, 与此处不一致说明参数文件选错了。
     ⚠ 这几个参数**不能**用 ros2 param set 热改: 插件只在 configure 期读一次,
       且没有注册 onSetParameters 回调 —— set 会成功返回但毫无效果。
       要降档 (见下面 e) 只能改 yaml 后重启。
  b) 脱困可视化话题存在 (平时为空, 只有真脱困才有内容):
       ros2 topic list | grep recovery_markers
     RViz 里已预置显示项 "Recovery (MINCO)" (共用 nav2_default_view.rviz):
       绿线 = 脱困路径规划结果, 蓝箭头 = 本周期下发的那一步, 球 = 当前净空。
  c) 观察脱困状态 (只筛脱困字段, 不刷屏):
       ros2 topic echo /FollowPath/diagnostics | grep -E "recovery_(active|trigger|exit|command_count|stale_abort|repeat_suppressed)"
     验收看 **recovery_command_count**: 它从 0 变成非 0 才说明脱困**真的输出了逃逸速度**。
     只看到 recovery_active=true 而 command_count 一直是 0, 等于"进了脱困但没推车"。
     (日志里那行"输出 N 个周期"2026-11-03 之前永远打 0 —— leaveRecovery() 先把计数清零
      再打印, 已修; 修之前只能看这个话题。)
     **recovery_repeat_suppressed_count** 增长而 enter_count 不增长 = 判为"原地来回荡",
     脱困被刻意跳过 (窄带里推不出来), 此时该查通道宽度/机器人半径。
  d) 实测脱困速度到没到底盘 (脱困期间在另一个终端跑):
       ros2 topic echo /cmd_vel_chassis
     脱困期间应看到**恒定**的 vx/vy (合速度约 escape_speed × 串口 linear_velocity_scale),
     wz 恒 0。若这里没有数据, 查 smoother 是否把它吃掉了。
  e) 首次落地验证请**降档**, 不要直接用 0.5 m/s:
       改 config/real/nav2_params_srm_minco.yaml 的 FollowPath.recovery:
         escape_speed: 0.2      # 0.5 -> 0.2, 先确认方向对不对
         window_sec: 1.5        # 3.0 -> 1.5, 缩短单次推离时长
       然后重启导航栈。**只在周围空旷的位置人为造卡死**做首次验证,
       不要在窄通道/障碍密集区试。确认方向与符号都对之后, 再逐步回到 0.5 / 3.0。
  f) 怎么把它"逼"进脱困 (两个条件必须同时成立, 最容易白等的地方):
       条件 1: 重规划连续失败 fail_threshold(10) 次;
       条件 2: 车**自身**净空 < stuck_clearance(0.40 m)。
     ⚠ 条件 2 的净空是相对**原始致命栅格**算的, 膨胀层不计入 ——
       "被膨胀层包住、规划失败"不等于"卡死", 那种情况判为"前方路障", 只绕路不脱困。
     ⚠ **不要用 Gazebo 那套 ign service 瞬移**来造卡死: 实车没有 Gazebo,
       而且瞬移/挪车会让 Nav2 取消目标, 目标一取消控制器就不再被调用, 脱困发不出速度
       (仿真里已实测: 报"脱困成功"但 recovery_command_count=0)。
       实车请用"把车开进死角"或者直接推车到位后再下目标。
  g) 两条脱困链路的分工 (先后兜底, 不是互相打架):
       控制器内部脱困 (recovery.window_sec) 先试;
       没成功 -> 回重规划; 仍失败 -> 等建轨宽限抛给行为树
       -> ClearEntireCostmap x2 + BackUpFreeSpace (同样走 ESDF 梯度选方向)。
       ⚠ 行为树那条链路在仿真里出现过"±90° 交替推、原地来回摆约 30 s 没走出去"的现象,
         实车同理会来回推车 —— 首次验证时尤其注意人流与周围空间。
EOF
fi

cat <<'EOF'

上层状态:
  ros2 topic echo /local_costmap/scan --once                # 局部代价地图是否有数据
  ros2 topic echo /terrain_map --once                       # 地形分析输出
  ros2 lifecycle get /map_server                            # map_server 是否 active

下发目标点:
  在 RViz 用 "Nav2 Goal" / "2D Goal Pose" 点一个近处目标,
  先确认车头方向与正负号都对 (y 方向横移、原地转向是否与预期一致), 再放开跑。

清理残留 (本条会结束实车链路节点, 含 RViz; 请不要在仿真同时运行时用):
  ./script/start_real_nav.sh --stop
  ./script/kill_rviz.sh
EOF
