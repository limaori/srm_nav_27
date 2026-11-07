# Copyright 2026 SRM
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""SRM 实车导航入口（与仿真 `nav_srm_simulation_launch.py` 同构的唯一链路）。

2026-11-06：把仿真验证过的 fakeyaw 迁移链原样搬到实车。数据流（与仿真**只差执行端**）::

    MINCO/Nav2(base_link_fake) ─cmd_vel_nav_fake─► fake_vel_transform
        ─cmd_vel_nav_body─► srm_cmd_mux ─cmd_vel_chassis─► srm27_nav_protocol ─► 串口 ─► C 板

仿真里最后一段是 `cmd_vel_sim → srm_velocity_adapter → Gazebo`；实车是
`cmd_vel_chassis → srm27_nav_protocol`。**mux 之后的差别只有话题名**，这也是
`output_topic` 成为本入口参数的原因。

`wz` 归属与仿真一致：由上位机（导航栈）拥有。三件必须同时成立的事本入口都会核对：
`use_fakeyaw_transform=True`、`command_frame=base_link_fake`、
参数文件里 `FollowPath.command_frame` 与之相同（不一致直接报错，不让"转一次还是转两次"
变成隐含约定）。

与仿真入口的区别（其余全部复用）：
* 没有 Gazebo / 真值里程计 / 点云格式转换器；odom->base_link 由 `sensor_scan_generation`
  从 LIO 反推（`use_lio_odometry:=True` 或 `use_pcd_localization:=True`）。
* 雷达驱动、车体 TF、底盘串口由 `script/start_real_nav.sh` 分标签页拉起（与仿真一致：
  Gazebo 也是脚本单独拉），所以本入口只起"导航 + 速度出口 + RViz"。
* `speed_limits_file` 默认取 `config/real/speed_limits.yaml`；`@fakeyaw` 段会按参数文件
  内容自动命中（放开 smoother 的 yaw 界），并给 mux 提供 wz_max。
* 云台：仿真里 `--gimbal-spin` 由 Gazebo 插件直接施加；实车云台由下位机驱动，上位机
  只缺"关节角回传"。`gimbal_joint_topic` 非空时起 `gimbal_joint_state_relay`，
  把下位机回传角转成 `/joint_states`（协议补齐字段后立即可用）。

用法::

    # 默认：--lio + MINCO + fakeyaw + mux（推荐入口是 script/start_real_nav.sh）
    ros2 launch srm27_nav_bringup nav_srm_real_launch.py \\
        map:=/abs/map.yaml prior_pcd_file:=/abs/prior.pcd \\
        params_file:=/abs/config/real/nav2_params_srm_minco_fakeyaw.yaml \\
        use_lio_odometry:=True

    # 小陀螺（自转请求进 MINCO，wz 仍由导航输出）
    ros2 launch srm27_nav_bringup nav_srm_real_launch.py ... yaw_mode:=spin

    # A/B 对照：导航不控自转（旧链行为，但仍是 base_link_fake 参考系）
    ros2 launch srm27_nav_bringup nav_srm_real_launch.py ... yaw_mode:=xy_only

⚠ 安全：本入口不启动底盘串口节点，但一旦 `srm27_nav_protocol` 在跑、`cmd_vel_chassis`
有数据，车就会动。首跑请架起车辆或守物理急停；C 板固件没有超时清零，进程被 kill -9
时底盘会保持最后一帧速度。
"""

import os
import sys

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile, ParameterValue
from nav2_common.launch import RewrittenYaml

# 速度限幅的合并模块装在 lib/<包名> 下（scripts/srm_speed_limits.py），
# 预检脚本与本 launch 共用同一份实现。
_SCRIPTS_DIR = os.path.join(
    get_package_prefix("srm27_nav_bringup"), "lib", "srm27_nav_bringup"
)
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from srm_speed_limits import SpeedLimitParams  # noqa: E402

#: fakeyaw 迁移链上 Nav2 的输出话题（fake_vel_transform 的输入）。
FAKEYAW_NAV_TOPIC = "cmd_vel_nav_fake"
#: fake 转换后的车体系速度话题（mux 的 nav_topic）。
FAKEYAW_MUX_TOPIC = "cmd_vel_nav_body"
#: 实车执行端话题：srm27_nav_protocol 唯一订阅的速度话题（串口节点的出口名）。
REAL_EXEC_TOPIC = "cmd_vel_chassis"


def _as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _find_node_section(cfg, node_name):
    """在参数文件里递归找 `<node_name>: ros__parameters: {...}`（兼容命名空间嵌套）。"""
    if not isinstance(cfg, dict):
        return None
    section = cfg.get(node_name)
    if isinstance(section, dict):
        if isinstance(section.get("ros__parameters"), dict):
            return section["ros__parameters"]
        return section
    for value in cfg.values():
        found = _find_node_section(value, node_name)
        if found is not None:
            return found
    return None


def _report_speed_chain(context, params_file, use_fakeyaw, command_frame,
                        cmd_vel_nav_topic, mux_nav_topic, mux_output_topic,
                        use_mux, yaw_mode):
    """启动时核对并打印最终速度链与 TF owner（实车版，规则与仿真入口一致）。

    这里只**读参数文件核对**，不改写任何东西：
    * fake 节点的话题与 launch 的 `cmd_vel_nav_topic` / mux `nav_topic` 必须首尾相接；
    * `FollowPath.command_frame` 与 launch 的 `command_frame` 不一致直接报错；
    * 开了 mux 时，mux 的输出**必须**是 `cmd_vel_chassis` —— 否则串口节点收不到速度，
      现场表现是"链路全绿、车不动"。
    """
    if not _as_bool(use_fakeyaw):
        raise RuntimeError(
            "use_fakeyaw_transform=False 不支持实车迁移链：本入口就是 fakeyaw 链路"
            "（导航拥有 wz、参考系 base_link_fake）。要用旧链（导航不控自转）请走"
            " nav2_stack_launch.py（script/start_real_nav.sh -c omni）。"
        )
    if command_frame != "base_link_fake":
        raise RuntimeError(
            f"fakeyaw 链上 command_frame 必须是 base_link_fake，实际是 {command_frame!r}："
            "速度必须先由 fake_vel_transform 转一次，控制器再转一次就是重复旋转。"
        )

    messages = []
    warnings = []
    try:
        import yaml

        with open(params_file, "r") as stream:
            cfg = yaml.safe_load(stream) or {}
    except Exception as error:  # noqa: BLE001 - 参数文件读不了不该阻止启动
        messages.append(
            f"[速度链] 参数文件无法预读（{error}）；跳过一致性核对，链路以节点启动日志为准。"
        )
        cfg = None

    fake_section = _find_node_section(cfg, "fake_vel_transform") if cfg else None
    controller = _find_node_section(cfg, "controller_server") if cfg else None
    follow_path = controller.get("FollowPath") if isinstance(controller, dict) else None

    if not fake_section:
        warnings.append(
            "[速度链] 参数文件里没有 fake_vel_transform 段："
            "fake 节点会退回节点默认话题，链路大概率断开。"
        )
    else:
        fake_in = str(fake_section.get("input_cmd_vel_topic", ""))
        fake_out = str(fake_section.get("output_cmd_vel_topic", ""))
        if fake_in and fake_in != cmd_vel_nav_topic:
            warnings.append(
                f"[速度链] Nav2 输出话题 {cmd_vel_nav_topic} 与 fake_vel_transform."
                f"input_cmd_vel_topic={fake_in} 不一致：链路断开（车不会动）。"
            )
        if fake_out and fake_out != mux_nav_topic:
            warnings.append(
                f"[速度链] mux 输入 {mux_nav_topic} 与 fake_vel_transform."
                f"output_cmd_vel_topic={fake_out} 不一致：链路断开（车不会动）。"
            )
        mode = str(fake_section.get("transform_mode", "legacy"))
        if mode != "nav_se2":
            warnings.append(
                f"[速度链] fake_vel_transform.transform_mode={mode}，本链路应为 nav_se2"
                "（legacy 会叠加 cmd_spin，造成角速度双重叠加）。"
            )
        odom_timeout = fake_section.get("odom_timeout")
        if isinstance(odom_timeout, (int, float)) and odom_timeout < 0.3:
            warnings.append(
                f"[速度链] fake_vel_transform.odom_timeout={odom_timeout}s 对实车偏小："
                "实车 /odometry 跟随雷达帧率（约 10 Hz），建议 >= 0.6 s，"
                "否则真值/里程计偶尔慢一拍就会清零速度。"
            )
        extrapolation_topic = str(fake_section.get("chassis_cmd_vel_topic", ""))
        if extrapolation_topic and extrapolation_topic != mux_output_topic:
            warnings.append(
                f"[速度链] yaw 外推的指令源 chassis_cmd_vel_topic={extrapolation_topic} "
                f"不是 mux 的输出 {mux_output_topic}：外推会用错的 wz 序列。"
            )
    if isinstance(follow_path, dict):
        configured = str(follow_path.get("command_frame", ""))
        if configured and configured != command_frame:
            raise RuntimeError(
                "参数文件里 FollowPath.command_frame="
                f"{configured} 与 launch 的 command_frame={command_frame} 不一致："
                "两者必须同时是 base_link_fake（fake_vel_transform 转一次）。"
            )
        configured_mode = ""
        yaw_policy = follow_path.get("yaw_policy")
        if isinstance(yaw_policy, dict):
            configured_mode = str(yaw_policy.get("mode", ""))
        if configured_mode and configured_mode != yaw_mode:
            # launch 会在下面用 RewrittenYaml 覆盖成 yaw_mode，所以这只是提示。
            messages.append(
                f"[速度链] 参数文件 yaw_policy.mode={configured_mode}，"
                f"启动时改写为 {yaw_mode}（launch 负责，不是配置错误）。"
            )

    if use_mux and mux_output_topic != REAL_EXEC_TOPIC:
        raise RuntimeError(
            f"实车链路里 mux 的输出必须是 {REAL_EXEC_TOPIC}（串口节点订阅的话题），"
            f"实际是 {mux_output_topic!r}：否则 srm27_nav_protocol 收不到速度。"
        )

    chain = (
        f"    Nav2[{cmd_vel_nav_topic}] → fake_vel_transform[{mux_nav_topic}]"
        + (f" → srm_cmd_mux[{mux_output_topic}]" if use_mux else " → （不经 mux）")
        + " → srm27_nav_protocol → 串口\n"
    )
    if not use_mux:
        warnings.append(
            f"[速度链] 未启用 mux：导航出口直接是 {mux_nav_topic}，而串口节点订阅的是 "
            f"{REAL_EXEC_TOPIC} —— 除非你把 srm27_nav_protocol 重映射过去，否则车不会动。"
        )
    messages.append("[速度链] 实车 fakeyaw 迁移链:\n" + chain +
                    "    TF owner: odom->base_link = 定位(LIO/GICP/SLAM)；"
                    "base_link->base_link_fake = fake_vel_transform")
    messages.append(
        f"[速度链] 参数: use_fakeyaw_transform={use_fakeyaw}, command_frame={command_frame}, "
        f"yaw_mode={yaw_mode}, use_chassis_mux={use_mux}"
    )
    if yaw_mode in ("se2", "spin"):
        messages.append(
            "⚠ wz 由导航产生：C 板固件必须已实现 wz（协议 §3.3 早年写着「被忽略」）。"
            "首跑请架起车辆或守物理急停，先确认符号与增益。"
        )
    for warning in warnings:
        messages.append("⚠ " + warning)
    return [LogInfo(msg="\n".join(messages))]


def generate_launch_description():
    bringup_dir = get_package_share_directory("srm27_nav_bringup")
    launch_dir = os.path.join(bringup_dir, "launch")

    namespace = LaunchConfiguration("namespace")
    slam = LaunchConfiguration("slam")
    map_yaml_file = LaunchConfiguration("map")
    prior_pcd_file = LaunchConfiguration("prior_pcd_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    speed_limits_file = LaunchConfiguration("speed_limits_file")
    chassis_mux_params_file = LaunchConfiguration("chassis_mux_params_file")
    autostart = LaunchConfiguration("autostart")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    use_rviz = LaunchConfiguration("use_rviz")
    use_pcd_localization = LaunchConfiguration("use_pcd_localization")
    use_lio_odometry = LaunchConfiguration("use_lio_odometry")
    odometry_frontend = LaunchConfiguration("odometry_frontend")
    zlio_config = LaunchConfiguration("zlio_config")
    use_velocity_smoother = LaunchConfiguration("use_velocity_smoother")
    cmd_vel_nav_topic = LaunchConfiguration("cmd_vel_nav_topic")
    map_to_odom_x = LaunchConfiguration("map_to_odom_x")
    map_to_odom_y = LaunchConfiguration("map_to_odom_y")
    map_to_odom_yaw = LaunchConfiguration("map_to_odom_yaw")

    use_fakeyaw_transform = LaunchConfiguration("use_fakeyaw_transform")
    command_frame = LaunchConfiguration("command_frame")

    use_chassis_mux = LaunchConfiguration("use_chassis_mux")
    mux_nav_topic = LaunchConfiguration("mux_nav_topic")
    mux_output_topic = LaunchConfiguration("mux_output_topic")

    gimbal_joint_topic = LaunchConfiguration("gimbal_joint_topic")

    yaw_mode_arg = LaunchConfiguration("yaw_mode")
    # 实车默认 se2（与仿真一致）；显式 spin = 小陀螺（外部自转请求进 MINCO）。
    effective_yaw_mode = PythonExpression([
        "'", yaw_mode_arg, "' if '", yaw_mode_arg, "' != 'auto' else 'se2'",
    ])
    # 判停器与航向策略必须配套：spin 模式下车到点仍在自转，StoppedGoalChecker 的角速度
    # 判停永远不会通过，必须换成只看平移的 OmniStoppedGoalChecker。
    effective_goal_checker = PythonExpression([
        "'srm27_nav_plugins::OmniStoppedGoalChecker' if '", yaw_mode_arg,
        "' == 'spin' else 'nav2_controller::StoppedGoalChecker'",
    ])

    # Nav2 栈的参数文件：在通用文件之上注入航向策略与判停器（点分全路径，
    # 见 nav2_common RewrittenYaml 的 substitute_params "total path" 分支）。
    # root_key 留空：nav2_stack 内部的 RewrittenYaml 会再做命名空间包装。
    nav_stack_params = RewrittenYaml(
        source_file=params_file,
        root_key=None,
        param_rewrites={
            "controller_server.ros__parameters.FollowPath.yaw_policy.mode": effective_yaw_mode,
            "controller_server.ros__parameters.general_goal_checker.plugin":
                effective_goal_checker,
        },
        convert_types=True,
    )

    # 1) Nav2 栈（定位 + 导航 + fake_vel_transform，全部由 nav2_stack 负责）
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "nav2_stack_launch.py")),
        launch_arguments={
            "namespace": namespace,
            "slam": slam,
            "map": map_yaml_file,
            "prior_pcd_file": prior_pcd_file,
            "use_sim_time": use_sim_time,
            "params_file": nav_stack_params,
            "speed_limits_file": speed_limits_file,
            "autostart": autostart,
            "use_composition": use_composition,
            "use_respawn": use_respawn,
            "use_pcd_localization": use_pcd_localization,
            "use_lio_odometry": use_lio_odometry,
            "odometry_frontend": odometry_frontend,
            "zlio_config": zlio_config,
            # 实车的 ZLIO 输入接线（与仿真那份不同：消息格式、话题名、IMU 单位、帧名）。
            # 抄仿真会让滤波器当场发散：仿真 IMU 是 m/s²（scale 1.0），实车 MID-360 是 g（9.81）。
            "zlio_input_type": "livox",
            "zlio_lidar_topic": "livox/lidar",
            "zlio_imu_topic": "livox/imu_saturated",
            "zlio_imu_acceleration_scale": "9.81",
            "zlio_body_frame": "livox_imu",
            # 迁移链路：fake_vel_transform 夹在 Nav2 输出与 mux 之间。
            "use_fake_vel_transform": use_fakeyaw_transform,
            "use_velocity_smoother": use_velocity_smoother,
            "cmd_vel_nav_topic": cmd_vel_nav_topic,
            "map_to_odom_x": map_to_odom_x,
            "map_to_odom_y": map_to_odom_y,
            "map_to_odom_yaw": map_to_odom_yaw,
        }.items(),
    )

    # 2) 速度出口：srm_cmd_mux（cmd_vel_chassis 的唯一发布者）
    chassis_mux_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("srm27_chassis_control"),
                "launch",
                "srm_chassis_control.launch.py",
            )
        ),
        condition=IfCondition(use_chassis_mux),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            # 显式传底盘控制自己的参数文件：限速（vx_max/vy_max/v_max/wz_max）与超时是
            # 这条链的权威值，必须与控制器规划用的 limits 一致。
            "params_file": SpeedLimitParams(
                chassis_mux_params_file, speed_limits_file
            ),
            # mux 的输入就是 fake_vel_transform 的输出。
            "nav_topic": mux_nav_topic,
            # 实车出口必须是串口节点订阅的话题。
            "output_topic": mux_output_topic,
        }.items(),
    )

    # 3) 云台关节角 -> /joint_states（**必须起**，见下）
    #    实车现在与仿真一致：gimbal_joint_type=revolute，base_link->gimbal_yaw 这条 TF
    #    只来自 /joint_states。没有发布者时 robot_state_publisher 一个动态 TF 都不发，
    #    livox_frame/livox_imu 整棵子树从 base_link 断开 —— 实测表现是 RViz 全红、
    #    loam_interface 永远初始化不了、nav2 transform 全失败。
    #      · 给了 gimbal_joint_topic -> 用真实回传角（协议补齐后用这个）；
    #      · 没给 -> constant 模式，暂位 0 度：TF 与 fixed 完全等价，但结构已就绪，
    #        以后只传 gimbal_joint_topic 即可，不用再动模型/launch。
    gimbal_relay_cmd = Node(
        package="srm27_nav_bringup",
        executable="gimbal_joint_state_relay.py",
        name="gimbal_joint_state_relay",
        output="screen",
        namespace=namespace,
        parameters=[
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
            {"input_topic": gimbal_joint_topic},
            {
                "input_type": PythonExpression([
                    "'float32' if '", gimbal_joint_topic, "' != '' else 'constant'",
                ])
            },
            {"constant_angle": 0.0},
        ],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "rviz_launch.py")),
        condition=IfCondition(use_rviz),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "rviz_config": rviz_config_file,
        }.items(),
    )

    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            "namespace", default_value="", description="Top-level namespace (实车默认根命名空间)"
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "yaw_mode",
            default_value="auto",
            description=(
                "MINCO 航向策略：auto/se2=导航拥有 wz（车头跟路径切线）；"
                "spin=外部自转请求进线（小陀螺，配合 script/spin_control.sh）；"
                "xy_only=导航不控自转（A/B 对照）。spin 时判停器自动换成 "
                "OmniStoppedGoalChecker（只看平移）"
            ),
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "slam", default_value="False", description="Whether run SLAM"
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_pcd_localization",
            default_value="False",
            description="Use prior-PCD localization (small_gicp) for map->odom",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_lio_odometry",
            default_value="True",
            description=(
                "LIO 提供 odom->base_link（实车默认 True：没有 Gazebo 真值，"
                "必须有里程计来源，否则 Nav2 起不来）"
            ),
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "odometry_frontend",
            default_value="point_lio",
            description=(
                "里程计前端: point_lio（默认，实车已验证）或 zlio（实车参数见 "
                "config/real/zlio_mid360.yaml）。zlio 需要 use_lio_odometry:=True"
            ),
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "zlio_config",
            default_value=os.path.join(
                bringup_dir, "config", "real", "zlio_mid360.yaml"
            ),
            description="ZLIO 算法配置（实车那份：IMU 单位 g、livox CustomMsg、实车外参）",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "map",
            default_value=os.path.join(
                bringup_dir, "map", "real", "srm_site_01.yaml"
            ),
            description="Full path to the map YAML file to load",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "prior_pcd_file",
            default_value=os.path.join(bringup_dir, "pcd", "real", "srm_site_01.pcd"),
            description="Full path to prior PCD file to load (仅 --reloc 需要)",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_sim_time", default_value="False", description="实车使用系统时钟"
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(
                bringup_dir, "config", "real", "nav2_params_srm_minco_fakeyaw.yaml"
            ),
            description="实车迁移链参数文件（MINCO + base_link_fake）",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "speed_limits_file",
            default_value=os.path.join(
                bringup_dir, "config", "real", "speed_limits.yaml"
            ),
            description=(
                "实车速度限幅的唯一来源。启动时合并进 Nav2 参数文件与 srm_cmd_mux 配置；"
                "`@fakeyaw` 段按参数文件内容自动命中（放开 smoother 的 yaw 界）。"
            ),
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "chassis_mux_params_file",
            default_value=os.path.join(
                get_package_share_directory("srm27_chassis_control"),
                "config",
                "srm_chassis_control.yaml",
            ),
            description="srm_cmd_mux 的基础配置（限速由 speed_limits_file 合并进来）",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_chassis_mux",
            default_value="True",
            description=(
                "是否串 srm_cmd_mux。它是 cmd_vel_chassis 的唯一发布者，负责钳位、"
                "导航超时清零与 stop_all/resume_all 急停服务。建议常开"
            ),
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "mux_nav_topic",
            default_value=FAKEYAW_MUX_TOPIC,
            description="mux 的输入（fake_vel_transform 的输出）",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "mux_output_topic",
            default_value=REAL_EXEC_TOPIC,
            description=(
                "mux 的输出 = 串口节点订阅的话题。实车必须 cmd_vel_chassis，"
                "改成别的名字 launch 会直接报错"
            ),
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "gimbal_joint_topic",
            default_value="",
            description=(
                "云台关节角回传话题（空=不起中继）。实车云台由下位机驱动，"
                "协议补齐关节角字段后指过去；在此之前请保持云台固定，否则 base_link yaw 会差一个云台角"
            ),
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "autostart", default_value="true", description="Automatically startup the nav2 stack"
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_composition", default_value="True", description="Whether to use composed bringup"
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_respawn",
            default_value="False",
            description="Whether to respawn if a node crashes (composition disabled only)",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_velocity_smoother",
            default_value="True",
            description="True: controller→velocity_smoother→cmd_vel_nav_fake；False: 直接输出",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_fakeyaw_transform",
            default_value="True",
            description="是否启用 fake_vel_transform（实车唯一链路，必须为 True）",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "command_frame",
            default_value="base_link_fake",
            description=(
                "MINCO 速度命令坐标系。实车迁移链必须是 base_link_fake："
                "控制器输出 odom 对齐的平移速度，由 fake_vel_transform 做唯一一次 R(-yaw)"
            ),
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "cmd_vel_nav_topic",
            default_value=FAKEYAW_NAV_TOPIC,
            description=(
                "导航链路最终速度出口（fakeyaw 链上是 fake_vel_transform 的输入）。"
                "必须与参数文件 fake_vel_transform.input_cmd_vel_topic 一致"
            ),
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=os.path.join(bringup_dir, "rviz", "nav2_default_view.rviz"),
            description="Full path to the RVIZ config file to use",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_rviz", default_value="True", description="Whether to start RVIZ"
        )
    )
    ld.add_action(DeclareLaunchArgument("map_to_odom_x", default_value="0.0"))
    ld.add_action(DeclareLaunchArgument("map_to_odom_y", default_value="0.0"))
    ld.add_action(DeclareLaunchArgument("map_to_odom_yaw", default_value="0.0"))

    # 启动时核对并打印最终速度链与 TF owner。放在最前面：链路配错时第一条日志就说清楚，
    # 而不是等 10 s 后 mux 报"导航输入超时"或车干脆不动。
    def _speed_chain_report(context, _mux_nav_topic, _mux_output_topic):
        return _report_speed_chain(
            context,
            context.launch_configurations.get("params_file", ""),
            context.launch_configurations.get("use_fakeyaw_transform", "True"),
            context.launch_configurations.get("command_frame", "base_link_fake"),
            context.launch_configurations.get("cmd_vel_nav_topic", ""),
            # 用与 include 完全同一个替换求值，避免"核对的是另一套表达式"。
            perform_substitutions(
                context, normalize_to_list_of_substitutions(_mux_nav_topic)
            ),
            perform_substitutions(
                context, normalize_to_list_of_substitutions(_mux_output_topic)
            ),
            _as_bool(context.launch_configurations.get("use_chassis_mux", "True")),
            context.launch_configurations.get("yaw_mode", "auto"),
        )

    ld.add_action(
        OpaqueFunction(
            function=lambda context: _speed_chain_report(
                context, mux_nav_topic, mux_output_topic
            )
        )
    )

    ld.add_action(bringup_cmd)
    ld.add_action(chassis_mux_cmd)
    ld.add_action(gimbal_relay_cmd)
    ld.add_action(rviz_cmd)

    return ld
