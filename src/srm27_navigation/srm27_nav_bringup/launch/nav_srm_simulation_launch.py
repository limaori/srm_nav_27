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

"""SRM 仿真导航入口。

数据流（旧基线，wz 由独立自转链路负责）：:

    Nav2 controller ──cmd_vel_nav──┐
                                   ├─ srm_cmd_mux ──cmd_vel_sim──► SRM 速度适配器
    rotation_test_sender ─ rotation_controller ─ rotation_velocity ┘

数据流（wz 控制迁移 / fakeyaw，use_fakeyaw_transform:=True）::

    MINCO/Nav2（base_link_fake）──cmd_vel_nav_fake──► fake_vel_transform
        ──cmd_vel_nav_body──► srm_cmd_mux(wz_control_mode=nav_se2) ──cmd_vel_sim──► 适配器

三个开关必须与参数文件一致，本入口按 `params_file` 的文件名给出默认值，并在启动时
核对/打印最终速度链：

* `use_fakeyaw_transform`：是否启用 fake_vel_transform（迁移链路必须为 True）。
* `wz_control_mode`：`nav`（导航拥有 wz → mux 用 nav_se2）或 `rotation`（旧链路）。
* `command_frame`：MINCO 的速度命令坐标系；迁移链路是 `base_link_fake`。

其余约定：

* 复用现有点云转换、地形分析、定位与 Nav2 启动链路。旧基线里机器人参考系是真实
  随底盘自转的 base_link；迁移配置里是 base_link_fake（与 odom 平行、朝向稳定）。
* 速度出口由 `cmd_vel_nav_topic` 决定，再由 srm_cmd_mux 合成 cmd_vel_sim；底盘执行由
  srm27_gazebo_simulator 的 SRM 速度插件负责。
* 通过 start_simulation / start_chassis_control 选择是否由本入口一并拉起仿真与
  底盘控制链路；默认与旧流程一致，由 script/start_sim_nav.sh 分标签页启动。

用法：
  ros2 launch srm27_nav_bringup nav_srm_simulation_launch.py \\
    map:=.../rmuc_2025_tunnel.yaml \\
    rotation_mode:=constant rotation_speed:=1.0

  # wz 迁移链路（空场 + 真值里程计）
  ros2 launch srm27_nav_bringup nav_srm_simulation_launch.py \\
    params_file:=.../nav2_params_srm_fakeyaw.yaml
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
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
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


def _as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _normalize_map_argument(context):
    """Accept a map directory while passing map_server a YAML file."""
    map_value = context.launch_configurations.get("map", "")
    world_value = context.launch_configurations.get("world", "rmuc_2025")
    if not map_value or not os.path.isdir(map_value):
        return []

    candidates = (
        os.path.join(map_value, "map", "simulation", f"{world_value}.yaml"),
        os.path.join(map_value, "simulation", f"{world_value}.yaml"),
        os.path.join(map_value, f"{world_value}.yaml"),
    )
    for candidate in candidates:
        if os.path.isfile(candidate):
            return [SetLaunchConfiguration("map", candidate)]
    return []


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


def _report_speed_chain(context, params_file, use_fakeyaw, wz_control_mode, command_frame,
                        cmd_vel_nav_topic, mux_nav_topic):
    """启动时核对并打印最终速度链与 TF owner（方案 §4.4、§4.6 的记录要求）。

    这里做的是**读参数文件核对**，不做任何改写：
    * fake 节点的话题与 launch 的 `cmd_vel_nav_topic` / mux `nav_topic` 必须首尾相接，
      否则车不动而日志里只有"导航输入超时"这种间接症状；
    * 参数文件里的 `FollowPath.command_frame` 与 launch 的 `command_frame` 不一致时
      直接报错 —— 那意味着"转一次还是转两次"变成了隐含约定，正是要消灭的坑。
    """
    messages = []
    warnings = []
    if wz_control_mode not in ("nav", "rotation"):
        raise RuntimeError(
            f"wz_control_mode={wz_control_mode!r} 非法：只能是 nav（导航拥有 wz）或 "
            "rotation（wz 来自独立自转链路）。"
        )
    if _as_bool(use_fakeyaw) and command_frame != "base_link_fake":
        raise RuntimeError(
            f"use_fakeyaw_transform=True 时 command_frame 必须是 base_link_fake，"
            f"实际是 {command_frame!r}：fakeyaw 链上的速度必须先由 fake_vel_transform "
            "转一次，控制器再转一次就是重复旋转。"
        )
    if not _as_bool(use_fakeyaw) and command_frame != "base_link":
        raise RuntimeError(
            f"use_fakeyaw_transform=False 时 command_frame 必须是 base_link，"
            f"实际是 {command_frame!r}：没有 fake 转换节点，命令必须是真实车体系速度。"
        )
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
    follow_path = _find_node_section(cfg, "controller_server") if cfg else None
    if isinstance(follow_path, dict):
        follow_path = follow_path.get("FollowPath")

    if _as_bool(use_fakeyaw):
        if not fake_section:
            warnings.append(
                "[速度链] use_fakeyaw_transform=True，但参数文件里没有 fake_vel_transform 段："
                "fake 节点会退回节点默认话题（cmd_vel_nav_fake/cmd_vel_nav_body）。"
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
                    f"[速度链] srm_cmd_mux.nav_topic={mux_nav_topic} 与 fake_vel_transform."
                    f"output_cmd_vel_topic={fake_out} 不一致：链路断开（车不会动）。"
                )
            mode = str(fake_section.get("transform_mode", "legacy"))
            if mode != "nav_se2":
                warnings.append(
                    f"[速度链] fake_vel_transform.transform_mode={mode}，迁移链路应为 nav_se2"
                    "（legacy 会叠加 cmd_spin，造成角速度双重叠加）。"
                )
        if isinstance(follow_path, dict):
            configured = str(follow_path.get("command_frame", ""))
            if configured and configured != command_frame:
                raise RuntimeError(
                    "参数文件里 FollowPath.command_frame="
                    f"{configured} 与 launch 的 command_frame={command_frame} 不一致："
                    "两者必须同时是 base_link（控制器转一次）或 base_link_fake"
                    "（fake_vel_transform 转一次），否则会重复旋转或完全不旋转。"
                )
        messages.append(
            "[速度链] 导航 → fakeyaw 转换（wz 由导航拥有）:\n"
            f"    Nav2[{cmd_vel_nav_topic}] → fake_vel_transform"
            f"[{mux_nav_topic}] → srm_cmd_mux(nav_se2) → cmd_vel_sim → Gazebo\n"
            "    TF owner: odom->base_link = 定位/真值里程计；"
            "base_link->base_link_fake = fake_vel_transform"
        )
    else:
        warnings.append(
            "[速度链] 旧基线：导航只给平移，wz 来自独立自转链路（mux 走 "
            "legacy_independent_rotation，导航 angular.z 被丢弃）。"
        )
        messages.append(
            "[速度链] 旧基线（导航只给平移，wz 来自独立自转链路）:\n"
            f"    Nav2[{cmd_vel_nav_topic}] + rotation_velocity → srm_cmd_mux(legacy) "
            "→ cmd_vel_sim → Gazebo\n"
            "    TF owner: odom->base_link = 定位/真值里程计"
        )

    messages.append(
        f"[速度链] 参数: use_fakeyaw_transform={use_fakeyaw}, wz_control_mode={wz_control_mode}, "
        f"command_frame={command_frame}"
    )
    for warning in warnings:
        messages.append("⚠ " + warning)
    return [LogInfo(msg="\n".join(messages))]


def generate_launch_description():
    bringup_dir = get_package_share_directory("srm27_nav_bringup")
    launch_dir = os.path.join(bringup_dir, "launch")

    namespace = LaunchConfiguration("namespace")
    slam = LaunchConfiguration("slam")
    world = LaunchConfiguration("world")
    map_yaml_file = LaunchConfiguration("map")
    prior_pcd_file = LaunchConfiguration("prior_pcd_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    speed_limits_file = LaunchConfiguration("speed_limits_file")
    autostart = LaunchConfiguration("autostart")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    use_rviz = LaunchConfiguration("use_rviz")
    use_pcd_localization = LaunchConfiguration("use_pcd_localization")
    use_lio_odometry = LaunchConfiguration("use_lio_odometry")
    use_velocity_smoother = LaunchConfiguration("use_velocity_smoother")
    cmd_vel_nav_topic = LaunchConfiguration("cmd_vel_nav_topic")
    map_to_odom_x = LaunchConfiguration("map_to_odom_x")
    map_to_odom_y = LaunchConfiguration("map_to_odom_y")
    map_to_odom_yaw = LaunchConfiguration("map_to_odom_yaw")

    use_fakeyaw_transform = LaunchConfiguration("use_fakeyaw_transform")
    wz_control_mode = LaunchConfiguration("wz_control_mode")
    command_frame = LaunchConfiguration("command_frame")

    start_simulation = LaunchConfiguration("start_simulation")
    start_chassis_control = LaunchConfiguration("start_chassis_control")
    start_rotation_sender = LaunchConfiguration("start_rotation_sender")

    # 空场调试用的真值里程计：三种定位模式都不启用时，由仿真真值提供
    # odom -> base_link。三种定位模式下由 Point-LIO / GICP / SLAM 提供。
    use_ground_truth_odometry = PythonExpression([
        "not ", slam, " and not ", use_pcd_localization,
        " and not ", use_lio_odometry,
    ])

    # fakeyaw 迁移链上 mux 的 nav_topic 是 fake 转换器的输出；
    # 旧链路仍然是 cmd_vel_nav_topic（= 导航直接输出）。
    mux_nav_topic = PythonExpression([
        "'", FAKEYAW_MUX_TOPIC, "' if ", use_fakeyaw_transform, " else '", cmd_vel_nav_topic, "'",
    ])
    # 迁移链上旋转测试发送器必须停掉：mux 在 nav_se2 下拒绝独立自转输入，
    # 但让发送器继续跑只会制造无意义的告警与"看起来在自转"的错觉。
    effective_rotation_sender = PythonExpression([
        "'False' if ", use_fakeyaw_transform, " else '", start_rotation_sender, "'",
    ])
    mux_wz_mode = PythonExpression([
        # 注意：wz_control_mode 的值是裸文本（nav / rotation），嵌进表达式时必须补引号，
        # 否则会变成 `'nav_se2' if nav == 'nav'` 这种 NameError（启动直接失败）。
        "'nav_se2' if '", wz_control_mode, "' == 'nav' else 'legacy_independent_rotation'",
    ])

    # 点云转换节点使用的参数：与旧入口一致，按命名空间 root_key 重写 use_sim_time。
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={"use_sim_time": use_sim_time},
            convert_types=True,
        ),
        allow_substs=True,
    )

    # 1) Gazebo 世界 + SRM 模型 + 传感器桥接 + 速度执行（可选，默认由启动脚本拉起）
    simulation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("srm27_gazebo_simulator"),
                "launch",
                "srm_sim.launch.py",
            )
        ),
        condition=IfCondition(start_simulation),
    )

    # 2) 点云转换：保持现有 ign_sim_pointcloud_tool 链路。
    #    该节点在参数文件里有自己的 pcd_topic / n_scan / horizon_scan，
    #    必须把参数文件传给它，只传 use_sim_time 会让它回退到别的默认值。
    start_velodyne_convert_tool = Node(
        package="ign_sim_pointcloud_tool",
        executable="ign_sim_pointcloud_tool_node",
        name="ign_sim_pointcloud_tool",
        output="screen",
        namespace=namespace,
        parameters=[configured_params],
    )

    # 3) 仿真真值里程计（空场与静态地图模式使用）
    start_simulation_odometry = Node(
        condition=IfCondition(use_ground_truth_odometry),
        package="srm27_nav_bringup",
        executable="simulation_ground_truth_odometry.py",
        name="simulation_ground_truth_odometry",
        output="screen",
        namespace=namespace,
        # The bridge already timestamps odometry in Gazebo simulation time.
        # Keep this lightweight adapter off the high-rate /clock subscription.
        parameters=[
            {"use_sim_time": False},
            {"base_frame": "base_link"},
        ],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    # 4) Nav2 栈：参数与速度出口都指向 SRM 版本
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "nav2_stack_launch.py")),
        launch_arguments={
            "namespace": namespace,
            "slam": slam,
            "map": map_yaml_file,
            "prior_pcd_file": prior_pcd_file,
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "speed_limits_file": speed_limits_file,
            "autostart": autostart,
            "use_composition": use_composition,
            "use_respawn": use_respawn,
            "use_pcd_localization": use_pcd_localization,
            "use_lio_odometry": use_lio_odometry,
            # 迁移链路：fake_vel_transform 夹在 Nav2 输出与 mux 之间（方案 §4.4）。
            "use_fake_vel_transform": use_fakeyaw_transform,
            "use_velocity_smoother": use_velocity_smoother,
            "cmd_vel_nav_topic": cmd_vel_nav_topic,
            "map_to_odom_x": map_to_odom_x,
            "map_to_odom_y": map_to_odom_y,
            "map_to_odom_yaw": map_to_odom_yaw,
        }.items(),
    )

    # 5) 速度合成与独立自转（mux 是 cmd_vel_sim 的唯一发布者）
    chassis_control_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("srm27_chassis_control"),
                "launch",
                "srm_chassis_control.launch.py",
            )
        ),
        condition=IfCondition(start_chassis_control),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            # 显式传底盘控制自己的参数文件：底盘限速（vx_max/vy_max/v_max/wz_max）与超时
            # 都是这条链的权威值，必须与控制器规划用的 limits 一致。虽然该 launch 的默认值
            # 已经指向包内配置，但显式传递能让"参数从哪来"在启动命令里可见，也避免将来
            # 默认值变化时静默改变行为。
            #
            # 限速本身已经搬到 config/simulation/speed_limits.yaml（唯一来源），这里用
            # SpeedLimitParams 在启动时把它合并进底盘配置：mux 与自转链拿到的是同一份
            # 速度档位，改档位不用再回来改这个文件。
            "params_file": SpeedLimitParams(
                os.path.join(
                    get_package_share_directory("srm27_chassis_control"),
                    "config",
                    "srm_chassis_control.yaml",
                ),
                speed_limits_file,
            ),
            # wz 所有权：nav_se2 时 mux 的输入是 fake 转换后的导航速度，
            # 且拒绝 rotation 输入（方案 §4.3）。
            "nav_topic": mux_nav_topic,
            "wz_control_mode": mux_wz_mode,
            "start_rotation_sender": effective_rotation_sender,
            "rotation_mode": LaunchConfiguration("rotation_mode"),
            "rotation_speed": LaunchConfiguration("rotation_speed"),
            "rotation_offset": LaunchConfiguration("rotation_offset"),
            "rotation_amplitude": LaunchConfiguration("rotation_amplitude"),
            "rotation_period": LaunchConfiguration("rotation_period"),
            "rotation_phase": LaunchConfiguration("rotation_phase"),
            "rotation_sine_wave": LaunchConfiguration("rotation_sine_wave"),
        }.items(),
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
            "namespace", default_value="red_standard_robot1", description="Top-level namespace"
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "slam",
            default_value="False",
            description="Whether run a SLAM. If True, it will disable small_gicp and send static tf (map->odom)",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_pcd_localization",
            default_value="False",
            description="Use prior-PCD localization instead of a fixed simulation origin",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_lio_odometry",
            default_value="False",
            description="Point-LIO odometry without GICP relocalization",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "world",
            default_value="rmuc_2025",
            description="Select world: 'rmul_2024' or 'rmuc_2024' or 'rmul_2025' or 'rmuc_2025' (map file share the same name as the this parameter)",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "map",
            default_value=[
                TextSubstitution(
                    text=os.path.join(
                        bringup_dir, "map", "simulation", "rmuc_2025_tunnel.yaml"
                    )
                )
            ],
            description="Full path to the tunnel map YAML file to load",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "prior_pcd_file",
            default_value=[
                TextSubstitution(text=os.path.join(bringup_dir, "pcd", "simulation", "")),
                world,
                TextSubstitution(text=".pcd"),
            ],
            description="Full path to prior pcd file to load",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="True",
            description="Use simulation (Gazebo) clock if True",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(
                bringup_dir, "config", "simulation", "nav2_params_srm.yaml"
            ),
            description="Full path to the ROS2 parameters file to use for all launched nodes",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "speed_limits_file",
            default_value=os.path.join(
                bringup_dir, "config", "simulation", "speed_limits.yaml"
            ),
            description=(
                "仿真速度限幅的唯一来源（控制器 / 平滑器 / mux / 恢复行为的限幅参数）。"
                "启动时合并进 Nav2 参数文件与底盘控制参数文件；控制器、平滑器、mux 三层"
                "的限速都在这里改。给空串可关闭合并，但那会退回插件默认值"
                "（Omni v_linear_max 默认 3.0 m/s），只适合排查问题。"
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
            description="Whether to respawn if a node crashes. Applied when composition is disabled.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_velocity_smoother",
            default_value="True",
            description="True: controller→velocity_smoother→cmd_vel_nav；False: controller 直接输出",
        )
    )
    # --- wz 控制迁移 / fakeyaw（方案 §4.4）-------------------------------------
    # 三个开关的默认值**跟随参数文件**：文件名里带 fakeyaw 的迁移配置默认开启迁移链路，
    # 旧基线（nav2_params_srm.yaml / nav2_params_srm_minco.yaml）默认保持旧行为。
    # 这样"用哪份配置"与"走哪条链"只有一处真相，不会出现"参数是 fakeyaw、链路是旧链路"。
    fakeyaw_default = PythonExpression([
        "'True' if 'fakeyaw' in '", params_file, "' else 'False'",
    ])
    ld.add_action(
        DeclareLaunchArgument(
            "use_fakeyaw_transform",
            default_value=fakeyaw_default,
            description=(
                "是否启用 fake_vel_transform（fakeyaw 迁移链路）。True 时 Nav2 输出 "
                "cmd_vel_nav_fake，经 fake 转换成为 cmd_vel_nav_body 后进入 mux。"
                "默认按 params_file 文件名自动选择（含 fakeyaw 即为 True）。"
            ),
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "wz_control_mode",
            default_value=PythonExpression([
                "'nav' if ", fakeyaw_default, " else 'rotation'",
            ]),
            description=(
                "wz 所有权：nav = 导航拥有 wz（mux 用 nav_se2，拒绝 rotation 输入）；"
                "rotation = 旧链路（wz 来自 rotation_velocity，导航 angular.z 被丢弃）。"
            ),
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "command_frame",
            default_value=PythonExpression([
                "'base_link_fake' if ", fakeyaw_default, " else 'base_link'",
            ]),
            description=(
                "MINCO 速度命令坐标系。base_link = 控制器做一次 R(-psi)；"
                "base_link_fake = 输出 odom 对齐的平移速度，由 fake_vel_transform 做唯一转换。"
                "必须与参数文件里的 FollowPath.command_frame 一致（不一致会直接报错）。"
            ),
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "cmd_vel_nav_topic",
            default_value=PythonExpression([
                "'", FAKEYAW_NAV_TOPIC, "' if ", fakeyaw_default, " else 'cmd_vel_nav'",
            ]),
            description=(
                "导航链路最终速度出口话题（fakeyaw 链上是 fake_vel_transform 的输入）。"
                "必须与参数文件里 fake_vel_transform.input_cmd_vel_topic 一致。"
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
    ld.add_action(
        DeclareLaunchArgument(
            "start_simulation",
            default_value="False",
            description="是否由本入口一并启动 Gazebo 与 SRM 模型（默认由启动脚本单独拉起）",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "start_chassis_control",
            default_value="True",
            description="是否启动 srm_cmd_mux 与独立自转控制链路",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "start_rotation_sender",
            default_value="True",
            description=(
                "是否启动独立自转测试发送器。fakeyaw 迁移链（use_fakeyaw_transform=True）"
                "会强制关掉它：导航拥有 wz，mux 也会拒绝 rotation 输入。"
            ),
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "rotation_mode",
            default_value="stop",
            description="stop | constant | periodic",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "rotation_speed",
            default_value="0.0",
            description="恒速模式的有符号角速度（rad/s）",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "rotation_offset", default_value="0.0", description="周期模式平均角速度（rad/s）"
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "rotation_amplitude", default_value="0.0", description="周期模式变化幅度（rad/s）"
        )
    )
    ld.add_action(
        DeclareLaunchArgument("rotation_period", default_value="4.0", description="周期（s）")
    )
    ld.add_action(
        DeclareLaunchArgument("rotation_phase", default_value="0.0", description="初相位（rad）")
    )
    ld.add_action(
        DeclareLaunchArgument(
            "rotation_sine_wave",
            default_value="true",
            description="true 为正弦，false 为方波",
        )
    )

    # Normalize a directory supplied as `map:=...` before map_server is configured.
    ld.add_action(OpaqueFunction(function=_normalize_map_argument))

    # 启动时核对并打印最终速度链与 TF owner（方案 §4.4 的启动日志要求）。
    # 放在最前面：链路配错时第一条日志就说清楚，而不是等 10 s 后 mux 报"导航输入超时"。
    def _speed_chain_report(context, _mux_nav_topic):
        return _report_speed_chain(
            context,
            context.launch_configurations.get("params_file", ""),
            context.launch_configurations.get("use_fakeyaw_transform", "False"),
            context.launch_configurations.get("wz_control_mode", "rotation"),
            context.launch_configurations.get("command_frame", "base_link"),
            context.launch_configurations.get("cmd_vel_nav_topic", ""),
            # 用与 include 完全同一个替换求值，避免"核对的是另一套表达式"。
            # perform_substitutions 只接受替换列表，单个替换要先归一化。
            perform_substitutions(
                context, normalize_to_list_of_substitutions(_mux_nav_topic)
            ),
        )

    ld.add_action(
        OpaqueFunction(
            function=lambda context: _speed_chain_report(context, mux_nav_topic)
        )
    )

    ld.add_action(simulation_cmd)
    ld.add_action(start_velodyne_convert_tool)
    ld.add_action(start_simulation_odometry)
    ld.add_action(bringup_cmd)
    ld.add_action(chassis_control_cmd)
    ld.add_action(rviz_cmd)

    return ld
