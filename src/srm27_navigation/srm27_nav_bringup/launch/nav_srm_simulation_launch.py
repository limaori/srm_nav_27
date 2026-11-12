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

数据流（**唯一**链路：fakeyaw 迁移 + 云台系，`wz` 与导航完全解耦）::

    MINCO/Nav2（gimbal_yaw_fake，只出 vx/vy）──cmd_vel_nav_fake──► fake_vel_transform
        ──cmd_vel_nav_body──► srm_cmd_mux ◄── rotation_cmd（决策层底盘 wz）
                                  └──cmd_vel_sim──► 适配器 ──► Gazebo
                                                     （插件按云台关节角把 vx/vy 旋到底盘）

2026-11-12：仿真与实车链对齐（云台系）。导航的"车"是云台 `gimbal_yaw`：
`odom -> gimbal_yaw` 只依赖固定外参，TF 树反过来（URDF `gimbal_is_root:=true`）。
导航**只出平移**（`yaw_policy.mode=xy_only`），底盘 `wz` 由决策层发
`rotation_cmd`，在 `srm_cmd_mux` 里合成 —— 不经过 MINCO，也不经过平滑器。
云台系导航必须走 fakeyaw 转换（`use_fakeyaw_transform:=False` 会直接报错）。

两个开关必须与参数文件一致，本入口按 `params_file` 的文件名给出默认值，并在启动时
核对/打印最终速度链：

* `use_fakeyaw_transform`：是否启用 fake_vel_transform（唯一链路，必须为 True）。
* `command_frame`：MINCO 的速度命令坐标系；本链路是 `gimbal_yaw_fake`。

其余约定：

* 复用现有点云转换、地形分析、定位与 Nav2 启动链路。机器人参考系统一为
  gimbal_yaw_fake（与 odom 平行、朝向稳定）；定位与里程计用真实云台 gimbal_yaw。
* 速度出口由 `cmd_vel_nav_topic` 决定，再由 srm_cmd_mux 合成 cmd_vel_sim；底盘执行由
  srm27_gazebo_simulator 的 SRM 速度插件负责（`velocity_command_frame:=gimbal`）。
* 通过 start_simulation / start_chassis_control 选择是否由本入口一并拉起仿真与
  底盘控制链路；默认由 script/start_sim_nav.sh 分标签页启动。

用法：
  ros2 launch srm27_nav_bringup nav_srm_simulation_launch.py \\
    map:=.../rmuc_2025_tunnel.yaml

  # 云台系迁移链路（空场 + 真值里程计）
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
from launch_ros.descriptions import ParameterValue
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


def _report_speed_chain(context, params_file, use_fakeyaw, command_frame,
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
    if not _as_bool(use_fakeyaw):
        # 独立自转链已于 2026-10-19 删除：没有 fakeyaw 就没有合法的仿真导航链路
        # （导航输出会在 base_link 系里含 wz，而 base_link 随底盘自转，代价地图与路径
        # 会被自转拖着转），所以这里直接失败而不是退回旧行为。
        raise RuntimeError(
            "use_fakeyaw_transform=False 已不支持：独立自转链（rotation_controller / "
            "rotation_velocity）已删除，仿真导航必须走 fakeyaw 迁移链。"
            "请用 params_file:=.../nav2_params_srm_fakeyaw.yaml。"
        )
    if command_frame != "gimbal_yaw_fake":
        raise RuntimeError(
            f"fakeyaw 链上 command_frame 必须是 gimbal_yaw_fake，实际是 {command_frame!r}："
            "速度必须先由 fake_vel_transform 转一次，控制器再转一次就是重复旋转。"
            "（2026-11-12 云台系：导航参考系是云台，不再是 base_link_fake。）"
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

    if not fake_section:
        warnings.append(
            "[速度链] 参数文件里没有 fake_vel_transform 段："
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
                f"[速度链] fake_vel_transform.transform_mode={mode}，本链路应为 nav_se2"
                "（legacy 会叠加 cmd_spin，造成角速度双重叠加）。"
            )
    if isinstance(follow_path, dict):
        configured = str(follow_path.get("command_frame", ""))
        if configured and configured != command_frame:
            raise RuntimeError(
                "参数文件里 FollowPath.command_frame="
                f"{configured} 与 launch 的 command_frame={command_frame} 不一致："
                "两者必须同时是 gimbal_yaw_fake（fake_vel_transform 转一次），"
                "否则会重复旋转或完全不旋转。"
            )
    messages.append(
        "[速度链] 导航 → fakeyaw 转换（云台系；导航只出平移，wz 归决策层）:\n"
        f"    Nav2[{cmd_vel_nav_topic}] → fake_vel_transform"
        f"[{mux_nav_topic}] → srm_cmd_mux ◄─ rotation_cmd（决策层 wz）"
        " → cmd_vel_sim → Gazebo\n"
        "    TF owner: odom->gimbal_yaw = 定位/真值里程计；"
        "gimbal_yaw->gimbal_yaw_fake = fake_vel_transform"
    )

    messages.append(
        f"[速度链] 参数: use_fakeyaw_transform={use_fakeyaw}, command_frame={command_frame}"
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
    # 里程计前端（方案 §6.2）：point_lio（默认 = 旧行为）或 zlio。只透传给 nav2_stack，
    # 分叉点在 localization_launch / navigation_launch。
    odometry_frontend = LaunchConfiguration("odometry_frontend")
    use_velocity_smoother = LaunchConfiguration("use_velocity_smoother")
    cmd_vel_nav_topic = LaunchConfiguration("cmd_vel_nav_topic")
    map_to_odom_x = LaunchConfiguration("map_to_odom_x")
    map_to_odom_y = LaunchConfiguration("map_to_odom_y")
    map_to_odom_yaw = LaunchConfiguration("map_to_odom_yaw")

    use_fakeyaw_transform = LaunchConfiguration("use_fakeyaw_transform")
    command_frame = LaunchConfiguration("command_frame")

    start_simulation = LaunchConfiguration("start_simulation")
    start_chassis_control = LaunchConfiguration("start_chassis_control")

    # 空场调试用的真值里程计：三种定位模式都不启用时，由仿真真值提供
    # odom -> gimbal_yaw。三种定位模式下由 Point-LIO / GICP / SLAM 提供。
    use_ground_truth_odometry = PythonExpression([
        "not ", slam, " and not ", use_pcd_localization,
        " and not ", use_lio_odometry,
    ])

    # mux 的 nav_topic 固定是 fake 转换器的输出（唯一链路）。
    mux_nav_topic = PythonExpression(["'", FAKEYAW_MUX_TOPIC, "'"])
    # 决策层底盘自转输入（只取 angular.z）：与实车链同一个话题名。
    mux_spin_topic = LaunchConfiguration("mux_spin_topic")

    # 航向策略（2026-11-12 云台系）：导航**只出平移**，底盘 wz 由决策层
    # rotation_cmd -> srm_cmd_mux 给，完全和导航解耦。
    #   mux_spin_topic 非空（默认 rotation_cmd）-> 固定 xy_only：mux 的 nav 输入
    #     永远忽略 angular.z，se2/spin 在本入口不可能生效。
    #   mux_spin_topic 留空（单输入回退旧行为）-> 才按 yaw_mode run（A/B 对照用）。
    yaw_mode_arg = LaunchConfiguration("yaw_mode")
    effective_yaw_mode = PythonExpression([
        "'xy_only' if '", mux_spin_topic, "' != '' else ('",
        yaw_mode_arg, "' if '", yaw_mode_arg, "' not in ('auto', 'se2') "
        "else 'xy_only')",
    ])

    # 云台自转：auto = 跟随航向策略（仅在 mux 单输入 + spin 的 A/B 下 = 2.0，其余 0）。
    gimbal_rate_arg = LaunchConfiguration("gimbal_spin_rate")
    effective_gimbal_rate = PythonExpression([
        "'2.0' if ('", gimbal_rate_arg, "' == 'auto' and '", effective_yaw_mode,
        "' == 'spin') else ('0.0' if '", gimbal_rate_arg, "' == 'auto' else '",
        gimbal_rate_arg, "')",
    ])

    # 判停器：导航只出平移（xy_only），到点时底盘可能仍在自转（决策层小陀螺驻车），
    # 带角速度判停的 StoppedGoalChecker 永远判不了到点 —— 一律用只看平移的
    # OmniStoppedGoalChecker（与实车链一致）。
    effective_goal_checker = PythonExpression([
        "'srm27_nav_plugins::OmniStoppedGoalChecker'",
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

    # Nav2 栈的参数文件：在通用文件之上注入航向策略与判停器（点分全路径，
    # 见 nav2_common RewrittenYaml 的 substitute_params"total path"分支）。
    # root_key 留空：nav2_stack 内部的 RewrittenYaml 会再做命名空间包装，这里先包
    # 会双重嵌套导致参数加载不到。实车链（start_real_nav.sh）直接走
    # nav2_stack_launch、不经过本包装，其 yaw_policy/判停器不受影响。
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
        launch_arguments={
            "gimbal_spin_rate": effective_gimbal_rate,
            # 云台系：vx/vy 按云台系解释（插件按关节角旋到底盘），真值里程计发 odom->gimbal_yaw。
            "velocity_command_frame": "gimbal",
            "odom_link": "gimbal_yaw",
        }.items(),
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
            # 云台系导航：真值里程计发布 odom -> gimbal_yaw（与导航参考系一致）。
            # z 由节点按云台高度（chassis_top）补偿回地面，下游 z 阈值不用改。
            {"base_frame": "gimbal_yaw"},
        ],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    # 3.5) LIO 模式专属：云台自转门控（等 Point-LIO 稳定后才开始转）。
    # 真值里程计模式不需要它 —— 那时转速由 SDF 直接交给仿真插件，起来就转。
    start_gimbal_spin_gate = Node(
        # IfCondition 接受 'True'/'true'/'1'，与脚本传入的写法兼容。
        condition=IfCondition(use_lio_odometry),
        package="srm27_nav_bringup",
        executable="gimbal_spin_gate.py",
        name="gimbal_spin_gate",
        output="screen",
        namespace=namespace,
        parameters=[
            {"use_sim_time": use_sim_time},
            # 必须显式声明类型：gimbal_spin_rate 可能是 'auto'，直接塞字符串会让
            # 节点的 double 参数声明抛 InvalidParameterTypeException（实测节点当场退出）。
            {
                "spin_rate": ParameterValue(
                    effective_gimbal_rate, value_type=float
                )
            },
            {"odometry_topic": "lidar_odometry"},
            {"command_topic": "cmd_gimbal_vel"},
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
            "params_file": nav_stack_params,
            "speed_limits_file": speed_limits_file,
            "autostart": autostart,
            "use_composition": use_composition,
            "use_respawn": use_respawn,
            "use_pcd_localization": use_pcd_localization,
            "use_lio_odometry": use_lio_odometry,
            "odometry_frontend": odometry_frontend,
            # 迁移链路：fake_vel_transform 夹在 Nav2 输出与 mux 之间（方案 §4.4）。
            "use_fake_vel_transform": use_fakeyaw_transform,
            "use_velocity_smoother": use_velocity_smoother,
            "cmd_vel_nav_topic": cmd_vel_nav_topic,
            "map_to_odom_x": map_to_odom_x,
            "map_to_odom_y": map_to_odom_y,
            "map_to_odom_yaw": map_to_odom_yaw,
        }.items(),
    )

    # 5) 速度出口（mux 是 cmd_vel_sim 的唯一发布者）
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
            # mux 的输入就是 fake_vel_transform 的输出（唯一链路）。
            "nav_topic": mux_nav_topic,
            # 决策层底盘自转（只取 angular.z）：与实车链同名同义。仿真里
            # --chassis-spin / script/spin_control.sh 发的就是这个话题。
            "spin_topic": mux_spin_topic,
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
            "yaw_mode",
            default_value="auto",
            description=(
                "MINCO 航向策略：云台系固定 xy_only（导航只出平移，底盘 wz 走 "
                "rotation_cmd -> srm_cmd_mux）。只有把 mux_spin_topic 留空（单输入回退）"
                "时才按本参数跑，用于 A/B 对照；判停器一律 OmniStoppedGoalChecker"
            ),
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
            "odometry_frontend",
            # 2026-11-05 切换：仿真链默认 ZLIO（这里改，导航栈/实车入口的默认值保持
            # point_lio —— 实车的 ZLIO 参数不同，见方案 §10，不该被仿真默认值带跑）。
            # 回退：odometry_frontend:=point_lio 或脚本开关 --lio-frontend point_lio。
            default_value="zlio",
            description=(
                "里程计前端: zlio (默认, z-localization 的 frontend 包) 或 point_lio (回退)。"
                "zlio 只在 use_lio_odometry:=True 时生效，并会同时起 zlio_bridge "
                "做坐标系改写与换代锚定。启动脚本开关: --lio-frontend"
            ),
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
            "gimbal_spin_rate",
            default_value="auto",
            description=(
                "云台匀速自转角速度 (rad/s, 负值反向, 0=不转)。auto = 跟随 yaw_mode："
                "spin 时 2.0，其它 0。由 SrmVelocitySystem 每个物理步直接施加，云台连同其上的"
                "雷达匀速转，Point-LIO 直接感知该自转。直接用 ros2 launch 起也不会忘记传。"
            ),
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
    # 唯一链路：默认恒为 True（独立自转链已删除，False 会在启动核对时直接报错）。
    fakeyaw_default = "True"
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
            "command_frame",
            default_value=PythonExpression([
                "'gimbal_yaw_fake' if ", fakeyaw_default, " else 'base_link'",
            ]),
            description=(
                "MINCO 速度命令坐标系。base_link = 控制器做一次 R(-psi)；"
                "gimbal_yaw_fake = 输出 odom 对齐的平移速度，由 fake_vel_transform 做唯一转换。"
                "必须与参数文件里的 FollowPath.command_frame 一致（不一致会直接报错）。"
            ),
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "mux_spin_topic",
            default_value="rotation_cmd",
            description=(
                "srm_cmd_mux 的第二路输入：决策层底盘自转（只取 angular.z）。"
                "与实车链同名同义；留空 = 单输入（旧行为，导航同时拥有 wz）。"
            ),
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "cmd_vel_nav_topic",
            default_value=FAKEYAW_NAV_TOPIC,
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
            description="是否启动速度出口 srm_cmd_mux",
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
            context.launch_configurations.get("use_fakeyaw_transform", "True"),
            context.launch_configurations.get("command_frame", "gimbal_yaw_fake"),
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
    ld.add_action(start_gimbal_spin_gate)
    ld.add_action(bringup_cmd)
    ld.add_action(chassis_control_cmd)
    ld.add_action(rviz_cmd)

    return ld
