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

数据流：

    Nav2 controller ──cmd_vel_nav──┐
                                   ├─ srm_cmd_mux ──cmd_vel_sim──► SRM 速度适配器
    rotation_test_sender ─ rotation_controller ─ rotation_velocity ┘

与旧步兵入口的区别：

* 复用现有点云转换、地形分析、定位与 Nav2 启动链路，但机器人参考系统一为真实
  随底盘自转的 base_link，关闭 fake_vel_transform，删除 gimbal_yaw_fake 链路。
* 速度出口为 cmd_vel_nav，由 srm_cmd_mux 与独立自转合成；底盘执行由
  srm27_gazebo_simulator 的 SRM 速度插件负责。
* 通过 start_simulation / start_chassis_control 选择是否由本入口一并拉起仿真与
  底盘控制链路；默认与旧流程一致，由 script/start_sim_nav.sh 分标签页启动。

用法：
  ros2 launch srm27_nav_bringup nav_srm_simulation_launch.py \\
    map:=.../rmuc_2025_tunnel.yaml \\
    rotation_mode:=constant rotation_speed:=1.0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


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

    start_simulation = LaunchConfiguration("start_simulation")
    start_chassis_control = LaunchConfiguration("start_chassis_control")
    start_rotation_sender = LaunchConfiguration("start_rotation_sender")

    # 空场调试用的真值里程计：三种定位模式都不启用时，由仿真真值提供
    # odom -> base_link。三种定位模式下由 Point-LIO / GICP / SLAM 提供。
    use_ground_truth_odometry = PythonExpression([
        "not ", slam, " and not ", use_pcd_localization,
        " and not ", use_lio_odometry,
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
            "autostart": autostart,
            "use_composition": use_composition,
            "use_respawn": use_respawn,
            "use_pcd_localization": use_pcd_localization,
            "use_lio_odometry": use_lio_odometry,
            "use_fake_vel_transform": "False",
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
            "params_file": os.path.join(
                get_package_share_directory("srm27_chassis_control"),
                "config",
                "srm_chassis_control.yaml",
            ),
            "start_rotation_sender": start_rotation_sender,
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
    ld.add_action(
        DeclareLaunchArgument(
            "cmd_vel_nav_topic",
            default_value="cmd_vel_nav",
            description="导航链路最终平移速度出口话题（mux 的导航输入）",
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
            description="是否启动独立自转测试发送器",
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

    ld.add_action(simulation_cmd)
    ld.add_action(start_velodyne_convert_tool)
    ld.add_action(start_simulation_odometry)
    ld.add_action(bringup_cmd)
    ld.add_action(chassis_control_cmd)
    ld.add_action(rviz_cmd)

    return ld
