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

"""SRM 仿真入口：世界 + SRM 模型 + 传感器桥接 + 速度执行。

改动要点（相对旧步兵的 bringup_sim.launch.py）：

* 机器人模型由 srm27_robot_description 按几何 YAML 现场生成，只含 SRM 圆柱底盘、
  轮组、MID360 和底盘 IMU；没有装甲、灯条、射击、云台关节链。
* 底盘速度执行走 srm_velocity_system 插件 + srm_velocity_adapter 适配器，
  不再使用 MecanumDrive2 / rmoss_gz_base 的底盘与云台控制器。
* 世界、初始位姿和速度参数统一来自 config/srm_sim.yaml。

用法：
  ros2 launch srm27_gazebo_simulator srm_sim.launch.py
  ros2 launch srm27_gazebo_simulator srm_sim.launch.py world:=srm_empty
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString

from srm27_robot_description.model_builder import build_sdf, build_urdf


def _load_config(pkg_share, config_file=""):
    path = config_file or os.path.join(pkg_share, "config", "srm_sim.yaml")
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def _resolve_world_sdf(pkg_share, config, world_override="", world_sdf_override=""):
    """按配置解析本包世界 SDF：先找 worlds/，再找 resource/worlds/。"""
    if world_sdf_override:
        if not os.path.isfile(world_sdf_override):
            raise FileNotFoundError(f"world_sdf 不存在: {world_sdf_override}")
        return world_sdf_override
    if config.get("world_sdf"):
        candidate = config["world_sdf"]
        if not os.path.isfile(candidate):
            raise FileNotFoundError(f"world_sdf 不存在: {candidate}")
        return candidate

    world = world_override or config.get("world", "rmuc_2025")
    local = os.path.join(pkg_share, "worlds", f"{world}.sdf")
    if os.path.isfile(local):
        return local

    arena = os.path.join(pkg_share, "resource", "worlds", f"{world}_world.sdf")
    if os.path.isfile(arena):
        return arena

    raise FileNotFoundError(
        f"找不到世界 '{world}'：既没有 {local}，也没有 {arena}"
    )


def _launch_setup(context):
    pkg_share = get_package_share_directory("srm27_gazebo_simulator")
    config_file = LaunchConfiguration("config_file").perform(context)
    config = _load_config(pkg_share, config_file)

    world_sdf = _resolve_world_sdf(
        pkg_share,
        config,
        world_override=LaunchConfiguration("world").perform(context),
        world_sdf_override=LaunchConfiguration("world_sdf").perform(context),
    )
    gui_config = config.get("gui_config") or os.path.join(
        pkg_share,
        "resource",
        "ign",
        "gui.config",
    )

    gui = LaunchConfiguration("gui").perform(context).lower() in ("1", "true", "yes")
    run_immediately = LaunchConfiguration("run_immediately").perform(
        context
    ).lower() in ("1", "true", "yes")

    # Gazebo 默认以暂停状态启动；-r 表示直接开始运行，-s 表示只跑 server。
    gz_flags = "" if gui else " -s"
    if run_immediately:
        gz_flags += " -r"

    robot = config["robot"]
    robot_name = LaunchConfiguration("robot_name").perform(context) or robot["name"]
    velocity = config["velocity"]

    # 机器人模型与 URDF 由同一份几何 YAML 生成，仿真与实车不会各自漂移。
    # 云台自转角速度：小陀螺时由启动脚本给非零值，仿真器每步直接施加。
    gimbal_spin_rate = float(
        LaunchConfiguration("gimbal_spin_rate").perform(context) or 2.0
    )
    gimbal_driver = LaunchConfiguration("gimbal_driver").perform(context) or "system"
    if gimbal_driver not in ("system", "external"):
        raise RuntimeError(f"gimbal_driver 只能是 system/external，收到 {gimbal_driver!r}")
    # 打印生效的云台转速：这条日志是"云台为什么转/不转"的第一现场证据。
    from launch.actions import LogInfo  # noqa: PLC0415 - 与文件内其它 import 风格一致
    if gimbal_driver == "external":
        gimbal_note = (
            f"云台转速交给外部命令（LIO 模式）：gimbal_spin_gate 会在 Point-LIO 稳定后"
            f"按 {gimbal_spin_rate} rad/s 启动，之前保持锁定"
        )
    elif abs(gimbal_spin_rate) > 0.0:
        gimbal_note = f"云台匀速自转 {gimbal_spin_rate} rad/s（仿真器直接施加，雷达随之转动）"
    else:
        gimbal_note = "云台锁定（转速 0）：不会自转，只跟随底盘"
    launch_note = LogInfo(msg=f"[云台] {gimbal_note}")
    robot_sdf = build_sdf(
        velocity_topic=velocity["gz_command_topic"],
        odometry_topic=velocity["odom_topic"],
        command_timeout=float(velocity["command_timeout"]),
        gimbal_spin_rate=gimbal_spin_rate,
        gimbal_driver=gimbal_driver,
    )
    robot_urdf = build_urdf()

    bridge_config = os.path.join(pkg_share, "config", "ros_gz_bridge.yaml")
    bridged = ReplaceString(
        source_file=bridge_config, replacements={"<robot_name>": robot_name}
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        launch_arguments={
            "gz_version": "6",
            "gz_args": [
                TextSubstitution(text=gz_flags),
                TextSubstitution(text=" "),
                TextSubstitution(text=world_sdf),
                TextSubstitution(text=" --gui-config "),
                TextSubstitution(text=gui_config),
            ],
        }.items(),
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-string",
            robot_sdf,
            "-name",
            robot_name,
            "-allow_renaming",
            "true",
            "-x",
            str(robot["x"]),
            "-y",
            str(robot["y"]),
            "-z",
            str(robot["z"]),
            "-Y",
            str(robot["yaw"]),
        ],
        output="screen",
    )

    # 场地世界使用 level performer，需要把新生成的模型设为当前关注的 performer。
    set_performer_service = ExecuteProcess(
        cmd=[
            "ign",
            "service",
            "-s",
            "/world/default/level/set_performer",
            "--reqtype",
            "ignition.msgs.StringMsg",
            "--reptype",
            "ignition.msgs.Boolean",
            "--timeout",
            "2000",
            "--req",
            f'data: "{robot_name}"',
        ],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=robot_name,
        output="screen",
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        parameters=[
            {"use_sim_time": True, "robot_description": robot_urdf},
        ],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        namespace=robot_name,
        parameters=[{"config_file": bridged}],
        output="screen",
    )

    # 速度适配器：校验 cmd_vel_sim 并转发到 Gazebo Transport，带单调时钟超时清零。
    velocity_adapter = Node(
        package="srm27_gazebo_simulator",
        executable="srm_velocity_adapter",
        name="srm_velocity_adapter",
        namespace=robot_name,
        output="screen",
        parameters=[
            {
                "robot_name": robot_name,
                "cmd_vel_topic": velocity["cmd_vel_topic"],
                "gz_command_topic": velocity["gz_command_topic"],
                "command_timeout": float(velocity["command_timeout"]),
                "publish_rate": float(velocity["publish_rate"]),
            }
        ],
    )

    return [
        launch_note,
        gazebo,
        spawn_robot,
        set_performer_service,
        robot_state_publisher,
        bridge,
        velocity_adapter,
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory("srm27_gazebo_simulator")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=os.path.join(pkg_share, "config", "srm_sim.yaml"),
                description="仿真入口配置（世界、初始位姿与速度参数）",
            ),
            DeclareLaunchArgument(
                "gimbal_driver",
                default_value="system",
                description=(
                    "云台转速由谁决定：system=仿真插件按 gimbal_spin_rate 直接驱动（默认）；"
                    "external=不驱动，改由 ROS 话题 /cmd_gimbal_vel + gz JointController 控制"
                    "（LIO 模式用，由 gimbal_spin_gate 在 Point-LIO 稳定后才发）"
                ),
            ),
            DeclareLaunchArgument(
                "gimbal_spin_rate",
                default_value="2.0",
                description=(
                    "云台匀速自转角速度 (rad/s, 负值反向, 0=不转)。默认 2.0 = 起来就一直转，"
                    "与有没有导航目标无关；由 SrmVelocitySystem 每个物理步直接施加。"
                    "要停就用 gimbal_spin_rate:=0。"
                ),
            ),

            DeclareLaunchArgument(
                "robot_name",
                default_value="",
                description="覆盖配置中的机器人名（同时作为命名空间与 Gazebo 模型名）",
            ),
            DeclareLaunchArgument(
                "world",
                default_value="",
                description="覆盖 config/srm_sim.yaml 中的世界名",
            ),
            DeclareLaunchArgument(
                "world_sdf",
                default_value="",
                description="直接指定世界 SDF 绝对路径，优先级最高",
            ),
            DeclareLaunchArgument(
                "gui", default_value="true", description="是否启动 Gazebo GUI"
            ),
            DeclareLaunchArgument(
                "run_immediately",
                default_value="false",
                description="true 时 Gazebo 直接开始运行（-r），否则保持暂停等待手动播放",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
