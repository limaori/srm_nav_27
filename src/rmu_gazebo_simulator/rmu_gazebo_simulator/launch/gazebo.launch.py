# Copyright 2025 Lihan Chen
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

# 导入 os，用于拼接功能包内的文件路径。
import os

# 获取 ROS 2 功能包安装后的 share 目录。
from ament_index_python.packages import get_package_share_directory
# 保存启动动作的容器。
from launch import LaunchDescription
# 声明 launch 参数，以及包含其他 launch 文件的动作。
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
# 指定 Python launch 文件的数据源。
from launch.launch_description_sources import PythonLaunchDescriptionSource
# 读取 launch 参数，并构造文本替换值。
from launch.substitutions import LaunchConfiguration, TextSubstitution
# 启动 ROS 2 节点。
from launch_ros.actions import Node


def generate_launch_description():
    # 查找本功能包安装后的 share 目录。
    pkg_simulator = get_package_share_directory("rmu_gazebo_simulator")

    # 获取运行时传入的世界文件和 GUI 配置文件路径。
    world_sdf_path = LaunchConfiguration("world_sdf_path")
    ign_config_path = LaunchConfiguration("ign_config_path")

    # 声明世界 SDF 路径参数；若父 launch 没有传值，则使用默认世界。
    declare_world_sdf_path = DeclareLaunchArgument(
        # 参数名。
        "world_sdf_path",
        # 默认加载 2024 场地世界文件。
        default_value=os.path.join(
            pkg_simulator, "resource", "worlds", "rmul_2024_world.sdf"
        ),
        # 参数说明。
        description="Path to the world SDF file",
    )

    # 声明 Ignition Gazebo 图形界面配置路径参数。
    declare_ign_config_path = DeclareLaunchArgument(
        # 参数名。
        "ign_config_path",
        # 默认使用本功能包提供的 GUI 配置。
        default_value=os.path.join(pkg_simulator, "resource", "ign", "gui.config"),
        # 参数说明。
        description="Path to the Ignition Gazebo GUI configuration file",
    )

    # 包含 ros_gz_sim 提供的通用 Gazebo 启动文件。
    gazebo = IncludeLaunchDescription(
        # 找到 ros_gz_sim 包中的 gz_sim.launch.py。
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        # 将版本、世界文件和 GUI 配置传递给 Gazebo 启动文件。
        launch_arguments={
            # 使用 Ignition/Gazebo Fortress 对应的版本号 6。
            "gz_version": "6",
            # gz_args 是传给 Gazebo 的命令行参数列表。
            "gz_args": [
                # 要加载的 SDF 世界文件。
                world_sdf_path,
                # 指定下面的文件作为 Gazebo GUI 配置。
                TextSubstitution(text=" --gui-config "),
                # GUI 配置文件路径。
                ign_config_path,
            ],
        # launch_arguments 要求传入键值对迭代器。
        }.items(),
    )

    # 启动 ros_gz_bridge，将 Gazebo 时钟转换为 ROS 2 /clock 话题。
    robot_ign_bridge = Node(
        # 节点所在功能包。
        package="ros_gz_bridge",
        # 节点可执行程序。
        executable="parameter_bridge",
        # 使用桥接语法声明 ROS 2 和 Gazebo 的消息类型及方向。
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
    )

    # 创建总启动描述，并按顺序加入各项动作。
    ld = LaunchDescription()

    # 注册世界文件路径参数。
    ld.add_action(declare_world_sdf_path)
    # 注册 GUI 配置路径参数。
    ld.add_action(declare_ign_config_path)
    # 注册 Gazebo 启动动作。
    ld.add_action(gazebo)
    # 注册 /clock 桥接节点。
    ld.add_action(robot_ign_bridge)

    # 将启动描述返回给 ROS 2 launch 系统执行。
    return ld
