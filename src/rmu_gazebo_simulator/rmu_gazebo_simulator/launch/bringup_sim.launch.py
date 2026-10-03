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

# 导入 os，用于拼接文件路径。
import os

# 导入 yaml，用于读取世界配置文件。
import yaml
# 导入获取 ROS 2 功能包 share 目录的工具。
from ament_index_python.packages import get_package_share_directory
# 导入 LaunchDescription，用于保存启动动作。
from launch import LaunchDescription
# 导入声明启动参数和包含子 launch 文件的动作。
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
# 导入条件判断动作。
from launch.conditions import IfCondition
# 导入 Python launch 文件的数据源类型。
from launch.launch_description_sources import PythonLaunchDescriptionSource
# 导入读取 launch 参数的配置对象。
from launch.substitutions import LaunchConfiguration


# ROS 2 launch 系统调用此函数来构建总启动描述。
def generate_launch_description():
    # 获取 rmu_gazebo_simulator 安装后的 share 目录。
    pkg_simulator = get_package_share_directory("rmu_gazebo_simulator")

    # 获取 use_referee 启动参数的运行时配置值。
    use_referee = LaunchConfiguration("use_referee")
    # 声明是否启动裁判系统的 launch 参数。
    declare_use_referee = DeclareLaunchArgument(
        # 参数名称。
        "use_referee",
        # 默认不启动裁判系统。
        default_value="False",
        # 参数说明。
        description="Whether to start the referee system (not needed for navigation)",
    )

    # 拼接 Gazebo 世界选择配置文件的完整路径。
    gz_world_path = os.path.join(pkg_simulator, "config", "gz_world.yaml")
    # 打开世界配置文件，退出 with 代码块时自动关闭文件。
    with open(gz_world_path) as file:
        # 将 YAML 文件解析为 Python 字典。
        config = yaml.safe_load(file)
        # 读取 world 字段，例如 rmuc_2025。
        selected_world = config.get("world")

    # 拼接当前世界对应的 SDF 文件路径。
    world_sdf_path = os.path.join(
        # 依次组合功能包目录、资源目录、世界目录和世界文件名。
        pkg_simulator, "resource", "worlds", f"{selected_world}_world.sdf"
    )
    # 拼接 Gazebo 图形界面配置文件的路径。
    ign_config_path = os.path.join(pkg_simulator, "resource", "ign", "gui.config")

    # 创建 Gazebo 子 launch 的包含动作。
    gazebo_launch = IncludeLaunchDescription(
        # 指定 Gazebo 子 launch 的 Python 文件。
        PythonLaunchDescriptionSource(
            # 拼接 gazebo.launch.py 的完整路径。
            os.path.join(pkg_simulator, "launch", "gazebo.launch.py")
        ),
        # 将世界文件和界面配置路径传给 Gazebo 子 launch。
        launch_arguments={
            # 传入 SDF 世界文件路径。
            "world_sdf_path": world_sdf_path,
            # 传入 Gazebo 界面配置文件路径。
            "ign_config_path": ign_config_path,
        # 将参数字典转换为 launch 需要的键值对迭代器。
        }.items(),
    )

    # 创建机器人生成子 launch 的包含动作。
    spawn_robots_launch = IncludeLaunchDescription(
        # 指定机器人生成子 launch 的 Python 文件。
        PythonLaunchDescriptionSource(
            # 拼接 spawn_robots.launch.py 的完整路径。
            os.path.join(pkg_simulator, "launch", "spawn_robots.launch.py")
        ),
        # 将世界配置路径和世界名称传给机器人生成子 launch。
        launch_arguments={
            # 传入 gz_world.yaml 的路径。
            "gz_world_path": gz_world_path,
            # 传入当前选择的世界名称。
            "world": selected_world,
        # 将参数字典转换为 launch 需要的键值对迭代器。
        }.items(),
    )

    # 创建裁判系统子 launch 的包含动作。
    referee_system_launch = IncludeLaunchDescription(
        # 指定裁判系统子 launch 的 Python 文件。
        PythonLaunchDescriptionSource(
            # 拼接 referee_system.launch.py 的完整路径。
            os.path.join(pkg_simulator, "launch", "referee_system.launch.py")
        ),
        # 仅当 use_referee 为 True 时启动裁判系统。
        condition=IfCondition(use_referee),
    )

    # 创建空的启动描述，用于按顺序添加各项动作。
    ld = LaunchDescription()

    # 注册 use_referee 参数声明。
    ld.add_action(declare_use_referee)
    # 注册 Gazebo 启动动作。
    ld.add_action(gazebo_launch)
    # 注册机器人生成、传感器和 bridge 启动动作。
    ld.add_action(spawn_robots_launch)
    # 注册条件启动的裁判系统动作。
    ld.add_action(referee_system_launch)

    # 返回完整启动描述，交给 ROS 2 launch 执行。
    return ld
