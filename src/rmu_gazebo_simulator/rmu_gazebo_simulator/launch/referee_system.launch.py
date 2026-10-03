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

# 导入 os，用于拼接配置文件路径。
import os

# 获取 ROS 2 功能包安装后的 share 目录。
from ament_index_python.packages import get_package_share_directory
# 保存要执行的 launch 动作。
from launch import LaunchDescription
# 用于启动 ROS 2 节点。
from launch_ros.actions import Node


def generate_launch_description():
    # 查找本功能包安装后的资源目录。
    pkg_simulator = get_package_share_directory("rmu_gazebo_simulator")

    # 拼接裁判系统参数文件的完整路径。
    referee_config_path = os.path.join(
        pkg_simulator, "config", "referee_system_1v1.yaml"
    )

    # 桥接 Gazebo 中的攻击和射击信息到 ROS 2。
    referee_ign_bridge = Node(
        # 使用 ros_gz_bridge 提供的通用参数桥。
        package="ros_gz_bridge",
        executable="parameter_bridge",
        # 将相关话题放在 referee_system 命名空间下。
        namespace="referee_system",
        # 语法：ROS 话题@ROS 类型[Gazebo 类型，左括号表示 GZ 到 ROS。
        arguments=[
            "/referee_system/attack_info@std_msgs/msg/String[ignition.msgs.StringMsg",
            "/referee_system/shoot_info@std_msgs/msg/String[ignition.msgs.StringMsg",
        ],
    )

    # 启动位姿桥，将 Gazebo 中的机器人位姿转换为裁判系统可用的 ROS 话题。
    referee_ign_pose_bridge = Node(
        package="rmoss_gz_bridge",
        executable="pose_bridge",
        namespace="referee_system",
    )

    # 启动 RFID 桥，处理 Gazebo 与 ROS 之间的 RFID 信息。
    referee_ign_rfid_bridge = Node(
        package="rmoss_gz_bridge",
        executable="rfid_bridge",
        namespace="referee_system",
    )

    # 启动本功能包的一对一比赛裁判节点。
    referee_system = Node(
        package="rmu_gazebo_simulator",
        executable="simple_competition_1v1.py",
        namespace="referee_system",
        # 读取最大血量、初始弹药和初始资源等参数。
        parameters=[referee_config_path],
    )

    # 创建总启动描述。
    ld = LaunchDescription()

    # 按顺序注册桥接节点和裁判节点。
    ld.add_action(referee_ign_bridge)
    ld.add_action(referee_ign_pose_bridge)
    ld.add_action(referee_ign_rfid_bridge)
    ld.add_action(referee_system)

    # 返回启动描述，由 ROS 2 launch 系统执行。
    return ld
