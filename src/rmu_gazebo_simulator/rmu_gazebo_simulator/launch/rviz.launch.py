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

# 导入 os，用于拼接 RViz 配置文件路径。
import os

# 获取 ROS 2 功能包安装后的 share 目录。
from ament_index_python.packages import get_package_share_directory
# 保存启动动作。
from launch import LaunchDescription
# 启动 RViz 节点。
from launch_ros.actions import Node


def generate_launch_description():
    # 启动 RViz，并加载预先保存的显示配置。
    start_rviz2 = Node(
        # RViz 所属功能包和可执行程序。
        package="rviz2",
        executable="rviz2",
        # 节点名称。
        name="rviz2",
        # 将 RViz 放入指定机器人的命名空间。
        namespace="red_standard_robot1",
        # -d 后面指定 RViz 配置文件。
        arguments=[
            "-d",
            os.path.join(
                # 从功能包 share 目录定位配置文件。
                get_package_share_directory("rmu_gazebo_simulator"),
                "rviz",
                "visualize.rviz",
            ),
        ],
        # 将绝对 /tf 话题映射到当前命名空间下的相对话题。
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    # 创建启动描述。
    ld = LaunchDescription()

    # 注册 RViz 启动动作。
    ld.add_action(start_rviz2)

    # 返回启动描述给 ROS 2 launch 系统。
    return ld
