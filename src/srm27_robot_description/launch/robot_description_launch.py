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

"""发布 SRM 的 robot_description / 静态 TF。

URDF 由 config/srm27_sentry_geometry.yaml 现场生成，与 Gazebo 使用的 SDF
同源，避免仿真与实车两份模型各自修改。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from srm27_robot_description.model_builder import build_urdf


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")

    robot_description = build_urdf()

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="", description="机器人命名空间"),
            DeclareLaunchArgument("use_sim_time", default_value="False"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                namespace=namespace,
                output="screen",
                remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "robot_description": robot_description,
                    }
                ],
            ),
        ]
    )
