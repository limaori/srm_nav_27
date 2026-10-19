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

"""速度出口链路（srm_cmd_mux）。

链路：
  <nav_topic> ─ srm_cmd_mux ── cmd_vel_sim

2026-10-19：独立自转链（rotation_controller / rotation_test_sender /
rotation_velocity）已删除。`wz` 只由上位机（导航栈）产生，本链路不再有第二路自转输入，
也没有 `wz_control_mode` 开关：mux 收到的导航 `angular.z` 就是最终执行的 wz。

`nav_topic` 由导航入口按链路给出：
  * fakeyaw 迁移链（唯一支持的链路）：cmd_vel_nav_body（fake_vel_transform 的输出）。

速度限幅（vx_max / vy_max / v_max / wz_max）与超时仍由
config/simulation/speed_limits.yaml 在启动时合并进来（唯一来源）。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_share = get_package_share_directory("srm27_chassis_control")

    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    nav_topic = LaunchConfiguration("nav_topic")

    # mux 的 nav_topic 也走 RewrittenYaml：配置文件里仍然写着这个键（"参数从哪来"在文件里
    # 可见），启动时按链路覆盖，避免"两处定义、一处静默生效"。
    configured = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={
                "use_sim_time": use_sim_time,
                "nav_topic": nav_topic,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    cmd_mux = Node(
        package="srm27_chassis_control",
        executable="cmd_mux",
        name="srm_cmd_mux",
        namespace=namespace,
        output="screen",
        parameters=[configured],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="srm_sentry"),
            DeclareLaunchArgument("use_sim_time", default_value="True"),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(
                    pkg_share, "config", "srm_chassis_control.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "nav_topic",
                default_value="cmd_vel_nav",
                description=(
                    "mux 的导航输入话题；fakeyaw 迁移链上是 fake_vel_transform 的输出 "
                    "cmd_vel_nav_body。"
                ),
            ),
            cmd_mux,
        ]
    )
