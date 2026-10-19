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

"""独立自转控制与速度合成链路。

链路：
  cmd_vel_nav ─┐
               ├─ srm_cmd_mux ── cmd_vel_sim
  rotation_cmd ─ rotation_controller ─ rotation_velocity ─┘

  rotation_test_sender ─ rotation_cmd

自转相关的波形参数全部由 launch 参数给出，速度统一使用
geometry_msgs/msg/Twist 的 angular.z，不新增消息类型。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_share = get_package_share_directory("srm27_chassis_control")

    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    start_rotation_sender = LaunchConfiguration("start_rotation_sender")

    rotation_mode = LaunchConfiguration("rotation_mode")
    rotation_speed = LaunchConfiguration("rotation_speed")
    rotation_offset = LaunchConfiguration("rotation_offset")
    rotation_amplitude = LaunchConfiguration("rotation_amplitude")
    rotation_period = LaunchConfiguration("rotation_period")
    rotation_phase = LaunchConfiguration("rotation_phase")
    rotation_sine_wave = LaunchConfiguration("rotation_sine_wave")

    configured = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={"use_sim_time": use_sim_time},
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

    rotation_controller = Node(
        package="srm27_chassis_control",
        executable="rotation_controller",
        name="rotation_controller",
        namespace=namespace,
        output="screen",
        parameters=[configured],
    )

    # 自转测试发送器：只写 rotation_cmd，不触碰导航话题。
    rotation_sender = Node(
        package="srm27_chassis_control",
        executable="rotation_test_sender",
        name="rotation_test_sender",
        namespace=namespace,
        output="screen",
        condition=IfCondition(start_rotation_sender),
        parameters=[
            configured,
            {
                "rotation_mode": rotation_mode,
                "angular_speed": rotation_speed,
                "offset": rotation_offset,
                "amplitude": rotation_amplitude,
                "period": rotation_period,
                "phase": rotation_phase,
                "sine_wave": rotation_sine_wave,
            },
        ],
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
                "start_rotation_sender",
                default_value="true",
                description="是否启动独立自转测试发送器",
            ),
            DeclareLaunchArgument(
                "rotation_mode",
                default_value="stop",
                description="stop | constant | periodic",
            ),
            DeclareLaunchArgument(
                "rotation_speed",
                default_value="0.0",
                description="恒速模式的有符号角速度（rad/s）",
            ),
            DeclareLaunchArgument(
                "rotation_offset",
                default_value="0.0",
                description="周期模式的平均角速度（rad/s）",
            ),
            DeclareLaunchArgument(
                "rotation_amplitude",
                default_value="0.0",
                description="周期模式的非负变化幅度（rad/s）",
            ),
            DeclareLaunchArgument(
                "rotation_period", default_value="4.0", description="周期（s）"
            ),
            DeclareLaunchArgument(
                "rotation_phase", default_value="0.0", description="初相位（rad）"
            ),
            DeclareLaunchArgument(
                "rotation_sine_wave",
                default_value="true",
                description="true 为正弦，false 为方波",
            ),
            cmd_mux,
            rotation_controller,
            rotation_sender,
        ]
    )
