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

链路（2026-11-08 起两路输入，方案 §4.3）：
  <nav_topic>  ─┐
                ├─ srm_cmd_mux ── <output_topic>
  <spin_topic> ─┘

`nav_topic` 给平移（angular.z 一律忽略），`spin_topic` 给底盘 wz（决策层）。
`spin_topic` **留空 = 单输入**（导航同时拥有平移与 wz）：仿真链与回退用这个。
实车入口（nav_srm_real_launch.py）会传 `spin_topic:=rotation_cmd`。

2026-10-19：独立自转链（rotation_controller / rotation_test_sender /
rotation_velocity）已删除，自转请求改由决策层直接发 `rotation_cmd`。
到点后 behavior/hold_spin 续租驻点转速；人工停车与决策许可门控覆盖所有输入。

`nav_topic` / `output_topic` 由导航入口按链路给出：
  * 仿真入口（nav_srm_simulation_launch.py）：cmd_vel_nav_body（fake_vel_transform 的输出），
    输出仍叫 cmd_vel_sim；
  * 实车入口（nav_srm_real_launch.py）：同样是 cmd_vel_nav_body，但**输出必须改成
    cmd_vel_chassis** —— 那是 srm27_nav_protocol 唯一订阅的话题（串口节点的出口名）。
    这两个话题都由 launch 参数给出，配置文件里保留默认值只为"参数从哪来"可见。

速度限幅（vx_max / vy_max / v_max / wz_max）与超时仍由
config/{simulation,real}/speed_limits.yaml 在启动时合并进来（唯一来源）：
仿真入口传 config/simulation 那份，实车入口传 config/real 那份。
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
    spin_topic = LaunchConfiguration("spin_topic")
    output_topic = LaunchConfiguration("output_topic")

    # mux 的 nav_topic / spin_topic / output_topic 也走 RewrittenYaml：配置文件里仍然写着
    # 这些键（"参数从哪来"在文件里可见），启动时按链路覆盖，避免"两处定义、一处静默生效"。
    configured = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={
                "use_sim_time": use_sim_time,
                "nav_topic": nav_topic,
                "spin_topic": spin_topic,
                "output_topic": output_topic,
                "decision_required": LaunchConfiguration("decision_required"),
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
                    "mux 的导航输入话题（只取平移）；fakeyaw 迁移链上是 "
                    "fake_vel_transform 的输出 cmd_vel_nav_body。"
                ),
            ),
            DeclareLaunchArgument(
                "spin_topic",
                default_value="",
                description=(
                    "决策层自转输入话题（只取 angular.z）。**留空 = 单输入**：导航同时拥有"
                    "平移与 wz（仿真链/回退）。实车入口传 rotation_cmd。"
                ),
            ),
            DeclareLaunchArgument(
                "output_topic",
                default_value="cmd_vel_sim",
                description=(
                    "mux 的输出话题，也是它**唯一**发布的话题。仿真默认 cmd_vel_sim"
                    "（速度适配器订阅它）；实车入口必须传 cmd_vel_chassis"
                    "（srm27_nav_protocol 订阅它），否则串口节点收不到任何速度。"
                ),
            ),
            DeclareLaunchArgument("decision_required", default_value="true"),
            cmd_mux,
        ]
    )
