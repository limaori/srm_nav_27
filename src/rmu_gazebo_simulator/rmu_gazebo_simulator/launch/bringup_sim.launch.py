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

"""旧步兵仿真入口已下线。

本包现在只提供场地素材（世界 SDF、GUI 配置、mid360 模型）；SRM 模型、传感器
桥接、速度执行插件和初始位姿都由 srm27_gazebo_simulator 负责，避免出现新旧两套
底盘执行器同时存在。旧的 SRM 模型、`spawn_robots.launch.py`、`gz_world.yaml`、
`base_params.yaml` 和 `ros_gz_bridge.yaml` 已随装甲、灯条、射击、云台关节链和
MecanumDrive2 底盘控制器一起删除。

请改用：

  ros2 launch srm27_gazebo_simulator srm_sim.launch.py

场地世界仍可单独启动（不含机器人）：

  ros2 launch rmu_gazebo_simulator gazebo.launch.py
"""

from launch import LaunchDescription


def generate_launch_description():
    raise RuntimeError(
        "rmu_gazebo_simulator bringup_sim.launch.py 已下线：\n"
        "  请改用  ros2 launch srm27_gazebo_simulator srm_sim.launch.py\n"
        "  场地世界可单独用  ros2 launch rmu_gazebo_simulator gazebo.launch.py\n"
        "新旧底盘执行器互斥，不要同时启动两套仿真。"
    )
