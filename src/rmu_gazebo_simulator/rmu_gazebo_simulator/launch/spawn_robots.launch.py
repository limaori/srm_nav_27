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

# 导入 os，用于拼接资源文件路径。
import os

# 导入 yaml，用于解析机器人和世界配置。
import yaml
# 获取 ROS 2 功能包安装后的 share 目录。
from ament_index_python.packages import get_package_share_directory
# 保存启动动作。
from launch import LaunchDescription
# 执行外部命令，例如调用 Gazebo 服务。
from launch.actions import ExecuteProcess
# 启动 ROS 2 节点。
from launch_ros.actions import Node
# 在启动时替换 YAML 中的占位符。
from nav2_common.launch import ReplaceString
# 将 SDF 格式转换为 URDF 格式。
from sdformat_tools.urdf_generator import UrdfGenerator
# 读取并展开 xmacro SDF 模板。
from xmacro.xmacro4sdf import XMLMacro4sdf


def generate_launch_description():
    # 将绝对 TF 话题映射为相对话题，使机器人命名空间能够自动添加。
    # 对 /tf 和 /tf_static 做此处理，可以让每个机器人拥有独立的 TF 话题。
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno): 将来可使用 PushNodeRemapping 替代此方式。
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    # 获取当前仿真包的 share 目录。
    pkg_simulator = get_package_share_directory("rmu_gazebo_simulator")
    robot_xmacro_path = os.path.join(
        pkg_simulator, "resource", "models", "srm_sentry_robot", "model.sdf.xmacro"
    )
    # 定位 Gazebo-ROS 桥接配置和机器人控制参数文件。
    bridge_config = os.path.join(pkg_simulator, "config", "ros_gz_bridge.yaml")
    robot_config = os.path.join(pkg_simulator, "config", "base_params.yaml")

    # 读取当前世界以及该世界中需要生成的机器人初始位姿。
    gz_world_path = os.path.join(pkg_simulator, "config", "gz_world.yaml")
    with open(gz_world_path) as file:
        # 将 YAML 文件解析为 Python 字典。
        config = yaml.safe_load(file)
        # 获取当前选择的世界名称，例如 rmuc_2025。
        selected_world = config.get("world")
        # 根据世界名称取出对应的机器人列表。
        robots = config["robots"].get(selected_world)

    # 创建 xmacro 解析器并加载机器人 SDF 模板。
    xmacro = XMLMacro4sdf()
    xmacro.set_xml_file(robot_xmacro_path)

    # 创建总启动描述；下面每个机器人都会向其中加入多个动作。
    ld = LaunchDescription()

    # 遍历配置文件中的机器人，为每个机器人生成独立的启动动作。
    for robot in robots:
        # 使用机器人颜色等宏参数展开 SDF 模板。
        xmacro.generate({"global_initial_color": robot["color"]})
        robot_xml = xmacro.to_string()

        # 将生成的 SDF 转成 URDF，供 robot_state_publisher 发布 TF。
        urdf_generator = UrdfGenerator()
        urdf_generator.parse_from_sdf_string(robot_xml)
        robot_urdf_xml = urdf_generator.to_string()

        # 将桥接配置中的 <robot_name> 替换为当前机器人的实际名称。
        aft_replace_ros_bridge_params = ReplaceString(
            source_file=bridge_config,
            replacements={"<robot_name>": robot["name"]},
        )

        # 调用 ros_gz_sim 的 create 节点，将机器人 SDF 插入 Gazebo 世界。
        spawn_robot = Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-string",
                robot_xml,
                "-name",
                robot["name"],
                "-allow_renaming",
                "true",
                "-x",
                robot["x_pose"],
                "-y",
                robot["y_pose"],
                "-z",
                robot["z_pose"],
                "-Y",
                robot["yaw"],
            ],
        )

        # 启动机器人底盘、云台和射击等仿真控制节点。
        robot_base = Node(
            package="rmoss_gz_base",
            executable="rmua19_robot_base",
            namespace=robot["name"],
            parameters=[robot_config, {"robot_name": robot["name"]}],
        )

        # 发布机器人关节和 TF；robot_description 使用刚生成的 URDF。
        robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=robot["name"],
            remappings=remappings,
            parameters=[
                {
                    "use_sim_time": True,
                    "robot_description": robot_urdf_xml,
                }
            ],
        )

        # 启动当前机器人专属的 Gazebo-ROS 话题桥。
        robot_ign_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            namespace=robot["name"],
            parameters=[{"config_file": aft_replace_ros_bridge_params}],
        )

        # 机器人生成后调用 Gazebo 服务，将它设置为当前关注的 performer。
        # https://gazebosim.org/api/gazebo/6.9/levels.html#Runtime-performers
        set_performer_service = ExecuteProcess(
            cmd=[
                # Gazebo/Ignition 服务命令。
                "ign",
                "service",
                "-s",
                # 当前世界的 level performer 服务。
                "/world/default/level/set_performer",
                # 请求和响应的消息类型。
                "--reqtype",
                "ignition.msgs.StringMsg",
                "--reptype",
                "ignition.msgs.Boolean",
                # 服务调用超时时间，单位为毫秒。
                "--timeout",
                "2000",
                # 把当前机器人名称作为请求内容。
                "--req",
                f'data: "{robot["name"]}"',
            ],
            # 将命令输出显示在终端。
            output="screen",
        )

        # 将当前机器人的所有动作加入总启动描述。
        ld.add_action(spawn_robot)
        ld.add_action(robot_base)
        ld.add_action(robot_state_publisher)
        ld.add_action(robot_ign_bridge)
        ld.add_action(set_performer_service)

    return ld

