import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("srm27_nav_protocol"),
        "config",
        "srm27_nav_protocol.yaml",
    )

    srm27_nav_protocol_node = Node(
        package="srm27_nav_protocol",
        executable="srm27_nav_protocol_node",
        name="srm27_nav_protocol",
        namespace="",
        output="screen",  # 原为 log: 调试期需要在终端看到串口/链路状态
        emulate_tty=True,
        parameters=[config],
    )

    return LaunchDescription([srm27_nav_protocol_node])
