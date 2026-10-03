import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cur_config_path = os.path.join(
        get_package_share_directory('livox_ros_driver2'), 'config')

    config_file = LaunchConfiguration('config_file')
    rviz_config = LaunchConfiguration('rviz_config')
    frame_id = LaunchConfiguration('frame_id')
    publish_freq = LaunchConfiguration('publish_freq')

    # xfer_format=4 publishes both the Point-LIO CustomMsg and a PointCloud2
    # topic for RViz: /livox/lidar and /livox/lidar/pointcloud respectively.
    livox_ros2_params = [{
        'xfer_format': 4,
        'multi_topic': 0,
        'data_src': 0,
        'publish_freq': publish_freq,
        'output_data_type': 0,
        'frame_id': frame_id,
        'lvx_file_path': '',
        'user_config_path': config_file,
        'cmdline_input_bd_code': 'livox0000000001',
    }]

    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params,
    )

    livox_rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['--display-config', rviz_config],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=os.path.join(cur_config_path, 'MID360_config.json'),
            description='Livox Mid-360 JSON network configuration',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(cur_config_path, 'display_point_cloud.rviz'),
            description='RViz configuration file',
        ),
        DeclareLaunchArgument(
            'frame_id', default_value='front_mid360',
            description='Frame ID attached to the point cloud',
        ),
        DeclareLaunchArgument(
            'publish_freq', default_value='20.0',
            description='Point cloud publish frequency in Hz',
        ),
        livox_driver,
        livox_rviz,
    ])
