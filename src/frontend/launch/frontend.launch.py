"""独立启动前端、前端可视化适配器及专属 RViz。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """复用里程计启动文件，并配置前端显示节点与 RViz。"""
    share = FindPackageShare('frontend')
    clock = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true', description='打开前端专属 RViz'),
        DeclareLaunchArgument('map_voxel_size', default_value='0.15', description='显示点云体素边长，米'),
        DeclareLaunchArgument('map_max_voxels', default_value='200000', description='显示点云的体素数量上限'),
        DeclareLaunchArgument('map_publish_period', default_value='0.2', description='显示快照最小发布间隔，秒'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([share, 'launch', 'lio.launch.py']))
        ),
        Node(
            package='frontend', executable='zlio_visualizer', output='screen',
            parameters=[clock, {
                'map_voxel_size': ParameterValue(LaunchConfiguration('map_voxel_size'), value_type=float),
                'map_max_voxels': ParameterValue(LaunchConfiguration('map_max_voxels'), value_type=int),
                'map_publish_period': ParameterValue(LaunchConfiguration('map_publish_period'), value_type=float),
            }], condition=IfCondition(LaunchConfiguration('rviz'))
        ),
        Node(
            package='rviz2', executable='rviz2', name='frontend_rviz', output='screen',
            arguments=['-d', PathJoinSubstitution([share, 'rviz', 'frontend.rviz'])],
            parameters=[clock], condition=IfCondition(LaunchConfiguration('rviz'))
        ),
    ])
