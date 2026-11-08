"""配置并启动激光惯性里程计 ROS 节点。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """构建节点参数与启动动作，保持配置值的明确类型。"""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'config',
                default_value=PathJoinSubstitution(
                    [FindPackageShare('frontend'), 'config', 'lio.yaml']
                ),
                description='算法配置文件的绝对路径'
            ),
            DeclareLaunchArgument(
                'input_type',
                default_value='livox',
                description='雷达输入类型：pointcloud2 或 livox'
            ),
            DeclareLaunchArgument(
                'lidar_topic',
                default_value='/livox/lidar',
                description='雷达数据输入话题'
            ),
            DeclareLaunchArgument(
                'imu_topic',
                default_value='/livox/imu',
                description='IMU 数据输入话题'
            ),
            DeclareLaunchArgument(
                'imu_acceleration_scale',
                default_value='9.81',
                description='输入加速度转为 m/s² 的倍率；以 g 为单位的 Livox 输入用 9.81'
            ),
            DeclareLaunchArgument('use_sim_time', default_value='false'),
            DeclareLaunchArgument(
                'wait_for_start', default_value='false',
                description='等待 /localization_start 的 SetBool data=true 请求后接收传感器输入'
            ),
            DeclareLaunchArgument('reset_mode', default_value='warm'),
            DeclareLaunchArgument(
                'coordinated_reset', default_value='false',
                description='已输出估计后的输入中断由后端协调重置；关闭时前端独立切换代次'
            ),
            Node(
                package='frontend',
                executable='zlio_node',
                output='screen',
                parameters=[
                    {
                        'config_file': LaunchConfiguration('config'),
                        'input_type': LaunchConfiguration('input_type'),
                        'lidar_topic': LaunchConfiguration('lidar_topic'),
                        'imu_topic': LaunchConfiguration('imu_topic'),
                        'imu_acceleration_scale': ParameterValue(
                            LaunchConfiguration('imu_acceleration_scale'),
                            value_type=float
                        ),
                        'use_sim_time': ParameterValue(
                            LaunchConfiguration('use_sim_time'),
                            value_type=bool
                        ),
                        'wait_for_start': ParameterValue(
                            LaunchConfiguration('wait_for_start'),
                            value_type=bool
                        ),
                        'reset_mode': LaunchConfiguration('reset_mode'),
                        'coordinated_reset': ParameterValue(
                            LaunchConfiguration('coordinated_reset'),
                            value_type=bool
                        )
                    }
                ]
            )
        ]
    )
