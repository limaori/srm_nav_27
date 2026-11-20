"""启动传感器到车体二维外参标定节点。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """声明标定输入、采样条件和目标车体姿态参数。"""
    map_frame = LaunchConfiguration('map_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    odom_topic = LaunchConfiguration('odom_topic')
    use_sim_time = LaunchConfiguration('use_sim_time')
    max_sync_dt = LaunchConfiguration('max_sync_dt')
    sample_period = LaunchConfiguration('sample_period')
    print_period = LaunchConfiguration('print_period')
    angle_sample_sec = LaunchConfiguration('angle_sample_sec')
    angle_min_samples = LaunchConfiguration('angle_min_samples')
    offset_min_samples = LaunchConfiguration('offset_min_samples')
    offset_min_yaw_span_deg = LaunchConfiguration('offset_min_yaw_span_deg')
    offset_radius_warn_m = LaunchConfiguration('offset_radius_warn_m')
    target_roll_deg = LaunchConfiguration('target_roll_deg')
    target_pitch_deg = LaunchConfiguration('target_pitch_deg')
    target_yaw_deg = LaunchConfiguration('target_yaw_deg')

    return LaunchDescription(
        [
            DeclareLaunchArgument('map_frame', default_value='map'),
            DeclareLaunchArgument('odom_frame', default_value='odom'),
            DeclareLaunchArgument('odom_topic', default_value='/odometry'),
            DeclareLaunchArgument('use_sim_time', default_value='false'),
            DeclareLaunchArgument('max_sync_dt', default_value='0.08'),
            DeclareLaunchArgument('sample_period', default_value='0.05'),
            DeclareLaunchArgument('print_period', default_value='1.0'),
            DeclareLaunchArgument('angle_sample_sec', default_value='5.0'),
            DeclareLaunchArgument('angle_min_samples', default_value='80'),
            DeclareLaunchArgument('offset_min_samples', default_value='80'),
            DeclareLaunchArgument('offset_min_yaw_span_deg', default_value='180.0'),
            DeclareLaunchArgument('offset_radius_warn_m', default_value='0.02'),
            DeclareLaunchArgument('target_roll_deg', default_value='0.0'),
            DeclareLaunchArgument('target_pitch_deg', default_value='0.0'),
            DeclareLaunchArgument('target_yaw_deg', default_value='0.0'),
            Node(
                package='srm27_zlocalization_backend',
                executable='srm27_zlocalization_backend_body_calibration_node',
                name='body_calibration_node',
                output='screen',
                parameters=[
                    {
                        'map_frame': map_frame,
                        'odom_frame': odom_frame,
                        'odom_topic': odom_topic,
                        'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                        'max_sync_dt': ParameterValue(max_sync_dt, value_type=float),
                        'sample_period': ParameterValue(sample_period, value_type=float),
                        'print_period': ParameterValue(print_period, value_type=float),
                        'angle_sample_sec': ParameterValue(angle_sample_sec, value_type=float),
                        'angle_min_samples': ParameterValue(angle_min_samples, value_type=int),
                        'offset_min_samples': ParameterValue(offset_min_samples, value_type=int),
                        'offset_min_yaw_span_deg': ParameterValue(
                            offset_min_yaw_span_deg,
                            value_type=float
                        ),
                        'offset_radius_warn_m': ParameterValue(
                            offset_radius_warn_m,
                            value_type=float
                        ),
                        'target_roll_deg': ParameterValue(target_roll_deg, value_type=float),
                        'target_pitch_deg': ParameterValue(target_pitch_deg, value_type=float),
                        'target_yaw_deg': ParameterValue(target_yaw_deg, value_type=float)
                    }
                ]
            )
        ]
    )
