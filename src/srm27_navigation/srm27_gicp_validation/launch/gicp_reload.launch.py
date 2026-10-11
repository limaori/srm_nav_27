# 逐轮重启的被测节点（方案 §6.8）：只启动 small_gicp_relocalization，
# 让每轮试验都从干净的 GICP 内部状态开始，而 Gazebo / Point-LIO / TF 链路保持不动。
#
# init_pose 与所有初始化策略参数都通过 launch 参数传入；launch 参数列表在
# params_file 之后，因此这里的默认值会覆盖参数文件里的同名项。

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory("srm27_nav_bringup")
    default_params = os.path.join(bringup_dir, "config", "simulation", "nav2_params_srm.yaml")

    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    prior_pcd_file = LaunchConfiguration("prior_pcd_file")
    log_level = LaunchConfiguration("log_level")
    init_pose = LaunchConfiguration("init_pose")
    diagnostic_topic = LaunchConfiguration("diagnostic_topic")
    run_id = LaunchConfiguration("run_id")
    max_scan_age = LaunchConfiguration("max_scan_age")
    init_acceptance_enable = LaunchConfiguration("init_acceptance_enable")
    init_max_translation_step = LaunchConfiguration("init_max_translation_step")
    init_max_rotation_step = LaunchConfiguration("init_max_rotation_step")
    init_accept_count = LaunchConfiguration("init_accept_count")
    init_hypothesis_enable = LaunchConfiguration("init_hypothesis_enable")
    init_hypothesis_angles = LaunchConfiguration("init_hypothesis_angles")
    init_hypothesis_quality_gap = LaunchConfiguration("init_hypothesis_quality_gap")
    max_dist_sq = LaunchConfiguration("max_dist_sq")
    global_leaf_size = LaunchConfiguration("global_leaf_size")
    registered_leaf_size = LaunchConfiguration("registered_leaf_size")
    max_rotation_step = LaunchConfiguration("max_rotation_step")
    max_translation_step = LaunchConfiguration("max_translation_step")
    min_inlier_ratio = LaunchConfiguration("min_inlier_ratio")
    max_registration_rmse = LaunchConfiguration("max_registration_rmse")
    registration_interval = LaunchConfiguration("registration_interval")

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={"use_sim_time": use_sim_time},
            convert_types=True,
        ),
        allow_substs=True,
    )

    node = Node(
        package="small_gicp_relocalization",
        executable="small_gicp_relocalization_node",
        name="small_gicp_relocalization",
        output="screen",
        parameters=[
            configured_params,
            {"prior_pcd_file": prior_pcd_file},
            {"init_pose": init_pose},
            {"diagnostic_topic": diagnostic_topic},
            {"run_id": run_id},
            {"max_scan_age": max_scan_age},
            {"init_acceptance_enable": init_acceptance_enable},
            {"init_max_translation_step": init_max_translation_step},
            {"init_max_rotation_step": init_max_rotation_step},
            {"init_accept_count": init_accept_count},
            {"init_hypothesis_enable": init_hypothesis_enable},
            {"init_hypothesis_angles": init_hypothesis_angles},
            {"init_hypothesis_quality_gap": init_hypothesis_quality_gap},
            {"max_dist_sq": max_dist_sq},
            {"global_leaf_size": global_leaf_size},
            {"registered_leaf_size": registered_leaf_size},
            {"max_rotation_step": max_rotation_step},
            {"max_translation_step": max_translation_step},
            {"min_inlier_ratio": min_inlier_ratio},
            {"max_registration_rmse": max_registration_rmse},
            {"registration_interval": registration_interval},
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="red_standard_robot1"),
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("use_sim_time", default_value="True"),
            DeclareLaunchArgument("prior_pcd_file", default_value=""),
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument("init_pose", default_value="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"),
            DeclareLaunchArgument("diagnostic_topic", default_value="gicp_diagnostics"),
            DeclareLaunchArgument("run_id", default_value=""),
            DeclareLaunchArgument("max_scan_age", default_value="3.0"),
            DeclareLaunchArgument("init_acceptance_enable", default_value="False"),
            DeclareLaunchArgument("init_max_translation_step", default_value="2.0"),
            DeclareLaunchArgument("init_max_rotation_step", default_value="1.75"),
            DeclareLaunchArgument("init_accept_count", default_value="3"),
            DeclareLaunchArgument("init_hypothesis_enable", default_value="False"),
            DeclareLaunchArgument(
                "init_hypothesis_angles",
                default_value="[-1.5707963267948966, -1.0471975511965976, -0.5235987755982988, "
                "0.0, 0.5235987755982988, 1.0471975511965976, 1.5707963267948966]",
            ),
            DeclareLaunchArgument("init_hypothesis_quality_gap", default_value="0.10"),
            DeclareLaunchArgument("max_dist_sq", default_value="1.5"),
            DeclareLaunchArgument("global_leaf_size", default_value="0.3"),
            DeclareLaunchArgument("registered_leaf_size", default_value="0.2"),
            DeclareLaunchArgument("max_rotation_step", default_value="0.5"),
            DeclareLaunchArgument("max_translation_step", default_value="2.0"),
            DeclareLaunchArgument("min_inlier_ratio", default_value="0.3"),
            DeclareLaunchArgument("max_registration_rmse", default_value="0.3"),
            DeclareLaunchArgument("registration_interval", default_value="0.5"),
            GroupAction(
                actions=[
                    PushRosNamespace(namespace),
                    node,
                ]
            ),
        ]
    )
