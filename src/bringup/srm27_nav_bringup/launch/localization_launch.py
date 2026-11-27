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


import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterFile, ParameterValue
from nav2_common.launch import RewrittenYaml


def _backend_package_share():
    """backend 包是可选的实验依赖：没装（例如本机没有 GTSAM）时整条默认链路仍要能起。

    只在 use_zlocalization_backend:=true 时把"包缺失"变成启动错误，
    否则这里返回空串，默认的 --lio/--reloc/静态 TF 链路完全不受影响。
    """
    try:
        return get_package_share_directory("srm27_zlocalization_backend")
    except PackageNotFoundError:
        return ""


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("srm27_nav_bringup")
    backend_share = _backend_package_share()
    default_backend_config = (
        os.path.join(backend_share, "config", "srm27_localization_sim.yaml")
        if backend_share
        else ""
    )

    namespace = LaunchConfiguration("namespace")
    map_yaml_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    prior_pcd_file = LaunchConfiguration("prior_pcd_file")
    params_file = LaunchConfiguration("params_file")
    use_composition = LaunchConfiguration("use_composition")
    container_name = LaunchConfiguration("container_name")
    container_name_full = (namespace, "/", container_name)
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")
    use_pcd_localization = LaunchConfiguration("use_pcd_localization")
    # 只跑 Point-LIO 里程计、不做 GICP 重定位（"纯 LIO"模式）。
    # 与 use_pcd_localization 分开是为了让 "谁来提供里程计" 和 "要不要 GICP 修正"
    # 可以独立选择：纯 LIO 模式下 map->odom 由静态 TF 给出（见下文的
    # start_static_transform_node，它的条件是 use_pcd_localization 为假）。
    use_lio_odometry = LaunchConfiguration("use_lio_odometry")
    # 谁来产生里程计（方案 §6.2）：point_lio（默认 = 旧行为）或 zlio。
    # ZLIO 只能替第 ① 层（激光惯性里程计）；第 ② 层（把 LIO 世界系改写成 odom）由新增的
    # zlio_bridge 承担，所以两者成对起停。--gicp 下暂不启用 ZLIO（方案 §10）。
    odometry_frontend = LaunchConfiguration("odometry_frontend")
    zlio_config = LaunchConfiguration("zlio_config")
    # ZLIO 的输入接线（仿真/实车不同，见下方 start_zlio_node 的注释）：
    #   仿真: pointcloud2 + velodyne_points + 秒制 time 字段 + IMU 单位 m/s² + front_mid360
    #   实车: livox(CustomMsg) + livox/lidar + livox_imu + IMU 单位 g(scale 9.81)
    # 这些不是"算法参数"（算法参数在 zlio_config 里），而是**话题/单位/帧名**，
    # 所以用 launch 参数暴露，由入口按目标平台给值。
    zlio_input_type = LaunchConfiguration("zlio_input_type")
    zlio_lidar_topic = LaunchConfiguration("zlio_lidar_topic")
    zlio_imu_topic = LaunchConfiguration("zlio_imu_topic")
    zlio_point_time_field = LaunchConfiguration("zlio_point_time_field")
    zlio_point_time_scale = LaunchConfiguration("zlio_point_time_scale")
    zlio_point_time_absolute = LaunchConfiguration("zlio_point_time_absolute")
    zlio_imu_acceleration_scale = LaunchConfiguration("zlio_imu_acceleration_scale")
    zlio_body_frame = LaunchConfiguration("zlio_body_frame")
    map_to_odom_x = LaunchConfiguration("map_to_odom_x")
    map_to_odom_y = LaunchConfiguration("map_to_odom_y")
    map_to_odom_yaw = LaunchConfiguration("map_to_odom_yaw")
    # Z-Localization backend 实验模式（默认关闭）。打开后 map->odom 的唯一发布者
    # 由静态 TF / small_gicp 换成 backend_node —— 三者互斥，冲突在启动前就失败。
    use_zlocalization_backend = LaunchConfiguration("use_zlocalization_backend")
    backend_stabilized_body = LaunchConfiguration("backend_stabilized_body")
    zlocalization_backend_config = LaunchConfiguration("zlocalization_backend_config")
    # 空 = 用包内默认 profile（入口脚本不关心平台细节时不用传）。
    effective_backend_config = PythonExpression(
        ["'", zlocalization_backend_config, "' or '", default_backend_config, "'"]
    )
    zlocalization_prior_enabled = LaunchConfiguration("zlocalization_prior_enabled")
    # 先验只在初始化时匹配一次（首次匹配后 map->odom 冻结，定位全走前端 ZLIO）。
    zlocalization_prior_initialization_only = LaunchConfiguration(
        "zlocalization_prior_initialization_only")
    zlocalization_loop_enabled = LaunchConfiguration("zlocalization_loop_enabled")
    zlocalization_uwb_enabled = LaunchConfiguration("zlocalization_uwb_enabled")
    zlocalization_prior_map = LaunchConfiguration("zlocalization_prior_map")
    zlocalization_trajectory_directory = LaunchConfiguration("zlocalization_trajectory_directory")

    lifecycle_nodes = ["map_server"]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {"use_sim_time": use_sim_time, "yaml_filename": map_yaml_file}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    colorized_output_envvar = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map", description="Full path to map yaml file to load"
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_prior_pcd_file_cmd = DeclareLaunchArgument(
        "prior_pcd_file",
        default_value="",
        description="Full path to prior PCD file to load",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, "config", "simulation", "nav2_params.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        "use_composition",
        default_value="False",
        description="Use composed bringup if True",
    )

    declare_container_name_cmd = DeclareLaunchArgument(
        "container_name",
        default_value="nav2_container",
        description="the name of container that nodes will load in if use composition",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    declare_use_pcd_localization_cmd = DeclareLaunchArgument(
        "use_pcd_localization", default_value="False",
        description="Use small_gicp localization against a prior PCD",
    )
    declare_use_lio_odometry_cmd = DeclareLaunchArgument(
        "use_lio_odometry", default_value="False",
        description=(
            "Use Point-LIO odometry without GICP relocalization; "
            "map->odom stays a static transform"
        ),
    )
    declare_map_to_odom_x_cmd = DeclareLaunchArgument("map_to_odom_x", default_value="0.0")
    declare_map_to_odom_y_cmd = DeclareLaunchArgument("map_to_odom_y", default_value="0.0")
    declare_map_to_odom_yaw_cmd = DeclareLaunchArgument("map_to_odom_yaw", default_value="0.0")

    declare_use_zlocalization_backend_cmd = DeclareLaunchArgument(
        "use_zlocalization_backend",
        default_value="False",
        description=(
            "用 srm27_zlocalization_backend 发布 map->odom（实验模式，默认关闭）。"
            "要求 use_lio_odometry:=True + odometry_frontend:=zlio；先验/回环/UWB 独立选择；"
            "与 use_pcd_localization / 静态 map->odom / slam 互斥"
        ),
    )
    declare_zlocalization_backend_config_cmd = DeclareLaunchArgument(
        "zlocalization_backend_config",
        default_value=default_backend_config,
        description=(
            "backend 算法配置（继承包内 backend.yaml；"
            "实车入口默认 srm27_localization_real.yaml，仿真用 sim 那份）"
        ),
    )
    declare_backend_stabilized_body_cmd = DeclareLaunchArgument(
        "backend_stabilized_body",
        default_value="False",
        description=(
            "backend 输入改用导航的稳定体坐标系（gimbal_yaw_fake）：位姿取 odom 朝向、"
            "点云取世界点云相对车体。云台持续自转/自瞄时 backend 的 body-rate 闸门"
            "不再把云台当车体。实车入口默认打开；仿真入口不传（保持原行为）"
        ),
    )
    declare_zlocalization_prior_enabled_cmd = DeclareLaunchArgument(
        "zlocalization_prior_enabled", default_value="False",
        description="启用先验配准，独立开关，默认关闭",
    )
    declare_zlocalization_prior_initialization_only_cmd = DeclareLaunchArgument(
        "zlocalization_prior_initialization_only", default_value="False",
        description=(
            "先验只在初始化时匹配一次；首次匹配成功后 map->odom 冻结，"
            "定位完全由前端 ZLIO 里程计传播（需要 zlocalization_prior_enabled:=True）"
        ),
    )
    declare_zlocalization_loop_enabled_cmd = DeclareLaunchArgument(
        "zlocalization_loop_enabled", default_value="False",
        description="启用运行轨迹回环，独立开关，默认关闭",
    )
    declare_zlocalization_uwb_enabled_cmd = DeclareLaunchArgument(
        "zlocalization_uwb_enabled", default_value="False",
        description="启用UWB 融合，独立开关，默认关闭",
    )
    declare_zlocalization_prior_map_cmd = DeclareLaunchArgument(
        "zlocalization_prior_map",
        default_value="",
        description="先验地图包目录（keyframes.txt + clouds/ + 描述子），必须是绝对路径",
    )
    declare_zlocalization_trajectory_directory_cmd = DeclareLaunchArgument(
        "zlocalization_trajectory_directory",
        default_value="",
        description="backend 轨迹地图包绝对目录；先验定位模式不写轨迹，建图模式必填",
    )

    # 条件表达式里必须**显式加括号**：PythonExpression 拼出来的是一整串 Python 表达式，
    # `A or B and C` 的优先级会让 and 只挂在最后一个操作数上（旧版本的坑）。
    use_point_lio_frontend = PythonExpression(["'", odometry_frontend, "' == 'point_lio'"])
    use_zlio_frontend = PythonExpression(["'", odometry_frontend, "' == 'zlio'"])

    declare_odometry_frontend_cmd = DeclareLaunchArgument(
        "odometry_frontend",
        default_value="point_lio",
        description=(
            "里程计前端: point_lio (默认, 旧行为) 或 zlio。"
            "zlio 只在 use_lio_odometry 为真时生效, 且会同时启动 zlio_bridge"
            "(它负责把 ZLIO 的局部 IMU 系位姿/点云改写成 odom 系)"
        ),
    )
    declare_zlio_config_cmd = DeclareLaunchArgument(
        "zlio_config",
        default_value=os.path.join(bringup_dir, "config", "simulation", "zlio_mid360.yaml"),
        description="ZLIO 算法配置文件绝对路径 (z-loc 的严格校验会拒绝未知键)",
    )
    # ZLIO 的输入接线。默认值 = 仿真（pointcloud2/velodyne_points/front_mid360）；
    # 实车入口会覆盖成 livox/livox_lidar/livox_imu 且 imu_acceleration_scale=9.81。
    declare_zlio_input_type_cmd = DeclareLaunchArgument(
        "zlio_input_type",
        default_value="pointcloud2",
        description="LiDAR 消息格式: pointcloud2（仿真）或 livox（实车 CustomMsg）",
    )
    declare_zlio_lidar_topic_cmd = DeclareLaunchArgument(
        "zlio_lidar_topic",
        default_value="velodyne_points",
        description="LiDAR 输入话题（仿真 velodyne_points / 实车 livox/lidar）",
    )
    declare_zlio_imu_topic_cmd = DeclareLaunchArgument(
        "zlio_imu_topic",
        default_value="livox/imu_saturated",
        description="IMU 输入话题（默认取饱和保护的输出，不订原始 IMU）",
    )
    declare_zlio_point_time_field_cmd = DeclareLaunchArgument(
        "zlio_point_time_field",
        default_value="time",
        description="PointCloud2 的逐点时间字段名（livox 输入不使用）",
    )
    declare_zlio_point_time_scale_cmd = DeclareLaunchArgument(
        "zlio_point_time_scale",
        default_value="1.0",
        description="逐点时间换算成秒的倍率：纳秒 1e-9 / 秒 1.0（须 >0）",
    )
    declare_zlio_point_time_absolute_cmd = DeclareLaunchArgument(
        "zlio_point_time_absolute",
        default_value="False",
        description="逐点时间是否为绝对时间；关闭时按相对 header.stamp 的偏移读",
    )
    declare_zlio_imu_acceleration_scale_cmd = DeclareLaunchArgument(
        "zlio_imu_acceleration_scale",
        default_value="1.0",
        description="IMU 加速度单位换算：输入为 g 时取 9.81（实车），m/s² 时取 1.0（仿真）",
    )
    declare_zlio_body_frame_cmd = DeclareLaunchArgument(
        "zlio_body_frame",
        default_value="front_mid360",
        description="里程计 child frame（仿真 front_mid360 / 实车 livox_imu）",
    )

    start_point_lio_node = Node(
        condition=IfCondition(
            PythonExpression([
                "(", use_pcd_localization, " or ", use_lio_odometry, ") and (",
                use_point_lio_frontend, ")",
            ])
        ),
        package="point_lio",
        executable="pointlio_mapping",
        name="point_lio",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[
            configured_params,
            {"use_sim_time": use_sim_time},
            # 注意: Point-LIO 的 prior_pcd(先验地图配准)在本工程里会导致状态发散
            # (实测开启后 odom 会漂到几十万米)。这里的 "先验地图求 map->odom"
            # 实际由 small_gicp_relocalization 完成, 因此 Point-LIO 只需跑纯
            # 里程计(prior_pcd 关闭), 保持稳定即可。故强制设为 False,
            # 不再跟随 use_pcd_localization。
            {"prior_pcd.enable": False},
            {"prior_pcd.prior_pcd_map_path": prior_pcd_file},
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # ZLIO 前端（z-localization 的 frontend 包）。仿真专有参数见方案 §5，逐项都有实测依据：
    #
    #   * imu_acceleration_scale: 1.0 —— 仿真 IMU 输出单位是 m/s²（point_lio 的
    #     acc_norm: 9.81 就是证据）。z-loc 的 lio.launch.py 默认 9.81 是给实车（g）用的，
    #     照抄会让加速度被放大 9.81 倍、滤波器当场发散。所以这里**不 include 它的 launch**。
    #   * input_type/lidar_topic —— 必须吃 velodyne_points（ign_sim_pointcloud_tool 生成的
    #     PointXYZIRT），不能吃 livox/lidar（gz 直出的 PointCloud2 没有逐点时间字段）。
    #   * point_time_field/scale/absolute —— 默认值是 offset_time / 1e-9（纳秒），不显式
    #     覆盖会静默把秒当纳秒，去畸变等于没做，位移会快 3~10 倍（仿真 scan_period 那段
    #     注释踩过同一个坑）。
    #   * publish_tf: false —— URDF 里 front_mid360 的父节点是 gimbal_yaw，再发
    #     odom -> front_mid360 会有两个父；odom -> base_link 仍只由
    #     sensor_scan_generation 发布。
    #   * body_frame: front_mid360 —— 仿真的 IMU 与雷达同 link；实车是 livox_imu。
    #   * output 全部 remap 进 zlio/ 子命名空间 —— 它的 odometry / lidar_odometry 与本仓库
    #     sensor_scan_generation 的 /odometry、zlio_bridge 的 lidar_odometry 撞名，
    #     不 remap 会让 fake_vel_transform 收到两路语义不同的 odom（方案 §5.5）。
    # IMU 加速度饱和保护（记录 §5.4）：ZLIO 运行时没有 Point-LIO 的 check_satu/satu_acc，
    # 撞墙那一下的冲量会被原样积分成几十 m/s 的速度，而"正对一面大墙"时激光匹配是退化的
    # （法向可观、切向不可观）救不回来 —— 表现就是航向还准、位置飞出去几千米。
    # 所以 zlio 前面串一级保护：livox/imu -> livox/imu_saturated
    # （|a| > 30 m/s² 时把该帧加速度替换成上一次未饱和值，而不是按比例缩小）。
    # 与 zlio 同条件起停，保证"谁订谁发"永远配对（不存在 zlio 在订一个没人发的话题的情况）。
    start_imu_saturation_guard = Node(
        condition=IfCondition(
            PythonExpression([use_lio_odometry, " and (", use_zlio_frontend, ")"])
        ),
        package="srm27_localization_adapter",
        executable="imu_saturation_guard.py",
        name="imu_saturation_guard",
        output="screen",
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", log_level],
    )

    start_zlio_node = Node(
        condition=IfCondition(
            PythonExpression([use_lio_odometry, " and (", use_zlio_frontend, ")"])
        ),
        package="frontend",
        executable="zlio_node",
        name="zlio",
        output="screen",
        # 刻意不 respawn：前端一旦失败，重启会重置代次与坐标原点而不报错，
        # 比"停下来"危险得多（方案 §4.3 的同类考虑）。
        parameters=[
            {"use_sim_time": use_sim_time},
            {"config_file": zlio_config},
            {"input_type": zlio_input_type},
            {"lidar_topic": zlio_lidar_topic},
            # 订饱和保护的输出，不订原始 IMU（守卫与 zlio 同条件起停，见上）。
            {"imu_topic": zlio_imu_topic},
            {"point_time_field": zlio_point_time_field},
            {"point_time_scale": ParameterValue(zlio_point_time_scale, value_type=float)},
            {"point_time_absolute": ParameterValue(zlio_point_time_absolute, value_type=bool)},
            # ⚠ 实车 MID-360 的 linear_acceleration 单位是 **g**（取 9.81），仿真才是 m/s²（取 1.0）。
            # 抄错的表现是滤波器当场发散（放大 9.81 倍）或加速度被压小 9.81 倍。
            {"imu_acceleration_scale": ParameterValue(zlio_imu_acceleration_scale, value_type=float)},
            {"publish_tf": False},
            {"world_frame": "odom"},
            {"body_frame": zlio_body_frame},
            # warm 重置保留速度/倾角/零偏。
            # coordinated_reset 的语义（frontend/apps/node.cpp:346-353）：
            #   false -> 输入中断后前端**自己**换代（非 backend 模式需要这样，
            #            bridge 的锚定正是为此准备）；
            #   true  -> 只置 reset_required 等后端协调（backend 模式必须这样，
            #            否则一次断流会触发两次换代，问题分析 §B5）。
            {"reset_mode": "warm"},
            {"coordinated_reset": ParameterValue(use_zlocalization_backend, value_type=bool)},
        ],
        remappings=[
            ("odometry", "zlio/odometry"),
            ("lidar_odometry", "zlio/lidar_odometry"),
            ("frontend/frame", "zlio/frame"),
            ("frontend/odometry", "zlio/frontend_odometry"),
            ("frontend/session", "zlio/session"),
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # 坐标系改写（方案 §4）：ZLIO 的位姿/点云在局部 IMU 系，必须左乘当时的位姿才能进
    # odom 系；同时做换代锚定，避免输入中断后 odom 帧跳变、terrain_analysis 地形图错位。
    # 输出沿用 registered_scan / lidar_odometry 原名，下游不需要任何 remap。
    start_zlio_bridge_node = Node(
        condition=IfCondition(
            PythonExpression([use_lio_odometry, " and (", use_zlio_frontend, ")"])
        ),
        package="srm27_localization_adapter",
        executable="zlio_bridge.py",
        name="zlio_bridge",
        output="screen",
        parameters=[
            configured_params,
            # backend 输出适配必须长在这个节点里：它和 lidar_odometry 共用同一个
            # T_off，另起一个节点会各自抓"第一帧"，两个 odom 系差一个常量变换。
            {"backend_enabled": ParameterValue(use_zlocalization_backend, value_type=bool)},
            # 稳定体输入：backend 拿到的位姿/点云已经把云台自转和自瞄去掉。
            # sensor_frame 仍用前端 body_frame（fake 点云只在 backend 适配层里改名），
            # backend 的逐帧 frame 校验和 scan-context lidar_height 都不用动。
            {"backend_stabilized_body":
                ParameterValue(backend_stabilized_body, value_type=bool)},
            # backend 的 common.sensor_frame 必须等于前端的 body_frame，
            # 否则它逐帧静默丢帧。
            {"backend_sensor_frame": zlio_body_frame},
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    start_static_transform_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher_map2odom",
        output="screen",
        # backend 模式下 map->odom 由 backend_node 发布：静态 TF 必须同时关掉，
        # 否则两个发布者会抢同一条 TF（问题分析 §B1）。
        condition=IfCondition(
            PythonExpression(["not (", use_pcd_localization, " or ",
                              use_zlocalization_backend, ")"])
        ),
        arguments=[
            "--x", map_to_odom_x, "--y", map_to_odom_y, "--z", "0.0",
            "--roll", "0.0", "--pitch", "0.0", "--yaw", map_to_odom_yaw,
            "--frame-id", "map", "--child-frame-id", "odom",
        ],
    )

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(["not ", use_composition])),
        actions=[
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                condition=IfCondition(use_pcd_localization),
                package="small_gicp_relocalization",
                executable="small_gicp_relocalization_node",
                name="small_gicp_relocalization",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, {"prior_pcd_file": prior_pcd_file}],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization",
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": autostart},
                    {"node_names": lifecycle_nodes},
                ],
            ),
        ],
    )

    load_composable_nodes = LoadComposableNodes(
        condition=IfCondition(use_composition),
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                package="nav2_map_server",
                plugin="nav2_map_server::MapServer",
                name="map_server",
                parameters=[configured_params],
            ),
            ComposableNode(
                package="nav2_lifecycle_manager",
                plugin="nav2_lifecycle_manager::LifecycleManager",
                name="lifecycle_manager_localization",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": autostart,
                        "node_names": lifecycle_nodes,
                    }
                ],
            ),
        ],
    )

    # ComposableNode conditions are not consistently honored across ROS 2
    # distributions. Keep the optional GICP component behind an action-level
    # condition so it is never loaded for static-map localization.
    load_pcd_localization_node = LoadComposableNodes(
        condition=IfCondition(use_pcd_localization),
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                package="small_gicp_relocalization",
                plugin="small_gicp_relocalization::SmallGicpRelocalizationNode",
                name="small_gicp_relocalization",
                parameters=[configured_params, {"prior_pcd_file": prior_pcd_file}],
            )
        ],
    )

    # backend 实验模式：定位后端整条链路由包内 launch 起（配置/地图/可视化都在那里），
    # 这里只负责"按本仓库的接线传参"。
    # backend 包缺失时不生成该动作；use_zlocalization_backend:=true 会在冲突检查里
    # 给出明确报错，而不是让整条默认链路一起起不来。
    start_zlocalization_backend = None
    if backend_share:
        start_zlocalization_backend = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(backend_share, "launch", "backend.launch.py")
            ),
            condition=IfCondition(use_zlocalization_backend),
            launch_arguments={
                "config_file": effective_backend_config,
                "prior_enabled": zlocalization_prior_enabled,
                "prior_initialization_only": zlocalization_prior_initialization_only,
                "loop_enabled": zlocalization_loop_enabled,
                "uwb_enabled": zlocalization_uwb_enabled,
                "prior_map": zlocalization_prior_map,
                "trajectory_directory": zlocalization_trajectory_directory,
                # backend 的 odom 就是本仓库 odom（与 zlio_bridge 的 odom_frame 一致）。
                "odom_frame": "odom",
                # 前端的 body_frame 就是 backend 的传感器系（livox_imu / front_mid360）。
                "sensor_frame": zlio_body_frame,
                "use_sim_time": use_sim_time,
                "rviz": "false",
            }.items(),
        )

    # 启动前冲突检查：backend 模式与另外三种 map->odom 所有者互斥，且必须是
    # ZLIO 纯 LIO 链路（backend 只吃 FrontendFrame，point_lio/legacy 输入没有
    # 本仓库的 odom/T_off 契约）。宁可启动失败，也不要两个发布者抢同一条 TF。
    def _validate_zlocalization_backend(context):
        def flag(name, default="False"):
            value = context.launch_configurations.get(name, default)
            return str(value).strip().lower() in ("1", "true", "yes", "on")

        if not flag("use_zlocalization_backend"):
            return []
        problems = []
        if not backend_share:
            problems.append(
                "未找到 srm27_zlocalization_backend 包（先构建它；它需要 GTSAM 4.2.x）"
            )
        if flag("use_pcd_localization"):
            problems.append("use_pcd_localization:=True（small_gicp 也要发 map->odom）")
        if not flag("use_lio_odometry"):
            problems.append("use_lio_odometry:=False（backend 需要前端里程计链路）")
        frontend = str(context.launch_configurations.get("odometry_frontend", "")).strip()
        if frontend != "zlio":
            problems.append(f"odometry_frontend:={frontend or '<empty>'}（backend 只接 ZLIO 观测）")
        config_file = (
            str(context.launch_configurations.get("zlocalization_backend_config", "")).strip()
            or default_backend_config
        )
        if not config_file:
            problems.append("backend 配置为空且包内没有默认 profile")
        elif not os.path.isfile(config_file):
            problems.append(f"backend 配置不存在: {config_file}")
        prior_map = str(context.launch_configurations.get("zlocalization_prior_map", "")).strip()
        if flag("zlocalization_prior_enabled") and not prior_map:
            problems.append("zlocalization_prior_map 为空（启用先验配准需要先验地图包）")
        elif prior_map and not os.path.isabs(prior_map):
            problems.append(f"zlocalization_prior_map={prior_map} 不是绝对路径")
        if problems:
            raise RuntimeError(
                "use_zlocalization_backend:=True 需要: use_lio_odometry:=True + "
                "odometry_frontend:=zlio（启用先验时还需绝对路径的地图包）；"
                "且不能与 GICP/静态 TF/slam 同时启用 map->odom。冲突项: "
                + "; ".join(problems)
            )
        return []

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(colorized_output_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_prior_pcd_file_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_use_pcd_localization_cmd)
    ld.add_action(declare_use_lio_odometry_cmd)
    ld.add_action(declare_odometry_frontend_cmd)
    ld.add_action(declare_zlio_config_cmd)
    ld.add_action(declare_zlio_input_type_cmd)
    ld.add_action(declare_zlio_lidar_topic_cmd)
    ld.add_action(declare_zlio_imu_topic_cmd)
    ld.add_action(declare_zlio_point_time_field_cmd)
    ld.add_action(declare_zlio_point_time_scale_cmd)
    ld.add_action(declare_zlio_point_time_absolute_cmd)
    ld.add_action(declare_zlio_imu_acceleration_scale_cmd)
    ld.add_action(declare_zlio_body_frame_cmd)
    ld.add_action(declare_map_to_odom_x_cmd)
    ld.add_action(declare_map_to_odom_y_cmd)
    ld.add_action(declare_map_to_odom_yaw_cmd)
    ld.add_action(declare_use_zlocalization_backend_cmd)
    ld.add_action(declare_zlocalization_backend_config_cmd)
    ld.add_action(declare_backend_stabilized_body_cmd)
    ld.add_action(declare_zlocalization_prior_enabled_cmd)
    ld.add_action(declare_zlocalization_prior_initialization_only_cmd)
    ld.add_action(declare_zlocalization_loop_enabled_cmd)
    ld.add_action(declare_zlocalization_uwb_enabled_cmd)
    ld.add_action(declare_zlocalization_prior_map_cmd)
    ld.add_action(declare_zlocalization_trajectory_directory_cmd)

    # Add the actions to launch all of the localiztion nodes
    ld.add_action(start_point_lio_node)
    ld.add_action(start_imu_saturation_guard)
    ld.add_action(start_zlio_node)
    ld.add_action(start_zlio_bridge_node)
    ld.add_action(start_static_transform_node)
    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)
    ld.add_action(load_pcd_localization_node)
    ld.add_action(OpaqueFunction(function=_validate_zlocalization_backend))
    if start_zlocalization_backend is not None:
        ld.add_action(start_zlocalization_backend)

    return ld
