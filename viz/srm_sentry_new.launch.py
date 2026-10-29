"""在 RViz 里查看车体模型（纯展示，不启动导航/驱动）。

唯一模型：srm27_robot_description/urdf/srm27_sentry.urdf.xacro
——实车建图/导航/独立调试用的都是这一份，这里看到的车就是跑车时的车。

两种看法（同一份文件，只改坐标系契约参数）:
  · 默认（实车契约）: base_link 原点在地面、轮子与云台 fixed、雷达帧 livox_frame
      ros2 launch viz/srm_sentry_new.launch.py
  · CAD 原稿视图   : base_footprint 根 + base_link 离地 0.1009 + 连续关节
      ros2 launch viz/srm_sentry_new.launch.py cad_view:=true

常用覆盖（留空 = 用 xacro 文件里的默认值）:
  model:=/abs/path/other.xacro   换一个模型（比如你自己改的试验稿）
  rviz:=false                    只发 /robot_description 与 TF，不开窗口
  cad_view:=true                 切到 CAD 装配体坐标系（见上）
  wheel_joint_type:=continuous   轮子改成连续关节（要配 joint_state_publisher）
  lidar_link_name:=front_mid360  用仿真链路的雷达帧名
  use_yaw:=false                 不画 yaw 半透明圆柱，雷达黑点看得更清
  gimbal_fixed_yaw:=0.8          把云台（连同它上面的雷达）摆到 0.8 rad 看几何关系
                                 （仅 gimbal_joint_type=fixed 时生效，不需要 /joint_states）
  lidar_xyz:="x y z"             临时改雷达位置（地面=0，米）
  lidar_roll:=0.0 等             临时改雷达姿态（弧度）
  例: ros2 launch viz/srm_sentry_new.launch.py use_yaw:=false

RViz 里要确认的三件事:
  · Global Options -> Fixed Frame 默认是 base_link（本文件已配好）；
    用 cad_view:=true 时请自己切到 base_footprint
  · RobotModel 显示插件已启用, Description Topic = /robot_description
  · 想看关节动起来: 用 wheel_joint_type:=continuous（或 cad_view:=true），
    本文件会同时起一个 joint_state_publisher

为什么还要 joint_state_publisher:
  实车契约下所有关节都是 fixed，本来不需要它；但 continuous/revolute 的关节要有
  /joint_states 才有 TF，缺了 RViz 会报 "No transform from [wheel_*] to [base_link]"
  并且不画。起一个全 0 的 joint_state_publisher 对两种模型都安全（fixed 模型上
  它只发空消息）。
"""

import os

import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
MODEL = os.path.join(
    REPO, "src/srm27_robot_description/urdf/srm27_sentry.urdf.xacro"
)
RVIZ_CONFIG = os.path.join(HERE, "srm_sentry_new.rviz")

# 允许从命令行覆盖的 xacro 参数；空字符串表示"沿用 xacro 里的 default"
XACRO_ARGS = (
    "use_yaw",
    "lidar_xyz",
    "lidar_roll",
    "lidar_pitch",
    "lidar_yaw",
    "lidar_radius",
    "lidar_post_length",
    "lidar_link_name",
    "root_frame",
    "wheel_joint_type",
    "gimbal_joint_type",
    "gimbal_fixed_yaw",
)


def launch_setup(context):
    model = LaunchConfiguration("model").perform(context)
    if not os.path.isfile(model):
        raise FileNotFoundError(model)

    mappings = {}
    for name in XACRO_ARGS:
        value = LaunchConfiguration(name).perform(context).strip()
        if value:
            mappings[name] = value
    # cad_view:=true 只是把三个契约参数一起切过去（也可以单独覆盖）。
    if LaunchConfiguration("cad_view").perform(context).lower() in ("1", "true", "yes"):
        mappings.setdefault("root_frame", "base_footprint")
        mappings.setdefault("wheel_joint_type", "continuous")
        mappings.setdefault("gimbal_joint_type", "revolute")

    try:
        robot_xml = xacro.process_file(model, mappings=mappings).toxml()
    except Exception as error:  # xacro 语法/表达式错误直接打出来，不静默吞掉
        raise RuntimeError(f"xacro processing failed for {model}: {error}") from error

    use_sim_time = LaunchConfiguration("use_sim_time")
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[
                {"robot_description": robot_xml},
                {"use_sim_time": use_sim_time},
            ],
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", RVIZ_CONFIG],
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ]


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("model", default_value=MODEL, description="xacro 文件路径"),
        DeclareLaunchArgument("rviz", default_value="true", description="是否打开 RViz"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "cad_view",
            default_value="false",
            description="按 CAD 装配体坐标系看（base_footprint 根 + 连续关节）",
        ),
    ]
    arguments += [
        DeclareLaunchArgument(name, default_value="", description=f"覆盖 xacro 的 {name}（空=用默认）")
        for name in XACRO_ARGS
    ]
    return LaunchDescription([*arguments, OpaqueFunction(function=launch_setup)])
