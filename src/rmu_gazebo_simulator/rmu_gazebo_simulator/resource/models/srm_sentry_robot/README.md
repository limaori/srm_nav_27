# SRM 哨兵 Fortress 模型

底盘和轮组尺寸来自 `srm27_nav_bringup` 包内的 `urdf/sentry_robot_cylinder.xacro`（SRM 实车模型的本地副本，上游为 `~/srm_auto_sentry` 的 `rm_nav_bringup`）：底盘为半径 0.27 m、高 0.20 m 的圆柱，质量 8.2 kg；四轮半径 0.075 m、宽 0.04 m，轮心位于 `(±0.18, ±0.18, 0.075)`。

MID360 安装位姿与本工作区 SRM 实车入口一致：`xyz = 0.15 -0.15 0.22`，`rpy = -0.06981317007977318 0 -1.5707963267948966`。仿真继续使用 `front_mid360` 坐标系，点云和 IMU 话题保持为机器人命名空间下的 `livox/lidar` 和 `livox/imu`。

SRM 源模型使用 Gazebo Classic。本模型适配 Gazebo Fortress，保留现有 MecanumDrive2、云台、装甲和裁判系统模块以兼容仿真控制接口；这些模块是通用仿真组件，并非 SRM 实车外观复刻。仿真 Nav2 碰撞半径设为 0.33 m，以覆盖轮组外廓。

`spawn_robots.launch.py` 展开本目录的 SDF XMacro 并生成用于 TF 发布的 URDF。模型与 MID360 资源随 `rmu_gazebo_simulator` 安装。通用仿真组件由 `rmoss_gz_resources` 提供。
