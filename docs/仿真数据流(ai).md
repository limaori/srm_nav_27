# 仿真导航算法与数据流

> 本文由 AI 生成并随 SRM 仿真重构更新：原文对应的旧步兵链路（`gimbal_yaw_fake` /
> `fake_vel_transform` / `MecanumDrive2` / `base_footprint`）已从 SRM 仿真中删除，
> 本文描述重构后的实际链路。落地用法见 [`SRM仿真与自转控制实现(ai).md`](./SRM仿真与自转控制实现%28ai%29.md)。

本文说明 `nav_srm_simulation_launch.py` 启动的 SRM 仿真导航系统中，各模块使用的算法和数据流。
默认命名空间为 `red_standard_robot1`，下文话题名前缀 `<ns>` 即指它。所有速度接口都是车体系
（x 前、y 左、z 上）的 `geometry_msgs/msg/Twist`。

## 一、当前仿真模式

当前使用的启动方式是：

```bash
./script/start_sim_nav.sh                       # 默认 rmuc_2025 + 隧道地图

# 等价的拆开启动
ros2 launch srm27_gazebo_simulator srm_sim.launch.py
ros2 launch srm27_nav_bringup nav_srm_simulation_launch.py \
  namespace:=red_standard_robot1 \
  world:=rmuc_2025 \
  map:=$HOME/srm_nav_27/src/srm27_navigation/srm27_nav_bringup/map/simulation/rmuc_2025_tunnel.yaml \
  params_file:=$HOME/srm_nav_27/src/srm27_navigation/srm27_nav_bringup/config/simulation/nav2_params_srm.yaml \
  slam:=False \
  use_pcd_localization:=False
```

在这个模式下：

- 机器人模型由 `srm27_robot_description` 按 `config/srm27_sentry_geometry.yaml` 现场生成，
  只含圆柱底盘、四个轮组、MID360 与底盘 IMU；
- 底盘速度由 `srm27_gazebo_simulator` 的 `srm_velocity_system` 插件执行，
  命令来自适配器转发的 `/<ns>/cmd_vel_sim`；
- 地图由 `map_server` 直接读取 PGM/YAML 文件；
- `map -> odom` 使用静态 TF（`map_to_odom_x/y/yaw` 默认全为 `0.0`）；
- `odom -> base_link` 由 `simulation_ground_truth_odometry` 使用 Gazebo 真值发布；
- 不需要 SLAM、PCD 重定位或 Point-LIO 里程计；
- `small_gicp_relocalization`、`point_lio` 不应该运行。

如果运行中仍看到 `/<ns>/small_gicp_relocalization` 或 `/<ns>/point_lio`，说明有旧进程残留，
或者命令行传入了 `use_pcd_localization:=True` / `use_lio_odometry:=True`。GICP 没有点云时会发布
无效的旧时间 TF，可能导致 TF 外推错误；它也可能与静态 TF 争抢 `map -> odom`。

三种定位模式（互斥）与 `odom -> base_link` 的来源：

| 模式 | `map -> odom` 发布者 | `odom -> base_link` 发布者 | 仿真真值里程计 |
| --- | --- | --- | --- |
| 默认（三者都 False） | `static_transform_publisher` | `simulation_ground_truth_odometry` | 启动 |
| `use_lio_odometry:=True` | `static_transform_publisher` | `sensor_scan_generation`（Point-LIO 里程计） | 不启动 |
| `use_pcd_localization:=True` | `small_gicp_relocalization` | `sensor_scan_generation`（Point-LIO 里程计） | 不启动 |
| `slam:=True` | `slam_toolbox` | `sensor_scan_generation`（Point-LIO 里程计） | 不启动 |

## 二、涉及的主要算法和模块

### 1. Gazebo 仿真

Gazebo 负责仿真世界、机器人模型、碰撞和运动，并发布：

- `/clock`：仿真时间（由 `ros_gz_bridge` 桥接到 ROS）；
- `/<ns>/livox/lidar`：MID360 点云；`/<ns>/livox/imu`：MID360 IMU；
- `/<ns>/joint_states`：关节状态，供 `robot_state_publisher` 发布轮组 TF；
- `/<ns>/chassis_odometry_gt`：速度插件按 50 Hz 发布的真值里程计；
- 接收 Gazebo Transport 上的 `<robot_name>/cmd_vel` 速度指令。

底盘速度执行由 SRM 自有插件完成：

```text
ROS cmd_vel_sim ──► srm_velocity_adapter ──► <robot_name>/cmd_vel（Gazebo Transport）
                                                   │
                                                   v
                                       SrmVelocitySystem（PreUpdate 设置 Link 平面速度）
                                                   │
                                                   v
                                       物理引擎推进位姿、接触与传感器
                                                   │
                                                   v
                            PostUpdate 发布 <robot_name>/odometry（50 Hz 真值）
```

插件按 **Link 坐标系（车体系）** 调用 `SetLinearVelocity` / `SetAngularVelocity`，只覆盖平面
`vx/vy/wz`，保留 z 平移与 roll/pitch 角速度；不做轮速 PID、力矩控制或额外旋转策略。
仿真暂停、仿真重置与命令超时（用仿真时间，默认 0.1 s）时清零。

底盘速度命令**不走 bridge**：适配器直接写 Gazebo Transport，以免桥接直通绕过 NaN/Inf 校验与超时清零。

### 2. 点云格式转换

`ign_sim_pointcloud_tool` 把 `livox/lidar` 点云转换为 Velodyne 风格的点云，主要补充：

- 扫描线编号 `ring`；
- 点的相对时间；
- 统一的 `velodyne_points` 点云格式（32 线、1875 点/线、`scan_period: 0.0`）。

这不是定位算法，只是传感器数据预处理。

### 3. 仿真里程计和 TF

`simulation_ground_truth_odometry.py` 使用 `chassis_odometry_gt` 生成相对里程计（50 Hz）：

```text
/<ns>/chassis_odometry_gt
        |
        v
simulation_ground_truth_odometry
        |
        +-- /<ns>/odometry            （相对位姿 + 实际速度，child_frame_id = base_link）
        +-- odom -> base_link         （TF）
        +-- /<ns>/lidar_odometry      （叠加雷达外参后的雷达位姿；sensor_scan_generation 与
                                        terrain_analysis(_ext) 订阅它，把点云和雷达位姿配对）
```

- 首帧位姿作为里程计原点，并抑制静止时的物理噪声（小于 0.002 的分量置零）。
- 雷达外参来自 `srm27_robot_description` 的几何 YAML（`base_link -> front_mid360`），
  读取失败时回退到内置默认值并打印告警；`base_frame` 参数默认 `base_link`，`lidar_frame` 默认
  `front_mid360`。该节点不发布雷达 TF，雷达静态 TF 由 `robot_state_publisher` 负责。
- 该节点在 launch 里显式设置 `use_sim_time: False`：bridge 已经给里程计消息打了仿真时间戳，
  它不需要再订阅高频 `/clock`。

`robot_state_publisher` 根据 `build_urdf()` 生成的 URDF 发布机器人内部固定 TF：

```text
base_link ─┬─► front_mid360
           ├─► front_left_wheel
           ├─► front_right_wheel
           ├─► rear_left_wheel
           └─► rear_right_wheel
```

仿真中完整的导航 TF 链为：

```text
map ──► odom ──► base_link ──┬─► front_mid360
                             └─► *_wheel（4 个轮组）
```

- `base_link` 是与实际底盘刚性固定的坐标系，**随底盘自转**，Nav2 的
  `robot_base_frame`、代价地图 `robot_base_frame` 与速度参考系统一使用它；
- 模型里没有 `base_footprint`、`chassis`、`gimbal_yaw`、`gimbal_yaw_fake` 等中间坐标系
  （那些是旧步兵链路，已从 SRM 仿真删除）；
- 每条 TF 边保持唯一发布者：真值模式下 `sensor_scan_generation` 的 `publish_tf` 被置为 `false`，
  只有 Point-LIO / GICP / SLAM 模式才由它发布 `odom -> base_link`。

### 4. SLAM 和定位分支

当前 `slam:=False`，所以正常情况下不使用 SLAM。

如果使用 `slam:=True`，系统会启用：

- `pointcloud_to_laserscan`：点云转二维激光扫描；
- `slam_toolbox`：基于二维激光的建图/定位，发布 `map -> odom`；
- `point_lio`：融合激光和 IMU 的里程计/建图算法。

如果使用 `use_pcd_localization:=True`，会启用：

- `point_lio` + `small_gicp_relocalization`：将当前点云与先验 PCD 地图做 GICP 配准，估计 `map -> odom`。

如果使用 `use_lio_odometry:=True`（纯 LIO），只用 Point-LIO 里程计，`map -> odom` 仍是静态 TF。
这三种模式与默认模式互斥，否则会有多个节点同时发布 `map -> odom`。

### 5. 地形分析

`terrain_analysis` 和 `terrain_analysis_ext` 对实时点云进行处理，主要包括：

- 体素下采样；
- 地面高度估计；
- 根据点相对地面的高度判断障碍；
- 动态障碍物过滤和清除；
- 地形连通性判断；
- 高度范围过滤，用于排除过高点或无效点。

输入话题随定位模式切换：无定位里程计时订阅 `velodyne_points`，启用 Point-LIO / GICP / SLAM 时
订阅 `registered_scan`。输出话题为：

```text
terrain_analysis     -> terrain_map
terrain_analysis_ext -> terrain_map_ext
```

### 6. 代价地图

全局和局部代价地图都使用以下图层：

```text
static_layer
intensity_voxel_layer
inflation_layer
```

作用分别是：

- `static_layer`：加载 PGM 静态地图；
- `intensity_voxel_layer`：根据实时地形点云添加障碍；
- `inflation_layer`：将障碍周围区域按安全距离膨胀。

当前主要参数为：

```text
robot_radius: 0.33 m       # SRM 整车碰撞包络半径（几何 YAML 的 collision.envelope_radius）
inflation_radius: 0.7 m
```

`robot_radius` 取整车外接包络而不是圆柱半径 0.27 m：轮组外廓、雷达偏置安装和传感器外形都要覆盖。
局部代价地图使用 `odom` 作为全局坐标系并跟随机器人滚动，`robot_base_frame` 为 `base_link`；
全局代价地图使用 `map` 作为全局坐标系。

### 7. 全局规划

规划器为：

```text
nav2_theta_star_planner/ThetaStarPlanner
```

Theta* 在栅格地图上搜索路径，同时利用栅格之间的可视性减少不必要的折点。输入是全局代价地图、起点和目标点，输出是全局路径。

之后由：

```text
nav2_smoother::SimpleSmoother
```

对路径进行平滑处理。

### 8. 局部控制

局部控制器为：

```text
srm27_omni_pid_controller::OmniPidPursuitController
```

它结合：

- 全向底盘控制；
- 前视点跟踪；
- 平移误差 PID；
- 路径曲率计算；
- 高曲率降速；
- 速度上下限限制。

控制器以 20 Hz 运行，把路径变换到代价地图的机器人系（`base_link`）后输出速度指令。
SRM 仿真参数关闭了独立旋转控制：`enable_rotation: false`、`use_rotate_to_heading: false`，
因此**导航链路只产生 `vx`、`vy`**；`min_y_velocity_threshold` 由 0.5 降到 0.001，
避免抹掉低速横移反馈。

### 9. 速度出口、合成与底盘控制

导航链路的最终速度出口是 `/<ns>/cmd_vel_nav`：

```text
use_velocity_smoother:=True （默认）
  controller_server ─ cmd_vel_controller ─► velocity_smoother ─► cmd_vel_nav
use_velocity_smoother:=False
  controller_server ─► cmd_vel_nav
```

两条分支互斥，`lifecycle_manager` 的 `node_names` 会随是否启用平滑器变化。
`behavior_server` 与 `bt_navigator` 的 `cmd_vel` 也重映射到同一个出口，其中的角速度分量会被 mux 丢弃。

合成与执行：

```text
/<ns>/cmd_vel_nav ──────┐
                        ├─ srm_cmd_mux ──► /<ns>/cmd_vel_sim ──► srm_velocity_adapter ──► Gazebo
/<ns>/rotation_velocity ┘

rotation_test_sender（stop / constant / periodic）
        │  /<ns>/rotation_cmd
        v
rotation_controller
        │  /<ns>/rotation_velocity
        └──────────────────────────────► srm_cmd_mux 的自转输入
```

- `srm_cmd_mux` 是 `cmd_vel_sim` 的**唯一发布者**：`vx/vy` 先单轴限幅再按向量模长限到 `v_max`，
  `wz` 只取 `rotation_velocity` 并限到 `wz_max=2.0`；导航输入的 `angular.z` 一律丢弃并统计次数。
- 默认限幅与超时：`vx_max=vy_max=v_max=0.5 m/s`、`wz_max=2.0 rad/s`、`nav_timeout=0.3 s`、
  `rotation_timeout=0.1 s`、合成频率 200 Hz。
- 自转请求超时默认 0.5 s（`rotation_controller`），自转波形时间取仿真时间。
- `srm_velocity_adapter` 超时默认 0.1 s、转发频率 200 Hz，含 NaN/Inf 时按零速处理并计数。
- 急停/暂停/退出都会清零：`/<ns>/srm_cmd_mux/stop_all` 保持零输出直到 `resume_all`；
  仿真暂停与重置由插件直接清空速度缓存。

导航与自转互不覆盖：导航停发后平移清零，但自转仍按发送器的启用状态继续；反之亦然。

### 10. 行为树和恢复行为

`bt_navigator` 使用 Nav2 行为树组织整个导航流程，包括：

- 接收目标；
- 请求全局规划；
- 请求路径跟踪；
- 检查目标是否到达；
- 规划失败或卡住时执行恢复行为。

恢复行为包括：

- 原地旋转 `Spin`；
- 后退 `BackUpFreeSpace`；
- 沿航向行驶 `DriveOnHeading`；
- 辅助遥控；
- 等待。

行为树本次未修改；恢复行为输出的角速度分量同样由 mux 丢弃。

### 11. 诊断

四个节点各自发布 `diagnostic_msgs/msg/DiagnosticArray` 到 `/<ns>/diagnostics`：

| 状态名 | 关键字段 |
| --- | --- |
| `srm27_chassis_control: cmd_mux` | `nav_vx`、`nav_vy`、`nav_wz_dropped_count`、`non_finite_*_count`、`stopped`、`mixed_vx/vy`、`rotation_wz`、`translation_scale`、`clamped`、`nav_timeout`、`rotation_timeout` |
| `srm27_chassis_control: rotation_controller` | `target_wz`、`output_wz`、`request_timeout`、`non_finite_count` |
| `srm27_chassis_control: rotation_test_sender` | `mode`、`angular_speed`、`offset`、`amplitude`、`period`、`phase`、`sine_wave`、`waveform_time`、`rotation_wz` |
| `srm27_gazebo_simulator: velocity_adapter` | `gz_topic`、`command_timeout`、`commands`、`non_finite_commands` |

三类计数（丢弃的导航角速度、非有限输入、超时）在诊断里可见，是排查“速度被清零/被限幅”的第一手依据。

## 三、完整数据流

```text
                         +---------------------+
                         |   Gazebo Fortress   |
                         | 世界、SRM 模型、传感器 |
                         +----------+----------+
                                    |
        +---------------------------+-----------------------------+
        |                           |                             |
        v                           v                             v
 /clock、chassis_odometry_gt    livox/lidar、livox/imu        joint_states
        |                           |                             |
        v                           v                             v
 simulation_ground_truth_     ign_sim_pointcloud_tool     robot_state_publisher
        odometry                    |                      base_link -> front_mid360 / *_wheel
        |                           +--> velodyne_points
        +--> odometry               |
        +--> odom -> base_link      v
        +--> lidar_odometry   terrain_analysis(_ext)
                                    |
                    +---------------+---------------+
                    |                               |
                    v                               v
               terrain_map                     terrain_map_ext
                    |                               |
                    +---------------+---------------+
                                    v
                     Global/Local Costmap（robot_base_frame: base_link）
                        static + voxel + inflation layers
                                    |
             +----------------------+----------------------+
             |                                             |
             v                                             v
        Theta* 全局规划                            当前机器人 TF
             |                                map -> odom -> base_link
             v                                             |
       SimpleSmoother                                      |
             |                                             |
             +--------------> Omni PID Pursuit <-----------+
                                    |
                                    v
                        cmd_vel_controller（20 Hz）
                                    |
                                    v
                          velocity_smoother
                                    |
                                    v
                              cmd_vel_nav ─────────────┐
                                                       ├─ srm_cmd_mux
                        rotation_test_sender           │       │
                              │                        │       v
                              v                        │  cmd_vel_sim（200 Hz）
                      rotation_controller              │       │
                              │                        │       v
                              └──► rotation_velocity ──┘  srm_velocity_adapter
                                                               │
                                                               v
                                                 Gazebo Transport <model>/cmd_vel
                                                               │
                                                               v
                                                     SrmVelocitySystem 执行
```

RViz 主要是显示和交互工具：

```text
2D Pose Estimate -> initialpose
2D Goal Pose    -> Nav2 导航目标
Map/Costmap/TF  <- 各模块发布的数据
```

## 四、通过洞口时需要关注的内容

过洞不需要额外的专用导航算法，标准的二维规划和控制器即可完成。需要确保：

1. PGM 静态地图中洞口是自由空间；
2. Global Costmap 和 Local Costmap 中洞口都没有致命障碍；
3. `terrain_map` 和 `terrain_map_ext` 没有将洞顶、桥面误判为地面障碍；
4. `map -> odom -> base_link` TF 时间连续；
5. 洞的有效宽度大于整车碰撞包络（`robot_radius: 0.33 m`）和安全膨胀范围；
6. 第一次测试应使用洞口前、洞内中心、出口中心等多个连续目标点，并降低速度；
7. 若同时启用自转叠加，还要检查自转扫掠范围内的碰撞（包络按实际姿态取外接圆）。

桥下通道的特殊风险是：三维点云投影到二维代价地图后，桥面或洞顶可能与地面落在相同的 `(x, y)` 栅格中。如果静态地图是白色但代价地图重新变黑，应优先检查地形高度过滤和体素层，而不是直接缩小机器人半径。
