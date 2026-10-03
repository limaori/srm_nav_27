# 仿真导航算法与数据流

本文说明 `nav_simulation_launch.py` 启动的仿真导航系统中，各模块使用的算法和数据流。

## 一、当前仿真模式

当前使用的启动方式是：

```bash
ros2 launch srm27_nav_bringup nav_simulation_launch.py \
  world:=rmuc_2025 \
  map:=$HOME/srm_nav_27/src/srm27_navigation/srm27_nav_bringup/map/simulation/rmuc_2025_tunnel.yaml \
  slam:=False \
  use_pcd_localization:=False
```

在这个模式下：

- 地图由 `map_server` 直接读取 PGM/YAML 文件；
- `map -> odom` 使用静态 TF；
- `odom -> base_footprint` 使用 Gazebo 的真实里程计；
- 不需要 SLAM 或 PCD 重定位；
- `small_gicp_relocalization` 不应该运行。

如果运行中仍看到 `/red_standard_robot1/small_gicp_relocalization`，说明有旧进程残留，或者组合容器加载了不应加载的 GICP 组件。它没有点云时会发布无效的旧时间 TF，可能导致 TF 外推错误。

## 二、涉及的主要算法和模块

### 1. Gazebo 仿真

Gazebo 负责仿真世界、机器人模型、碰撞和运动，并发布：

- `/clock`：仿真时间；
- 激光雷达点云；
- `chassis_odometry_gt`：机器人真实里程计；
- 接收最终的 `/cmd_vel` 控制指令。

### 2. 点云格式转换

`ign_sim_pointcloud_tool` 将 Gazebo 点云转换为 Velodyne 风格的点云，主要补充：

- 扫描线编号 `ring`；
- 点的相对时间；
- 统一的 `velodyne_points` 点云格式。

这不是定位算法，只是传感器数据预处理。

### 3. 仿真里程计和 TF

`simulation_ground_truth_odometry.py` 使用 Gazebo 的真实位姿生成相对里程计：

```text
chassis_odometry_gt
        |
        v
simulation_ground_truth_odometry
        |
        +-- /red_standard_robot1/odometry
        +-- odom -> base_footprint
```

`robot_state_publisher` 根据 URDF 发布机器人内部固定 TF，例如：

```text
base_footprint -> chassis -> gimbal_yaw -> gimbal_yaw_fake
                              |
                              +-> front_mid360
                              +-> front_rplidar_a2
```

仿真中完整的导航 TF 链应为：

```text
map -> odom -> base_footprint -> chassis -> gimbal_yaw_fake
```

其中 `gimbal_yaw_fake` 是 Nav2 使用的机器人参考坐标系。

### 4. SLAM 和定位分支

当前 `slam:=False`，所以正常情况下不使用 SLAM。

如果使用 `slam:=True`，系统会启用：

- `pointcloud_to_laserscan`：点云转二维激光扫描；
- `slam_toolbox`：基于二维激光的建图/定位；
- `Point-LIO`：融合激光和 IMU 的里程计/建图算法。

如果使用 `use_pcd_localization:=True`，会启用：

- `small_gicp_relocalization`：将当前点云与先验 PCD 地图进行 GICP 配准，估计 `map -> odom`。

当前使用静态地图和仿真真实里程计时，不应同时启用 GICP，否则可能出现多个节点同时发布 `map -> odom`。

### 5. 地形分析

`terrain_analysis` 和 `terrain_analysis_ext` 对实时点云进行处理，主要包括：

- 体素下采样；
- 地面高度估计；
- 根据点相对地面的高度判断障碍；
- 动态障碍物过滤和清除；
- 地形连通性判断；
- 高度范围过滤，用于排除过高点或无效点。

输出话题为：

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
robot_radius: 0.2 m
inflation_radius: 0.7 m
```

局部代价地图使用 `odom` 作为全局坐标系，并跟随机器人滚动；全局代价地图使用 `map` 作为全局坐标系。

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

控制器根据当前机器人位姿和全局路径输出速度指令：

```text
cmd_vel_nav2_result
```

### 9. 速度平滑和底盘控制

速度指令经过 `velocity_smoother` 和 `fake_vel_transform` 处理后，输出：

```text
/red_standard_robot1/cmd_vel
```

Gazebo 接收该话题，驱动车辆运动。

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

## 三、完整数据流

```text
                         +----------------+
                         |     Gazebo     |
                         | 世界、机器人、传感器 |
                         +--------+-------+
                                  |
                 +----------------+----------------+
                 |                                 |
                 v                                 v
          /clock、真实里程计                    激光点云
                 |                                 |
                 v                                 v
 simulation_ground_truth_odometry       ign_sim_pointcloud_tool
                 |                                 |
                 +--> odometry                    +--> velodyne_points
                 +--> odom -> base_footprint              |
                                                          v
                                              terrain_analysis
                                              terrain_analysis_ext
                                                          |
                                     +--------------------+--------------------+
                                     |                                         |
                                     v                                         v
                                terrain_map                            terrain_map_ext
                                     |                                         |
                                     +--------------------+--------------------+
                                                          v
                                       Global/Local Costmap
                              static + voxel + inflation layers
                                                          |
                         +--------------------------------+----------------+
                         |                                                 |
                         v                                                 v
                    Theta* 全局规划                              当前机器人 TF
                         |                                      map -> odom -> base
                         v                                                 |
                    全局路径                                             |
                         |                                                 |
                         v                                                 |
                   SimpleSmoother                                      |
                         |                                                 |
                         +-------------------> Omni PID Pursuit <-----------+
                                                       |
                                                       v
                                           cmd_vel_nav2_result
                                                       |
                                                       v
                                      velocity_smoother/fake_vel_transform
                                                       |
                                                       v
                                                  /cmd_vel
                                                       |
                                                       v
                                                    Gazebo
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
4. `map -> odom -> gimbal_yaw_fake` TF 时间连续；
5. 洞的有效宽度大于机器人本体尺寸和安全膨胀范围；
6. 第一次测试应使用洞口前、洞内中心、出口中心等多个连续目标点，并降低速度。

桥下通道的特殊风险是：三维点云投影到二维代价地图后，桥面或洞顶可能与地面落在相同的 `(x, y)` 栅格中。如果静态地图是白色但代价地图重新变黑，应优先检查地形高度过滤和体素层，而不是直接缩小机器人半径。
