# srm_nav_27

深圳北理莫斯科大学北极熊战队哨兵机器人 ROS 2 工作空间。

本文以**框架架构**为主：分层、包职责、数据流、TF 链和启动入口。具体现场操作步骤见 [`docs/`](./docs/)。

---

## 1. 项目概览

| 项 | 内容 |
| --- | --- |
| 目标平台 | RoboMaster 哨兵机器人（SRM 实车 + Gazebo 仿真） |
| 操作系统 | Ubuntu 22.04 |
| 中间件 | ROS 2 Humble |
| 仿真器 | Ignition Gazebo Fortress |
| 主传感器 | Livox Mid-360（网口，点云 + 内置 IMU） |
| 决策框架 | BehaviorTree.CPP 4.x + BehaviorTree.ROS2 |
| 导航框架 | Nav2（Theta\* 全局规划 + 自研全向 PID 局部控制） |
| 仿真执行 | `srm27_gazebo_simulator`（SRM 模型、自有速度插件、速度适配器） |

工作空间同时承载**实车**与**仿真**两条链路，二者共用同一套导航栈，只在传感器来源、里程计来源和坐标系命名上不同。

---

## 2. 架构总览

### 2.1 分层

```text
┌────────────────────────────────────────────────────────────────────┐
│ 决策层    srm27_behavior                                           │
│   BehaviorTree.CPP 服务端 + 行为树插件（条件 / 动作 / 控制 / 装饰）    │
│   输入：裁判系统、RFID、受击状态    输出：导航目标、云台角、底盘转速     │
└──────────────────────────────┬─────────────────────────────────────┘
                               │ NavigateToPose / NavigateThroughPoses
┌──────────────────────────────v─────────────────────────────────────┐
│ 导航层    srm27_nav_bringup（Nav2）                                 │
│   bt_navigator → planner(Theta*) → smoother → controller(OmniPID)  │
│   代价地图：static + intensity_voxel + inflation                    │
│   行为恢复：Spin / BackUpFreeSpace / DriveOnHeading / Wait          │
└───────┬──────────────────────────────────────────┬─────────────────┘
        │ TF、代价地图、里程计                       │ 速度出口（实车 / 仿真分叉）
┌───────v────────────────────────────┐   ┌─────────v──────────────────────┐
│ 定位与感知层                        │   │ 实车：cmd_vel_controller        │
│   point_lio            激光惯性里程计│   │        → /cmd_vel              │
│   small_gicp_reloc.    先验图重定位  │   │        → srm27_nav_protocol│
│   slam_toolbox         二维建图      │   │ 仿真：cmd_vel_nav              │
│   terrain_analysis(_ext) 地形分析    │   │        → srm_cmd_mux           │
│   sensor_scan_generation  odom→base  │   │        → cmd_vel_sim           │
│   loam_interface / livox_ros_driver2 │   │        → srm_velocity_adapter  │
└───────┬──────────────────────────────┘   │        → SrmVelocitySystem     │
        │ 传感器数据                        └────────────────────────────────┘
┌───────v────────────────────────────────────────────────────────────┐
│ 硬件 / 仿真层                                                       │
│   实车：Livox Mid-360、下位机 C 板                                   │
│   仿真：srm27_gazebo_simulator（SRM 模型、速度执行、传感器桥接）       │
│         同包提供世界 SDF、GUI 配置、MID-360 等场地素材              │
└────────────────────────────────────────────────────────────────────┘
```

跨层契约由 `interfaces/rm_decision_interfaces` 统一提供（裁判系统、决策、云台消息）。

### 2.2 工作空间布局

```text
srm_nav_27/
├── src/
│   ├── srm27_bringup/        实车总启动入口、node_params、RViz 配置
│   ├── srm27_behavior/       行为树服务端 / 客户端 / 插件
│   ├── srm27_navigation/     导航相关 ROS 包集合（每个子目录是独立包）
│   ├── srm27_robot_description/  SRM 几何与外参的唯一定义（URDF/SDF 生成）
│   ├── srm27_chassis_control/    速度合成（srm_cmd_mux）与独立自转控制
│   ├── srm27_gazebo_simulator/   SRM 仿真入口、速度执行、场地与传感器素材
│   ├── srm27_nav_protocol/  下位机串口通信
│   ├── interfaces/           统一消息接口
│   ├── tools/                开发工具（pcd2pgm、录包、键盘云台）
│   └── dependencies/         第三方依赖（vcs 拉取，勿手改）
├── script/                   现场启动与诊断脚本
├── docs/                     使用指南与设计记录
├── maps/                     实车场地地图（按场地名分目录）
├── build/ install/ log/      colcon 构建输出
└── dependencies.repos        vcs 依赖清单
```

关于 `maps/` 与包内 `map/`、`pcd/` 的分工：**包内目录是随包发布的默认数据**（如 `srm27_nav_bringup/map/real/srm_site_01.*`），**工作区根 `maps/` 是现场采集的场地地图**。两者都可用 `--map` 指定。

### 2.3 包清单

**决策层**

| 包 | 职责 |
| --- | --- |
| `srm27_behavior` | 行为树服务端与客户端；SRM 自定义 BT 插件（条件/动作/控制/装饰节点） |

**导航层**

| 包 | 职责 |
| --- | --- |
| `srm27_nav_bringup` | Nav2 启动、参数、地图、PCD、RViz、行为树 XML |
| `srm27_nav_plugins` | Nav2 扩展插件：`srm27_nav_behaviors/BackUpFreeSpace`、`srm27_nav_costmap::IntensityVoxelLayer` |
| `srm27_omni_pid_controller` | 全向底盘 PID 路径跟踪控制器 `OmniPidPursuitController` |
| `srm27_teleop_twist_joy` | 手柄速度与云台控制 |
| `srm27_navigation` | 导航包集合的元包（meta package） |

**定位与感知层**

| 包 | 职责 |
| --- | --- |
| `point_lio` | 激光惯性里程计（LOAM 系） |
| `small_gicp_relocalization` | 用先验 PCD 做 GICP 配准，发布 `map → odom` |
| `sensor_scan_generation` | 生成传感器系下的扫描，发布 `odom → base` |
| `loam_interface` | Point-LIO 等里程计算法的接口层 |
| `terrain_analysis` / `terrain_analysis_ext` | 车体 4 m 内 / 外的地形分析，把障碍离地高度写入点云 `intensity` |
| `fake_vel_transform` | 旧步兵链路：构造不随云台旋转的虚拟底盘系 `gimbal_yaw_fake`。**SRM 仿真已不再使用**（见 §3.3），仅为 `nav2_params_upstream.yaml` / `srm27_bringup` 的旧配置保留 |
| `livox_ros_driver2` | Mid-360 驱动 |
| `ign_sim_pointcloud_tool` | 仿真点云转 Velodyne 格式（补 `ring` 与相对时间） |
| `pointcloud_to_laserscan` | 点云降维为二维激光（仅 SLAM 模式使用） |

**通信与接口**

| 包 | 职责 |
| --- | --- |
| `srm27_nav_protocol` | 下位机串口协议；实车 `/cmd_vel` 的唯一消费者，同时发布关节状态用于整车 TF |
| `rm_decision_interfaces` | 统一的裁判系统、决策、云台消息接口 |

**仿真与工具**

| 包 | 职责 |
| --- | --- |
| `srm27_robot_description` | SRM 几何与外参的**唯一定义**（`config/srm27_sentry_geometry.yaml`）；由同一份 YAML 生成 URDF 与 Gazebo SDF，并带有与之一致的公共 xacro（`urdf/srm27_sentry.urdf.xacro`）；实车链路目前仍用 `srm27_nav_bringup/urdf/sentry_robot_cylinder.xacro` |
| `srm27_chassis_control` | 速度合成与独立自转：`srm_cmd_mux`（导航平移 + 独立自转，`cmd_vel_sim` 的唯一发布者）、`rotation_controller`、`rotation_test_sender`，含纯逻辑 gtest |
| `srm27_gazebo_simulator` | SRM 仿真入口 `srm_sim.launch.py`、自有 Gazebo 速度插件 `srm_velocity_system`、速度适配器 `srm_velocity_adapter`、空场世界 `worlds/srm_empty.sdf`，以及 `resource/` 下的比赛场地、GUI 配置和 MID-360 模型 |
| `pcd2pgm` | `.pcd` 转 `.pgm` 栅格图 |
| `rosbag2_composable_recorder` | 按裁判系统状态触发录包 |
| `teleop_gimbal_keyboard` | 键盘控制云台 |

> [!NOTE]
> 仿真与实车的速度出口不同：**实车**是
> `controller_server → cmd_vel_controller → velocity_smoother → cmd_vel_nav2_result → fake_vel_transform → cmd_vel_chassis → srm27_nav_protocol`；
> **仿真**是 `cmd_vel_nav → srm_cmd_mux → cmd_vel_sim → srm_velocity_adapter → SrmVelocitySystem`。
> 注意底盘串口节点订阅的是 **`cmd_vel_chassis`**（不是 `/cmd_vel`，见 `srm27_nav_protocol.cpp`），
> `fake_vel_transform` 就是这一级改名的地方；若绕过它，必须把导航出口话题直接设成 `cmd_vel_chassis`，
> 否则链路断开、车不动。

---

## 3. 数据流

### 3.1 实车链路

```text
Livox Mid-360 ──► livox_ros_driver2 ──► Point-LIO ──► loam_interface
                        │                    │              │
                        │                    │              v
                        │                    │      registered_scan（map 系）
                        │                    v
                        │           sensor_scan_generation ──► odom → base_link
                        v
                 terrain_analysis ──► terrain_map      ─┐
                 terrain_analysis_ext ──► terrain_map_ext ─┤
                                                          v
                                              代价地图 intensity_voxel_layer
                                                          │
  ┌───────────────────────────────────────────────────────┘
  v
Theta* 全局规划 ──► SimpleSmoother ──► OmniPID 局部控制
                                            │
                                            v
                                   cmd_vel_controller ──► velocity_smoother
                                                                │
                                                                v
                                                  cmd_vel_nav2_result
                                                                │
                                                                v
                                                     fake_vel_transform
                                                                │
                                                                v
                                                        cmd_vel_chassis
                                                                │
                                                                v
                                              srm27_nav_protocol ──► 串口 ──► 下位机 C 板
```

### 3.2 仿真链路

传感器来源与执行方式与实车不同：Gazebo 提供点云与真值，速度经合成后由自有插件执行。

```text
Gazebo ──┬─► livox/lidar ──► ign_sim_pointcloud_tool ──► velodyne_points
         │                                                      │
         │                                         terrain_analysis(_ext)
         │                                                      │
         └─► chassis_odometry_gt（真值 50 Hz）                  v
                     │                              代价地图 / 规划 / 控制
                     v                                          │
   simulation_ground_truth_odometry                             v
                     ├─► odometry                     cmd_vel_nav（导航平移）
                     └─► odom → base_link                       │
                                                                v
   rotation_test_sender ─► rotation_controller ─► rotation_velocity ─┐
                                                                     ├─ srm_cmd_mux
                                     cmd_vel_nav ────────────────────┘      │
                                                                            v
                                                    cmd_vel_sim（200 Hz，车体系）
                                                                            │
                                                                            v
                                           srm_velocity_adapter（校验 + 超时清零）
                                                                            │
                                                                            v
                                      SrmVelocitySystem（Gazebo Transport cmd_vel）
                                                                            │
                                                                            v
                                                                         Gazebo
```

速度链路规则（详见 [`docs/SRM仿真与自转控制实现(ai).md`](./docs/SRM仿真与自转控制实现%28ai%29.md)）：

```text
Nav2 controller ─ cmd_vel_nav ─┐
                               ├─ srm_cmd_mux ─ cmd_vel_sim ─► srm_velocity_adapter ─► Gazebo
自转测试链路 ─ rotation_velocity ┘
```

- 导航链路只输出 `vx`、`vy`（`enable_rotation: false`、`use_rotate_to_heading: false`）；
  `srm_cmd_mux` **丢弃**导航输入的 `angular.z` 并计数，`wz` 只来自 `rotation_velocity`。
- 合成限幅：`vx/vy` 先单轴限到 0.5 m/s，再按向量模长限到 `v_max=0.5 m/s`；`wz` 限到 2.0 rad/s。
- 超时清零：导航输入 0.3 s、自转输入 0.1 s（mux），执行适配器 0.1 s；计时用单调时钟，
  波形相位用仿真时间。
- `cmd_vel_sim` 的**唯一发布者**是 `srm_cmd_mux`；适配器把车体系 Twist 转成
  `ignition.msgs.Twist` 直接写 Gazebo Transport（不走 `ros_gz_bridge`，以免绕过校验）。
- 急停服务：`/<ns>/srm_cmd_mux/stop_all` 与 `/<ns>/srm_cmd_mux/resume_all`。

### 3.3 坐标系（TF 链）

**实车**（SRM 自建模型，`config/real/nav2_params_srm.yaml`）

```text
map ──► odom ──► base_link ──┬─► livox_frame ──► livox_imu
                             ├─► livox_scan
                             └─► wheel_1 .. wheel_4
```

- `odom → base_link`：`sensor_scan_generation` 发布
- `map → odom`：由定位方式决定，见 §5.3（四种方式互斥）
- `base_link → *`：`real_robot_state_publisher_launch.py` 发布（雷达安装外参 + 轮组）

**仿真**（SRM 生成模型，`config/simulation/nav2_params_srm.yaml`）

```text
map ──► odom ──► base_link ──┬─► front_mid360
                             └─► front_left_wheel / front_right_wheel
                                 rear_left_wheel / rear_right_wheel
```

- `base_link` 是与实际底盘刚性固定的坐标系，**随底盘自转**，仿真与实车都用它做导航速度参考系。
- 模型里**没有** `base_footprint`、`chassis`、`gimbal_yaw`、`gimbal_yaw_fake` 这些中间坐标系：
  旧步兵链路的 `base_footprint → chassis → gimbal_yaw → gimbal_yaw_fake` 已从 SRM 模型与仿真参数中删除。
- `odom → base_link`：默认（静态地图 + 真值）由 `simulation_ground_truth_odometry` 发布（50 Hz）；
  启用 `use_lio_odometry` / `use_pcd_localization` / `slam` 时由 `sensor_scan_generation` +
  对应定位链路提供，真值里程计随之关闭。
- `base_link → front_mid360` 与轮组 TF：`robot_state_publisher` 按生成的 URDF 发布。

> [!IMPORTANT]
> frame 命名必须与参数文件配套。**SRM 仿真与实车都用 `base_link`**；上游对照版
> （`config/simulation/nav2_params.yaml`、`config/real/nav2_params_upstream.yaml`）与
> `srm27_bringup/params/node_params.yaml` 仍保留旧步兵的
> `base_footprint / gimbal_yaw / gimbal_yaw_fake`（**旧链路**），与 SRM 参数混用会满屏 TF 报错且重定位卡死。

---

## 4. 导航栈内部

| 环节 | 实现 |
| --- | --- |
| 全局规划 | `nav2_theta_star_planner/ThetaStarPlanner` |
| 路径平滑 | `nav2_smoother::SimpleSmoother` |
| 局部控制 | `srm27_omni_pid_controller::OmniPidPursuitController` |
| 代价地图图层 | `static_layer` + `intensity_voxel_layer` + `inflation_layer` |
| 恢复行为 | `behavior_server` 插件：`Spin` / `BackUpFreeSpace` / `DriveOnHeading` / `Wait` / `AssistedTeleop`；行为树实际使用的是 `ClearEntireCostmap` + `BackUp`（`BackUpFreeSpace`），**脱困只用平移、不涉及旋转** |
| 流程编排 | Nav2 `bt_navigator` + 行为树 XML |
| 速度出口 | `controller → cmd_vel_controller → velocity_smoother → cmd_vel_nav`（SRM 仿真）；`use_velocity_smoother:=False` 时由 controller 直接发布（详见 §3.2） |

全局代价地图以 `map` 为全局坐标系；局部代价地图以 `odom` 为全局坐标系并跟随车体滚动，
机器人参考系统一为 `base_link`（SRM 仿真参数中 `robot_radius` 取整车碰撞包络 0.33 m）。

对上层暴露的标准 Nav2 action 为 `NavigateToPose` 与 `NavigateThroughPoses`，决策层通过它们下发目标。

---

## 5. 启动入口

### 5.1 启动文件命名约定

`srm27_nav_bringup/launch/` 分**入口层**（直接运行）和**内部层**（被入口 include），前缀即用途：

| 启动文件 | 层 | 用途 |
| --- | --- | --- |
| `nav_real_launch.py` | 入口 | 实车导航（雷达 + 导航栈 + RViz + 手柄） |
| `nav_srm_simulation_launch.py` | 入口 | **SRM 仿真导航（当前入口）**：点云转换 + 真值里程计 + Nav2 + 速度合成/自转 + RViz，可选一并拉起 Gazebo |
| `nav_simulation_launch.py` | 入口（**已弃用**） | 旧步兵仿真入口，现只转发到 `nav_srm_simulation_launch.py`，默认参数文件已改为 `nav2_params_srm.yaml` |
| `nav_multi_simulation_launch.py` | 入口 | 多机仿真导航 |
| `real_mapping_launch.py` | 入口 | 实车 MID360 建图 |
| `real_robot_state_publisher_launch.py` | 入口 | 实车整车 TF（SRM 模型 + 雷达外参） |
| `standalone_robot_state_publisher_launch.py` | 入口 | 独立调试用备用 TF（读 `urdf/srm_robot.urdf`） |
| `nav2_stack_launch.py` | 内部 | Nav2 主栈，按 `slam` 参数二选一拉起建图或定位，并透传速度出口参数 |
| `nav2_slam_launch.py` | 内部 | `slam:=True` 分支（slam_toolbox + map_saver + Point-LIO） |
| `localization_launch.py` | 内部 | `slam:=False` 分支（静态 TF / small_gicp 重定位 / 纯 LIO） |
| `navigation_launch.py` | 内部 | 控制器、规划器与行为服务器；速度出口由 `use_velocity_smoother` 与 `cmd_vel_nav_topic` 决定 |
| `rviz_launch.py` / `joy_teleop_launch.py` | 内部 | RViz 与手柄遥控 |

`srm27_nav_bringup/nav_srm_simulation_launch.py` 的关键参数：`namespace`（默认 `red_standard_robot1`）、
`world`、`map`、`params_file`（默认 `config/simulation/nav2_params_srm.yaml`）、`slam`、
`use_pcd_localization`、`use_lio_odometry`、`use_sim_time`、`use_rviz`、`use_velocity_smoother`、
`cmd_vel_nav_topic`（默认 `cmd_vel_nav`）、`start_simulation`（默认 `False`）、
`start_chassis_control`、`start_rotation_sender` 与自转波形参数。

`srm27_gazebo_simulator/srm_sim.launch.py` 的关键参数：`config_file`（默认 `config/srm_sim.yaml`）、
`robot_name`（默认取配置里的 `red_standard_robot1`）、`world`、`world_sdf`、`gui`、
`run_immediately`（默认 `false`，Gazebo 以暂停状态启动）。

`srm27_chassis_control/srm_chassis_control.launch.py` 的关键参数：`namespace`、`use_sim_time`、
`params_file`、`start_rotation_sender`、`rotation_mode`、`rotation_speed`、`rotation_offset`、
`rotation_amplitude`、`rotation_period`、`rotation_phase`、`rotation_sine_wave`。

`srm27_robot_description/robot_description_launch.py` 用 `config/srm27_sentry_geometry.yaml`
现场生成 URDF 并启动 `robot_state_publisher`（参数：`namespace`、`use_sim_time`）。

`srm27_bringup` 是**整车总入口**，把串口、雷达、导航、行为树、RViz 和录包组装在一起。

> [!NOTE]
> 旧仿真包已删除，所需场地、GUI 和 MID-360 资源均迁入 `srm27_gazebo_simulator/resource/`。
> `ros2 launch srm27_gazebo_simulator gazebo.launch.py` 可单独启动场地世界和时钟桥接
> （不含机器人，默认 `resource/worlds/rmul_2024_world.sdf`，用 `world_sdf_path:=` 指定其他世界）。

### 5.2 脚本入口（推荐现场使用）

`script/` 下当前提供的脚本：

| 脚本 | 用途 |
| --- | --- |
| `script/start_real_nav.sh` | **SRM 实车导航**一键启动：标签页依次为雷达驱动 → 车体 TF → 底盘串口 → Nav2 导航栈 → RViz（→ 可选手柄）。默认 `--lio`、地图自动选择 |
| `script/start_sim_nav.sh` | **SRM 仿真导航**一键启动：标签页 1 = `srm_sim.launch.py`（Gazebo + SRM 模型），标签页 2 = `nav_srm_simulation_launch.py`（导航 + 速度合成 + RViz），标签页 3 = 可选手柄自转。默认 `rmuc_2025` + 隧道地图、默认不自转 |
| `script/kill_gzb.sh` / `kill_rviz.sh` | 清理残留的 Gazebo / RViz 进程（`kill_gzb.sh` 已覆盖 `srm27_gazebo_simulator`、`srm_velocity_adapter` 等新进程名） |

实车：

```bash
./script/start_real_nav.sh -h                    # 打印脚本头部的完整用法
./script/start_real_nav.sh                       # 默认 --lio + 自动选地图，启动全链路
./script/start_real_nav.sh -m xjl0914            # 指定地图（maps/ 下的名字）
./script/start_real_nav.sh --reloc --prior-pcd /abs/map.pcd
./script/start_real_nav.sh --slam                # 边建图边导航
./script/start_real_nav.sh --map-to-odom 0 0 0   # 起步位姿（map 系）
./script/start_real_nav.sh --list-maps           # 列出可用地图与先验 PCD
./script/start_real_nav.sh -n                    # 只解析并打印将执行的命令
./script/start_real_nav.sh --stop                # 先发零速，再结束实车链路节点
```

> [!WARNING]
> `srm27_nav_protocol` 在后台线程里按固定频率重发**最近一次**收到的速度，源码中没有超时
> 清零逻辑，因此**杀掉发速度的节点后车不会自己停**。停节点前先发零速
> （`ros2 topic pub -r 20 /cmd_vel_chassis geometry_msgs/msg/Twist "{}"`），或直接按物理急停；
> `--stop` 已内置"先发零速再结束进程"。

仿真：

```bash
./script/start_sim_nav.sh -h                    # 打印脚本头部的完整用法
./script/start_sim_nav.sh                       # 默认 rmuc_2025 + 隧道地图，不自转
./script/start_sim_nav.sh -m rmuc_2025          # 换普通场地地图
./script/start_sim_nav.sh -w srm_empty --run    # 空场 + Gazebo 直接开始运行
./script/start_sim_nav.sh --rotation-mode constant --rotation-speed 1.0
./script/start_sim_nav.sh --rotation-mode periodic \
    --rotation-offset 1.0 --rotation-amplitude 0.5 --rotation-period 4.0
./script/start_sim_nav.sh --no-rviz --no-rotation
DRY_RUN=1 ./script/start_sim_nav.sh             # 只打印将执行的命令
```

常用参数：`-w/--world`（`rmuc_2025` / `rmuc_2024` / `rmul_2024` / `rmul_2025` / `srm_empty`）、
`-m/--map`（地图名或 YAML 绝对路径）、`-p/--params`（默认 `config/simulation/nav2_params_srm.yaml`）、
`--gui/--no-gui`、`--run/--no-run`、`--smoother/--no-smoother`、`--rviz/--no-rviz`、
`--rotation-mode/-speed/-offset/-amplitude/-period/-phase/-wave`、`--rotation/--no-rotation`、
`--teleop/--no-teleop`。

仿真脚本除了按顺序拉起各节点，还多了几层保护：`flock` 保证 Gazebo 单实例启动、清理上次 `ros2 launch`
遗留的孤儿节点（避免新旧 `/clock` 同时发布）、地图与参数文件的存在性校验（`--params` 必须是绝对路径）、
以及从 `srm27_gazebo_simulator/config/srm_sim.yaml` 读取世界名并与 `-w` 做一致性检查。

`start_sim_nav.sh` 固定使用**导航模式**（`slam:=False`、`use_pcd_localization:=False`），只加载现成栅格图；
建图与重定位需另起 launch，不能与它并行。它默认启动自转测试发送器但模式为 `stop`（零自转），
要真正自转需显式指定 `--rotation-mode`。

> [!NOTE]
> 实车一键脚本 `script/start_real_nav.sh` 已回到工作区（见 §5.2 与 §7.3）。`start_real_slam.sh` 与另外两个仿真脚本（`start_slam.sh`、`start_nav_reloc.sh`）仍不在工作区中（git 历史里可找回）；实车建图请用 `start_real_nav.sh --slam`。

### 5.3 四种定位方式（互斥）

区别只在**谁发布 `map → odom`**（下表左列是实车一键脚本的写法，对应 launch 参数为 `slam` 与 `use_pcd_localization`）：

| 方式 | `map → odom` 来源 | 需要先验数据 | 说明 |
| --- | --- | --- | --- |
| `--slam` | `slam_toolbox` | 否 | 边跑边建图，不需要先验地图 |
| `--reloc` | `small_gicp_relocalization` | 先验 PCD | 与先验点云做 GICP 配准 |
| `--lio` | 静态 TF | 否 | 只用 Point-LIO 里程计（`use_lio_odometry:=True`），**实车推荐**；`start_real_nav.sh` 的默认方式 |
| `--static` | 静态 TF | 先验栅格图 | 只加载栅格图、没有任何里程计来源，仅当车体模块自己发 `odom → base_link` 时可用 |
| 默认 | 静态 TF | 先验栅格图 | `map_server` 加载 PGM，需要外部里程计；SRM 仿真下由 `simulation_ground_truth_odometry` 提供 `odom → base_link`，即 `start_sim_nav.sh` 使用的模式 |

> [!WARNING]
> `slam`、`use_pcd_localization`、`use_lio_odometry` 三者与默认模式互斥，否则会有多个节点争抢
> `map → odom` 或 `odom → base_link`。仿真真值里程计只在三者都不启用时才启动。
> `start_real_nav.sh` 把四个模式做成显式互斥开关，同时给两个会直接报错退出（而不是后者覆盖前者）；
> `--lio` / `--static` 下 `map → odom` 是静态 TF，其数值 `--map-to-odom X Y YAW` 表示
> **起步点在地图坐标系里的位姿**，车不停在地图原点时必须显式指定，否则 RViz 与全局代价地图都会错位。

---

## 6. 配置与数据组织

`src/srm27_navigation/srm27_nav_bringup/` 下按**实车 / 仿真**分目录：

```text
config/real/          nav2_params_srm.yaml        SRM 实车导航参数（当前使用）
                      nav2_params_upstream.yaml   上游原版参数（对照保留，旧 gimbal 链路）
                      mapping_params.yaml         实车建图参数
                      mid360_user_config.json     雷达网络配置
config/simulation/    nav2_params_srm.yaml        SRM 仿真导航参数（当前使用）
                      nav2_params.yaml            旧步兵仿真参数（对照保留，含 gimbal_yaw_fake）
map/real/  pcd/real/          实车栅格图与先验点云
map/simulation/  pcd/simulation/  仿真栅格图与先验点云
rviz/                 mapping.rviz / nav2_default_view.rviz
behavior_trees/       Nav2 行为树 XML
urdf/                 srm_robot.urdf                    独立调试用备用模型
                      sentry_robot_cylinder.xacro       SRM 实车车体模型（实车入口使用）
meshes/               mid360.stl                        实车模型的雷达 mesh
```

本次新增包各自带自己的配置，跨包只通过话题与服务交互：

```text
src/srm27_robot_description/config/srm27_sentry_geometry.yaml   几何与外参（唯一来源）
src/srm27_chassis_control/config/srm_chassis_control.yaml       mux / 自转控制器 / 波形默认参数
src/srm27_gazebo_simulator/config/srm_sim.yaml                  世界名、SRM 初始位姿、速度执行参数
src/srm27_gazebo_simulator/config/ros_gz_bridge.yaml            /clock、真值里程计、joint_states、雷达
src/srm27_gazebo_simulator/worlds/srm_empty.sdf                 空场调试世界（物理步长 1 ms）
src/srm27_gazebo_simulator/resource/                           比赛场地、MID-360 模型与 GUI 配置
```

实车参数有两份，用途写在文件名里：**`nav2_params_srm.yaml` 是 SRM 车实际使用的那份**，`nav2_params_upstream.yaml` 仅作上游对照，两者的 frame 体系不兼容。仿真参数同样是两份：**`nav2_params_srm.yaml` 是当前使用的那份**（`robot_base_frame: base_link`），`nav2_params.yaml` 是旧步兵链路对照版。

实车车体模型已随包提供（`urdf/sentry_robot_cylinder.xacro` + `meshes/mid360.stl`），实车入口**不再依赖外部工作区**。xacro 的雷达 mesh 现在直接指向 `package://srm27_robot_description/meshes/mid360.stl`；`real_mapping_launch.py` 与 `real_robot_state_publisher_launch.py` 仍会把历史遗留的 `package://pb_rm_simulation/` 前缀改写为描述包内的同路径资源。**仿真模型（SDF）与实车模型（xacro）的几何数值必须与 `srm27_robot_description/config/srm27_sentry_geometry.yaml` 保持一致。**

---

## 7. 构建与运行

### 7.1 依赖

```bash
sudo apt install git-lfs
sudo pip install vcstool2
vcs import --recursive . < dependencies.repos
```

另有源码依赖需自行安装：[small_gicp](https://github.com/koide3/small_gicp)。

### 7.2 构建

```bash
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 2
```

> [!NOTE]
> 推荐 `--symlink-install`：本工作空间大量使用 `launch.py` 和 YAML，改参数后通常只需重启节点。
> `--parallel-workers` **不要调大**：`point_lio`、`livox_ros_driver2`、`rmoss_gz_*` 是重模板 C++ 包，并发过高会打满内存导致系统卡死。

### 7.3 运行

实车：推荐一键脚本 `script/start_real_nav.sh`（用法见 §5.2；`--lio` / `--reloc` / `--slam` / `--static`
四者互斥，见 §5.3，默认 `--lio`）。它内部等价于按顺序启动六层：

```bash
# 1) 雷达驱动（nav2_stack_launch.py 不负责起驱动，必须单独启动）
ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
  -r __node:=livox_ros_driver2 \
  --params-file /absolute/path/to/config/real/nav2_params_srm.yaml \
  -p user_config_path:=/absolute/path/to/config/real/mid360_user_config.json

# 2) 整车 TF：SRM 车体模型 + 雷达外参，参数必须用 SRM 那份
ros2 launch srm27_nav_bringup real_robot_state_publisher_launch.py \
  use_sim_time:=False \
  params_file:=/absolute/path/to/config/real/nav2_params_srm.yaml

# 3) 底盘串口：/cmd_vel_chassis 的唯一消费者
ros2 launch srm27_nav_protocol srm27_nav_protocol.launch.py

# 4) 导航栈：slam / use_pcd_localization / use_lio_odometry 按 §5.3 选择
ros2 launch srm27_nav_bringup nav2_stack_launch.py \
  map:=/absolute/path/to/<YOUR_MAP>.yaml \
  params_file:=/absolute/path/to/config/real/nav2_params_srm.yaml \
  use_sim_time:=False

# 5) RViz / 6) 手柄：见 §5.1 的入口层 launch 列表
```

> [!NOTE]
> 雷达驱动那两行不是多余的：驱动代码里的节点名是 `livox_driver_node`，而参数文件顶层键是
> `livox_ros_driver2`，ROS 2 按节点名匹配 `--params-file`，对不上时整段参数被忽略；且参数里
> `user_config_path` 用的是 launch 专有的 `$(find-pkg-share ...)` 语法，`ros2 run` 不会展开。
> `start_real_nav.sh` 已内置这两处处理。

整车总入口：

```bash
ros2 launch srm27_bringup bringup.launch.py \
  map:=/absolute/path/to/<YOUR_MAP>.yaml \
  params_file:=/absolute/path/to/node_params.yaml \
  use_rviz:=True
```

> [!NOTE]
> `srm27_bringup/params/node_params.yaml` 目前仍是**旧步兵链路**配置（`fake_vel_transform` +
> `gimbal_yaw_fake`），本次 SRM 仿真改动未涉及它。

**SRM 仿真**（推荐一键脚本，内部包含下面两条 launch）：

```bash
./script/start_sim_nav.sh                    # 默认 rmuc_2025 + 隧道地图，不自转
./script/start_sim_nav.sh -m rmuc_2025       # 换普通场地地图
./script/start_sim_nav.sh -w srm_empty --run # 空场调试，Gazebo 直接开始运行
```

等价的拆开手动启动方式：

```bash
# 终端 1：Gazebo 世界 + SRM 模型 + 传感器桥接 + 速度适配器
ros2 launch srm27_gazebo_simulator srm_sim.launch.py

# 终端 2：点云转换 + 真值里程计 + Nav2 + 速度合成 + RViz
ros2 launch srm27_nav_bringup nav_srm_simulation_launch.py \
  namespace:=red_standard_robot1 world:=rmuc_2025 \
  map:=/absolute/path/to/map/simulation/rmuc_2025_tunnel.yaml \
  params_file:=/absolute/path/to/config/simulation/nav2_params_srm.yaml \
  slam:=False use_pcd_localization:=False use_sim_time:=True use_rviz:=True
```

> [!NOTE]
> `map`、`prior_pcd_file`、`params_file` 一律使用**绝对路径**。
> `nav_simulation_launch.py` 已改为**弃用壳**（转发到 `nav_srm_simulation_launch.py`），
> 旧仿真包已删除，仿真统一使用 `srm27_gazebo_simulator srm_sim.launch.py`。

### 7.4 自转测试模式

自转由独立链路产生，与导航平移在 `srm_cmd_mux` 里合成，因此**不影响导航状态**：

```text
rotation_test_sender ─ rotation_cmd ─► rotation_controller ─ rotation_velocity ─┐
                                                                                ├─ srm_cmd_mux
Nav2 controller ──────────────────────────────── cmd_vel_nav ──────────────────┘
```

| 模式 | 含义 | 相关参数 |
| --- | --- | --- |
| `stop` | 持续输出零角速度（默认） | — |
| `constant` | 恒速自转 | `rotation_speed`（有符号，rad/s） |
| `periodic` | 周期自转（正弦或方波） | `rotation_offset`、`rotation_amplitude`、`rotation_period`、`rotation_phase`、`rotation_sine_wave` |

同一组参数在三个位置出现，含义一致：脚本参数 `--rotation-mode/-speed/-offset/-amplitude/-period/-phase/-wave`、
`nav_srm_simulation_launch.py` 的 `rotation_*` launch 参数、`rotation_test_sender` 的节点参数。
波形时间取**仿真时间**，从启用或参数改变时开始计算；周期性发布本身不重置相位。

```bash
# 恒速自转 1.0 rad/s
./script/start_sim_nav.sh --rotation-mode constant --rotation-speed 1.0

# 正弦周期：1.0 ± 0.5 rad/s，周期 4 s
./script/start_sim_nav.sh --rotation-mode periodic \
  --rotation-offset 1.0 --rotation-amplitude 0.5 --rotation-period 4.0

# 方波换向
./script/start_sim_nav.sh --rotation-mode periodic \
  --rotation-offset 0.0 --rotation-amplitude 1.0 --rotation-wave square
```

运行中切换（无需重启）：

```bash
ros2 service call /<ns>/rotation_test_sender/disable std_srvs/srv/Trigger
ros2 param set /<ns>/rotation_test_sender rotation_mode periodic
ros2 param set /<ns>/rotation_test_sender offset 1.0
ros2 param set /<ns>/rotation_test_sender amplitude 0.5
ros2 param set /<ns>/rotation_test_sender period 4.0
ros2 service call /<ns>/rotation_test_sender/enable std_srvs/srv/Trigger
```

急停与自查：

```bash
ros2 service call /<ns>/srm_cmd_mux/stop_all std_srvs/srv/Trigger     # 两路清零并保持零输出
ros2 service call /<ns>/srm_cmd_mux/resume_all std_srvs/srv/Trigger  # 重新启用两路输入
ros2 service call /<ns>/rotation_controller/clear_rotation std_srvs/srv/Trigger

ros2 topic echo /<ns>/cmd_vel_nav --once        # 导航平移速度（angular.z 应为 0）
ros2 topic echo /<ns>/rotation_velocity --once  # 独立自转速度
ros2 topic echo /<ns>/cmd_vel_sim --once        # 合成后的最终执行命令
ros2 topic hz   /<ns>/cmd_vel_sim               # 应约 200 Hz
ros2 topic echo /<ns>/diagnostics --once        # 超时 / 限幅 / 丢弃计数
```

`<ns>` 默认 `red_standard_robot1`。诊断消息里 `nav_wz_dropped_count` 表示被 mux 丢弃的导航角速度次数，
`translation_scale` / `clamped` 表示平移是否被模长限幅，`non_finite_*_count` 表示 NaN/Inf 输入次数。
限幅与超时的完整默认值见 [`docs/SRM仿真与自转控制实现(ai).md`](./docs/SRM仿真与自转控制实现%28ai%29.md)。

雷达网卡配置（`192.168.1.50/24`）与驱动验证见 [`docs/mid360使用指南.md`](./docs/mid360使用指南.md)。

---

## 8. 文档索引

| 文档 | 内容 |
| --- | --- |
| [`docs/mid360使用指南.md`](./docs/mid360使用指南.md) | Mid-360 网络配置与驱动启动 |
| [`docs/SRM仿真与导航自转控制实施方案.md`](./docs/SRM仿真与导航自转控制实施方案.md) | 本次 SRM 仿真与自转控制重构的权威方案 |
| [`docs/SRM仿真与自转控制实现(ai).md`](./docs/SRM仿真与自转控制实现%28ai%29.md) | **（AI 生成）** 落地说明：三个新包的职责、速度/自转接口表、合成与安全优先级、启动方式、测试矩阵与实测结论 |
| [`docs/仿真数据流(ai).md`](./docs/仿真数据流%28ai%29.md) | 仿真链路各模块算法与数据流详解（已按 SRM 链路更新） |
| [`docs/哨兵导航TF与下位机数据接口(ai).md`](./docs/哨兵导航TF与下位机数据接口%28ai%29.md) | MID-360 安装与底盘自旋的组合、传感器分工及上下位机通信约定 |
| [`docs/局部规划器相关(ai).md`](./docs/局部规划器相关%28ai%29.md) | 当前控制器实现与候选方案对比 |
| [`docs/局部规划器迁移MINCO_MPC方案(ai).md`](./docs/局部规划器迁移MINCO_MPC方案%28ai%29.md) | 局部规划器迁移 MINCO/MPC 的分阶段方案 |
| [`docs/TF与fake底盘详解(ai).md`](./docs/TF与fake底盘详解%28ai%29.md) | 旧步兵 fake 虚拟底盘与 TF 详解（历史资料） |
| [`docs/tdt开源.md`](./docs/tdt开源.md) | 上游导航实现调研记录 |
| [`src/srm27_navigation/README.md`](./src/srm27_navigation/README.md) | 导航包集合详细说明（含完整启动参数表） |
| [`src/srm27_behavior/README.md`](./src/srm27_behavior/README.md) | 行为树框架与插件开发说明 |

---

## 9. 许可

Apache-2.0。第三方依赖各自遵循其原始许可；`rm_decision_interfaces` 中合入的消息版权见该包内 `LICENSE.pb_rm_interfaces`。
