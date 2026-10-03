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
        │ TF、代价地图、里程计                       │ cmd_vel_controller
┌───────v────────────────────────────┐   ┌─────────v─────────────────┐
│ 定位与感知层                        │   │ 通信层                     │
│   point_lio            激光惯性里程计│   │  standard_robot_pp_ros2   │
│   small_gicp_reloc.    先验图重定位  │   │   串口 ↔ 下位机 C 板       │
│   slam_toolbox         二维建图      │   │  （唯一 /cmd_vel 消费者）   │
│   terrain_analysis(_ext) 地形分析    │   └───────────────────────────┘
│   sensor_scan_generation  odom→base  │
│   loam_interface / livox_ros_driver2 │
└───────┬──────────────────────────────┘
        │ 传感器数据
┌───────v────────────────────────────────────────────────────────────┐
│ 硬件 / 仿真层                                                       │
│   实车：Livox Mid-360、下位机 C 板                                   │
│   仿真：rmu_gazebo_simulator（世界、SRM 模型、传感器、仿真真值）        │
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
│   ├── standard_robot_pp_ros2/  下位机串口通信
│   ├── rmu_gazebo_simulator/ 仿真世界与 SRM 机器人模型
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
| `fake_vel_transform` | 构造不随云台旋转的虚拟底盘系，供 Nav2 使用 |
| `livox_ros_driver2` | Mid-360 驱动 |
| `ign_sim_pointcloud_tool` | 仿真点云转 Velodyne 格式（补 `ring` 与相对时间） |
| `pointcloud_to_laserscan` | 点云降维为二维激光（仅 SLAM 模式使用） |

**通信与接口**

| 包 | 职责 |
| --- | --- |
| `standard_robot_pp_ros2` | 下位机串口协议；`/cmd_vel` 的唯一消费者，同时发布关节状态用于整车 TF |
| `rm_decision_interfaces` | 统一的裁判系统、决策、云台消息接口 |

**仿真与工具**

| 包 | 职责 |
| --- | --- |
| `rmu_gazebo_simulator` | Gazebo 世界、SRM 圆柱底盘模型、传感器、仿真真值里程计 |
| `pcd2pgm` | `.pcd` 转 `.pgm` 栅格图 |
| `rosbag2_composable_recorder` | 按裁判系统状态触发录包 |
| `teleop_gimbal_keyboard` | 键盘控制云台 |

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
                                   cmd_vel_controller ──► /cmd_vel
                                            │
                                            v
                              standard_robot_pp_ros2 ──► 串口 ──► 下位机 C 板
```

### 3.2 仿真链路

与实车的差异只在传感器来源和里程计来源：

```text
Gazebo ──┬─► 激光点云 ──► ign_sim_pointcloud_tool ──► velodyne_points
         │                                                    │
         │                                        terrain_analysis(_ext)
         │                                                    │
         └─► chassis_odometry_gt                              v
                     │                              代价地图 / 规划 / 控制
                     v                                        │
      simulation_ground_truth_odometry                        v
                     ├─► odometry                        /cmd_vel
                     └─► odom → base_footprint                 │
                                                               v
                                                            Gazebo
```

### 3.3 坐标系（TF 链）

**实车**（SRM 自建模型，`nav2_params_srm.yaml`）

```text
map ──► odom ──► base_link ──┬─► livox_frame ──► livox_imu
                             ├─► livox_scan
                             └─► wheel_1 .. wheel_4
```

- `odom → base_link`：`sensor_scan_generation` 发布
- `map → odom`：由定位方式决定，见 §5.3（四种方式互斥）
- `base_link → *`：`real_robot_state_publisher_launch.py` 发布（雷达安装外参 + 轮组）

**仿真**（Gazebo 模型，`config/simulation/nav2_params.yaml`）

```text
map ──► odom ──► base_footprint ──► chassis ──► gimbal_yaw_fake
                                          ├─► front_mid360
                                          └─► front_rplidar_a2
```

`gimbal_yaw_fake` 是 Nav2 使用的机器人参考坐标系。

> [!IMPORTANT]
> 实车与仿真两套 frame 命名**不能混用**：参数文件与车体模型必须配套。实车那份用 `base_link / livox_*`，上游对照版用 `base_footprint / gimbal_yaw / front_mid360`。混用会满屏 TF 报错且重定位卡死。

---

## 4. 导航栈内部

| 环节 | 实现 |
| --- | --- |
| 全局规划 | `nav2_theta_star_planner/ThetaStarPlanner` |
| 路径平滑 | `nav2_smoother::SimpleSmoother` |
| 局部控制 | `srm27_omni_pid_controller::OmniPidPursuitController` |
| 代价地图图层 | `static_layer` + `intensity_voxel_layer` + `inflation_layer` |
| 恢复行为 | `Spin` / `BackUpFreeSpace` / `DriveOnHeading` / `Wait` / `AssistedTeleop` |
| 流程编排 | Nav2 `bt_navigator` + 行为树 XML |

全局代价地图以 `map` 为全局坐标系；局部代价地图以 `odom` 为全局坐标系并跟随车体滚动。

对上层暴露的标准 Nav2 action 为 `NavigateToPose` 与 `NavigateThroughPoses`，决策层通过它们下发目标。

---

## 5. 启动入口

### 5.1 启动文件命名约定

`srm27_nav_bringup/launch/` 分**入口层**（直接运行）和**内部层**（被入口 include），前缀即用途：

| 启动文件 | 层 | 用途 |
| --- | --- | --- |
| `nav_real_launch.py` | 入口 | 实车导航（雷达 + 导航栈 + RViz + 手柄） |
| `nav_simulation_launch.py` | 入口 | 仿真导航 |
| `nav_multi_simulation_launch.py` | 入口 | 多机仿真导航 |
| `real_mapping_launch.py` | 入口 | 实车 MID360 建图 |
| `real_robot_state_publisher_launch.py` | 入口 | 实车整车 TF（SRM 模型 + 雷达外参） |
| `standalone_robot_state_publisher_launch.py` | 入口 | 独立调试用备用 TF（读 `urdf/srm_robot.urdf`） |
| `nav2_stack_launch.py` | 内部 | Nav2 主栈，按 `slam` 参数二选一拉起建图或定位 |
| `nav2_slam_launch.py` | 内部 | `slam:=True` 分支（slam_toolbox + map_saver + Point-LIO） |
| `localization_launch.py` | 内部 | `slam:=False` 分支（AMCL / small_gicp 重定位） |
| `navigation_launch.py` | 内部 | 控制器、规划器与行为服务器 |
| `rviz_launch.py` / `joy_teleop_launch.py` | 内部 | RViz 与手柄遥控 |

`srm27_bringup` 是**整车总入口**，把串口、雷达、导航、行为树、RViz 和录包组装在一起。

### 5.2 脚本入口（推荐现场使用）

`script/` 下当前提供的脚本：

| 脚本 | 用途 |
| --- | --- |
| `script/start_sim_nav.sh` | **仿真导航**一键启动（Gazebo + Nav2 + RViz，可选键鼠控制），默认 `rmuc_2025` + 隧道地图 |
| `script/kill_gzb.sh` / `kill_rviz.sh` | 清理残留的 Gazebo / RViz 进程 |

仿真脚本除了按顺序拉起各节点，还多了几层保护：`flock` 保证 Gazebo 单实例启动、清理上次 `ros2 launch` 遗留的孤儿节点（避免新旧 `/clock` 同时发布）、地图与参数文件的存在性校验、`gz_world.yaml` 里的世界名与 `-w` 参数的一致性检查。

`start_sim_nav.sh` 固定使用**导航模式**（`slam:=False`、`use_pcd_localization:=False`），只加载现成栅格图；建图与重定位需另起 launch，不能与它并行。

> [!NOTE]
> 实车一键脚本（`start_real_nav.sh`、`start_real_slam.sh`）与另外两个仿真脚本（`start_slam.sh`、`start_nav_reloc.sh`）当前不在工作区中（git 历史里仍可找回）。实车启动请直接使用 §7.3 的 `ros2 launch` 入口。

### 5.3 四种定位方式（互斥）

区别只在**谁发布 `map → odom`**（下表左列是实车一键脚本的写法，对应 launch 参数为 `slam` 与 `use_pcd_localization`）：

| 方式 | `map → odom` 来源 | 需要先验数据 | 说明 |
| --- | --- | --- | --- |
| `--slam` | `slam_toolbox` | 否 | 边跑边建图，不需要先验地图 |
| `--reloc` | `small_gicp_relocalization` | 先验 PCD | 与先验点云做 GICP 配准 |
| `--lio` | 静态 TF | 否 | 只用 Point-LIO 里程计，**实车推荐** |
| 默认 | 静态 TF | 先验栅格图 | `map_server` 加载 PGM，需要外部里程计；适用于仿真真值或底盘自带里程计，即 `start_sim_nav.sh` 使用的模式 |

> [!WARNING]
> `slam` 与 `use_pcd_localization` 不能同时开启，否则会有多个节点争抢 `map → odom`。

---

## 6. 配置与数据组织

`src/srm27_navigation/srm27_nav_bringup/` 下按**实车 / 仿真**分目录：

```text
config/real/          nav2_params_srm.yaml        SRM 实车导航参数（当前使用）
                      nav2_params_upstream.yaml   上游原版参数（对照保留）
                      mapping_params.yaml         实车建图参数
                      mid360_user_config.json     雷达网络配置
config/simulation/    nav2_params.yaml            仿真导航参数
map/real/  pcd/real/          实车栅格图与先验点云
map/simulation/  pcd/simulation/  仿真栅格图与先验点云
rviz/                 mapping.rviz / nav2_default_view.rviz
behavior_trees/       Nav2 行为树 XML
urdf/                 srm_robot.urdf                    独立调试用备用模型
                      sentry_robot_cylinder.xacro       SRM 实车车体模型（实车入口使用）
meshes/               mid360.stl                        实车模型的雷达 mesh
```

实车参数有两份，用途写在文件名里：**`nav2_params_srm.yaml` 是 SRM 车实际使用的那份**，`nav2_params_upstream.yaml` 仅作上游对照，两者的 frame 体系不兼容。

实车车体模型已随包提供（`urdf/sentry_robot_cylinder.xacro` + `meshes/mid360.stl`），实车入口**不再依赖外部工作区**。xacro 里的 `package://pb_rm_simulation/meshes/mid360.stl` 由两个实车 launch 在加载时改写为 `package://srm27_nav_bringup/meshes/mid360.stl`；改车体尺寸只需改这一份 xacro。

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

实车：一键脚本 `script/start_real_nav.sh` 当前不在工作区中（git 历史里可找回，`--lio` / `--reloc` / `--slam` 三者互斥，见 §5.3）。它内部等价于按顺序启动三层：

```bash
# 1) 整车 TF：SRM 车体模型 + 雷达外参，参数必须用 SRM 那份
ros2 launch srm27_nav_bringup real_robot_state_publisher_launch.py \
  use_sim_time:=False \
  params_file:=/absolute/path/to/config/real/nav2_params_srm.yaml

# 2) 导航栈：slam 与 use_pcd_localization 按 §5.3 选择，默认静态 map→odom
ros2 launch srm27_nav_bringup nav2_stack_launch.py \
  map:=/absolute/path/to/<YOUR_MAP>.yaml \
  params_file:=/absolute/path/to/config/real/nav2_params_srm.yaml \
  use_sim_time:=False

# 3) RViz / 手柄 / 底盘串口节点：见 §5.1 的入口层 launch 列表
```

整车总入口：

```bash
ros2 launch srm27_bringup bringup.launch.py \
  map:=/absolute/path/to/<YOUR_MAP>.yaml \
  params_file:=/absolute/path/to/node_params.yaml \
  use_rviz:=True
```

仿真导航（Gazebo 世界 + Nav2 + RViz，脚本内部已包含下面那条 Gazebo 启动）：

```bash
./script/start_sim_nav.sh                    # 默认 rmuc_2025 + 隧道地图
./script/start_sim_nav.sh -m rmuc_2025       # 换普通场地地图
```

等价的拆开手动启动方式：

```bash
ros2 launch rmu_gazebo_simulator bringup_sim.launch.py
ros2 launch srm27_nav_bringup nav_simulation_launch.py world:=rmuc_2025 slam:=False use_pcd_localization:=False use_sim_time:=True use_rviz:=True
```

> [!NOTE]
> `map`、`prior_pcd_file`、`params_file` 一律使用**绝对路径**。

雷达网卡配置（`192.168.1.50/24`）与驱动验证见 [`docs/mid360使用指南.md`](./docs/mid360使用指南.md)。

---

## 8. 文档索引

| 文档 | 内容 |
| --- | --- |
| [`docs/mid360使用指南.md`](./docs/mid360使用指南.md) | Mid-360 网络配置与驱动启动 |
| [`docs/仿真数据流(ai).md`](./docs/仿真数据流%28ai%29.md) | 仿真链路各模块算法与数据流详解 |
| [`docs/局部规划器相关(ai).md`](./docs/局部规划器相关%28ai%29.md) | 当前控制器实现与候选方案对比 |
| [`docs/局部规划器迁移MINCO_MPC方案(ai).md`](./docs/局部规划器迁移MINCO_MPC方案%28ai%29.md) | 局部规划器迁移 MINCO/MPC 的分阶段方案 |
| [`docs/tdt开源.md`](./docs/tdt开源.md) | 上游导航实现调研记录 |
| [`src/srm27_navigation/README.md`](./src/srm27_navigation/README.md) | 导航包集合详细说明（含完整启动参数表） |
| [`src/srm27_behavior/README.md`](./src/srm27_behavior/README.md) | 行为树框架与插件开发说明 |

---

## 9. 许可

Apache-2.0。第三方依赖各自遵循其原始许可；`rm_decision_interfaces` 中合入的消息版权见该包内 `LICENSE.pb_rm_interfaces`。
