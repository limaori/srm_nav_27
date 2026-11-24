# ROS 包清点（2026-11-24，TDT 拆包并入 planner 前后）

统计口径：`colcon list` / `src/**/package.xml`，共 **41** 个 ROS 包。

| 分类 | 数量 | 说明 |
|---|---:|---|
| 项目自有/维护 | 29 | `srm27_*`、`frontend`、`localization_interfaces`、`rm_decision_interfaces` |
| 集成/改造的第三方导航包 | 10 | Point-LIO、ZLIO 周边、fake_vel_transform、terrain_analysis 等 |
| 外部依赖 | 2 | `src/dependencies/BehaviorTree.ROS2` 下的 BT 运行库与接口 |

> 2026-11-14 已删除 19 个旧包：11 个 `rmoss_*`、`joint_state_publisher`、
> `joint_state_publisher_gui`、`sdformat_tools`、`pcd2pgm`、`teleop_gimbal_keyboard`、
> `btcpp_ros2_samples`、旧总入口 `srm27_bringup`、`rosbag2_composable_recorder`。
> `src/tools/` 已随工具包一起删除；`src/dependencies/` 现在只保留 BehaviorTree.ROS2。

## 目录模块

| 目录 | 包数 | 内容 |
|---|---:|---|
| `interfaces` | 2 | 定位协议、决策/裁判/云台消息 |
| `localization` | 9 | 前端 5 + 后端 4（含迁入的 `srm27_zlocalization_backend`） |
| `planner` | 13 | 前端 6 + 后端 4 + 共用底座 1 + 地图 2 |
| `controller` | 6 | 控制器、速度变换、底盘、串口、手柄 |
| `decision` | 3 | 前端 1（BT 插件）+ 后端 1（服务端/决策树）+ 共用底座 1 |
| `simulation` | 3 | Gazebo、模型、仿真点云 |
| `bringup` | 3 | Nav2 装配、工具与导航元包 |
| `dependencies` | 2 | BehaviorTree.ROS2 |

## 项目自有/维护（29）

| 包 | 路径 | 角色 |
|---|---|---|
| `srm27_behavior` | `src/decision/backend/srm27_behavior` | 决策后端：执行服务端/客户端、裁判输入、决策树/参数/launch |
| `srm27_behavior_plugins` | `src/decision/frontend/srm27_behavior_plugins` | BehaviorTree 动作/条件/控制/装饰器插件（安装到 `bt_plugins/`） |
| `srm27_behavior_core` | `src/decision/common/srm27_behavior_core` | 决策共享底座（header-only）：航点/语义区、导航到点状态、自转参考、姿态状态 |
| `srm27_chassis_control` | `src/controller/srm27_chassis_control` | `srm_cmd_mux`：速度合成、限幅、看门狗 |
| `srm27_nav_protocol` | `src/controller/srm27_nav_protocol` | 下位机串口协议、速度反馈 |
| `srm27_robot_description` | `src/simulation/srm27_robot_description` | 实车/仿真共用 URDF/xacro、雷达/云台外参 |
| `srm27_gazebo_simulator` | `src/simulation/srm27_gazebo_simulator` | Gazebo Fortress 世界、SRM 模型、速度执行插件 |
| `rm_decision_interfaces` | `src/interfaces/rm_decision_interfaces` | 决策/裁判/云台消息接口 |
| `localization_interfaces` | `src/interfaces/localization_interfaces` | 前后端同步观测、会话身份、重置协议 |
| `frontend` | `src/localization/frontend/frontend` | ZLIO 激光惯性里程计前端 |
| `srm27_navigation` | `src/bringup/srm27_navigation` | 导航包集合的元包 |
| `srm27_nav_bringup` | `src/bringup/srm27_nav_bringup` | Nav2 启动、参数、地图、RViz、launch 契约测试 |
| `srm27_nav_tools` | `src/bringup/srm27_nav_tools` | 导航/建图复用的 ROS 节点与工具脚本 |
| `srm27_minco_core` | `src/planner/backend/srm27_minco_core` | MINCO 轨迹优化 + MPC 纯算法核心 |
| `srm27_minco_controller` | `src/controller/srm27_minco_controller` | Nav2 FollowPath 控制器插件 |
| `srm27_minco_vendor` | `src/planner/backend/srm27_minco_vendor` | vendored GCOPTER MINCO/L-BFGS |
| `srm27_qpoases_vendor` | `src/planner/backend/srm27_qpoases_vendor` | vendored qpOASES |
| `srm27_costmap_plugins` | `src/planner/frontend/srm27_costmap_plugins` | 强度体素代价地图层 |
| `srm27_nav_behavior_plugins` | `src/planner/frontend/srm27_nav_behavior_plugins` | Nav2 脱困行为、目标检查器、路线途经点 BT 节点 |
| `srm27_nav_plugins` | `src/planner/frontend/srm27_nav_plugins` | 兼容元包；依赖上述两个拆分插件包 |
| `srm27_nav_planner` | `src/planner/frontend/srm27_nav_planner` | Nav2 全局规划器插件 `YAstarGlobalPlanner`；vendored YAstar 源码内置 `vendor/`（MIT） |
| `srm27_nav_smoother` | `src/planner/backend/srm27_nav_smoother` | Nav2 平滑器插件 `MinimumSnapSmoother`；vendored MinimumSnap/SfcSquare 源码内置 `vendor/`（MIT） |
| `srm27_nav_common` | `src/planner/common/srm27_nav_common` | 规划前后端共用的路径工具：Path 转换/加密/切角（无第三方源码） |
| `srm27_map_core` | `src/planner/map/srm27_map_core` | 规划地图底座（纯 C++）：GridSnapshot 栅格语义/几何/编码、ESDF、占据几何、SFC 视图 |
| `srm27_map_adapter` | `src/planner/map/srm27_map_adapter` | 地图适配层：Nav2 Costmap2D(ROS) → GridSnapshot，带内容版本号 |
| `srm27_localization_adapter` | `src/localization/backend/srm27_localization_adapter` | ZLIO 消息桥接、IMU 饱和保护、backend 输入适配与 reset 代理 |
| `srm27_zlocalization_backend` | `src/localization/backend/srm27_zlocalization_backend` | Z-Localization backend 迁入版：因子图定位、6 个地图/标定工具（GPL-2.0-only，实验模式默认关闭） |
| `srm27_omni_pid_controller` | `src/controller/srm27_omni_pid_controller` | Omni Pure Pursuit 控制器（实车回退链） |
| `srm27_teleop_twist_joy` | `src/controller/srm27_teleop_twist_joy` | 底盘/云台手柄遥控 |

## 集成/改造的第三方导航包（10）

| 包 | 路径 | 说明 |
|---|---|---|
| `fake_vel_transform` | `src/controller/fake_vel_transform` | 当前 fakeyaw 链路必需：`cmd_vel_nav_fake → cmd_vel_nav_body` |
| `point_lio` | `src/localization/frontend/point_lio` | Point-LIO 里程计/建图；PCD 输出已改运行期 `SRM27_PCD_DIR` |
| `small_gicp_relocalization` | `src/localization/backend/small_gicp_relocalization` | 先验 PCD 的 GICP 重定位 |
| `sensor_scan_generation` | `src/localization/backend/sensor_scan_generation` | 发布 `odom→gimbal_yaw` / `sensor_scan` |
| `loam_interface` | `src/localization/frontend/loam_interface` | Point-LIO 输出到 `registered_scan` 的适配 |
| `pointcloud_to_laserscan` | `src/localization/frontend/pointcloud_to_laserscan` | 3D 点云转 2D 扫描 |
| `terrain_analysis` | `src/planner/frontend/terrain_analysis` | 局部地形分析 |
| `terrain_analysis_ext` | `src/planner/frontend/terrain_analysis_ext` | 扩展地形分析 |
| `ign_sim_pointcloud_tool` | `src/simulation/ign_sim_pointcloud_tool` | Gazebo 点云转 velodyne 格式 |
| `livox_ros_driver2` | `src/localization/frontend/livox_ros_driver2` | MID-360 驱动 |

## 外部依赖（2）

`src/dependencies/BehaviorTree.ROS2/`：

| 包 | 说明 |
|---|---|
| `behaviortree_ros2` | BehaviorTree.CPP 的 ROS 2 封装，`srm27_behavior` 使用 |
| `btcpp_ros2_interfaces` | BT 的 ROS 2 动作/服务接口 |

## 入口与包的对应

四个正式入口只依赖上述包的一部分：

- 实车导航：`srm27_nav_bringup` + `srm27_minco_*` + `srm27_costmap_plugins` +
  `srm27_nav_behavior_plugins` + `srm27_nav_smoother`（默认平滑器）+
  `srm27_nav_planner`（`--yastar-planner` 时）+
  `srm27_chassis_control` + `srm27_nav_protocol` + `fake_vel_transform` + 定位前端；
- 仿真导航：上述 + `srm27_gazebo_simulator`；
- 实车建图：`srm27_nav_bringup` + `srm27_robot_description` + `point_lio`/`frontend` +
  `loam_interface`/`zlio_bridge` + `sensor_scan_generation` + `slam_toolbox`；
- 仿真建图：上述 + `srm27_gazebo_simulator` + `ign_sim_pointcloud_tool`。

> `src/perception_localization` 已更名为 `src/localization`，
> `src/trajectory_planning` 已更名为 `src/planner`；MINCO core/vendor 属于规划后端。
> 2026-11-24：原 `src/tdt_nav_kit` 拆成 `srm27_nav_planner`（前端）、
> `srm27_nav_smoother`（后端）与 `srm27_nav_common`（planner/common，vendored 算法 + 共用转换层），
> 见 [`../records/2026-11-24-tdt-merge-into-planner.md`](../records/2026-11-24-tdt-merge-into-planner.md)。
>
> 2026-11-24：决策模块从单包 `srm27_behavior` 拆成
> `srm27_behavior_core`（common）、`srm27_behavior_plugins`（frontend）、
> `srm27_behavior`（backend），见
> [`../records/2026-11-24-decision-module-layering.md`](../records/2026-11-24-decision-module-layering.md)。
>
> 2026-11-24：地图/栅格代码从 `srm27_minco_core`、`srm27_nav_common`、
> `srm27_nav_smoother`、`srm27_minco_controller` 抽到 `src/planner/map`，见
> [`../records/2026-11-24-planner-map-module.md`](../records/2026-11-24-planner-map-module.md)。
>
> 2026-11-24：上游算法源码拆入各自插件包的 `vendor/`，`srm27_nav_common` 只保留路径工具，见
> [`../records/2026-11-24-vendor-sources-into-plugins.md`](../records/2026-11-24-vendor-sources-into-plugins.md)。
> 是否把 `srm27_navigation` 拆成多个源码目录，见
> [`../records/2026-11-14-workspace-cleanup.md`](../records/2026-11-14-workspace-cleanup.md) §9。
