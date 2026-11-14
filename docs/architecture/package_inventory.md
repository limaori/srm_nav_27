# ROS 包清点（2026-11-14）

统计口径：`src/**/package.xml`，共 **48** 个 ROS 包。

| 分类 | 数量 | 说明 |
|---|---:|---|
| 项目自有/维护 | 18 | `srm27_*`、`frontend`、`localization_interfaces`、`rm_decision_interfaces` |
| 集成/改造的第三方导航包 | 10 | Point-LIO、ZLIO 周边、fake_vel_transform、terrain_analysis 等 |
| 外部依赖 | 17 | `src/dependencies/` 下的 BehaviorTree.ROS2、rmoss、joint_state_publisher 等 |
| 工具包 | 3 | `pcd2pgm`、`rosbag2_composable_recorder`、`teleop_gimbal_keyboard` |

## 项目自有/维护（18）

| 包 | 路径 | 角色 |
|---|---|---|
| `srm27_behavior` | `src/srm27_behavior` | 行为树、BT 插件、决策参数 |
| `srm27_bringup` | `src/srm27_bringup` | 旧总入口（雷达/RSP/导航/手柄组合），当前正式入口在 `srm27_nav_bringup` |
| `srm27_chassis_control` | `src/srm27_chassis_control` | `srm_cmd_mux`：速度合成、限幅、看门狗 |
| `srm27_nav_protocol` | `src/srm27_nav_protocol` | 下位机串口协议、速度反馈 |
| `srm27_robot_description` | `src/srm27_robot_description` | 实车/仿真共用 URDF/xacro、雷达/云台外参 |
| `srm27_gazebo_simulator` | `src/srm27_gazebo_simulator` | Gazebo Fortress 世界、SRM 模型、速度执行插件 |
| `rm_decision_interfaces` | `src/interfaces/rm_decision_interfaces` | 决策/裁判/云台消息接口 |
| `localization_interfaces` | `src/localization_interfaces` | 前后端同步观测、会话身份、重置协议 |
| `frontend` | `src/frontend` | ZLIO 激光惯性里程计前端 |
| `srm27_navigation` | `src/srm27_navigation/srm27_navigation` | 导航包集合的元包 |
| `srm27_nav_bringup` | `src/srm27_navigation/srm27_nav_bringup` | Nav2 启动、参数、地图、RViz、launch 契约测试 |
| `srm27_minco_core` | `src/srm27_navigation/srm27_minco_core` | MINCO 轨迹优化 + MPC 纯算法核心 |
| `srm27_minco_controller` | `src/srm27_navigation/srm27_minco_controller` | Nav2 FollowPath 控制器插件 |
| `srm27_minco_vendor` | `src/srm27_navigation/srm27_minco_vendor` | vendored GCOPTER MINCO/L-BFGS |
| `srm27_qpoases_vendor` | `src/srm27_navigation/srm27_qpoases_vendor` | vendored qpOASES |
| `srm27_nav_plugins` | `src/srm27_navigation/srm27_nav_plugins` | 自研 Nav2 插件（判停、路线途经点、强度体素层） |
| `srm27_omni_pid_controller` | `src/srm27_navigation/srm27_omni_pid_controller` | Omni Pure Pursuit 控制器（实车回退链） |
| `srm27_teleop_twist_joy` | `src/srm27_navigation/srm27_teleop_twist_joy` | 底盘/云台手柄遥控 |

## 集成/改造的第三方导航包（10）

| 包 | 路径 | 说明 |
|---|---|---|
| `fake_vel_transform` | `src/srm27_navigation/fake_vel_transform` | 当前 fakeyaw 链路必需：`cmd_vel_nav_fake → cmd_vel_nav_body` |
| `point_lio` | `src/srm27_navigation/point_lio` | Point-LIO 里程计/建图；PCD 输出已改运行期 `SRM27_PCD_DIR` |
| `small_gicp_relocalization` | `src/srm27_navigation/small_gicp_relocalization` | 先验 PCD 的 GICP 重定位 |
| `sensor_scan_generation` | `src/srm27_navigation/sensor_scan_generation` | 发布 `odom→gimbal_yaw` / `sensor_scan` |
| `loam_interface` | `src/srm27_navigation/loam_interface` | Point-LIO 输出到 `registered_scan` 的适配 |
| `pointcloud_to_laserscan` | `src/srm27_navigation/pointcloud_to_laserscan` | 3D 点云转 2D 扫描 |
| `terrain_analysis` | `src/srm27_navigation/terrain_analysis` | 局部地形分析 |
| `terrain_analysis_ext` | `src/srm27_navigation/terrain_analysis_ext` | 扩展地形分析 |
| `ign_sim_pointcloud_tool` | `src/srm27_navigation/ign_sim_pointcloud_tool` | Gazebo 点云转 velodyne 格式 |
| `livox_ros_driver2` | `src/srm27_navigation/livox_ros_driver2` | MID-360 驱动 |

## 外部依赖（17）

`src/dependencies/`：

- BehaviorTree.ROS2：`behaviortree_ros2`、`btcpp_ros2_interfaces`、`btcpp_ros2_samples`
- joint_state_publisher：`joint_state_publisher`、`joint_state_publisher_gui`
- rmoss_core：`rmoss_base`、`rmoss_cam`、`rmoss_core`、`rmoss_projectile_motion`、`rmoss_util`
- rmoss_gazebo：`rmoss_gz_base`、`rmoss_gz_bridge`、`rmoss_gz_cam`、`rmoss_gz_plugins`、`rmoss_gz_resources`
- `rmoss_interfaces`、`sdformat_tools`

## 工具包（3）

| 包 | 路径 | 说明 |
|---|---|---|
| `pcd2pgm` | `src/tools/pcd2pgm` | PCD 转 2D 栅格图 |
| `rosbag2_composable_recorder` | `src/tools/rosbag2_composable_recorder` | 可组合录包 |
| `teleop_gimbal_keyboard` | `src/tools/teleop_gimbal_keyboard` | 键盘→JointState 云台遥控 |

## 入口与包的对应

四个正式入口只依赖上述包的一部分：

- 实车导航：`srm27_nav_bringup` + `srm27_minco_*` + `srm27_nav_plugins` +
  `srm27_chassis_control` + `srm27_nav_protocol` + `fake_vel_transform` + 定位前端；
- 仿真导航：上述 + `srm27_gazebo_simulator`；
- 实车建图：`srm27_nav_bringup` + `srm27_robot_description` + `point_lio`/`frontend` +
  `loam_interface`/`zlio_bridge` + `sensor_scan_generation` + `slam_toolbox`；
- 仿真建图：上述 + `srm27_gazebo_simulator` + `ign_sim_pointcloud_tool`。

> 包路径本轮保持不变。是否把 `srm27_navigation` 拆成多个源码目录，见
> [`../records/2026-11-14-workspace-cleanup.md`](../records/2026-11-14-workspace-cleanup.md) §9。
