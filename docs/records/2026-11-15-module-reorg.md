# 2026-11-15 ROS 包目录模块化重组

> 状态：已完成
> 范围：`src/` 下 29 个 ROS 包的目录归属
> 未改动：ROS 包名、`package.xml` 包名、launch 入口名、算法实现

## 1. 目标

按“七个业务模块 + 支撑目录”整理目录，但不把代码合并成巨型 ROS 包：

- 业务模块：感知定位前端/后端、轨迹规划前端/后端、控制器、决策、仿真；
- 支撑目录：接口、系统装配、第三方依赖。

ROS 包仍保持独立，只调整目录归属和依赖方向。

## 2. 目标结构

```text
src/
├── interfaces/
│   ├── localization_interfaces
│   └── rm_decision_interfaces
├── localization/
│   ├── frontend/
│   │   ├── frontend                 # ZLIO
│   │   ├── point_lio
│   │   ├── livox_ros_driver2
│   │   ├── loam_interface
│   │   └── pointcloud_to_laserscan
│   └── backend/
│       ├── small_gicp_relocalization
│       └── sensor_scan_generation
├── planner/
│   ├── frontend/
│   │   ├── terrain_analysis
│   │   ├── terrain_analysis_ext
│   │   └── srm27_nav_plugins
│   └── backend/
│       ├── srm27_minco_core
│       ├── srm27_minco_vendor
│       └── srm27_qpoases_vendor
├── controller/
│   ├── srm27_minco_controller
│   ├── srm27_omni_pid_controller
│   ├── fake_vel_transform
│   ├── srm27_chassis_control
│   ├── srm27_nav_protocol
│   └── srm27_teleop_twist_joy
├── decision/
│   └── srm27_behavior
├── simulation/
│   ├── srm27_gazebo_simulator
│   ├── srm27_robot_description
│   └── ign_sim_pointcloud_tool
├── bringup/
│   ├── srm27_nav_bringup
│   └── srm27_navigation
└── dependencies/
    └── BehaviorTree.ROS2
```

`src/localization/frontend/frontend` 是“感知定位前端”分组下的
ZLIO 包 `frontend`；目录层级看起来重复，但包名仍是 `frontend`。

## 3. 包归属

| 模块 | 包 |
|---|---|
| 接口 | `localization_interfaces`、`rm_decision_interfaces` |
| 感知定位前端 | `frontend`、`point_lio`、`livox_ros_driver2`、`loam_interface`、`pointcloud_to_laserscan` |
| 感知定位后端 | `small_gicp_relocalization`、`sensor_scan_generation` |
| 轨迹规划前端 | `terrain_analysis`、`terrain_analysis_ext`、`srm27_nav_plugins` |
| 轨迹规划后端 | `srm27_minco_core`、`srm27_minco_vendor`、`srm27_qpoases_vendor` |
| 控制器 | `srm27_minco_controller`、`srm27_omni_pid_controller`、`fake_vel_transform`、`srm27_chassis_control`、`srm27_nav_protocol`、`srm27_teleop_twist_joy` |
| 决策 | `srm27_behavior` |
| 仿真 | `srm27_gazebo_simulator`、`srm27_robot_description`、`ign_sim_pointcloud_tool` |
| 装配 | `srm27_nav_bringup`、`srm27_navigation` |
| 依赖 | `BehaviorTree.ROS2`（`behaviortree_ros2`、`btcpp_ros2_interfaces`） |

## 4. 同步修改

- 全部使用 `git mv`，未改包名；
- `dependencies.repos`：
  - 删除 `src/srm27_navigation` 条目（该仓库的包已拆到新模块，不能再用它整仓覆盖）；
  - `src/srm27_behavior` → `src/decision/srm27_behavior`；
  - `src/dependencies/BehaviorTree.ROS2` → `src/dependencies/BehaviorTree.ROS2`；
- 脚本里的 `PKG_SRC`、`SIM_SRC`、`PROTOCOL_CFG`、PCD 搜索路径更新到新目录；
- 测试里的工作区路径常量更新：
  - `srm27_nav_protocol/test/test_feedback_protocol.py` 的 `WORKSPACE_ROOT`；
  - `test_speed_limits.py` 的 chassis 参数路径；
  - `test_gimbal_tree_direction.py` 的协议配置路径；
  - `test_point_lio_gravity_matches_model.py` 的 URDF 路径；
- `.gitignore` 的 Point-LIO PCD 与 nav_bringup 地图/PCD 规则更新；
- 原 `src/srm27_navigation/README.md` / `README_en.md` 归档到 `docs/archive/`；
- 旧 `srm27_navigation/LICENSE` 保留在 `src/bringup/srm27_navigation/LICENSE`。

## 5. 已知边界

1. `srm27_nav_plugins` 的 **测试**通过 CMake 读取
   `src/bringup/srm27_nav_bringup/behavior_trees/navigate_through_poses_route_aware.xml`。
   这仍是一个“规划插件测试依赖装配层文件”的反向依赖，下一轮应把该 XML 抽到共享测试数据或
   planning 包内。
2. `zlio_bridge.py` 仍在 `srm27_nav_bringup/scripts/`，按计划应抽成独立定位适配包。
3. `srm27_nav_plugins` 仍同时包含 costmap、GoalChecker、行为树插件，后续拆成
   `srm27_planning_plugins` 与 `srm27_decision_plugins`。
4. 本轮不改包名；包名统一放到后续单独提交，并保留一个版本的兼容入口。

## 6. 验证

构建前因 CMakeCache 仍指向旧目录，已清理 `build/`、`install/` 后重新构建。

- `colcon list`：29 个包，路径全部为新的模块目录；
- `colcon build --symlink-install --cmake-args -DBUILD_TESTING=ON`：29/29 成功，最终布局 clean build 约 3 分 26 秒；
- `srm27_nav_bringup`：14/14 通过；
- `srm27_nav_protocol`：2/2 通过；
- 代码测试（排除本机缺失 `.clang-format` 的 lint 项）：
  - `srm27_minco_core` 13/13；
  - `srm27_minco_controller` 8/8；
  - `srm27_nav_plugins` 3/4：`test_intensity_voxel_layer` 在 clean rebuild 后稳定失败
    （清空静态观测后期望 `FREE_SPACE`，实际 cost=114），其余 3 个 gtest 通过；
    该测试源码未在本轮修改，先记为待查；
  - `srm27_chassis_control` 通过；
  - `fake_vel_transform` 2/2；
  - `sensor_scan_generation` 1/1；
- 四个正式入口 `ros2 launch ... --show-args` 全部成功；
- `start_real_nav.sh -n`、`start_sim_slam.sh` dry-run 正常，路径已指向新模块；
- `docs` 中当前文档链接检查通过；`docs/archive` 的历史 README 按当时目录保留，
  已在文首标注“历史资料”。

已知问题与本次目录移动无关：部分包缺少 `.clang-format`，`localization_interfaces`
的 `wire_roundtrip_test` 仍受本机 `launch_testing/pluggy` 版本不兼容影响；
`srm27_nav_plugins` 的 intensity layer 测试需单独排查。
